#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import cv2
import numpy as np
import json
import math
import time
import tf_transformations
from collections import deque
import os
import pathlib
import sys
import uuid

# ── 共享配置：从主空间根定位 runtime_config（外参与权重统一来源）──
# 明确失败优于静默回退：runtime_config / 活动外参缺失或非法时直接报错退出。
_WS_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_WS_ROOT) not in sys.path:
    sys.path.insert(0, str(_WS_ROOT))
try:
    import runtime_config as _rc
except ImportError as _exc:  # pragma: no cover
    raise RuntimeError(
        f"无法导入主空间 runtime_config（期望 {_WS_ROOT}/runtime_config.py）: {_exc}"
    ) from _exc

# 模型权重：models/yolo/fruits12.pt（不再硬编码 orange_dataset 路径）
_MODEL_PATH = str(_rc.resolve_resource("yolo_weights"))
# 活动外参：config/calibration/d455_eye_to_hand.json（缺失/非法明确失败）
_D455_HANDEYE = _rc.load_handeye("d455")
from camera_geometry import CameraGeometry, image_qos, fresh_image
from runtime_geometry import object_centroid_from_rgbd, short_axis_tool_rotation
from vision.tracker import (ClassVoteStabilizer, Detection, PointKalmanCV,
                            ScalarKalmanCV, deduplicate_detections,
                            expand_detection, iou_xyxy, merge_fragment_detections,
                            nearest_cross_camera_match, refine_bbox_grabcut)


class ObjectTrack:
    """Track state with constant-velocity Kalman filtering and class voting."""

    def __init__(self, track_id, class_name, obs, frame_count, alpha=0.25):
        self.track_id = track_id
        self.class_name = class_name
        self.class_votes = ClassVoteStabilizer(class_name)
        self.stable_class = class_name
        self.cross_class = None
        self.cross_class_until = 0.
        self.alpha = alpha
        self.last_center_u = obs['center_u']
        self.last_center_v = obs['center_v']
        self.kf_u = ScalarKalmanCV(obs['center_u'], measurement_var=16., process_var=20.)
        self.kf_v = ScalarKalmanCV(obs['center_v'], measurement_var=16., process_var=20.)
        self.s_x1 = float(obs['x1'])
        self.s_y1 = float(obs['y1'])
        self.s_x2 = float(obs['x2'])
        self.s_y2 = float(obs['y2'])
        self.s_depth = obs.get('depth')
        self.s_cam = list(obs['camera_coords']) if obs.get('camera_coords') is not None else None
        self.s_ee = list(obs['ee_coords']) if obs.get('ee_coords') is not None else None
        self.s_base = list(obs['base_coords']) if obs.get('base_coords') is not None else None
        self.base_kf = PointKalmanCV(self.s_base) if self.s_base is not None else None
        self.last_update_frame = frame_count
        self.miss_count = 0
        self.hits = 1
        self.confidence = obs.get('conf', 0.0)
        self.last_obs = obs

    def update(self, obs, frame_count, class_name, alpha=None, dt=0.1):
        a = alpha if alpha is not None else self.alpha
        self.class_name = class_name
        self.stable_class = self.class_votes.update(class_name)
        if self.cross_class and time.monotonic() < self.cross_class_until:
            self.stable_class = self.cross_class
        center_u = self.kf_u.update(obs['center_u'], dt)
        center_v = self.kf_v.update(obs['center_v'], dt)
        width = a * (float(obs['x2']) - float(obs['x1'])) + (1 - a) * (self.s_x2 - self.s_x1)
        height = a * (float(obs['y2']) - float(obs['y1'])) + (1 - a) * (self.s_y2 - self.s_y1)
        self.last_center_u, self.last_center_v = center_u, center_v
        self.s_x1, self.s_x2 = center_u - width / 2., center_u + width / 2.
        self.s_y1, self.s_y2 = center_v - height / 2., center_v + height / 2.
        d = obs.get('depth')
        if d is not None:
            self.s_depth = d if self.s_depth is None else a * d + (1 - a) * self.s_depth
        cam = obs.get('camera_coords')
        if cam is not None:
            self.s_cam = list(cam) if self.s_cam is None else [a * cam[i] + (1 - a) * self.s_cam[i] for i in range(3)]
        ee = obs.get('ee_coords')
        if ee is not None:
            self.s_ee = list(ee) if self.s_ee is None else [a * ee[i] + (1 - a) * self.s_ee[i] for i in range(3)]
        base = obs.get('base_coords')
        if base is not None:
            if self.base_kf is None:
                self.base_kf = PointKalmanCV(base)
                self.s_base = list(base)
            else:
                self.s_base = self.base_kf.update(base, dt)
        self.last_update_frame = frame_count
        self.miss_count = 0
        self.hits += 1
        self.confidence = .7 * self.confidence + .3 * float(obs.get('conf', self.confidence))
        self.last_obs = obs

# ==========================================================================
# 双相机默认参数
#   d455  = Intel RealSense D455  (默认值 = 改造前的硬编码值, 行为保持不变)
#   dabai = Orbbec Dabai DC1      (默认关闭, 需显式 -p dabai.enabled:=true)
# 每项都可用 ROS 参数 "<相机名>.<字段>" 覆盖, 例如:
#   python3 realsense_yolo_node.py --ros-args -p d455.enabled:=false -p dabai.enabled:=true
# ==========================================================================
CAMERA_DEFAULTS = {
    "d455": {
        "enabled": True,
        # ── 话题 ──
        "rgb_topic": "/camera/camera/color/image_raw",
        "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
        "camera_info_topic": "/camera/camera/color/camera_info",
        "exposure_ctrl_topic": "/camera/exposure_ctrl",
        # ── 输出 ── "" → 发布到根命名空间 (与改造前逐字符一致)
        "output_prefix": "",
        "frame_child_id": "detected_object",
        # ── 眼在手外外参 (统一来源: config/calibration/d455_eye_to_hand.json) ──
        "cam_to_base_translation": list(_D455_HANDEYE["position"]),
        "cam_to_base_quat": list(_D455_HANDEYE["quaternion_normalized"]),
        # ── 内参 fallback (camera_info 到达后被覆盖) ──
        "intrinsics": [378.394659861614, 379.366916262423, 330.140969430714,
                       246.095530649072, 1.25667775035477],
        "dist_coeffs": [-0.0579748391767018, 0.104170020380745, -0.000760277417774888,
                        0.000473717313218756, -0.0946382536306512],
        "use_depth_camera": True,
        # ── 偏差补偿 ──
        "depth_offset_m": 0.0,
        "x_offset_m": 0.0,
        "y_offset_m": 0.0,
        "z_offset_m": 0.0,
        # ── 曝光后端: 用 ros2 param set 改 realsense-ros 动态参数 ──
        "exposure_backend": "realsense_param",
        "driver_node": "/camera/camera",
        "auto_exposure_param_name": "rgb_camera.enable_auto_exposure",
        "exposure_param_name": "rgb_camera.exposure",
        "exposure_service_prefix": "",
        "set_exposure_service_type": "",
    },
    "dabai": {
        # 默认关闭: 不传任何新参数时, 节点行为与改造前完全一致
        "enabled": False,
        # ── 话题 (驱动须以 camera_name:=dabai 启动) ──
        "rgb_topic": "/dabai/color/image_raw",
        # depth_registration:=true 时 /dabai/depth/image_raw 即为"对齐到彩色"的深度
        "depth_topic": "/dabai/depth/image_raw",
        "camera_info_topic": "/dabai/color/camera_info",
        "exposure_ctrl_topic": "/dabai/exposure_ctrl",
        # ── 输出 ──
        "output_prefix": "dabai",
        "frame_child_id": "detected_object_dabai",
        # ⚠️ 占位值: 沿用 D455 外参, 必须用 Dabai 自己的眼在手外标定结果覆盖
        "cam_to_base_translation": [0.0, 0.0, 0.0],
        "cam_to_base_quat": [0.0, 0.0, 0.0, 1.0],
        # ── 内参: 无硬编码 fallback, 完全依赖 /dabai/color/camera_info ──
        "intrinsics": [0.0, 0.0, 0.0, 0.0, 0.0],
        "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
        "use_depth_camera": True,
        "depth_offset_m": 0.0,
        "x_offset_m": 0.0,
        "y_offset_m": 0.0,
        "z_offset_m": 0.0,
        # ── 曝光后端: Orbbec 驱动服务 (ros2 param set 是 RealSense 专有, 对 Orbbec 无效) ──
        "exposure_backend": "orbbec_service",
        "driver_node": "/dabai/dabai",
        "auto_exposure_param_name": "",
        "exposure_param_name": "",
        "exposure_service_prefix": "/dabai",
        # 实测确认: ros2 service type /dabai/set_color_exposure
        "set_exposure_service_type": "orbbec_camera_msgs/srv/SetInt32",
    },
}


class CameraState:
    """单个相机的全部运行状态。

    改造前这些状态全部挂在 node 的 self.* 上 (只能支持一个相机), 现在
    每个相机一份, 由 detect_and_publish(cam) 等函数按相机读写。
    """

    def __init__(self, name):
        self.name = name
        self.motion_session = uuid.uuid4().hex
        self.enabled = False
        self.use_depth_camera = True

        # ── 话题 ──
        self.rgb_topic = ""
        self.depth_topic = ""
        self.camera_info_topic = ""
        self.exposure_ctrl_topic = ""

        # ── 输出 ──
        self.output_prefix = ""          # "" → 根命名空间; "dabai" → /dabai/...
        self.frame_child_id = "detected_object"

        # ── 眼在手外外参 (常量变换) ──
        self.transform_cam_to_base = np.eye(4)
        self.transform_base_to_cam = np.eye(4)
        self.cam_to_base_translation = np.zeros(3)
        self.cam_to_base_quat = np.array([0.0, 0.0, 0.0, 1.0])

        # ── 内参 (camera_matrix=None 表示还没有内参, 不能做单目反推) ──
        self.fx = self.fy = self.cx = self.cy = self.skew = 0.0
        self.camera_matrix = None
        self.dist_coeffs = np.zeros(5)
        self.cam_info_received = False

        # ── 偏差补偿 ──
        self.depth_offset_m = 0.0
        self.x_offset_m = 0.0
        self.y_offset_m = 0.0
        self.z_offset_m = 0.0

        # ── 图像缓存 ──
        self.latest_rgb = None            # BGR ndarray
        self.latest_rgb_stamp = None      # 秒
        self.latest_depth = None          # uint16 ndarray (mm)
        self.latest_depth_stamp = 0.0
        # 深度帧缓存 (时间戳, 图), 用于按 RGB 时间戳就近配对。
        # DaBai DC1 硬件不支持传感器同步 (enable_frame_sync 会让设备初始化失败),
        # 且两路时间戳对齐 (实测 |Δ| 中位 0.7ms)、但深度消息比彩色晚 ~64ms 到达,
        # 所以在处理彩色时"时间上匹配的那帧深度"通常还没发布出来。
        # 对策见 depth_pair_delay 与 _pick_depth。
        self.depth_buffer = deque(maxlen=8)
        self.last_depth_dt_ms = None      # 本帧选中的深度与 RGB 的时间差 (ms)
        self.held_rgb = None              # depth_pair_delay 用: 上一轮暂存的 (stamp, BGR 图)
        self.rgb_ready = False
        self.last_processed_stamp = None  # 避免同一帧重复推理
        self.min_track_hits = 2
        self.preprocess_clahe = False
        self.clahe_clip_limit = 2.
        self.inference_size = 640
        self.tta = False
        self.bbox_expand_ratio = 0.
        self.merge_fragments = False
        self.refine_bbox = False

        # ── 滤波 (EMA + 帧间匹配) ──
        self.tracks = {}                  # {track_id: ObjectTrack}
        self.next_track_id = 0

        # ── 计数 / 诊断 ──
        self.frame_count = 0
        self.detection_count = 0
        self.depth_fallback_count = 0
        self.depth_total_count = 0
        self.depth_diag_logged = False

        # ── 曝光 ──
        self.exposure_backend = ""
        self.driver_node = ""
        self.auto_exposure_param_name = ""
        self.exposure_param_name = ""
        self.exposure_service_prefix = ""
        self.set_exposure_service_type = ""
        self.auto_exposure = True
        self.current_exposure = 100
        self.exposure_min = 1
        self.exposure_max = 10000

        # ── 发布器 ──
        self.pub_object_pose = None
        self.pub_detection_info = None
        self.pub_annotated = None
        self.pub_object_marker = None
        self.pub_object_markers = None
        self.marker_ids = set()
        self.pub_object_pose_stamped = None

        # ── GUI ──
        self.show_gui_window = True
        self.window_name = ""


class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")

        # ================== 模型配置 ==================
        # 新权重支持 12 类水果检测 (apple / green_apple / banana / yellow_pear / green_pear /
        # strawberry / orange / peach / lemon / mango / star_fruit / pomegranate)
        self.MODEL_PATH = _MODEL_PATH  # models/yolo/fruits12.pt（共享配置解析）
        # 置信度阈值: 与另一台电脑 (model(frame) 默认 conf=0.25) 保持一致。
        # 之前 0.8 太高, 过滤掉大量有效检测导致识别率低。
        self.CONF_THRESHOLD = 0.25

        # ================== 显示模式 ==================
        # True: standalone 模式，弹出 OpenCV 窗口；False: GUI 内嵌模式，仅发布 ROS topic
        self.show_gui_window = bool(self.declare_parameter("show_gui_window", True).value)

        # ================== 已知物体真实尺寸 (单目深度反推用) ==================
        # 每个类别的真实物理尺寸 (单位: 米)。来源: 用户实测。
        # length=长(对应像素宽度), width=宽(深度方向), height=高(对应像素高度)
        # 单目深度估计用 length (像素宽度方向) 和 height (像素高度方向) 反推 depth。
        self.OBJECT_DIMENSIONS = {
            "apple":       {"length_m": 0.076, "width_m": 0.076, "height_m": 0.065},
            "green_apple": {"length_m": 0.0836, "width_m": 0.0836, "height_m": 0.0715},
            "banana":      {"length_m": 0.18, "width_m": 0.030, "height_m": 0.030},
            "yellow_pear": {"length_m": 0.090, "width_m": 0.075, "height_m": 0.074},
            "green_pear":  {"length_m": 0.090, "width_m": 0.075, "height_m": 0.074},
            "strawberry":  {"length_m": 0.090, "width_m": 0.060, "height_m": 0.046},
            "orange":      {"length_m": 0.069, "width_m": 0.069, "height_m": 0.055},
            "peach":       {"length_m": 0.075, "width_m": 0.076, "height_m": 0.065},
            "lemon":       {"length_m": 0.080, "width_m": 0.054, "height_m": 0.0648},
            "mango":       {"length_m": 0.090, "width_m": 0.070, "height_m": 0.060},
            "star_fruit":  {"length_m": 0.080, "width_m": 0.060, "height_m": 0.040},
            "pomegranate": {"length_m": 0.090, "width_m": 0.090, "height_m": 0.080},
        }
        # 默认尺寸 (兜底: 类别未在字典中时使用, 用 orange 的尺寸)
        self.known_object_width_m  = float(self.declare_parameter("known_object_width_m",  0.069).value)
        self.known_object_height_m = float(self.declare_parameter("known_object_height_m", 0.055).value)

        # ================== 通用参数 (两路相机共享) ==================
        # RViz 可视化用的基座坐标系 frame_id
        self.base_frame_id = self.declare_parameter("base_frame_id", "base_link").value

        # ── 滤波参数 (EMA + 空间关联) ──
        # filter_alpha: EMA 新观测权重, 越小越平滑 (0.1=强滤波, 0.5=弱滤波, 0.0=完全冻结)
        # filter_match_dist_px: 帧间匹配阈值 (像素), 同类物体中心距离 < 此值视为同一物体
        # filter_max_misses: 连续多少帧未匹配到则删除 track
        self.filter_alpha = float(self.declare_parameter("filter_alpha", 0.25).value)
        self.filter_match_dist_px = float(self.declare_parameter("filter_match_dist_px", 80.0).value)
        self.filter_max_misses = int(self.declare_parameter("filter_max_misses", 10).value)

        # 检测循环周期 (秒): 单次定时器遍历所有启用相机
        self.detect_period_s = float(self.declare_parameter("detect_period_s", 0.1).value)

        # 深度/彩色时间对齐: True 时每轮处理"上一轮暂存的彩色帧"(固定延迟一个周期),
        # 让与该帧时间对齐的深度帧有机会到达 —— 深度消息比彩色晚 ~64ms 到达本节点,
        # 不等就会用到 ~50ms 之前的老深度。代价: 检测结果晚 ~100ms 输出。
        # 实测: RGB↔深度时间差 中位 50ms → 8ms, 吞吐不变。
        # 关掉即回到"处理当前最新帧、用最新深度"的旧行为。
        self.depth_pair_delay = bool(self.declare_parameter("depth_pair_delay", True).value)
        # Keep inference on the workstation GPU. A missing CUDA runtime must be
        # visible instead of silently falling back to CPU and starving cameras.
        self.yolo_device = str(self.declare_parameter("yolo_device", "cuda:0").value)

        self.status_printed = False
        self.active_cam_name = None      # 键盘/曝光操作作用的相机 (按 1/2 切换)

        # ================== 双相机状态 ==================
        # 内参 / 外参 / 话题 / 偏移补偿 均按相机声明, 默认值见 CAMERA_DEFAULTS。
        # 覆盖方式: --ros-args -p d455.enabled:=false -p dabai.enabled:=true
        self.cameras = {
            name: self._build_camera(name, defaults)
            for name, defaults in CAMERA_DEFAULTS.items()
        }
        self._d455_fusion_objects = []
        self._d455_fusion_stamp = 0.
        for cam in self.cameras.values():
            self._log_camera_config(cam)

        # ── 补偿热更新（计划 4.4）──
        # 参数回调只做原子验证，验证阶段不改 CameraState 或 track；
        # 合法值进入 pending，在下一帧边界由 _apply_pending_offsets 一次性提交。
        self._pending_offsets = {}   # {cam_name: {field: value}}
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.camera_geometry = CameraGeometry(self)

        # 说明: 原来的单相机状态 (latest_rgb / _tracks / use_depth_camera /
        # depth_offset_m / x_offset_m / y_offset_m / z_offset_m / 计数等) 已全部
        # 迁移到 CameraState, 每个相机一份, 见 self.cameras。
        
        # ================== 相机发布/订阅 (仅启用的相机) ==================
        # RViz TF 广播 (两路共用同一个 broadcaster, child_frame_id 按相机区分)
        self.tf_broadcaster = TransformBroadcaster(self)

        for cam in self.cameras.values():
            if not cam.enabled:
                self.get_logger().info(
                    f"[{cam.name}] enabled=false → 不创建订阅/发布 (不影响另一路)")
                continue

            # 发布器: d455 的 output_prefix="" → 话题名与改造前逐字符一致
            p = cam.output_prefix
            cam.pub_object_pose = self.create_publisher(
                Float64MultiArray, self._topic(p, "object_3d_position"), 10)
            cam.pub_detection_info = self.create_publisher(
                String, self._topic(p, "detection_info"), 10)
            cam.pub_annotated = self.create_publisher(
                CompressedImage, self._topic(p, "yolo/annotated_image/compressed"), 1)
            cam.pub_object_marker = self.create_publisher(
                Marker, self._topic(p, "object_marker"), 10)
            cam.pub_object_markers = self.create_publisher(
                MarkerArray, self._topic(p, "objects_markers"), 10)
            cam.pub_object_pose_stamped = self.create_publisher(
                PoseStamped, self._topic(p, "object_pose"), 10)

            # 订阅器: 闭包绑定相机, 回调统一带 cam 参数
            self.create_subscription(
                Image, cam.rgb_topic, lambda m, c=cam: self.rgb_cb(c, m), image_qos(self,cam.rgb_topic,cam.name))
            self.create_subscription(
                Image, cam.depth_topic, lambda m, c=cam: self.depth_cb(c, m), image_qos(self,cam.depth_topic,cam.name))
            self.create_subscription(
                CameraInfo, cam.camera_info_topic,
                lambda m, c=cam: self.camera_info_cb(c, m), image_qos(self,cam.camera_info_topic,cam.name))
            self.create_subscription(
                String, cam.exposure_ctrl_topic,
                lambda m, c=cam: self._exposure_ctrl_cb(c, m), 10)

            # OpenCV 窗口 (仅 standalone 模式), 每个相机一个窗口
            cam.show_gui_window = bool(self.show_gui_window)
            if cam.show_gui_window:
                cam.window_name = f"Object Detection - {cam.name}"
                cv2.namedWindow(cam.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(cam.window_name, 960, 540)

            self.get_logger().info(
                f"[{cam.name}] 订阅: {cam.rgb_topic} | {cam.depth_topic} | "
                f"{cam.camera_info_topic}")
            self.get_logger().info(
                f"[{cam.name}] 发布: {self._topic(p, 'detection_info')} 等 5 个话题 "
                f"(TF child_frame: {cam.frame_child_id})")

        # 键盘/曝光操作默认作用于第一个启用的相机
        self.active_cam_name = next(
            (c.name for c in self.cameras.values() if c.enabled), None)

        # ================== YOLO 模型 (两路共用一份, 省显存) ==================
        self.get_logger().info("正在加载 YOLO 模型...")
        self.get_logger().info(
            f"📐 单目估计: 已知物体尺寸 W={self.known_object_width_m:.3f}m "
            f"H={self.known_object_height_m:.3f}m")
        from ultralytics import YOLO
        self.model = YOLO(self.MODEL_PATH)
        if self.yolo_device != "cpu":
            import torch
            if not torch.cuda.is_available():
                raise RuntimeError("YOLO 配置为使用 GPU，但当前 Python 环境无法访问 CUDA")
            self.get_logger().info(
                f"YOLO GPU: {torch.cuda.get_device_name(0)} ({self.yolo_device})")
        else:
            self.get_logger().warning("YOLO 已显式配置为 CPU，检测吞吐会明显降低")
        self.get_logger().info(f"✅ 模型加载成功，类别: {self.model.names}")

        # 定时器: 单个定时器遍历所有启用相机 (无新帧则跳过, 避免重复推理同一帧)
        self._detect_timer = self.create_timer(self.detect_period_s, self._detect_all)
        self.geometry_pub = self.create_publisher(String, '/workbench/vision_geometry', 10)
        self.geometry_timer = self.create_timer(1.0, self._publish_geometry)
        self.status_timer = self.create_timer(5.0, self.print_status)
        self._depth_status_timer = self.create_timer(5.0, self._depth_status_log)

        self.get_logger().info("✅ 节点启动，等待相机数据...")
        self.get_logger().info(f"🔍 检测阈值: {self.CONF_THRESHOLD}")
        self.get_logger().info("📷 眼在手外 (eye-to-hand): 相机变换为常量, 无需 TCP 位姿订阅")
        self.get_logger().info(f"🖼️  RViz TF 基座 frame: {self.base_frame_id}")
        self.get_logger().info(
            f"🎛️  键盘曝光控制作用于激活相机: {self.active_cam_name} (按 1/2 切换)")

        # 默认启用自动曝光 (相机未上线时命令会静默失败, 不影响节点启动)
        for cam in self.cameras.values():
            if cam.enabled:
                self.set_exposure(cam, True)

    # ==================== 双相机公共辅助 ====================
    @staticmethod
    def _topic(prefix, name):
        """拼接话题名: prefix="" → /name ; prefix="dabai" → /dabai/name"""
        return f"/{prefix}/{name}" if prefix else f"/{name}"

    # ── 补偿热更新（计划 4.4）: 水果仅相机 x/y/z/depth_offset_m ──
    _HOT_UPDATE_FIELDS = ("x_offset_m", "y_offset_m", "z_offset_m", "depth_offset_m")

    def _on_set_parameters(self, params):
        """参数热更新回调：原子验证有限数字，拒绝 NaN/Inf/非法类型。

        验证阶段不改 CameraState 或 track；合法值进入 pending 快照，
        在下一帧边界提交（一帧内全部点使用同一快照）。
        不自动写 YAML；相机拓扑/话题/外参变化仍需重启。
        """
        from rcl_interfaces.msg import SetParametersResult
        staged = {}
        for p in params:
            name = p.name
            if "." not in name:
                continue
            cam_name, field = name.split(".", 1)
            if cam_name not in self.cameras or field not in self._HOT_UPDATE_FIELDS + ("yolo_conf", "yolo_iou", "detect_period_s"):
                continue
            try:
                val = float(p.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f"{name} 必须是有限数值, 拒绝非法类型 "
                           f"{type(p.value).__name__}")
            if not math.isfinite(val):
                return SetParametersResult(
                    successful=False, reason=f"{name} 不能是 NaN/Inf")
            if field in self._HOT_UPDATE_FIELDS and val != 0.0:
                return SetParametersResult(successful=False, reason="标定模式禁止人工坐标补偿，偏置必须为0")
            if field in ("yolo_conf", "yolo_iou") and not 0.0 <= val <= 1.0:
                return SetParametersResult(successful=False, reason="置信度和重叠阈值应在0到1之间")
            if field == "detect_period_s" and val <= 0.:
                return SetParametersResult(successful=False, reason="检测周期必须大于零")
            staged.setdefault(cam_name, {})[field] = val
        # 整套原子提交：任一非法则整体拒绝（staged 未写入）
        for cam_name, fields in staged.items():
            self._pending_offsets.setdefault(cam_name, {}).update(fields)
        return SetParametersResult(successful=True)

    def _apply_pending_offsets(self, cam):
        """在帧边界提交热更新补偿快照；受影响相机的 track 清空，其他相机保留。"""
        pending = self._pending_offsets.pop(cam.name, None)
        if not pending:
            return
        changed = False
        for field, value in pending.items():
            if getattr(cam, field, None) != value:
                setattr(cam, field, value)
                changed = True
        if changed and any(field in self._HOT_UPDATE_FIELDS for field in pending):
            # 三维/深度平滑 track 清空: 下一帧体现完整补偿变化
            cam.tracks = {}
            cam.next_track_id = 0
            cam.motion_session = uuid.uuid4().hex
            cam.depth_fallback_count = 0
            self.get_logger().info(
                f"♻️ [{cam.name}] 热更新补偿已提交: "
                + ", ".join(f"{k}={v}" for k, v in sorted(pending.items()))
                + "（track 已清空，下一帧生效）")

    def _build_camera(self, name, defaults):
        """按 CAMERA_DEFAULTS 声明该相机全部 ROS 参数, 并构建 CameraState。

        参数覆盖方式: --ros-args -p <name>.<字段>:=<值>
        """
        cam = CameraState(name)
        d = defaults

        def p(field, default):
            return self.declare_parameter(f"{name}.{field}", default).value

        cam.enabled = bool(p("enabled", d["enabled"]))
        # 腕载Dabai的水果容易出现多个重叠框；使用现有YOLO NMS去重。
        cam.yolo_conf = float(p("yolo_conf", 0.22))
        cam.yolo_iou = float(p("yolo_iou", 0.45))
        cam.detect_period_s = float(p("detect_period_s", self.detect_period_s))
        cam.last_detect_at = -100.
        cam.yolo_agnostic_nms = bool(p("yolo_agnostic_nms", True))
        cam.min_track_hits = int(p("min_track_hits", 2))
        cam.preprocess_clahe = bool(p("preprocess_clahe", False))
        cam.clahe_clip_limit = float(p("clahe_clip_limit", 2.))
        cam.inference_size = int(p("inference_size", 640))
        cam.tta = bool(p("tta", False))
        cam.bbox_expand_ratio = float(p("bbox_expand_ratio", 0.))
        cam.merge_fragments = bool(p("merge_fragments", False))
        cam.refine_bbox = bool(p("refine_bbox", False))
        cam.use_depth_camera = bool(p("use_depth_camera", d["use_depth_camera"]))
        cam.rgb_topic = str(p("rgb_topic", d["rgb_topic"]))
        cam.depth_topic = str(p("depth_topic", d["depth_topic"]))
        cam.camera_info_topic = str(p("camera_info_topic", d["camera_info_topic"]))
        cam.exposure_ctrl_topic = str(p("exposure_ctrl_topic", d["exposure_ctrl_topic"]))
        cam.output_prefix = str(p("output_prefix", d["output_prefix"]))
        cam.frame_child_id = str(p("frame_child_id", d["frame_child_id"]))

        # 相机参考变换；Dabai每帧按图像时刻重新构建基座变换。
        if name == "dabai" and cam.enabled:
            he = _rc.load_handeye(name)
            d = dict(d, cam_to_base_translation=he["position"], cam_to_base_quat=he["quaternion_normalized"])
        cam.cam_to_base_translation = np.array(
            p("cam_to_base_translation", d["cam_to_base_translation"]), dtype=np.float64)
        cam.cam_to_base_quat = np.array(
            p("cam_to_base_quat", d["cam_to_base_quat"]), dtype=np.float64)
        if cam.cam_to_base_translation.shape != (3,) or cam.cam_to_base_quat.shape != (4,):
            raise ValueError(
                f"[{name}] cam_to_base_translation 需 3 个数, cam_to_base_quat 需 4 个数")
        cam.transform_cam_to_base = np.eye(4)
        cam.transform_cam_to_base[:3, :3] = self.quaternion_to_rotation_matrix(
            cam.cam_to_base_quat)
        cam.transform_cam_to_base[:3, 3] = cam.cam_to_base_translation
        cam.transform_base_to_cam = np.linalg.inv(cam.transform_cam_to_base)

        # ── 偏差补偿 ──
        cam.depth_offset_m = float(p("depth_offset_m", d["depth_offset_m"]))
        cam.x_offset_m = float(p("x_offset_m", d["x_offset_m"]))
        cam.y_offset_m = float(p("y_offset_m", d["y_offset_m"]))
        cam.z_offset_m = float(p("z_offset_m", d["z_offset_m"]))
        if any(getattr(cam, field) != 0. for field in self._HOT_UPDATE_FIELDS):
            raise ValueError("标定模式要求所有人工位置/深度偏置为0")

        # ── 内参 fallback (camera_info 到达后会被覆盖) ──
        fx, fy, cx, cy, skew = [float(v) for v in p("intrinsics", d["intrinsics"])]
        cam.fx, cam.fy, cam.cx, cam.cy, cam.skew = fx, fy, cx, cy, skew
        if fx > 0.0 and fy > 0.0:
            cam.camera_matrix = np.array([
                [fx, skew, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)
            cam.dist_coeffs = np.array(p("dist_coeffs", d["dist_coeffs"]), dtype=np.float64)
        else:
            cam.camera_matrix = None      # 无 fallback, 等 camera_info
            cam.dist_coeffs = np.zeros(5)

        # ── 曝光后端 ──
        cam.exposure_backend = str(p("exposure_backend", d["exposure_backend"]))
        cam.driver_node = str(p("driver_node", d["driver_node"]))
        cam.auto_exposure_param_name = str(
            p("auto_exposure_param_name", d["auto_exposure_param_name"]))
        cam.exposure_param_name = str(p("exposure_param_name", d["exposure_param_name"]))
        cam.exposure_service_prefix = str(
            p("exposure_service_prefix", d["exposure_service_prefix"]))
        cam.set_exposure_service_type = str(
            p("set_exposure_service_type", d["set_exposure_service_type"]))
        return cam

    def _log_camera_config(self, cam):
        """启动时打印单个相机的关键配置 (外参 / 内参 / 话题), 便于核对。"""
        rpy = tf_transformations.euler_from_quaternion([
            cam.cam_to_base_quat[0], cam.cam_to_base_quat[1],
            cam.cam_to_base_quat[2], cam.cam_to_base_quat[3]])
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"[{cam.name}] 手眼参考变换 (每帧使用图像时刻基座变换, enabled={cam.enabled}):")
        self.get_logger().info(
            f"   平移: X={cam.cam_to_base_translation[0]:.6f}m "
            f"Y={cam.cam_to_base_translation[1]:.6f}m "
            f"Z={cam.cam_to_base_translation[2]:.6f}m")
        self.get_logger().info(
            f"   四元数: qx={cam.cam_to_base_quat[0]:.6f} qy={cam.cam_to_base_quat[1]:.6f} "
            f"qz={cam.cam_to_base_quat[2]:.6f} qw={cam.cam_to_base_quat[3]:.6f}")
        self.get_logger().info(
            f"   RPY: roll={math.degrees(rpy[0]):.2f}° "
            f"pitch={math.degrees(rpy[1]):.2f}° yaw={math.degrees(rpy[2]):.2f}°")
        if cam.camera_matrix is not None:
            self.get_logger().info(
                f"   内参 fallback: fx={cam.fx:.4f} fy={cam.fy:.4f} "
                f"cx={cam.cx:.4f} cy={cam.cy:.4f}")
        else:
            self.get_logger().info(
                f"   内参: 无 fallback, 等待 {cam.camera_info_topic} 覆盖")
        self.get_logger().info("=" * 60)

    def _enabled_cameras(self):
        return [c for c in self.cameras.values() if c.enabled]

    def _active_cam(self):
        """键盘/曝光操作作用的相机 (active_cam_name 失效时回落到第一个启用相机)。"""
        cam = self.cameras.get(self.active_cam_name)
        if cam is not None and cam.enabled:
            return cam
        enabled = self._enabled_cameras()
        return enabled[0] if enabled else None

    def _publish_geometry(self):
        rows={}
        for cam in self._enabled_cameras():
            rows[cam.name]=dict(position=cam.cam_to_base_translation.tolist(),orientation=cam.cam_to_base_quat.tolist(),
                k=cam.camera_matrix.ravel().tolist() if cam.cam_info_received else [],d=cam.dist_coeffs.ravel().tolist() if cam.cam_info_received else [],
                vision={f'{cam.name}.{field}':getattr(cam,field) for field in ('yolo_conf','yolo_iou','detect_period_s')})
        self.geometry_pub.publish(String(data=json.dumps(rows)))

    def _detect_all(self):
        """定时器入口: 遍历所有启用相机, 只对"有新帧"的相机做推理, 最后统一处理键盘。

        depth_pair_delay 开启时 (默认), 每轮处理的是**上一轮暂存**的那帧彩色:
        它已存在一个检测周期 (~100ms), 与之时间对齐的深度帧此时必然已到达
        (深度消息从设备到本节点比彩色晚 ~64ms)。这样吞吐不变、只多一个周期延迟,
        却能把 RGB↔深度的时间差从 ~50ms 压到 ~8ms。

        注意不能改成"发现深度没到就跳过本轮": 那样会一直盯着最新帧等, 而
        "深度已追上"要求彩色已老 ~64ms, 时间窗口极窄, 实测吞吐会从 ~10Hz 崩到 1.4Hz。
        """
        for cam in self._enabled_cameras():
            try:
                # 帧边界：先提交上一轮的热更新补偿（一帧内所有点使用同一快照）
                self._apply_pending_offsets(cam)
                period = min(c.detect_period_s for c in self._enabled_cameras())
                if self._detect_timer.timer_period_ns != int(period*1e9):
                    self._detect_timer.timer_period_ns = int(period*1e9)
                now = time.monotonic()
                if now-cam.last_detect_at < cam.detect_period_s*.95:
                    continue
                cam.last_detect_at = now
                if cam.latest_rgb is None:
                    continue
                if self.depth_pair_delay and cam.use_depth_camera:
                    held = cam.held_rgb                       # 先取上一轮暂存的帧
                    cam.held_rgb = (cam.latest_rgb_stamp, cam.latest_rgb)
                    if held is None:
                        continue                              # 第一轮只暂存, 下一轮才处理
                    stamp, img = held
                    if stamp is None or stamp == cam.last_processed_stamp:
                        continue
                    cam.last_processed_stamp = stamp
                    self.detect_and_publish(cam, rgb=img, rgb_stamp=stamp)
                    continue
                stamp = cam.latest_rgb_stamp
                if stamp is None or stamp == cam.last_processed_stamp:
                    continue
                cam.last_processed_stamp = stamp
                self.detect_and_publish(cam)
            except Exception as exc:
                self.get_logger().error(f'[{cam.name}] 本帧处理失败: {exc}', throttle_duration_sec=2.)
                self._publish_invalid(cam, 'camera_processing_error')
        if self.show_gui_window:
            self._handle_gui_keys()

    def _handle_gui_keys(self):
        """waitKey 每轮只调用一次 (两个窗口各调一次会互相抢键)。

        1/2 切换激活相机; +/- 调曝光; a 切自动/手动; q/ESC 退出。
        """
        key = cv2.waitKey(1) & 0xFF
        if key == 255:
            return
        if key == ord('q') or key == 27:
            self.get_logger().info("用户退出")
            if rclpy.ok():
                rclpy.shutdown()
            return
        if key == ord('1') or key == ord('2'):
            names = [c.name for c in self._enabled_cameras()]
            idx = key - ord('1')
            if idx < len(names):
                self.active_cam_name = names[idx]
                self.get_logger().info(f"🎛️  激活相机切换为: {self.active_cam_name}")
            return
        active = self._active_cam()
        if active is None:
            return
        if key == ord('a'):
            self.set_exposure(active, not active.auto_exposure)
        elif key == ord('+') or key == ord('='):
            if not active.auto_exposure:
                step = 100 if active.current_exposure >= 100 else 10
                self.set_exposure(active, False, active.current_exposure + step)
        elif key == ord('-') or key == ord('_'):
            if not active.auto_exposure:
                step = 100 if active.current_exposure >= 100 else 10
                self.set_exposure(active, False, active.current_exposure - step)

    # ==================== 滤波: track 匹配与清理 ====================
    def _find_matching_track(self, cam, class_name, center_u, center_v, bbox, exclude_ids=None):
        """在现有 tracks 中找同类且像素中心距离最近的 track (贪心匹配)。
        exclude_ids: 本帧已匹配过的 track_id 集合, 避免重复匹配。
        返回 ObjectTrack 或 None。"""
        exclude_ids = exclude_ids or set()
        best_track = None
        best_dist = self.filter_match_dist_px
        best_other = None
        best_other_dist = self.filter_match_dist_px * 1.25
        for track in cam.tracks.values():
            if track.track_id in exclude_ids:
                continue
            dx = track.last_center_u - center_u
            dy = track.last_center_v - center_v
            dist = math.sqrt(dx * dx + dy * dy)
            if track.class_name == class_name and dist < best_dist:
                best_dist = dist
                best_track = track
            elif track.class_name != class_name and dist < best_other_dist:
                if iou_xyxy(bbox, (track.s_x1, track.s_y1, track.s_x2, track.s_y2)) >= .20:
                    best_other_dist = dist
                    best_other = track
        return best_track or best_other

    def _cleanup_stale_tracks(self, cam, frame_count):
        """删除连续 filter_max_misses 帧未匹配的 track, 并对未匹配 track miss_count++。"""
        stale_ids = [
            tid for tid, t in cam.tracks.items()
            if t.miss_count >= self.filter_max_misses
        ]
        for tid in stale_ids:
            del cam.tracks[tid]
        for t in cam.tracks.values():
            if t.last_update_frame < frame_count:
                t.miss_count += 1

    # ==================== 物体尺寸查询 ====================
    def _get_class_dimensions(self, class_name):
        """按类别名查询物体真实尺寸 (length_m, height_m)。
        类别名做小写匹配, 找不到时回落到默认 known_object_*_m。"""
        if not class_name:
            return self.known_object_width_m, self.known_object_height_m
        key = class_name.strip().lower()
        dim = self.OBJECT_DIMENSIONS.get(key)
        if dim is None:
            # 模糊匹配: 类别名包含字典 key (例如 "green apple" 包含 "apple")
            for k, v in self.OBJECT_DIMENSIONS.items():
                if k in key or key in k:
                    dim = v
                    break
        if dim is None:
            self.get_logger().warning(
                f"未注册的类别 '{class_name}', 回落到默认尺寸 "
                f"W={self.known_object_width_m:.3f}m H={self.known_object_height_m:.3f}m",
                throttle_duration_sec=5.0
            )
            return self.known_object_width_m, self.known_object_height_m
        return dim["length_m"], dim["height_m"]

    # ==================== 曝光调节 ====================
    def _exposure_ctrl_cb(self, cam, msg):
        """接收曝光控制命令 (每个相机一个控制话题)。
        JSON 格式: {"auto": true} 或 {"auto": false, "value": 5000}
        """
        try:
            cmd = json.loads(msg.data)
            auto = bool(cmd.get("auto", True))
            value = cmd.get("value", None)
            self.set_exposure(cam, auto, value if value is None else int(value))
        except Exception as e:
            self.get_logger().error(
                f"[{cam.name}] 解析曝光控制命令失败: {e} (raw: {msg.data[:100]})")

    def set_exposure(self, cam, auto, value=None):
        """调节指定相机的曝光, 后端按相机类型分派。

        - realsense_param (d455): ros2 param set 改 realsense-ros 动态参数,
          等效于 pyrealsense2 color_sensor.set_option(rs.option.enable_auto_exposure, ...)
        - orbbec_service (dabai): 调 Orbbec 驱动服务 (ros2 param set 是 RealSense 专有,
          对 Orbbec 无效)

        全部用 subprocess 异步调用, 不阻塞主检测循环; 相机未上线时命令会失败,
        但输出被丢弃, 不影响节点启动与另一路运行。
        """
        import subprocess
        cam.auto_exposure = auto
        ros_setup = "source /opt/ros/humble/setup.bash"
        bash = ['bash', '-c']

        if cam.exposure_backend == "orbbec_service":
            srv_prefix = cam.exposure_service_prefix or f"/{cam.name}"
            auto_srv = f"{srv_prefix}/set_color_auto_exposure"
            exp_srv = f"{srv_prefix}/set_color_exposure"
            auto_str = "true" if auto else "false"
            cmd = (f'{ros_setup} && ros2 service call {auto_srv} '
                   f'std_srvs/srv/SetBool "{{data: {auto_str}}}"')
            subprocess.Popen(bash + [cmd], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            if auto:
                self.get_logger().info(f"[{cam.name}] 🔄 已切换为自动曝光")
                return
            if value is not None:
                value = max(cam.exposure_min, min(cam.exposure_max, int(value)))
                cmd = (f'{ros_setup} && ros2 service call {exp_srv} '
                       f'{cam.set_exposure_service_type} "{{data: {value}}}"')
                subprocess.Popen(bash + [cmd], stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                cam.current_exposure = value
                self.get_logger().info(
                    f"[{cam.name}] � 曝光度: {value} "
                    f"(范围: {int(cam.exposure_min)} - {int(cam.exposure_max)})")
            return

        # realsense_param 后端 (d455, 行为与改造前一致)
        auto_str = "true" if auto else "false"
        cmd = (f'{ros_setup} && ros2 param set {cam.driver_node} '
               f'{cam.auto_exposure_param_name} {auto_str}')
        subprocess.Popen(bash + [cmd], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
        if auto:
            self.get_logger().info(f"[{cam.name}] 🔄 已切换为自动曝光")
            return
        if value is not None:
            value = max(cam.exposure_min, min(cam.exposure_max, int(value)))
            cmd = (f'{ros_setup} && ros2 param set {cam.driver_node} '
                   f'{cam.exposure_param_name} {value}')
            subprocess.Popen(bash + [cmd], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            cam.current_exposure = value
            self.get_logger().info(
                f"[{cam.name}] 📷 曝光度: {value} "
                f"(范围: {int(cam.exposure_min)} - {int(cam.exposure_max)})")

    def quaternion_to_rotation_matrix(self, quat):
        """
        将四元数转换为旋转矩阵
        quat: [qx, qy, qz, qw]
        """
        quat = np.asarray(quat, dtype=np.float64)
        norm = np.linalg.norm(quat)
        if quat.shape != (4,) or not np.isfinite(quat).all() or norm < 1e-9:
            raise ValueError("相机外参四元数必须为有限非零xyzw")
        qx, qy, qz, qw = quat / norm
        
        rotation_matrix = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        
        return rotation_matrix
    
    def rpy_to_rotation_matrix(self, rpy):
        """
        将 RPY 角度（弧度）转换为旋转矩阵
        rpy: [roll, pitch, yaw]
        """
        roll, pitch, yaw = rpy
        
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        
        R_y = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        
        R_z = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        rotation_matrix = R_z @ R_y @ R_x
        return rotation_matrix
    
    def pose_stamped_to_dict(self, pose_msg):
        """将 PoseStamped 转为可发布的字典。"""
        if pose_msg is None:
            return None
        return {
            "header_frame_id": pose_msg.header.frame_id,
            "position": {
                "x": round(pose_msg.pose.position.x, 4),
                "y": round(pose_msg.pose.position.y, 4),
                "z": round(pose_msg.pose.position.z, 4)
            },
            "orientation": {
                "x": round(pose_msg.pose.orientation.x, 6),
                "y": round(pose_msg.pose.orientation.y, 6),
                "z": round(pose_msg.pose.orientation.z, 6),
                "w": round(pose_msg.pose.orientation.w, 6)
            }
        }
    def _stamp_to_sec(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def print_status(self):
        """打印状态信息 (逐相机, 首次打印后取消定时器)"""
        if self.status_printed:
            return
        self.status_printed = True
        self.get_logger().info("=" * 60)
        self.get_logger().info("📊 状态统计:")
        for cam in self._enabled_cameras():
            if cam.rgb_ready and cam.latest_rgb is not None:
                h, w = cam.latest_rgb.shape[:2]
                rgb_str = f"就绪 {w}x{h}"
            else:
                rgb_str = "等待中"
            self.get_logger().info(
                f"  - [{cam.name}] RGB 数据: {rgb_str}; "
                f"已处理帧数: {cam.frame_count}; "
                f"检测到物体: {cam.detection_count} 次")
        self.get_logger().info("  - 眼在手外: 相机变换为常量 (无需机械臂位姿)")
        self.get_logger().info("=" * 60)
        self.status_timer.cancel()

    def _depth_status_log(self):
        """周期诊断 (每5s, 逐相机): 深度图接收状态 + 单目回退比率.
        结论性区分两种故障:
          (a) latest_depth=None → 深度话题未收到 → 全程单目 (直径≈预设/距离抖动/质心z=None→撞台面)
          (b) 话题活着但逐物体回退 → bbox中心5x5无效像素<3 或 深度超出0.2~3.0m
        """
        for cam in self._enabled_cameras():
            if not cam.use_depth_camera:
                continue
            if cam.latest_depth is None:
                self.get_logger().error(
                    f"❌ [{cam.name}] 深度图未收到 (latest_depth=None)! "
                    f"话题 {cam.depth_topic} 无数据 → 该路全程单目 "
                    f"(用预设尺寸: 直径≈预设, 距离抖动, 质心z=None→夹爪撞台面). "
                    f"修复: 确认该话题有数据且驱动已开启深度对齐")
                continue
            total = cam.depth_total_count
            fb = cam.depth_fallback_count
            if total > 0:
                ok = total - fb
                ratio = fb / total * 100.0
                if fb > 0:
                    self.get_logger().warn(
                        f"⚠️ [{cam.name}] 深度: {ok}/{total} 成功, {fb} 回退单目 "
                        f"({ratio:.0f}%). 回退时用预设尺寸 (直径≈预设, 距离抖动). "
                        f"可能原因: bbox中心5x5无效像素<3 或 深度超出0.2~3.0m")
                else:
                    self.get_logger().info(
                        f"✅ [{cam.name}] 深度: {ok}/{total} 全部成功 (无单目回退)")
            # 重置周期计数
            cam.depth_total_count = 0
            cam.depth_fallback_count = 0

    def rgb_cb(self, cam, msg):
        """接收指定相机的 RGB 图像"""
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            cam.latest_rgb = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cam.latest_rgb_stamp = self._stamp_to_sec(msg.header.stamp)

            if not cam.rgb_ready:
                cam.rgb_ready = True
                self.get_logger().info(
                    f"✅ [{cam.name}] RGB 就绪: {msg.width}x{msg.height}")
        except Exception as e:
            self.get_logger().error(f"[{cam.name}] RGB 转换错误: {e}")

    def depth_cb(self, cam, msg):
        """接收指定相机的对齐深度图 (uint16, mm). 像素坐标与 RGB 严格对齐.

        同时把 (时间戳, 图) 压入环形缓存, 供 _pick_depth 按 RGB 时间戳就近配对。
        """
        try:
            depth_img = np.frombuffer(
                msg.data, dtype=np.uint16
            ).reshape(msg.height, msg.width).copy()
            stamp = self._stamp_to_sec(msg.header.stamp)
            cam.latest_depth = depth_img
            cam.latest_depth_stamp = stamp
            cam.depth_buffer.append((stamp, depth_img))
        except Exception as e:
            self.get_logger().error(f"[{cam.name}] 深度图解析失败: {e}")
            cam.latest_depth = None

    def _pick_depth(self, cam, ref_stamp):
        """按 RGB 时间戳从深度缓存里挑时间上最接近的一帧, 并记录时间差。

        为什么需要: 该相机的彩色帧要走 MJPG 解码线程发布, 比深度晚几十毫秒,
        若直接用 cam.latest_depth, RGB 与深度的实际时间差中位约 52ms (最大 68ms);
        就近配对后可压到 ~8ms (最大 ~17ms)。

        ref_stamp缺失、缓存为空或时差超过50ms时拒绝本帧深度。
        返回 uint16 ndarray (mm) 或 None。
        """
        if ref_stamp is None or not cam.depth_buffer:
            cam.last_depth_dt_ms = None
            return None
        best_img, best_dt = None, None
        for stamp, img in cam.depth_buffer:
            dt = abs(stamp - ref_stamp)
            if best_dt is None or dt < best_dt:
                best_dt, best_img = dt, img
        cam.last_depth_dt_ms = best_dt * 1000.0
        return best_img if best_dt <= .05 else None
            
    def transform_camera_to_base(self, cam, camera_coords):
        """使用本帧冻结的相机→基座变换；Dabai已包含图像时刻腕部姿态。
        """
        if camera_coords is None:
            return None, None
        point_homogeneous = np.append(camera_coords, 1)
        base_coords = cam.transform_cam_to_base @ point_homogeneous
        # X/Y 标定偏差补偿 (仅修正 main detection 位置, 不影响 PCA/质心 z 计算)
        return base_coords[:3], None  # ee_coords 不再适用, 返回 None

    def get_robot_current_position(self):
        """eye-to-hand: 不再跟踪机械臂位姿, 返回 None"""
        return None
    
    def calculate_distance(self, point1, point2):
        """计算两点之间的欧氏距离"""
        if point1 is None or point2 is None:
            return None
        return np.linalg.norm(point1 - point2)

    def estimate_object_volume(self, width, height, shape="sphere"):
        """估计物体体积"""
        if shape == "sphere":
            diameter = (width + height) / 2
            radius = diameter / 2
            volume = (4/3) * math.pi * (radius ** 3)
        elif shape == "cube":
            volume = width * height * width
        else:
            volume = width * height * width
        return volume

    # ================== 单目RGB估计方法 ==================

    def monocular_depth_from_bbox(self, cam, box, class_name=None):
        """
        利用已知物体真实宽度，通过针孔模型反推深度。
        原理: depth = fx * real_width / pixel_width

        需要该相机内参已知 (camera_info 已到达, 或有硬编码 fallback), 否则返回 None。
        返回: depth_m (float) 或 None
        """
        if cam.camera_matrix is None or cam.fx <= 0.0 or cam.fy <= 0.0:
            self.get_logger().warning(
                f"[{cam.name}] 内参未知 (未收到 {cam.camera_info_topic}), 无法单目反推深度",
                throttle_duration_sec=10.0)
            return None
        x1, y1, x2, y2 = [int(v) for v in box]
        pixel_width = float(x2 - x1)
        pixel_height = float(y2 - y1)

        if pixel_width <= 0 or pixel_height <= 0:
            return None

        # 按检测到的类别查询真实尺寸 (未注册类别回落到默认 known_object_*_m)
        real_width, real_height = self._get_class_dimensions(class_name)
        # 用fx和已知宽度反推；同时用fy+已知高度做交叉验证
        depth_w = cam.fx * real_width / pixel_width
        depth_h = cam.fy * real_height / pixel_height
        depth = (depth_w + depth_h) / 2.0

        if depth < 0.05 or depth > 10.0:
            return None

        return depth

    def depth_from_camera(self, cam, box, depth_img):
        """从该相机的对齐深度图提取物体深度 (米).

        策略: 取 bbox 中心 5x5 像素区域的中位数.
        - 用中位数而非均值: 抗噪、抗无效像素
        - 用中心小区域而非整个 bbox: 避免 bbox 松散时混入背景深度
        - 水果类物体近似球/椭球, bbox 中心通常落在物体表面

        Args:
            cam: CameraState (用 cam.depth_offset_m)
            box: [x1, y1, x2, y2] 像素坐标
            depth_img: 该帧选定的对齐深度图 (uint16, mm), 由 _pick_depth 按 RGB 时间戳挑出

        Returns:
            depth_m (float) 或 None (无有效深度)
        """
        if depth_img is None:
            return None

        x1, y1, x2, y2 = [int(v) for v in box]
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        h, w = depth_img.shape
        x_start = max(0, cx - 2)
        x_end = min(w, cx + 3)
        y_start = max(0, cy - 2)
        y_end = min(h, cy + 3)

        region = depth_img[y_start:y_end, x_start:x_end]
        valid = region[region > 0]

        if len(valid) < 3:
            return None

        depth_mm = float(np.median(valid))
        depth_m = depth_mm / 1000.0

        if depth_m < 0.2 or depth_m > 3.0:
            return None

        if not cam.depth_diag_logged:
            cam.depth_diag_logged = True
            self.get_logger().info(f'[{cam.name}] 原始深度 {depth_m:.3f}m，人工深度/XYZ偏置已禁用')

        return depth_m

    def monocular_pixel_to_camera_coords(self, cam, u, v, depth):
        """
        单目模式: 用估算深度将像素坐标转为相机3D坐标
        cam: CameraState (用其内参)
        u, v: 像素坐标
        depth: 估算深度 (米)
        """
        x = (u - cam.cx) * depth / cam.fx
        y = (v - cam.cy) * depth / cam.fy
        z = depth
        return np.array([x, y, z])

    def publish_object_visualization(self, cam, camera_coords, base_coords):
        """将检测结果以 TF + Marker + PoseStamped 发布到 RViz

        两路相机各自使用自己的 TF child_frame_id 与 Marker 命名空间,
        避免 d455 / dabai 互相覆盖。
        """
        now = self.get_clock().now().to_msg()

        # 选择发布位置：优先基座坐标，否则用相机坐标
        if base_coords is not None:
            pos_x, pos_y, pos_z = base_coords
            parent_frame = self.base_frame_id  # base_coords 已在基座坐标系下
        else:
            pos_x, pos_y, pos_z = camera_coords
            parent_frame = "camera_frame"      # camera_coords 在相机坐标系下

        # --- TF ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = parent_frame
        t.child_frame_id = cam.frame_child_id
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = pos_z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # --- Marker (绿色半透明球体) ---
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = parent_frame
        marker.ns = f"detected_objects_{cam.name}"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pos_x
        marker.pose.position.y = pos_y
        marker.pose.position.z = pos_z
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.known_object_width_m
        marker.scale.y = self.known_object_height_m
        marker.scale.z = self.known_object_height_m
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime.sec = 1
        cam.pub_object_marker.publish(marker)

        # --- PoseStamped ---
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = parent_frame
        ps.pose.position.x = pos_x
        ps.pose.position.y = pos_y
        ps.pose.position.z = pos_z
        ps.pose.orientation.w = 1.0
        cam.pub_object_pose_stamped.publish(ps)

    def publish_object_markers(self, cam, objects):
        """Publish every tracked object so the workbench 3D view can show it."""
        array = MarkerArray()
        current = set()
        for obj in objects:
            base = obj.get("base_position_m")
            if not base:
                continue
            marker_id = int(obj.get("track_id", 0)) + 1
            current.add(marker_id)
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = self.base_frame_id
            marker.ns = f"{cam.name}_detected_objects"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(base["x"])
            marker.pose.position.y = float(base["y"])
            marker.pose.position.z = float(base["z"]) + .02
            marker.pose.orientation.w = 1.
            diameter = max(.02, min(float(obj.get("estimated_diameter_m", .05)), .15))
            marker.scale.x = marker.scale.y = marker.scale.z = diameter * .65
            marker.color.r, marker.color.g, marker.color.b = (.08, .48, 1.) if cam.name == "d455" else (.0, .75, .62)
            marker.color.a = .72
            marker.lifetime.sec = 1
            array.markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = f"{cam.name}_labels"
            label.id = marker.id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = marker.pose
            label.pose.position.z += diameter * .55
            label.pose.orientation.w = 1.
            label.scale.z = .045
            label.color.r = label.color.g = label.color.b = 1.
            label.color.a = .95
            label.text = f"T{obj.get('track_id')} {obj.get('object_name', 'object')}"
            label.lifetime.sec = 1
            array.markers.append(label)

        for old_id in cam.marker_ids - current:
            for name, marker_type in ((f"{cam.name}_detected_objects", Marker.SPHERE),
                                      (f"{cam.name}_labels", Marker.TEXT_VIEW_FACING)):
                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = self.base_frame_id
                marker.ns = name; marker.id = old_id
                marker.type = marker_type; marker.action = Marker.DELETE
                array.markers.append(marker)
        cam.marker_ids = current
        cam.pub_object_markers.publish(array)

    def camera_info_cb(self, cam, msg):
        """D455优先使用与手眼标定相同的批准内参；无文件时读取驱动。"""
        if cam.cam_info_received:
            return  # 仅首帧更新, 避免每帧重复解析
        try:
            k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            distortion = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
            source = "camera_info"
            intrinsics_file = _rc.workspace_root() / f"config/calibration/{cam.name}/intrinsics.yaml"
            if intrinsics_file.is_file():
                # 复用手眼标定的文件格式和已有校验，不把候选当活动参数。
                calibration_module = str(_WS_ROOT / "src/handeye_calibration_ros")
                if calibration_module not in sys.path:
                    sys.path.insert(0, calibration_module)
                from handeye_calibration_ros.charuco_detect import load_intrinsics
                from handeye_calibration_ros.validation import validate_intrinsics
                loaded = load_intrinsics(str(intrinsics_file))
                if loaded is None:
                    raise ValueError(f"无法读取批准内参: {intrinsics_file}")
                from workbench.calibration import selected_file
                if not selected_file(_WS_ROOT, "intrinsics", cam.name, intrinsics_file):
                    validate_intrinsics(loaded["raw"], cam.name, approved=True)
                if (loaded["image_width"], loaded["image_height"]) != (msg.width, msg.height):
                    raise ValueError("D455批准内参与当前图像分辨率不一致")
                k = loaded["camera_matrix"]
                distortion = loaded["dist_coeffs"].ravel()
                source = str(intrinsics_file)
            new_fx = float(k[0, 0])
            new_fy = float(k[1, 1])
            new_cx = float(k[0, 2])
            new_cy = float(k[1, 2])
            if not np.isfinite(k).all() or new_fx <= 0 or new_fy <= 0:
                return
            cam.fx, cam.fy, cam.cx, cam.cy = new_fx, new_fy, new_cx, new_cy
            cam.skew = float(k[0, 1])
            cam.camera_matrix = k
            cam.info_size = (msg.height,msg.width)
            cam.dist_coeffs = distortion
            cam.cam_info_received = True
            self.get_logger().info("=" * 60)
            self.get_logger().info(
                f"📥 [{cam.name}] 内参来源: {source}")
            self.get_logger().info(f"   fx={cam.fx:.4f} fy={cam.fy:.4f} "
                                   f"cx={cam.cx:.4f} cy={cam.cy:.4f}")
            self.get_logger().info("=" * 60)
        except Exception as e:
            self.get_logger().warning(f"[{cam.name}] 解析相机内参失败: {e}", throttle_duration_sec=3.)

    def _publish_invalid(self,cam,reason):
        msg=String()
        msg.data=json.dumps(dict(camera=cam.name,header_stamp=self.get_clock().now().nanoseconds*1e-9,
            detected=False,valid=False,invalid_reason=reason,objects=[],objects_count=0))
        cam.pub_detection_info.publish(msg)

    def detect_and_publish(self, cam, rgb=None, rgb_stamp=None):
        """主检测函数 (针对指定相机, 两路各自调用)。

        rgb / rgb_stamp: 指定要处理的彩色帧。默认用 cam.latest_rgb (最新帧)。
        depth_pair_delay 模式下, _detect_all 会传入"上一轮暂存的帧"。
        """
        if rgb is None:
            rgb = cam.latest_rgb
        if rgb_stamp is None:
            rgb_stamp = cam.latest_rgb_stamp
        if not cam.rgb_ready or not cam.cam_info_received or rgb is None or not fresh_image(self,rgb_stamp):
            self._publish_invalid(cam,"stale_rgb_or_missing_intrinsics")
            return

        try:
            cam.transform_cam_to_base = self.camera_geometry.camera(cam.name, rgb_stamp)
            cam.transform_base_to_cam = np.linalg.inv(cam.transform_cam_to_base)
        except Exception as exc:
            self.get_logger().warning(f"[{cam.name}] 图像时刻变换不可用: {exc}", throttle_duration_sec=3.)
            self._publish_invalid(cam,"image_time_transform_unavailable")
            return
        cam.frame_count += 1
        display_img = rgb.copy()
        # 按该 RGB 帧的时间戳就近配对深度帧 (见 _pick_depth), 本帧内所有深度读取共用这一帧。
        # DaBai DC1 硬件不支持传感器同步, 只能用这种软件配对。
        depth_img = self._pick_depth(cam, rgb_stamp)
        if depth_img is None or depth_img.shape[:2] != rgb.shape[:2] or rgb.shape[:2]!=cam.info_size:
            self._publish_invalid(cam,"rgb_depth_intrinsics_pair_unavailable")
            return
        try:
            # YOLO 检测
            inference_rgb = rgb
            if cam.preprocess_clahe:
                lab = cv2.cvtColor(inference_rgb, cv2.COLOR_BGR2LAB)
                light, a_channel, b_channel = cv2.split(lab)
                light = cv2.createCLAHE(clipLimit=cam.clahe_clip_limit,
                                        tileGridSize=(8, 8)).apply(light)
                inference_rgb = cv2.cvtColor(
                    cv2.merge((light, a_channel, b_channel)), cv2.COLOR_LAB2BGR)
            results = self.model(inference_rgb, conf=cam.yolo_conf, iou=cam.yolo_iou,
                                 device=self.yolo_device,
                                 agnostic_nms=cam.yolo_agnostic_nms,
                                 imgsz=cam.inference_size, augment=cam.tta,
                                 verbose=False)
            
            # ==================== 多目标检测 ====================
            # 遍历 YOLO 输出的所有检测框, 每个都: 绘制标注 + 坐标变换 + 收集到 objects 列表。
            # 主物体 (= 最高置信度的有效物体) 信息填到顶层字段, 保持与 auto_sorting_action.py
            # 的向后兼容 (_two_stage_refine 仍能读 detected / base_position_m)。
            raw_boxes = results[0].boxes
            detections = []
            for box in raw_boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                detections.append(Detection(
                    float(x1), float(y1), float(x2), float(y2),
                    float(box.conf[0].cpu().numpy()),
                    int(box.cls[0].cpu().numpy())))
            boxes_sorted = deduplicate_detections(detections)
            if cam.merge_fragments:
                boxes_sorted = merge_fragment_detections(boxes_sorted)
            if cam.bbox_expand_ratio > 0:
                boxes_sorted = [expand_detection(d, cam.bbox_expand_ratio, rgb.shape[1], rgb.shape[0])
                                for d in boxes_sorted]
            if cam.refine_bbox:
                boxes_sorted = [
                    Detection(*refine_bbox_grabcut(rgb, d.bbox), d.confidence, d.class_id)
                    for d in boxes_sorted]
            if boxes_sorted:
                cam.detection_count += 1

            # eye-to-hand: 无需选择机械臂位姿, 相机变换是常量
            selected_robot_pose = None
            pose_time_diff = None
            robot_position = None  # eye-to-hand: 不跟踪机械臂位置

            # 不同物体用不同颜色框 (BGR), 循环使用
            box_colors = [
                (0, 255, 0),    # 绿
                (255, 0, 0),    # 蓝
                (0, 165, 255),  # 橙
                (255, 0, 255),  # 品红
                (0, 255, 255),  # 黄
                (255, 255, 0),  # 青
                (128, 0, 128),  # 紫
            ]

            objects_list = []
            primary_info = None  # 主物体完整信息 (最高置信度的有效物体)
            matched_track_ids = set()  # 本帧已匹配的 track_id, 避免重复匹配

            for idx, box in enumerate(boxes_sorted):
                x1, y1, x2, y2 = box.bbox
                conf = box.confidence
                cls_id = box.class_id
                class_name = self.model.names[cls_id]

                # 中心点
                center_u = int((x1 + x2) / 2)
                center_v = int((y1 + y2) / 2)

                # 真实尺寸 (按类别查询, 用于深度估计和体积)
                real_width, real_height = self._get_class_dimensions(class_name)
                volume = self.estimate_object_volume(real_width, real_height, shape="sphere")

                # ========== 深度估计: 优先深度相机, 失败时回退到单目 ==========
                # depth_source 取值保持不变 ("d455_stereo" 等), 下游消费方按原字符串判定
                depth_source = "monocular"  # 默认单目 (深度失败或未启用时)
                if cam.use_depth_camera:
                    cam.depth_total_count += 1
                    depth_m = self.depth_from_camera(cam, [x1, y1, x2, y2], depth_img)
                    if depth_m is None:
                        # 深度无效 (反光/透明/超出范围), 自动回退到单目
                        depth_m = None  # 无真实深度不生成可执行抓取坐标
                        cam.depth_fallback_count += 1
                        depth_source = "monocular_fallback"  # 深度失败→单目(用预设尺寸)
                    else:
                        depth_source = "d455_stereo"  # 深度相机成功
                else:
                    depth_m = None  # calibration-only grasping requires measured depth
                    depth_source = "monocular_only"  # 未启用深度相机

                # 抓取目标是物体内部几何中心，表面点仅另行记录。
                camera_coords = None
                ee_coords = None
                base_coords = None
                centroid_data = None
                if depth_img is not None and cam.use_depth_camera:
                    try:
                        centroid_data = object_centroid_from_rgbd(depth_img,[x1,y1,x2,y2],
                            [cam.fx,cam.fy,cam.cx,cam.cy],cam.transform_cam_to_base,
                            distortion=cam.dist_coeffs if cam.name == "d455" else None)
                        base_coords = centroid_data['centroid']
                        surface = centroid_data['surface_position']
                        camera_coords = (cam.transform_base_to_cam @ np.r_[surface,1])[:3]
                        depth_m = float(camera_coords[2])
                        depth_source = cam.name + '_rgbd_centroid'
                    except ValueError:
                        depth_m = None  # 无中心估计时，不拿表面点/类别尺寸兜底。
                else:
                    depth_m = None

                # ========== 滤波: 匹配 track + EMA 更新 ==========
                # 用原始观测更新 track, 再用滤波值替换发布值 (画框/坐标都用滤波后)
                raw_obs = {
                    'x1': float(x1), 'y1': float(y1), 'x2': float(x2), 'y2': float(y2),
                    'center_u': center_u, 'center_v': center_v,
                    'depth': depth_m,
                    'camera_coords': camera_coords,
                    'ee_coords': ee_coords,
                    'base_coords': base_coords,
                    'conf': conf,
                }
                # 当前帧质心供动态抓取使用；静止抓取仍发布原来的 EMA 坐标。
                motion_observation = None
                if centroid_data is not None and rgb_stamp is not None:
                    motion_observation = {
                        'stamp_s': float(rgb_stamp),
                        'position_m': dict(zip(('x', 'y', 'z'), map(float, centroid_data['centroid']))),
                    }
                track = self._find_matching_track(
                    cam, class_name, center_u, center_v, (x1, y1, x2, y2), matched_track_ids)
                if track is None:
                    # 新物体, 创建 track
                    track_id = cam.next_track_id
                    cam.next_track_id += 1
                    track = ObjectTrack(track_id, class_name, raw_obs, cam.frame_count, self.filter_alpha)
                    cam.tracks[track_id] = track
                else:
                    matched_track_ids.add(track.track_id)
                    track.update(raw_obs, cam.frame_count, class_name,
                                 1.0 if cam.name=="dabai" else self.filter_alpha,
                                 cam.detect_period_s)
                if track.hits < cam.min_track_hits:
                    continue

                published_class = track.stable_class
                published_cls_id = next(
                    (cid for cid, name in self.model.names.items() if name == published_class),
                    cls_id)
                real_width, real_height = self._get_class_dimensions(published_class)
                volume = self.estimate_object_volume(real_width, real_height, shape="sphere")

                # 用滤波值替换 (画框/标注/发布都用平滑后的值)
                x1, y1, x2, y2 = track.s_x1, track.s_y1, track.s_x2, track.s_y2
                center_u = int((track.s_x1 + track.s_x2) / 2)
                center_v = int((track.s_y1 + track.s_y2) / 2)
                depth_m = track.s_depth if centroid_data is not None else None
                camera_coords = track.s_cam
                ee_coords = track.s_ee
                base_coords = track.s_base

                cross_source = None
                if cam.name == "dabai" and base_coords is not None and time.monotonic() - self._d455_fusion_stamp < .5:
                    hint = nearest_cross_camera_match(
                        base_coords, self._d455_fusion_objects, max_distance=.07)
                    if hint:
                        track.cross_class = hint.get("object_name")
                        track.cross_class_until = time.monotonic() + 1.2
                        track.stable_class = track.cross_class
                        published_class = track.stable_class
                        published_cls_id = next(
                            (cid for cid, name in self.model.names.items() if name == published_class),
                            cls_id)
                        real_width, real_height = self._get_class_dimensions(published_class)
                        volume = self.estimate_object_volume(real_width, real_height, shape="sphere")
                        cross_source = "d455_fusion"

                # 距离 (用滤波后的基座坐标重算)
                distance_to_robot = None
                if base_coords is not None and robot_position is not None:
                    distance_to_robot = self.calculate_distance(base_coords, robot_position)

                # ── 绘制检测框 + 类别标签 (用滤波后的 bbox) ──
                box_color = box_colors[idx % len(box_colors)]
                cv2.rectangle(display_img, (int(x1), int(y1)), (int(x2), int(y2)), box_color, 2)
                cv2.circle(display_img, (center_u, center_v), 4, (0, 0, 255), -1)
                label = f"#{idx+1} {published_class}: {conf:.2f} ({track.class_votes.stability:.0%})"
                cv2.putText(display_img, label, (int(x1), max(int(y1)-8, 12)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2)

                # 框旁标注位置 (优先放框右下外侧, 出界则放框内左上)
                coord_label_x = int(x2) + 4
                coord_label_y = int(y2)
                if coord_label_x > display_img.shape[1] - 200:
                    coord_label_x = int(x1) + 4
                    coord_label_y = int(y1) + 18

                if depth_m is None:
                    # 深度估计失败 (像素尺寸异常或超出合理范围, 或 track 从未成功估计深度)
                    cv2.putText(display_img, "depth=N/A",
                               (coord_label_x, coord_label_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
                    obj_info = {
                        "index": idx + 1,
                        "object_name": published_class,
                        "raw_object_name": class_name,
                        "class_stability": round(track.class_votes.stability, 3),
                        "class_source": cross_source or "camera",
                        "confidence": round(track.confidence, 4),
                        "raw_confidence": round(conf, 4),
                        "class_id": published_cls_id,
                        "bbox_pixel": {
                            "x1": int(x1), "y1": int(y1),
                            "x2": int(x2), "y2": int(y2)
                        },
                        "size_m": {
                            "width": round(real_width, 4),
                            "height": round(real_height, 4),
                            "diameter": round((real_width + real_height) / 2, 4),
                        },
                        "volume_m3": round(volume, 6),
                        "depth_available": False,
                        "depth_source": "none",  # D455+单目均失败
                        "filtered": True,
                        "track_id": track.track_id,
                    }
                    if rgb_stamp is not None:
                        obj_info["rgb_stamp_s"] = round(rgb_stamp, 6)
                    objects_list.append(obj_info)
                    continue

                # 深度成功, obj_x/obj_y/obj_z 从滤波后的 camera_coords 取
                obj_x, obj_y, obj_z = camera_coords

                # 框旁标注: 深度 + 基座坐标 (用滤波值)
                cv2.putText(display_img, f"d={depth_m:.2f}m",
                           (coord_label_x, coord_label_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                if base_coords is not None:
                    cv2.putText(display_img,
                               f"B({base_coords[0]:.2f},{base_coords[1]:.2f},{base_coords[2]:.2f})",
                               (coord_label_x, coord_label_y + 16),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)

                # ── 估计直径 (bbox像素 + D455深度, 不依赖预设尺寸) ──
                # diam = bbox平均像素尺寸 × 深度 / fx (针孔模型反投影)
                _est_diam = ((float(x2 - x1) + float(y2 - y1)) / 2.0) * depth_m / cam.fx
                # ── 收集物体信息到 objects 列表 (用滤波值) ──
                obj_info = {
                    "index": idx + 1,
                    "object_name": published_class,
                    "raw_object_name": class_name,
                    "class_stability": round(track.class_votes.stability, 3),
                    "class_source": cross_source or "camera",
                    "confidence": round(track.confidence, 4),
                    "raw_confidence": round(conf, 4),
                    "class_id": published_cls_id,
                    "bbox_pixel": {
                        "x1": int(x1), "y1": int(y1),
                        "x2": int(x2), "y2": int(y2)
                    },
                    "camera_position_m": {
                        "pixel_u": center_u,
                        "pixel_v": center_v,
                        "depth_m": round(obj_z, 4)
                    },
                    "end_effector_position_m": None,  # eye-to-hand: 不再计算 ee_coords
                    "monocular_depth_m": round(depth_m, 4),
                    "depth_source": depth_source,  # d455_stereo | monocular_fallback | monocular_only
                    "estimated_diameter_m": round(_est_diam, 4),  # D455深度估计, 不依赖预设
                    "size_m": {
                        "width": round(real_width, 4),
                        "height": round(real_height, 4),
                        "diameter": round((real_width + real_height) / 2, 4),
                        "note": "known_size, not estimated from depth"
                    },
                    "volume_m3": round(volume, 6),
                    "depth_available": True,
                    "filtered": True,  # 标记本帧坐标已经过 EMA 滤波
                    "track_id": track.track_id,
                    "motion_observation": motion_observation,
                }
                if centroid_data is not None:
                    obj_info['grasp_point_type'] = 'estimated_centroid'
                    obj_info['centroid_method'] = centroid_data['method']
                    obj_info['centroid_base_z'] = float(base_coords[2])
                    obj_info['surface_position_m'] = dict(zip(('x','y','z'),map(float,centroid_data['surface_position'])))
                    obj_info['support_z_m'] = centroid_data['support_z']
                    geometry = centroid_data['grasp_geometry']
                    obj_info['grasp_geometry'] = geometry
                    obj_info['elongation_ratio'] = geometry['elongation_ratio']
                    obj_info['grasp_orientation'] = None
                    if geometry['elongation_ratio'] >= 1.5:
                        R4 = np.eye(4)
                        R4[:3, :3] = short_axis_tool_rotation(geometry['short_axis_base'], [0., 0., -1.])
                        q = tf_transformations.quaternion_from_matrix(R4)
                        obj_info['grasp_orientation'] = dict(zip(('qx','qy','qz','qw'), map(float, q)))

                if base_coords is not None:
                    obj_info["base_position_m"] = {
                        "x": round(base_coords[0], 4),
                        "y": round(base_coords[1], 4),
                        "z": round(base_coords[2], 4)
                    }
                if distance_to_robot is not None:
                    obj_info["distance_to_robot_m"] = round(distance_to_robot, 4)
                if rgb_stamp is not None:
                    obj_info["rgb_stamp_s"] = round(rgb_stamp, 6)

                objects_list.append(obj_info)

                # ── 主物体 (第一个有效物体 = 最高置信度):
                #    发布 RViz 可视化 + /object_3d_position (Float64MultiArray, 向后兼容) ──
                if primary_info is None:
                    primary_info = obj_info
                    self.publish_object_visualization(cam, camera_coords, base_coords)
                    coord_msg = Float64MultiArray()
                    # eye-to-hand: ee_coords 不再计算, 用 0.0 占位 (向后兼容 Float64MultiArray 格式)
                    coord_data = [obj_x, obj_y, obj_z,
                                 0.0, 0.0, 0.0,  # ee_coords 占位 (eye-to-hand 不适用)
                                 real_width, real_height, volume,
                                 conf, float(published_cls_id), depth_m]
                    if distance_to_robot is not None:
                        coord_data.append(distance_to_robot)
                    coord_msg.data = coord_data
                    cam.pub_object_pose.publish(coord_msg)

            if cam.name == "d455":
                self._d455_fusion_objects = objects_list
                self._d455_fusion_stamp = time.monotonic()
            self.publish_object_markers(cam, objects_list)

            # 循环结束: 清理过期 track + 未匹配 track miss_count++
            self._cleanup_stale_tracks(cam, cam.frame_count)

            # eye-to-hand: 无需恢复 robot_current_pose (不再使用)

            # 画面左上角汇总: 检测到的物体数量 + 主物体坐标
            cv2.putText(display_img, f"Objects: {len(objects_list)}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if primary_info is not None:
                p = primary_info
                cv2.putText(display_img,
                           f"Primary: {p['object_name']} d={p['monocular_depth_m']:.2f}m",
                           (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 200), 1)
                bp = p.get("base_position_m")
                if bp:
                    cv2.putText(display_img,
                               f"Base: ({bp['x']:.3f},{bp['y']:.3f},{bp['z']:.3f})",
                               (10, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

            # ── 发布 /detection_info (顶层=主物体字段 + objects 列表, 向后兼容) ──
            info_dict = {
                "detected": primary_info is not None,
                "method": (primary_info.get("depth_source", "unknown") if primary_info is not None else "unknown"),
                "header_stamp": rgb_stamp,  # YOLO 检测时刻, 用于 grasp_pose_node 时间同步
                "camera": cam.name,                    # 新增: 标识本帧来自哪一路相机
                "motion_session": cam.motion_session,
                "objects": objects_list,
                "objects_count": len(objects_list),
            }
            if primary_info is not None:
                info_dict["object_name"] = primary_info["object_name"]
                info_dict["confidence"] = primary_info["confidence"]
                info_dict["bbox_pixel"] = primary_info["bbox_pixel"]
                info_dict["camera_position_m"] = primary_info["camera_position_m"]
                for key in ('grasp_point_type','centroid_method','surface_position_m','support_z_m','track_id'):
                    if key in primary_info:info_dict[key]=primary_info[key]
                info_dict["end_effector_position_m"] = primary_info["end_effector_position_m"]
                info_dict["monocular_depth_m"] = primary_info["monocular_depth_m"]
                info_dict["size_m"] = primary_info["size_m"]
                info_dict["volume_m3"] = primary_info["volume_m3"]
                if "base_position_m" in primary_info:
                    info_dict["base_position_m"] = primary_info["base_position_m"]
                if "distance_to_robot_m" in primary_info:
                    info_dict["distance_to_robot_m"] = primary_info["distance_to_robot_m"]
                info_dict["grasp_geometry"] = primary_info.get("grasp_geometry")
                # 两指实际闭合方向为工具Y轴。
                if "grasp_orientation" in primary_info:
                    info_dict["grasp_orientation"] = primary_info["grasp_orientation"]
                    info_dict["elongation_ratio"] = primary_info.get("elongation_ratio")
                # ── 质心 z (D455 深度点云估算, 不依赖预设尺寸, 供抓取高度补偿) ──
                if "centroid_base_z" in primary_info and primary_info["centroid_base_z"] is not None:
                    info_dict["centroid_base_z"] = primary_info["centroid_base_z"]
                # ── 估计直径 (bbox+D455深度, 不依赖预设尺寸, 供夹爪闭合目标) ──
                if "estimated_diameter_m" in primary_info:
                    info_dict["estimated_diameter_m"] = primary_info["estimated_diameter_m"]
                # ── 随动抓取接口占位 (中期实现: 光流/帧间位移估算物体速度) ──
                info_dict["velocity_mps"] = None

            # eye-to-hand: 不再发布 used_robot_pose / tcp_pose_m (相机变换为常量)
            if rgb_stamp is not None:
                info_dict["rgb_stamp_s"] = round(rgb_stamp, 6)

            info_msg = String()
            info_msg.data = json.dumps(info_dict, ensure_ascii=False)
            cam.pub_detection_info.publish(info_msg)

            # 终端日志 (多目标简要)
            if objects_list:
                self.get_logger().info("=" * 60)
                self.get_logger().info(f"🎯 [{cam.name}] 检测到 {len(objects_list)} 个物体:")
                for obj in objects_list:
                    bp = obj.get("base_position_m", {})
                    bp_str = f" 基座({bp['x']:.3f},{bp['y']:.3f},{bp['z']:.3f})" if bp else ""
                    self.get_logger().info(
                        f"  #{obj['index']} {obj['object_name']} conf={obj['confidence']:.3f} "
                        f"depth={obj['monocular_depth_m']:.3f}m{bp_str}"
                    )
                self.get_logger().info("=" * 60)
        except Exception as e:
            self.get_logger().error(f"[{cam.name}] 检测错误: {e}")
            cv2.putText(display_img, f"Error: {str(e)[:50]}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # ── 在画面上叠加相机名 + 曝光状态 (无论 standalone 还是 GUI 模式都显示) ──
        # 放在画面底部避免与检测信息 (y_offset 从 30 开始) 重叠
        exposure_text = (f"[{cam.name}] exposure: "
                         f"{'auto' if cam.auto_exposure else int(cam.current_exposure)}")
        cv2.putText(display_img, exposure_text, (10, display_img.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # ── 每帧都发布标注后的 JPEG，供 GUI 或外部订阅 ──
        try:
            ok, jpeg_buf = cv2.imencode('.jpg', display_img,
                                        [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                cimg = CompressedImage()
                cimg.header.stamp = self.get_clock().now().to_msg()
                cimg.format = "jpeg"
                cimg.data = jpeg_buf.tobytes()
                cam.pub_annotated.publish(cimg)
        except Exception:
            pass

        # ── standalone 模式: 显示该相机自己的窗口 ──
        # 键盘处理 (配合 waitKey) 统一放在 _handle_gui_keys, 每轮只调用一次,
        # 否则两个窗口各调一次 waitKey 会互相抢键。
        if self.show_gui_window and cam.show_gui_window:
            cv2.imshow(cam.window_name, display_img)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
