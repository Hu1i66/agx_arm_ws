#!/usr/bin/env python3
# -*-coding:utf8-*-
"""手眼标定主程序 (ChArUco 7x5 标定板)。

与旧版 (单 ArUco 标记 + aruco_ros 检测节点) 的区别:
  1. 不再依赖 aruco_ros, 直接订阅彩色图 + camera_info, 内联做 ChArUco 检测与 PnP 求解;
  2. 采集时逐帧质量门 (标记数 / 角点数 / 重投影 RMS / 方格像素边长 / 图像与位姿同步),
     不合格当场提示并重采, 不计数;
  3. 每个位姿落盘原始图像 + 逐帧 JSON, 支持事后离线复算;
  4. 求解改用 AX=XB 双算法互校 + 留出位姿验证; 方程残差与 cv2.calibrateHandEye
     仅作交叉参考, **不作为验收**。

输出语义 (与旧版保持一致, 不得更改):
  - eye_in_hand: 结果为 T_cam2gripper (相机相对 TCP 的位姿)
  - eye_to_hand: 结果为 T_base2cam (相机相对基座的位姿)

启动方式: 必须用 `ros2 run` (交互式 stdin 在 `ros2 launch` 下会因 termios 异常 crash)。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Image, CameraInfo
from rcl_interfaces.srv import GetParameters
from cv_bridge import CvBridge
import tf_transformations
from scipy.spatial.transform import Rotation
import cv2
import numpy as np
import json
import datetime
import sys
import termios
import tty
import time
import os
import threading

try:
    from .charuco_detect import (
        make_board, make_detector, detect_board, board_pose, load_intrinsics,
        DEFAULT_SQUARES_X, DEFAULT_SQUARES_Y, DEFAULT_SQUARE_LENGTH,
        DEFAULT_MARKER_LENGTH, DEFAULT_DICT, FULL_MARKERS, FULL_CORNERS)
    from .axxb import (solve_axxb, holdout_validate, format_criteria_report,
                       rt_to_T, rotation_angle_deg, CONSISTENCY_MAX_DEG,
                      MEDIAN_REL_ANGLE_MIN_DEG, MIN_USABLE_PAIRS, HOLDOUT_MAX_MM)
    from .validation import (validate_intrinsics, vector, camera_optical_frame,
                             CalibrationValidationError, validate_capture_frames, MIN_HANDEYE_SQUARE_PX)
except ImportError:
    from charuco_detect import (
        make_board, make_detector, detect_board, board_pose, load_intrinsics,
        DEFAULT_SQUARES_X, DEFAULT_SQUARES_Y, DEFAULT_SQUARE_LENGTH,
        DEFAULT_MARKER_LENGTH, DEFAULT_DICT, FULL_MARKERS, FULL_CORNERS)
    from axxb import (solve_axxb, holdout_validate, format_criteria_report,
                      rt_to_T, rotation_angle_deg, CONSISTENCY_MAX_DEG,
                      MEDIAN_REL_ANGLE_MIN_DEG, MIN_USABLE_PAIRS, HOLDOUT_MAX_MM)
    from validation import (validate_intrinsics, vector, camera_optical_frame,
                            CalibrationValidationError, validate_capture_frames, MIN_HANDEYE_SQUARE_PX)


def clear_input_buffer():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def create_dir(dir=""):
    if dir == "" or dir == "/":
        exit(-1)
    if not os.path.exists(dir) or not os.path.isdir(dir):
        try:
            if os.path.exists(dir):
                os.remove(dir)
            os.makedirs(dir)
        except IOError:
            print("cannot create directory: " + dir)
            exit(-2)


def create_pose_from_martix(translation_matrix, rotation_matrix) -> Pose:
    rotation_matrix_4x4 = rotation_matrix_4x4 = np.eye(4)
    rotation_matrix_4x4[:3, :3] = rotation_matrix
    quaternion = tf_transformations.quaternion_from_matrix(rotation_matrix_4x4)

    pose = Pose()
    pose.position.x = translation_matrix[0][0]
    pose.position.y = translation_matrix[1][0]
    pose.position.z = translation_matrix[2][0]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class PoseMartix():
    def __init__(self, pose_input: Pose, check_mode=False, mode='eye_in_hand'):
        position = pose_input.position
        orientation = pose_input.orientation
        self.position_list = [position.x, position.y, position.z]
        self.orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.rpy_list = [tf_transformations.euler_from_quaternion(self.orientation_list)]
        self.translation_matrix = np.array(self.position_list)
        self.rotation_matrix = Rotation.from_quat(self.orientation_list).as_matrix()
        if check_mode and mode == 'eye_to_hand':
            T = np.eye(4)
            T[:3, :3] = self.rotation_matrix
            T[:3, 3] = self.translation_matrix
            T_inv = np.linalg.inv(T)
            self.rotation_matrix = T_inv[:3, :3]
            self.translation_matrix = T_inv[:3, 3]


class HandEyeCalibrationNode(Node):
    def __init__(self):
        super().__init__("handeye_calibration")

        # ── 基本参数 ──
        self.declare_parameter('mode', 'eye_in_hand')
        self.declare_parameter('min_num', 15)
        self.declare_parameter('piper_topic', '/feedback/tcp_pose')
        self.declare_parameter('result_save_path', './result')
        # ── 相机 / 图像 ──
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('intrinsics_yaml', './result/camera_intrinsics.yaml')
        # ── 标定板参数 (唯一真相源; square_length 可按卡尺实测回填) ──
        self.declare_parameter('squares_x', DEFAULT_SQUARES_X)
        self.declare_parameter('squares_y', DEFAULT_SQUARES_Y)
        self.declare_parameter('square_length', DEFAULT_SQUARE_LENGTH)
        self.declare_parameter('marker_length', DEFAULT_MARKER_LENGTH)
        self.declare_parameter('aruco_dict', DEFAULT_DICT)
        # ── 逐帧质量门 ──
        self.declare_parameter('min_markers', 14)
        self.declare_parameter('min_corners', 20)
        self.declare_parameter('max_reproj_rms', 0.5)
        self.declare_parameter('min_square_px', MIN_HANDEYE_SQUARE_PX)
        self.declare_parameter('settle_delay', 0.8)
        self.declare_parameter('max_time_diff', 0.05)
        # ── 求解 / 验收 ──
        self.declare_parameter('holdout', 4)
        # ── 调试可视化 (替代原 aruco_ros 的画面预览) ──
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_topic', '/handeye/debug_image')
        # ── 相机标识 / 内参来源 / 新鲜度与同步 (计划 5.2/5.4) ──
        # use_camera_info: 仅显式 true 时允许用本相机驱动 CameraInfo 作内参来源
        # max_message_age_s: 图像/CameraInfo/机器人位姿按接收 monotonic 年龄校验
        # sync_time_source: receive(默认, 接收差<max_time_diff) | header(要求有效同域时间)
        self.declare_parameter('camera_id', 'd455')
        self.declare_parameter('use_camera_info', False)
        self.declare_parameter('max_message_age_s', 1.0)
        self.declare_parameter('sync_time_source', 'receive')

        p = lambda n: self.get_parameter(n).get_parameter_value()
        self.mode = p('mode').string_value
        self.min_num = p('min_num').integer_value
        self.piper_topic = p('piper_topic').string_value
        self.result_save_path = p('result_save_path').string_value
        self.image_topic = p('image_topic').string_value
        self.camera_info_topic = p('camera_info_topic').string_value
        self.intrinsics_yaml = p('intrinsics_yaml').string_value
        self.squares_x = p('squares_x').integer_value
        self.squares_y = p('squares_y').integer_value
        self.square_length = p('square_length').double_value
        self.marker_length = p('marker_length').double_value
        self.aruco_dict = p('aruco_dict').string_value
        self.min_markers = p('min_markers').integer_value
        self.min_corners = p('min_corners').integer_value
        self.max_reproj_rms = p('max_reproj_rms').double_value
        self.min_square_px = p('min_square_px').double_value
        self.settle_delay = p('settle_delay').double_value
        self.max_time_diff = p('max_time_diff').double_value
        self.holdout = p('holdout').integer_value
        self.publish_debug_image = p('publish_debug_image').bool_value
        self.debug_image_topic = p('debug_image_topic').string_value
        self.camera_id = p('camera_id').string_value
        self.use_camera_info = p('use_camera_info').bool_value
        self.max_message_age_s = p('max_message_age_s').double_value
        self.sync_time_source = p('sync_time_source').string_value

        print(f"mode: {self.mode}")
        print(f"min_num: {self.min_num}")
        print(f"piper_topic: {self.piper_topic}")
        print(f"image_topic: {self.image_topic}")
        print(f"camera_info_topic: {self.camera_info_topic}")
        print(f"intrinsics_yaml: {self.intrinsics_yaml}")
        print(f"标定板: {self.squares_x}x{self.squares_y} 方格={self.square_length}m "
              f"标记={self.marker_length}m 字典={self.aruco_dict} "
              f"(满分 {FULL_MARKERS} 标记 / {FULL_CORNERS} 角点)")
        print(f"质量门: 标记>={self.min_markers} 角点>={self.min_corners} "
              f"RMS<{self.max_reproj_rms}px 方格>={self.min_square_px}px "
              f"同步<{self.max_time_diff * 1000:.0f}ms 静止延时={self.settle_delay}s")
        print(f"result_save_path: {self.result_save_path}")
        print(f"camera_id: {self.camera_id}")
        print(f"use_camera_info: {self.use_camera_info} (仅显式 true 时用驱动内参)")
        print(f"max_message_age_s: {self.max_message_age_s}  "
              f"sync_time_source: {self.sync_time_source}")

        self.bridge = CvBridge()
        self.board = make_board(self.square_length, self.marker_length, self.aruco_dict,
                               self.squares_x, self.squares_y)
        self.detector = make_detector(self.board)

        self.piper_poses = []
        self.marker_poses = []
        self.frames = []

        self.camera_info = None
        self.camera_info_recv = None            # 接收 monotonic
        self.camera_matrix = None
        self.dist_coeffs = None
        self.intrinsics_size = None
        self.intrinsics_source = None           # 内参来源说明 (计划 5.2)
        self._camera_info_selected = False
        # 输入快照: 回调在后台 spin 线程写, 采样时一次取锁保护的快照 (计划 5.4)
        self._lock = threading.Lock()
        self._img_seq = 0
        self._pose_seq = 0
        self._info_seq = 0
        self.latest_image = None   # (Image msg, 接收 monotonic, 接收序号)
        self.latest_pose = None    # (PoseStamped msg, 接收 monotonic, 接收序号)
        self.last_sampled_img_seq = None        # 拒绝重复采样同一帧

        # ── 常驻订阅 (整个会话保持, 采集时取最新值) ──
        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(PoseStamped, self.piper_topic, self.pose_cb, 10)

        self.setup_intrinsics()
        self._read_tcp_metadata()

        # ── 会话目录: 原始图像 + 逐帧 JSON ──
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.session_name = stamp + "_session"
        self.session_dir = os.path.join(self.result_save_path, self.session_name)
        create_dir(self.session_dir)
        print(f"会话目录: {self.session_dir}")

        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 1)
            self.create_timer(0.2, self.debug_image_timer)
            print(f"调试图话题: {self.debug_image_topic} (可用 rqt_image_view 查看)")

        # ── 后台线程持续 spin (关键修复) ──
        # 主线程在 input() 等回车时会阻塞十几秒。若不 spin, 消息会在 DDS 里积压;
        # 回车后只 spin 0.8s (≈40 次 spin_once, 而 /feedback/tcp_pose 是 200Hz,
        # 回调机会基本被它吃光) → latest_image 会停在整个积压队列里最旧的一帧。
        # 实测现象: 图像戳比墙钟旧 17s, 但接收时刻才 0.8s 前 → 同步门误报"不同步"。
        # 常驻 spin 后, 等待输入期间消息照常被处理, latest_image 永远是最新帧。
        self._shutdown = False
        self._spin_thread = threading.Thread(
            target=self._spin_loop, name="handeye_spin", daemon=True)
        self._spin_thread.start()

    # ────────────────────────── 后台 spin ──────────────────────────

    def _spin_loop(self):
        """后台线程: 持续处理订阅 / 定时器回调 (整个进程唯一的 spin 点)。"""
        while rclpy.ok() and not getattr(self, "_shutdown", False):
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                if not rclpy.ok() or getattr(self, "_shutdown", False):
                    break
                time.sleep(0.05)

    def stop_spin(self):
        """停止后台 spin 线程 (退出前调用)。"""
        self._shutdown = True
        thread = getattr(self, "_spin_thread", None)
        if thread is not None:
            thread.join(timeout=1.0)

    def _read_tcp_metadata(self):
        """Read and freeze the driver endpoint; never enable/move the robot."""
        client = self.create_client(GetParameters, "/agx_arm_ctrl_single_node/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError("无法读取控制节点tcp_offset；请先启动T1")
            request = GetParameters.Request()
            request.names = ["tcp_offset"]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.)
            if not future.done() or future.exception() is not None:
                raise RuntimeError("读取tcp_offset超时/失败")
            response = future.result()
            if response is None or len(response.values) != 1 or response.values[0].type != 8:
                raise RuntimeError("控制节点tcp_offset不是double_array")
            self.tcp_offset = vector(list(response.values[0].double_array_value), 6, "tcp_offset")
            self.robot_pose_frame = "tcp_link" if any(self.tcp_offset) else "link6"
        finally:
            self.destroy_client(client)

    def pose_metadata(self):
        optical = camera_optical_frame(self.camera_id)
        return {"robot_base_frame": "base_link", "robot_pose_frame": self.robot_pose_frame,
                "camera_optical_frame": optical, "tcp_offset": self.tcp_offset,
                "tcp_offset_source": "/agx_arm_ctrl_single_node/tcp_offset",
                "transform_parent": ("base_link" if self.mode == "eye_to_hand"
                                     else self.robot_pose_frame),
                "transform_child": optical, "units": "m", "quaternion_order": "xyzw"}

    # ────────────────────────── 内参 ──────────────────────────

    def setup_intrinsics(self):
        """An invalid explicit file fails closed, even with CameraInfo enabled."""
        self._camera_info_selected = False
        if os.path.isfile(self.intrinsics_yaml):
            try:
                loaded = load_intrinsics(self.intrinsics_yaml)
                if loaded is None:
                    raise ValueError("内参无法解析")
                import runtime_config as rc
                from workbench.calibration import selected_file
                if not selected_file(rc.workspace_root(), "intrinsics", self.camera_id, self.intrinsics_yaml):
                    validate_intrinsics(loaded["raw"], self.camera_id, approved=True)
            except (ValueError, KeyError, TypeError) as exc:
                raise RuntimeError(f"内参文件未批准/非法/相机不符: {self.intrinsics_yaml}: {exc}") from exc
            self.camera_matrix = loaded["camera_matrix"]
            self.dist_coeffs = loaded["dist_coeffs"]
            self.intrinsics_size = (loaded["image_width"], loaded["image_height"])
            self.intrinsics_source = f"approved_intrinsics_file:{self.intrinsics_yaml}"
            print(f"已加载本相机批准内参: {self.intrinsics_yaml} {self.intrinsics_size}")
            return
        if not self.use_camera_info:
            raise RuntimeError(f"该相机批准内参不存在: {self.intrinsics_yaml}; "
                               "先采集并批准，或显式use_camera_info:=true")
        self._camera_info_selected = True
        deadline = time.monotonic() + 10.
        while self.camera_info is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
        if self.camera_info is None:
            raise RuntimeError(f"未收到 {self.camera_info_topic}")
        self._apply_camera_info_intrinsics(self.camera_info)

    def _apply_camera_info_intrinsics(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(msg.d, dtype=np.float64).reshape(-1)
        if dist.size == 0:
            dist = np.zeros(5, dtype=np.float64)
        self.dist_coeffs = dist.reshape(1, -1)
        self.intrinsics_size = (int(msg.width), int(msg.height))
        self.intrinsics_source = "camera_info"

    def camera_info_cb(self, msg):
        with self._lock:
            self._info_seq += 1
            self.camera_info = msg
            self.camera_info_recv = time.monotonic()
        if self.camera_matrix is not None:
            # 内参来自内参文件: 只做分辨率一致性检查 (文档第 1 条)
            if self.intrinsics_size and (msg.width, msg.height) != self.intrinsics_size:
                self.get_logger().warning(
                    f"内参文件分辨率 {self.intrinsics_size} 与当前相机 "
                    f"{msg.width}x{msg.height} 不一致, 内参可能失效!")
            return
        if not self._camera_info_selected:
            return   # Only the explicitly chosen source may initialize K/D.
        self._apply_camera_info_intrinsics(msg)

    def image_cb(self, msg):
        with self._lock:
            self._img_seq += 1
            self.latest_image = (msg, time.monotonic(), self._img_seq)

    def pose_cb(self, msg):
        with self._lock:
            self._pose_seq += 1
            self.latest_pose = (msg, time.monotonic(), self._pose_seq)

    # ────────────────────────── 调试可视化 ──────────────────────────

    def debug_image_timer(self):
        if self.latest_image is None or self.camera_matrix is None:
            return
        with self._lock:
            snap = self.latest_image
        if snap is None:
            return
        img_msg = snap[0]
        try:
            bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            vis = bgr.copy()
            obs = detect_board(gray, self.board, self.detector)
            if obs is None:
                cv2.putText(vis, "NO BOARD", (10, 32), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 0, 255), 2)
            else:
                cv2.aruco.drawDetectedCornersCharuco(vis, obs["corners_raw"],
                                                     obs["ids_raw"], (0, 255, 0))
                pose = board_pose(obs, self.board, self.camera_matrix, self.dist_coeffs,
                                  self.squares_x)
                if pose is not None:
                    cv2.drawFrameAxes(vis, self.camera_matrix, self.dist_coeffs,
                                      pose["rvec"], pose["tvec"], 0.03)
                    bad = (obs["n_markers"] < self.min_markers
                           or obs["n_corners"] < self.min_corners
                           or pose["reproj_rms_px"] >= self.max_reproj_rms
                           or pose["square_px"] < self.min_square_px)
                    color = (0, 0, 255) if bad else (0, 255, 0)
                    cv2.putText(vis,
                                f"m{obs['n_markers']}/{FULL_MARKERS} "
                                f"c{obs['n_corners']}/{FULL_CORNERS} "
                                f"rms{pose['reproj_rms_px']:.2f}px "
                                f"sq{pose['square_px']:.0f}px",
                                (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            out = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            out.header = img_msg.header
            self.debug_pub.publish(out)
        except Exception as exc:  # 调试话题不应干扰标定主流程
            self.get_logger().debug(f"调试图像发布失败: {exc}")

    # ────────────────────────── 采集 ──────────────────────────

    def spin_for(self, seconds):
        """等待 settle_delay 让机械臂停稳。

        消息由后台 _spin_loop 线程持续接收, 所以这里只 sleep。
        不能在这里 spin_once: 会和后台线程争抢同一个 executor。
        """
        time.sleep(seconds)

    def sync_status(self, img_msg, img_recv, pose_msg, pose_recv):
        """返回 (是否同步, 描述文字, 来源)。

        计划 5.4:
          - sync_time_source=receive (默认): 按接收 monotonic 差 < max_time_diff;
            说明为"接收同步(非硬件同步)"。
          - sync_time_source=header: 要求图像/位姿 header 时间戳均有效且同域,
            差 < max_time_diff; header 超限直接拒绝(不再用接收近掩盖)。
        """
        ts_img = stamp_to_sec(img_msg.header.stamp)
        ts_pose = stamp_to_sec(pose_msg.header.stamp)
        if self.sync_time_source == 'header':
            if ts_img <= 0.0 or ts_pose <= 0.0:
                return False, "header 时间戳无效 (显式 header 模式要求有效同域时间)", "header"
            diff = abs(ts_img - ts_pose)
            if diff < self.max_time_diff:
                return True, f"header 时间戳差 {diff * 1000:.1f}ms (显式 header 模式)", "header"
            return False, (f"header 时间戳差 {diff * 1000:.1f}ms 超限 "
                           f"(header 模式直接拒绝)"), "header"

        recv_diff = abs(img_recv - pose_recv)
        note = "接收同步(非硬件同步)"
        if recv_diff < self.max_time_diff:
            return True, f"{note} 接收差 {recv_diff * 1000:.1f}ms", "receive"
        return False, f"{note} 接收差 {recv_diff * 1000:.1f}ms 超限", "receive"

    def capture_frame(self):
        """采集一组 (图像 + 机器人位姿), 过质量门; 失败返回 None 且不计数。

        计划 5.4: 一次取锁保护输入快照; 按接收年龄校验新鲜度; 接收序号拒绝重复帧。
        """
        self.spin_for(self.settle_delay)

        if self.camera_matrix is None:
            print("!! 内参未知 (无已批准内参且未显式 use_camera_info), 无法检测 !!")
            return None

        # 锁保护的一次输入快照 (回调在后台 spin 线程写)
        with self._lock:
            snap_img = self.latest_image
            snap_pose = self.latest_pose
            snap_info = self.camera_info
            snap_info_recv = self.camera_info_recv
            last_seq = self.last_sampled_img_seq

        if snap_img is None:
            print("!! 未收到图像, 请检查 image_topic !!")
            return None
        if snap_pose is None:
            print("!! 未收到机器人位姿, 请检查 piper_topic !!")
            return None
        if snap_info is None:
            print("!! 未收到 camera_info (新鲜度/尺寸核对需要) !!")
            return None

        img_msg, img_recv, img_seq = snap_img
        pose_wrap, pose_recv, _pose_seq = snap_pose

        if last_seq is not None and img_seq == last_seq:
            print("!! 与上次采样是同一帧图像 (未收到新图), 请调整后重采 !!")
            return None

        now = time.monotonic()
        img_age = now - img_recv
        pose_age = now - pose_recv
        if img_age > self.max_message_age_s:
            print(f"!! 图像过期 (接收年龄 {img_age:.2f}s > {self.max_message_age_s}s), 该帧作废")
            return None
        if pose_age > self.max_message_age_s:
            print(f"!! 机器人位姿过期 (接收年龄 {pose_age:.2f}s > "
                  f"{self.max_message_age_s}s), 该帧作废")
            return None
        if snap_info_recv is not None and (now - snap_info_recv) > self.max_message_age_s:
            print(f"!! CameraInfo 过期 (接收年龄 {now - snap_info_recv:.2f}s), 该帧作废")
            return None

        if isinstance(pose_wrap, PoseStamped):
            pose_msg, pose_pose = pose_wrap, pose_wrap.pose
        else:
            print("!! 机器人位姿消息类型不是 PoseStamped !!")
            return None
        if pose_msg.header.frame_id and pose_msg.header.frame_id != "base_link":
            print("!! 机器人位姿不是base_link坐标，拒绝采样")
            return None
        if (img_msg.width, img_msg.height) != (snap_info.width, snap_info.height):
            print("!! 图像与CameraInfo尺寸不一致，拒绝采样")
            return None

        # 图像尺寸与内参分辨率一致性 (计划 5.2: 尺寸不符不能静默通过)
        if self.intrinsics_size and (img_msg.width, img_msg.height) != self.intrinsics_size:
            print(f"!! 图像尺寸 {img_msg.width}x{img_msg.height} 与内参分辨率 "
                  f"{self.intrinsics_size} 不符, 该帧作废 !!")
            return None

        try:
            bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as exc:
            print(f"!! 图像转换失败: {exc} !!")
            return None
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        obs = detect_board(gray, self.board, self.detector)
        if obs is None:
            print("!! 未检出标定板 (该帧作废) !! 请让标定板完整进入视野后重采")
            return None
        pose = board_pose(obs, self.board, self.camera_matrix, self.dist_coeffs,
                          self.squares_x)
        if pose is None:
            print(f"!! 标定板位姿求解失败 (仅检出 {obs['n_corners']} 角点, 该帧作废) !!")
            return None

        sync_ok, sync_desc, sync_src = self.sync_status(img_msg, img_recv, pose_msg, pose_recv)

        violations = []
        if obs["n_markers"] < self.min_markers:
            violations.append(f"检出标记数 {obs['n_markers']} (满分 {FULL_MARKERS}) "
                              f"< 门限 {self.min_markers}")
        if obs["n_corners"] < self.min_corners:
            violations.append(f"检出角点数 {obs['n_corners']} (满分 {FULL_CORNERS}) "
                              f"< 门限 {self.min_corners}")
        if pose["reproj_rms_px"] >= self.max_reproj_rms:
            violations.append(f"单帧重投影 RMS {pose['reproj_rms_px']:.3f}px "
                              f">= 门限 {self.max_reproj_rms}px (检查运动模糊/曝光)")
        if pose["square_px"] < self.min_square_px:
            violations.append(f"方格像素边长 {pose['square_px']:.1f}px "
                              f"< 门限 {self.min_square_px}px (请靠近或换大板)")
        if not sync_ok:
            violations.append(f"图像与机器人位姿不同步 ({sync_desc})")

        if violations:
            print("!! 质量门不通过, 该帧作废, 请调整后重采:")
            for item in violations:
                print(f"   - {item}")
            return None

        if obs["n_markers"] < FULL_MARKERS:
            print(f"   提示: 标记数 {obs['n_markers']} < {FULL_MARKERS} (未丢出画面为佳)")

        index = len(self.frames)
        filename = f"f{index:03d}.png"
        cv2.imwrite(os.path.join(self.session_dir, filename), bgr)

        T_t2c = pose["T_t2c"]
        marker_pose = PoseMartix(
            create_pose_from_martix(T_t2c[:3, 3].reshape(3, 1), T_t2c[:3, :3]),
            False, self.mode)
        piper_pose = PoseMartix(pose_pose, True, self.mode)
        raw_pose = PoseMartix(pose_pose, False, self.mode)
        T_g2b = rt_to_T(piper_pose.rotation_matrix, piper_pose.translation_matrix)
        T_g2b_raw = rt_to_T(raw_pose.rotation_matrix, raw_pose.translation_matrix)

        self.piper_poses.append(piper_pose)
        self.marker_poses.append(marker_pose)
        self.frames.append({
            "i": index,
            "img": filename,
            "markers": obs["n_markers"],
            "corners": obs["n_corners"],
            "reproj_rms_px": float(pose["reproj_rms_px"]),
            "square_px": float(pose["square_px"]),
            "T_g2b": T_g2b.tolist(),
            "T_g2b_raw": T_g2b_raw.tolist(),
            "T_t2c": T_t2c.tolist(),
            "ts_img": stamp_to_sec(img_msg.header.stamp),
            "ts_pose": stamp_to_sec(pose_msg.header.stamp),
            "sync": sync_desc,
            # 计划 5.4: 接收序号/新鲜度/同步来源 元数据
            "img_seq": img_seq,
            "img_age_s": round(float(img_age), 4),
            "pose_age_s": round(float(pose_age), 4),
            "sync_source": sync_src,
            "resolution": [int(img_msg.width), int(img_msg.height)],
            "camera_id": self.camera_id,
        })
        self.last_sampled_img_seq = img_seq   # 成功采样后记录, 拒绝重复帧
        self.write_frames_json()

        print(f"--- 采集成功 #{index} ---")
        print(f"  标记 {obs['n_markers']}/{FULL_MARKERS}  角点 {obs['n_corners']}/{FULL_CORNERS}  "
              f"RMS {pose['reproj_rms_px']:.3f}px  方格 {pose['square_px']:.1f}px")
        print(f"  同步: {sync_desc}")
        print(f"  标定板在相机系: t={np.round(pose['tvec'], 4)} |t|={np.linalg.norm(pose['tvec']):.4f}m")
        if self.mode == 'eye_to_hand':
            print(f"  TCP(原始): {np.round(raw_pose.translation_matrix, 4)}")
        self.print_span_hint()
        return self.frames[-1]

    def print_span_hint(self):
        """采集过程中即时提示方位跨度 (精度的唯一大杠杆, 文档 §5.1)。"""
        if len(self.frames) < 3:
            return
        rotations = [np.array(f["T_g2b"])[:3, :3] for f in self.frames]
        angles = []
        for i in range(len(rotations)):
            for j in range(i + 1, len(rotations)):
                angles.append(rotation_angle_deg(rotations[i], rotations[j]))
        median_angle = float(np.median(angles))
        if median_angle < 25.0:
            print(f"  [跨度提示] 当前相对转角中位 {median_angle:.1f}° < 25°: "
                  f"请补绕两个轴的 ±25~35° 倾斜与绕光轴自转 (文档 §5.1)")
        else:
            print(f"  [跨度提示] 当前相对转角中位 {median_angle:.1f}° (已达标)")

    def write_frames_json(self):
        """把逐帧元数据落盘 (文档 §5.5), 便于事后离线复算。"""
        payload = {
            "board": {
                "squares": [self.squares_x, self.squares_y],
                "square_m": self.square_length,
                "marker_m": self.marker_length,
                "dict": self.aruco_dict,
                "full_markers": FULL_MARKERS,
                "full_corners": FULL_CORNERS,
            },
            "camera": {
                "image_topic": self.image_topic,
                "camera_info_topic": self.camera_info_topic,
                "intrinsics_yaml": self.intrinsics_yaml,
                "resolution": list(self.intrinsics_size) if self.intrinsics_size else None,
                "K": self.camera_matrix.tolist() if self.camera_matrix is not None else None,
                "dist": self.dist_coeffs.tolist() if self.dist_coeffs is not None else None,
            },
            "mode": self.mode,
            "camera_id": self.camera_id,
            "intrinsics_source": self.intrinsics_source,
            "sync_time_source": self.sync_time_source,
            "max_message_age_s": self.max_message_age_s,
            "sync_note": ("receive=接收单调时间差(非硬件同步); header=显式同域时间戳"
                          if self.sync_time_source == 'receive'
                          else "显式 header 模式(要求有效同域时间戳)"),
            "frames": self.frames,
            "robot": self.pose_metadata(),
        }
        with open(os.path.join(self.session_dir, "frames.json"), 'w+') as fp:
            json.dump(payload, fp, indent=2)

    # ────────────────────────── 求解 ──────────────────────────

    def print_data_diagnostics(self, T_g2b, T_t2c):
        """保留旧版数据诊断: 位置范围 / 相对转角 / 标定板朝向一致性。"""
        print("\n========== 数据诊断 ==========")
        tcp_positions = np.array([T[:3, 3] for T in T_g2b])
        marker_positions = np.array([T[:3, 3] for T in T_t2c])
        print(f"TCP位置范围 X:[{tcp_positions[:, 0].min():.4f}, {tcp_positions[:, 0].max():.4f}] "
              f"Y:[{tcp_positions[:, 1].min():.4f}, {tcp_positions[:, 1].max():.4f}] "
              f"Z:[{tcp_positions[:, 2].min():.4f}, {tcp_positions[:, 2].max():.4f}]")
        print(f"marker位置范围 X:[{marker_positions[:, 0].min():.4f}, "
              f"{marker_positions[:, 0].max():.4f}] "
              f"Y:[{marker_positions[:, 1].min():.4f}, {marker_positions[:, 1].max():.4f}] "
              f"Z:[{marker_positions[:, 2].min():.4f}, {marker_positions[:, 2].max():.4f}]")

        tcp_angles, cam_angles = [], []
        for i in range(len(T_g2b) - 1):
            tcp_angles.append(rotation_angle_deg(T_g2b[i][:3, :3], T_g2b[i + 1][:3, :3]))
            cam_angles.append(rotation_angle_deg(T_t2c[i][:3, :3], T_t2c[i + 1][:3, :3]))
        if tcp_angles:
            print(f"TCP相对旋转角度: min={min(tcp_angles):.1f}° max={max(tcp_angles):.1f}° "
                  f"avg={np.mean(tcp_angles):.1f}°")
            print(f"相机相对旋转角度: min={min(cam_angles):.1f}° max={max(cam_angles):.1f}° "
                  f"avg={np.mean(cam_angles):.1f}°")
            if max(tcp_angles) < 5.0 and max(cam_angles) < 5.0:
                print("!! 警告: 相对旋转角度最大值<5°, 旋转约束不足, 标定结果不可靠 !!")

        signs = [1.0 if T[:3, :3][2, 2] > 0 else -1.0 for T in T_t2c]
        unique = set(signs)
        if len(unique) > 1:
            pos = sum(1 for s in signs if s > 0)
            neg = sum(1 for s in signs if s < 0)
            print(f"!! 标定板方向不一致! z>0:{pos}组 z<0:{neg}组, 混入野值将严重破坏标定 !!")
        else:
            print(f"标定板 Z 方向一致 (全部 z{'>' if unique == {1.0} else '<'}0), 方向无翻转")

    def process_handeye(self):
        T_g2b = [rt_to_T(p.rotation_matrix, p.translation_matrix) for p in self.piper_poses]
        T_t2c = [rt_to_T(m.rotation_matrix, m.translation_matrix) for m in self.marker_poses]

        self.print_data_diagnostics(T_g2b, T_t2c)

        # ── 主求解: AX=XB 双算法互校 + 留出位姿验证 ──
        result = solve_axxb(T_g2b, T_t2c)
        holdout = holdout_validate(T_g2b, T_t2c, self.holdout)
        print("\n" + format_criteria_report(result, holdout))
        print("\n主解 (Kronecker 9x9): t="
              f"{np.round(result['t'], 6)} |t|={np.linalg.norm(result['t']):.6f}m")

        # ── 交叉参考: cv2.calibrateHandEye 多方法 (残差仅参考, 不作为验收) ──
        methods = {
            'TSAI': cv2.CALIB_HAND_EYE_TSAI,
            'PARK': cv2.CALIB_HAND_EYE_PARK,
            'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
            'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
            'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
        }
        R_g2b = [T[:3, :3] for T in T_g2b]
        t_g2b = [T[:3, 3].reshape(3, 1) for T in T_g2b]
        R_t2c = [T[:3, :3] for T in T_t2c]
        t_t2c = [T[:3, 3].reshape(3, 1) for T in T_t2c]

        print("\n========== 交叉参考: cv2.calibrateHandEye ==========")
        print("(残差与距离均仅作参考, 不作验收; 以双算法解为准)")
        for name, method in methods.items():
            try:
                ret_R, ret_t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=method)
                rot_diff = rotation_angle_deg(result["R"], ret_R)
                print(f"{name:12s} | 与主解旋转差={rot_diff:7.3f}° "
                      f"| 距离={np.linalg.norm(ret_t.ravel()):.4f}m | pos={np.round(ret_t.ravel(), 4)}")
            except Exception as exc:
                print(f"{name:12s} | 失败: {exc}")

        return (PoseMartix(create_pose_from_martix(result["t"].reshape(3, 1), result["R"])),
                result, holdout)

    # ────────────────────────── 交互采集 ──────────────────────────

    def run(self):
        if self.mode != "eye_in_hand" and self.mode != "eye_to_hand":
            raise ValueError("'mode' 必须为 eye_in_hand 或 eye_to_hand ")
        if self.min_num < 5:
            raise ValueError("最少采集次数不得小于5次")

        self.get_poses()
        result_pose, solver_result, holdout = self.process_handeye()
        result = dict(
            position=result_pose.position_list,
            orientation=result_pose.orientation_list,
            rpy=result_pose.rpy_list
        )

        print("标定结果:")
        print("")
        print(json.dumps(result, indent=4))
        print("")
        create_dir(self.result_save_path)
        filename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        quality_status, quality_checks = self.build_quality(solver_result, holdout)
        output = dict(result)
        output["criteria"] = self.build_criteria(solver_result, holdout)
        output["metadata"] = {
            "kind": "handeye_candidate",
            "camera_id": self.camera_id,
            "mode": self.mode,
            "resolution": list(self.intrinsics_size) if self.intrinsics_size else None,
            "board": {
                "squares": [self.squares_x, self.squares_y],
                "square_m": self.square_length,
                "marker_m": self.marker_length,
                "dict": self.aruco_dict,
            },
            "frames_used": len(self.frames),
            "session_dir": self.session_dir,
            "intrinsics_source": self.intrinsics_source,
            "sync_time_source": self.sync_time_source,
            "max_message_age_s": self.max_message_age_s,
            "convention": ("p_base = T_base_camera @ p_camera"
                           if self.mode == 'eye_to_hand'
                           else "T_base_camera(t) = T_base_tcp(t) @ T_tcp_camera"),
            "quality_status": quality_status,
            "quality_checks": quality_checks,
            "approved": False,
            "note": ("候选结果；文件生成不等于验收通过；须操作者 approve-handeye "
                     "且 --scene-verified 后才成为活动外参"),
            **self.pose_metadata(),
        }
        with open(f"{self.result_save_path}/{filename}_calibration.json", 'w+') as json_file:
            json.dump(output, json_file, indent=4)
            print(f"writing to {self.result_save_path}/{filename}_calibration.json")
        print(f"质量状态: {quality_status}")
        for c in quality_checks:
            mark = "OK " if c["ok"] else "NG "
            print(f"  [{mark}] {c['check']} ({c['detail']})")
        print("提示: 这是候选结果, 不自动成为活动外参; 需 approve-handeye 且现场坐标核查通过。")

        raw_data = {
            "piper_poses": [{
                "position": p.position_list,
                "orientation": p.orientation_list,
                "rotation_matrix": p.rotation_matrix.tolist(),
                "translation": p.translation_matrix.tolist()
            } for p in self.piper_poses],
            "marker_poses": [{
                "position": m.position_list,
                "orientation": m.orientation_list,
                "rotation_matrix": m.rotation_matrix.tolist(),
                "translation": m.translation_matrix.tolist()
            } for m in self.marker_poses]
        }
        with open(f"{self.result_save_path}/{filename}_raw_data.json", 'w+') as raw_file:
            json.dump(raw_data, raw_file, indent=2)
            print(f"raw data saved to {self.result_save_path}/{filename}_raw_data.json")
        print(f"逐帧数据与原始图像: {self.session_dir}")

    def build_quality(self, result, holdout):
        """手眼候选质量状态（计划 5.3 保留原标准）: 达标 passed, 否则 failed。"""
        checks = []

        def _chk(name, ok, detail=""):
            checks.append({"check": name, "ok": bool(ok), "detail": detail})

        minimum = max(15, self.min_num)
        _chk("有效姿态数 >= max(15,min_num)", len(self.frames) >= minimum,
             f"{len(self.frames)}/{minimum}")
        try:
            validate_capture_frames(self.frames, len(self.frames))
            _chk("逐帧固定推广门", True, f"markers>=14 corners>=20 RMS<0.5px square>={MIN_HANDEYE_SQUARE_PX:g}px")
        except CalibrationValidationError as exc:
            _chk("逐帧固定推广门", False, str(exc))
        _chk(f"consistency_deg < {CONSISTENCY_MAX_DEG:g}", result["consistency_deg"] < CONSISTENCY_MAX_DEG,
             f"{result['consistency_deg']:.5f}")
        _chk(f"median_rel_angle_deg >= {MEDIAN_REL_ANGLE_MIN_DEG:g}", result["median_rel_angle_deg"] >= MEDIAN_REL_ANGLE_MIN_DEG,
             f"{result['median_rel_angle_deg']:.2f}")
        _chk(f"n_pairs_ge25deg >= {MIN_USABLE_PAIRS}", result["n_pairs_ge25deg"] >= MIN_USABLE_PAIRS,
             f"{result['n_pairs_ge25deg']}")
        if holdout is not None:
            _chk(f"holdout median_mm < {HOLDOUT_MAX_MM:g}", holdout["median_mm"] < HOLDOUT_MAX_MM,
                 f"median={holdout['median_mm']:.4f} p95={holdout['p95_mm']:.4f} "
                 f"max={holdout['max_mm']:.4f}")
        else:
            _chk("holdout 验证已执行", False, "无 holdout 结果")
        status = "passed" if all(c["ok"] for c in checks) else "failed"
        return status, checks

    def build_criteria(self, result, holdout):
        criteria = {
            "mode": self.mode,
            "session_dir": self.session_dir,
            "frames_used": len(self.frames),
            "median_rel_angle_deg": result["median_rel_angle_deg"],
            "consistency_deg": result["consistency_deg"],
            "n_pairs_ge25deg": result["n_pairs_ge25deg"],
            "eq_residual_mm_reference_only": result["eq_residual_mm"],
            "capture_frames": [{key: frame[key] for key in
                                ("markers", "corners", "reproj_rms_px", "square_px")}
                               for frame in self.frames],
            "validation_thresholds": {"holdout_median_mm_lt": HOLDOUT_MAX_MM,
                                      "median_rel_angle_deg_min": MEDIAN_REL_ANGLE_MIN_DEG,
                                      "consistency_deg_lt": CONSISTENCY_MAX_DEG,
                                      "min_usable_pairs_ge25deg": MIN_USABLE_PAIRS},
            "acquisition_thresholds": {"min_markers": self.min_markers,
                                       "min_corners": self.min_corners,
                                       "max_reproj_rms": self.max_reproj_rms,
                                       "min_square_px": self.min_square_px},
        }
        if holdout is not None:
            criteria["holdout"] = {
                "n": holdout["n"],
                "median_mm": holdout["median_mm"],
                "p95_mm": holdout["p95_mm"],
                "max_mm": holdout["max_mm"],
                "indices": holdout["holdout_idx"],
            }
        return criteria

    def get_poses(self):
        count = 1
        while True:
            print(f"\n------- 第{count}次采集 ------")
            if count == 1:
                menu_str = "请输入(Enter-采集):"
            elif count <= self.min_num:
                menu_str = "请输入(Enter-采集, d-回退):"
            else:
                menu_str = "请输入(Enter-采集, d-回退, q-计算并退出):"
            try:
                clear_input_buffer()
                user_input = input(menu_str + " ")
                if user_input == '':
                    if self.capture_frame() is not None:
                        count += 1
                elif count > 1 and user_input == 'd':
                    count -= 1
                    self.piper_poses.pop()
                    self.marker_poses.pop()
                    removed = self.frames.pop()
                    image_path = os.path.join(self.session_dir, removed["img"])
                    if os.path.exists(image_path):
                        os.remove(image_path)
                    self.write_frames_json()
                    print(f"已回退第{removed['i']}组")
                elif user_input == 'c':
                    print("exit")
                    exit(0)
                elif count > self.min_num and user_input == 'q':
                    print("开始计算标定结果")
                    break
                else:
                    print("无效输入")
            except Exception as e:
                print(f"\n程序异常或中断退出: {e}")
                exit(-1)


def main(args=None):
    rclpy.init(args=args)
    calibration_node = HandEyeCalibrationNode()
    try:
        calibration_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        calibration_node.stop_spin()
        calibration_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
