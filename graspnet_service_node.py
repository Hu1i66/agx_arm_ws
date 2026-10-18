#!/usr/bin/env python3
"""GraspNet 抓取位姿服务节点 (运行在 orange_dataset/.venv)

眼在手外 (eye-to-hand) 配置: 相机固定在传送带上方, T_cam_to_base 是常量.
启动时加载 GraspNet 模型, 持续订阅 D455 深度图,
按需推理并通过 /graspnet/req → /graspnet/result 话题通信返回 6DoF 抓取位姿 (base 系).

启动:
  source /home/lxf/orange_dataset/.venv/bin/activate
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///home/lxf/ROS2-Gazebo-GO2/src/docker/cyclonedds.xml
  cd /home/lxf/agx_arm_ws
  python3 graspnet_service_node.py
"""
import os
import sys
import json
import math
import time
import threading
from collections import deque
import traceback

# ── 共享配置：从主空间根定位 runtime_config（外部实现与资源统一来源）──
import pathlib
_WS_ROOT = pathlib.Path(__file__).resolve().parent
if str(_WS_ROOT) not in sys.path:
    sys.path.insert(0, str(_WS_ROOT))
try:
    import runtime_config as _rc
except ImportError as _exc:  # pragma: no cover
    raise RuntimeError(
        f"无法导入主空间 runtime_config（期望 {_WS_ROOT}/runtime_config.py）: {_exc}"
    ) from _exc

# ── sys.path 注入 graspnet-baseline（外部实现路径来自共享配置）──
ROOT = str(_rc.resolve_resource("graspnet_baseline"))
for _sub in ["", "models", "utils", "pointnet2", "knn"]:
    _p = os.path.join(ROOT, _sub) if _sub else ROOT
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
from camera_geometry import CameraGeometry, image_qos
from runtime_geometry import graspnet_tcp_target, object_centroid_from_rgbd


# ========================================================================
# 常量
# ========================================================================
_HANDEYE_JSON_PATH = str(_rc.load_handeye("d455")["source"])
_GRASPNET_CHECKPOINT = str(_rc.resolve_resource("graspnet_checkpoint"))
_DEPTH_OFFSET_M = 0.0           # eye-to-hand: 相机在可靠深度范围 (0.5-0.8m), 无需近距校正
_NUM_POINT = 20000
_NUM_VIEW = 300
_APPROACH_Z_MIN = 0.30          # 相机系 R[:,0]_z 下限 (approach 朝前=朝下)
_PROXIMITY_MAX_M = 0.08         # 邻近过滤半径 (base 系 pick_pose 到 grasp translation)
_MAX_GRIPPER_WIDTH = 0.085      # 夹爪最大开合


# ========================================================================
# 工具函数
# ========================================================================
def _quat_to_rotmat(qx, qy, qz, qw):
    """四元数 -> 3x3 旋转矩阵 (复用 grasp_pose_node.py L94-104)"""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def _load_handeye_matrix():
    """加载手眼标定矩阵 camera_color_optical_frame -> base_link (eye-to-hand, 常量变换)"""
    with open(_HANDEYE_JSON_PATH, "r") as f:
        data = json.load(f)
    pos = data.get("position", [0.0, 0.0, 0.0])
    ori = data.get("orientation", [0.0, 0.0, 0.0, 1.0])
    R = _quat_to_rotmat(float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3]))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [float(pos[0]), float(pos[1]), float(pos[2])]
    return T


def _rotmat_to_quat(R):
    """3x3 旋转矩阵 -> [qx, qy, qz, qw]

    与 auto_sorting_action._matrix_to_quaternion 算法一致.
    R 列含义: R[:,0]=x_axis(opening), R[:,1]=y_axis(ortho), R[:,2]=z_axis(approach)
    """
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return [float(qx), float(qy), float(qz), float(qw)]


# ========================================================================
# GraspNet 可复用函数 (从 verify_grasp_pose.py 复制)
# ========================================================================
# 模块级 logger: 由 GraspNetServiceNode.__init__ 设置, 确保 print 输出进 ROS2 日志
_logger = None

def _log(msg):
    """统一日志输出: 优先用 ROS2 logger (进 ~/.ros/log), 否则 print 到 stdout"""
    if _logger is not None:
        _logger.info(msg)
    else:
        print(msg)


def depth_to_pointcloud(depth_uint16, color, intrinsics, depth_offset_m):
    """深度图转 Open3D 点云 (相机系). ⚠️ mask 在校正前计算."""
    import open3d as o3d
    from data_utils import (
        CameraInfo as GraspNetCameraInfo,
        create_point_cloud_from_depth_image,
    )

    fx = intrinsics["fx"]; fy = intrinsics["fy"]
    cx = intrinsics["cx"]; cy = intrinsics["cy"]
    width = intrinsics["width"]; height = intrinsics["height"]

    # ⚠️ mask 必须在深度校正之前计算!
    mask = depth_uint16 > 0

    if depth_offset_m != 0.0:
        correction_mm = int(round(depth_offset_m * 1000))
        depth_uint16 = np.clip(
            depth_uint16.astype(np.int32) - correction_mm, 1, 65535
        ).astype(np.uint16)

    cam = GraspNetCameraInfo(
        float(width), float(height), float(fx), float(fy), float(cx), float(cy), 1000.0
    )
    depth_m = depth_uint16.astype(np.float32)
    cloud = create_point_cloud_from_depth_image(depth_m, cam, organized=True)

    points = cloud[mask]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))

    if color is not None and color.shape[:2] == (height, width):
        colors = color[mask].astype(np.float64) / 255.0
        pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd, cloud, mask


def load_graspnet(checkpoint_path, num_view, device):
    """加载 GraspNet 模型"""
    import torch
    from graspnet import GraspNet

    if not os.path.exists(checkpoint_path):
        raise RuntimeError(f"checkpoint 不存在: {checkpoint_path}")

    net = GraspNet(
        input_feature_dim=0, num_view=num_view, num_angle=12, num_depth=4,
        cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04],
        is_training=False,
    )
    net.to(device)
    checkpoint = torch.load(checkpoint_path, map_location=device)
    net.load_state_dict(checkpoint["model_state_dict"])
    net.eval()
    _log(f"  ✅ GraspNet 模型加载成功 (epoch={checkpoint.get('epoch', '?')})")
    return net


def run_inference(net, pcd, num_point, device):
    """GraspNet 推理 → GraspGroup"""
    import torch
    from graspnet import pred_decode
    from graspnetAPI import GraspGroup

    points = np.asarray(pcd.points, dtype=np.float32)
    colors = (
        np.asarray(pcd.colors, dtype=np.float32)
        if pcd.has_colors() else np.zeros_like(points)
    )
    if len(points) < 100:
        raise RuntimeError(f"点云点数过少 ({len(points)}), 无法推理")

    if len(points) >= num_point:
        idxs = np.random.choice(len(points), num_point, replace=False)
    else:
        idxs1 = np.arange(len(points))
        idxs2 = np.random.choice(len(points), num_point - len(points), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)

    cloud_sampled = points[idxs]
    color_sampled = colors[idxs]
    cloud_tensor = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(device)
    end_points = {"point_clouds": cloud_tensor, "cloud_colors": color_sampled}

    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.cuda.reset_peak_memory_stats()

    t0 = time.time()
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    t1 = time.time()

    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    _log(f"  ✅ 推理完成: {len(gg)} 个候选, 耗时 {t1 - t0:.2f}s")
    return gg


def segment_object_points(pcd, distance_thresh=0.01, ransac_n=3,
                          num_iterations=2000, max_depth=0.8,
                          eps=0.012, min_points=20):
    """RANSAC 去桌面 + 深度过滤 + 体素降采样 + DBSCAN 聚类"""
    plane_model, inliers = pcd.segment_plane(distance_thresh, ransac_n, num_iterations)
    object_cloud = pcd.select_by_index(inliers, invert=True)
    _log(f"  去桌面: {len(pcd.points)} → {len(object_cloud.points)} 个非平面点")

    pts = np.asarray(object_cloud.points)
    depth_mask = pts[:, 2] < max_depth
    object_cloud = object_cloud.select_by_index(np.where(depth_mask)[0])
    _log(f"  深度过滤 (<{max_depth}m): → {len(object_cloud.points)} 个近处点")

    if len(object_cloud.points) < 50:
        return object_cloud

    if len(object_cloud.points) > 5000:
        object_cloud = object_cloud.voxel_down_sample(0.005)
        _log(f"  体素降采样(5mm): → {len(object_cloud.points)} 点")

    labels = np.array(object_cloud.cluster_dbscan(eps=eps, min_points=min_points))
    n_clusters = labels.max() + 1 if labels.max() >= 0 else 0
    if n_clusters == 0:
        _log(f"  ⚠️ DBSCAN 无簇, 用深度过滤后的点云")
        return object_cloud

    unique, counts = np.unique(labels[labels >= 0], return_counts=True)
    largest_label = int(unique[np.argmax(counts)])
    cluster_mask = labels == largest_label
    cluster_cloud = object_cloud.select_by_index(np.where(cluster_mask)[0])
    _log(f"  DBSCAN: {n_clusters} 簇 → 最大簇 {len(cluster_cloud.points)} 点")
    return cluster_cloud


def analyze_object_shape(object_cloud):
    """PCA 分析物体形状 → centroid + 长轴/短轴 (XY 投影)"""
    points = np.asarray(object_cloud.points)
    centroid = points.mean(axis=0)
    centered = points - centroid
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    long_axis = eigenvectors[:, 2]
    short_axis = eigenvectors[:, 0]
    long_xy = long_axis[:2].copy()
    short_xy = short_axis[:2].copy()
    long_xy /= max(np.linalg.norm(long_xy), 1e-9)
    short_xy /= max(np.linalg.norm(short_xy), 1e-9)
    xy = points[:, :2]
    size_xy = max(float(np.linalg.norm(xy.max(axis=0) - xy.min(axis=0))), 0.01)
    elongation = float(eigenvalues[2] / max(eigenvalues[0], 1e-9))
    return {
        'centroid': centroid,
        'centroid_xy': centroid[:2],
        'long_axis_xy': long_xy,
        'short_axis_xy': short_xy,
        'elongation': elongation,
        'size_xy': size_xy,
    }


def filter_grasps_short_edge_center(gg, shape, approach_scores=None, pick_dists=None,
                                    w_align=0.30, w_center=0.20, w_score=0.10,
                                    w_vert=0.25, w_pick=0.15):
    """重排候选: 闭合方向对齐短轴 + 位置接近中心 + score + approach垂直度 + YOLO邻近

    approach_scores: 各候选 base 系 approach z 分量 (负=朝下, -1.0=正下方). None=不考虑.
    pick_dists: 各候选到 YOLO 检测位置的 3D 距离 (米). None=不考虑.
    """
    from graspnetAPI import GraspGroup

    short_xy = shape['short_axis_xy']
    centroid_xy = shape['centroid_xy']
    size_xy = shape['size_xy']

    max_score = max(abs(g.score) for g in gg) or 1.0
    records = []
    for i in range(len(gg)):
        g = gg[i]
        R = g.rotation_matrix
        closing_xy = R[:2, 1].copy()
        closing_xy /= max(np.linalg.norm(closing_xy), 1e-9)
        align = abs(float(np.dot(closing_xy, short_xy)))
        center_dist = float(np.linalg.norm(g.translation[:2] - centroid_xy))
        center_score = float(np.exp(-center_dist / size_xy * 3.0))
        score_norm = abs(g.score) / max_score
        # 垂直度: |approach_z|, 1.0=正下方, 0.7≈45°倾斜
        if approach_scores is not None:
            vert = abs(approach_scores[i])
        else:
            vert = 1.0
        # YOLO 邻近: 距检测位置越近越好 (4cm 处衰减到 e^-1≈0.37)
        if pick_dists is not None:
            pick_score = float(np.exp(-pick_dists[i] / 0.04))
        else:
            pick_score = 1.0
        total = (w_align * align + w_center * center_score + w_score * score_norm
                 + w_vert * vert + w_pick * pick_score)
        records.append((total, i, align, center_dist, g.score, vert, pick_score))

    records.sort(key=lambda x: -x[0])
    _log(f"  过滤重排 (短边+中心+垂直度+YOLO邻近):")
    _log(f"    centroid_xy=({centroid_xy[0]:.3f},{centroid_xy[1]:.3f}) "
         f"size={size_xy:.3f} elongation={shape['elongation']:.2f}")
    for total, idx, align, cd, sc, vt, pk in records[:5]:
        _log(f"    idx={idx:3d} total={total:.3f} align={align:.3f} "
             f"c_dist={cd:.3f} vert={vt:.3f} pick={pk:.3f} score={sc:.3f}")

    order = [r[1] for r in records]
    arr = gg.grasp_group_array[order]
    return GraspGroup(arr)


# ========================================================================
# GraspNet 服务节点
# ========================================================================
class GraspNetServiceNode(Node):
    def __init__(self):
        super().__init__("graspnet_service_node")

        # ── 设置模块级 logger, 让 helper 函数的日志也进 ROS2 日志文件 ──
        global _logger
        _logger = self.get_logger()

        self.default_camera = self.declare_parameter('camera_id', 'd455').value
        self.geometry = CameraGeometry(self)
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._detections = {}
        self.create_subscription(String,'/detection_info',self._detection_cb,10)
        self._frames = {name:dict(depth=deque(maxlen=40),color=deque(maxlen=15),info=None) for name in ('d455','dabai')}
        cameras = _rc.load_runtime_config()['cameras']
        for name,cfg in cameras.items():
            self.create_subscription(Image,cfg['depth_topic'],lambda msg,n=name:self._image_cb(n,'depth',msg),image_qos(self,cfg['depth_topic'],name))
            self.create_subscription(Image,cfg['rgb_topic'],lambda msg,n=name:self._image_cb(n,'color',msg),image_qos(self,cfg['rgb_topic'],name))
            self.create_subscription(CameraInfo,cfg['camera_info_topic'],lambda msg,n=name:self._cam_info_cb(n,msg),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String,'/graspnet/req',self._req_cb,10)
        self._result_pub=self.create_publisher(String,'/graspnet/result',10)

        # ── 加载 GraspNet 模型 ──
        import torch
        self._device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"设备: {self._device}")
        self._net = load_graspnet(_GRASPNET_CHECKPOINT, _NUM_VIEW, self._device)
        self.get_logger().info("✅ graspnet_service_node 就绪, 等待请求...")

    def _detection_cb(self,msg):
        try:
            data=json.loads(msg.data)
            camera=data.get('camera')
            if camera in ('d455','dabai'):
                with self._lock:self._detections[camera]=data
        except (ValueError,TypeError):pass

    def _centroid_snapshot(self,request,camera,depth,info,T,stamp):
        with self._lock:data=self._detections.get(camera,{})
        target_id=request.get('target_track_id')
        detection_stamp=float(data.get('header_stamp',0.))
        if target_id is None or not np.isfinite(detection_stamp) or abs(detection_stamp-stamp)>.25:
            raise ValueError('fresh_selected_object_detection_unavailable')
        obj=next((o for o in data.get('objects',[]) if o.get('track_id')==target_id),None)
        if obj is None:raise ValueError('selected_object_track_lost')
        box=obj['bbox_pixel'];bbox=[box[k] for k in ('x1','y1','x2','y2')]
        centre=object_centroid_from_rgbd(depth,bbox,[info.k[0],info.k[4],info.k[2],info.k[5]],T)
        return centre['centroid']

    def _image_cb(self,camera,kind,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        array=self.bridge.imgmsg_to_cv2(msg,desired_encoding='rgb8' if kind=='color' else 'passthrough')
        if kind=='depth' and array.dtype.kind=='f':
            array=np.nan_to_num(array,nan=0.,posinf=0.,neginf=0.)
            array=np.clip(array*1000.,0,65535).astype(np.uint16)
        with self._lock:self._frames[camera][kind].append((stamp,array,time.monotonic()))

    def _cam_info_cb(self,camera,msg):
        with self._lock:self._frames[camera]['info']=(msg,time.monotonic())

    def _snapshot(self,camera):
        with self._lock:
            frames=self._frames[camera]
            colors=list(frames['color']);depths=list(frames['depth']);info=frames['info']
        if not colors or not depths or info is None:raise ValueError('missing_rgb_depth_or_intrinsics')
        stamp,color,received=colors[-1];ds,depth,dr=min(depths,key=lambda row:abs(row[0]-stamp))
        if max(time.monotonic()-received,time.monotonic()-dr,time.monotonic()-info[1])>1.:
            raise ValueError('stale_camera_snapshot')
        now=self.get_clock().now().nanoseconds*1e-9
        if stamp<=0 or not -.05<=now-stamp<=1. or abs(ds-stamp)>.05:raise ValueError('camera_timestamp_or_pair_mismatch')
        ci=info[0]
        if color.shape[:2]!=depth.shape[:2] or depth.shape[:2]!=(ci.height,ci.width):raise ValueError('rgb_depth_intrinsics_size_mismatch')
        if not np.isfinite(ci.k).all() or ci.k[0]<=0 or ci.k[4]<=0:raise ValueError('invalid_camera_intrinsics')
        T=self.geometry.camera(camera,stamp)
        return depth.copy(),color.copy(),ci,T,stamp

    # ── 请求处理 ──
    def _req_cb(self, msg):
        request_id = ''
        camera = self.default_camera
        try:
            req = json.loads(msg.data)
            if not isinstance(req, dict):
                raise ValueError('request_must_be_object')
            request_id = req.get('request_id', '')
            camera = req.get('camera_id', self.default_camera)
            if req.get('pick_semantics') != 'estimated_centroid':
                raise ValueError('pick_must_be_estimated_centroid')
            if req.get('cmd') != 'get_grasp':
                raise ValueError('unknown_cmd')
            if camera not in self._frames:
                raise ValueError('unknown_camera')
            pick = tuple(float(req[key]) for key in ('pick_x', 'pick_y', 'pick_z'))
            if not np.isfinite(pick).all():
                raise ValueError('invalid_pick')
            self.get_logger().info(f'收到{camera}抓取请求 {request_id}: {pick}')
            result = self._handle_get_grasp(pick, camera, req)
        except Exception as exc:
            self.get_logger().error(f'get_grasp失败: {exc}')
            result = {'success': False, 'error': str(exc)}
        result.update(request_id=request_id, camera_id=camera)
        self._publish_result(result)

    def _handle_get_grasp(self, pick_base, camera="d455", request=None):
        """主推理流水线: 深度图 → GraspNet → 过滤 → 坐标变换 → 返回 base 系位姿"""
        from graspnetAPI import GraspGroup

        depth,color,cam_info,T_cam_to_base,image_stamp = self._snapshot(camera)
        pick_base = self._centroid_snapshot(request or {},camera,depth,cam_info,T_cam_to_base,image_stamp)
        T_base_to_cam = np.linalg.inv(T_cam_to_base)
        self.get_logger().info(f"T_cam_to_base 平移: {T_cam_to_base[:3, 3]}")

        # 3. 内参
        intr = {
            "fx": float(cam_info.k[0]), "fy": float(cam_info.k[4]),
            "cx": float(cam_info.k[2]), "cy": float(cam_info.k[5]),
            "width": int(cam_info.width), "height": int(cam_info.height),
        }

        # 4. 深度图 → 点云
        pcd, _, _ = depth_to_pointcloud(depth, color, intr, _DEPTH_OFFSET_M)
        self.get_logger().info(f"点云: {len(pcd.points)} 个点")
        if len(pcd.points) < 200:
            return {"success": False, "error": "too_few_points"}

        # 5. GraspNet 推理
        gg = run_inference(self._net, pcd, _NUM_POINT, self._device)
        if len(gg) == 0:
            return {"success": False, "error": "no_grasp_candidates"}

        # 6. 接近方向过滤 (相机系): R[:,0]=approach, z>0.3 = 朝前(朝下)
        keep_approach = []
        for i in range(len(gg)):
            R = gg[i].rotation_matrix
            if R[2, 0] > _APPROACH_Z_MIN:
                keep_approach.append(i)
        if not keep_approach:
            return {"success": False, "error": "no_forward_approach"}
        gg = GraspGroup(gg.grasp_group_array[keep_approach])
        self.get_logger().info(f"接近方向过滤: → {len(gg)} 个候选")

        # 7. 邻近过滤: pick_base → 相机系, 用 XY 距离 (图像平面) 过滤
        #    eye-to-hand: 相机在可靠深度范围 (0.5-0.8m), 深度数据可靠,
        #    用 XY 距离过滤确保候选在同一图像区域 (同一物体)
        pick_cam = (T_base_to_cam @ np.array(
            [pick_base[0], pick_base[1], pick_base[2], 1.0]))[:3]
        all_dists = [float(np.linalg.norm(gg[i].translation[:2] - pick_cam[:2]))
                     for i in range(len(gg))]
        keep_prox = []
        _prox_level = ""
        for _prox_thresh, _label in [(0.04, "严格4cm"), (0.06, "放宽6cm"),
                                      (_PROXIMITY_MAX_M, "兜底8cm")]:
            keep_prox = [i for i, d in enumerate(all_dists) if d < _prox_thresh]
            if keep_prox:
                _prox_level = _label
                break
        if not keep_prox:
            _min_xy = min(all_dists) if all_dists else -1.0
            _min_3d = min(float(np.linalg.norm(gg[i].translation - pick_cam))
                          for i in range(len(gg))) if len(gg) > 0 else -1.0
            self.get_logger().info(
                f"❌ 邻近过滤失败: {len(gg)} 候选无一通过 "
                f"(最近 XY={_min_xy*1000:.0f}mm 3D={_min_3d*1000:.0f}mm "
                f"pick_cam={pick_cam})")
            return {"success": False, "error": "no_grasp_near_pick"}
        gg = GraspGroup(gg.grasp_group_array[keep_prox])
        pick_dists = [all_dists[i] for i in keep_prox]
        self.get_logger().info(
            f"邻近过滤 ({_prox_level}): → {len(gg)} 个候选 (pick_cam={pick_cam})")

        # 7.5 垂直度硬过滤 (base 系): 优先接近正下方, 避免斜向下抓取导致抓空
        #     approach_z = (R_cam_to_base @ R_cam[:,0])[2], -1.0=正下方
        #     严格 < -0.90(≈26°) → 放宽 < -0.80(≈37°) → 全保留 (兜底)
        R_cam_to_base = T_cam_to_base[:3, :3]
        approach_scores = []
        for i in range(len(gg)):
            R = gg[i].rotation_matrix
            approach_scores.append(float((R_cam_to_base @ R[:, 0])[2]))

        strict = [i for i, z in enumerate(approach_scores) if z < -0.90]
        relaxed = [i for i, z in enumerate(approach_scores) if z < -0.80]
        if len(strict) >= 1:
            keep_vert = strict;  level = "严格(<26°)"
        elif len(relaxed) >= 1:
            keep_vert = relaxed; level = "放宽(<37°)"
        else:
            keep_vert = list(range(len(gg))); level = "无垂直候选(全保留)"
        if len(keep_vert) < len(gg):
            gg = GraspGroup(gg.grasp_group_array[keep_vert])
            approach_scores = [approach_scores[i] for i in keep_vert]
            pick_dists = [pick_dists[i] for i in keep_vert]
        self.get_logger().info(
            f"垂直度过滤 ({level}): → {len(gg)} 个候选 "
            f"(approach_z 范围 {min(approach_scores):.3f}~{max(approach_scores):.3f})")

        # 8. 短边+中心+垂直度+YOLO邻近 过滤
        try:
            object_cloud = segment_object_points(pcd)
            if len(object_cloud.points) >= 50:
                shape = analyze_object_shape(object_cloud)
                gg = filter_grasps_short_edge_center(
                    gg, shape, approach_scores=approach_scores, pick_dists=pick_dists)
            else:
                gg.sort_by_score()
        except Exception as e:
            self.get_logger().info(f"短边过滤失败 ({e}), 用 score 排序")
            gg.sort_by_score()

        if len(gg) == 0:
            return {"success": False, "error": "no_grasp_after_filter"}

        # 9. Top-1 → base 系
        best = gg[0]
        R_cam = best.rotation_matrix
        t_cam = best.translation
        width = float(min(best.width, _MAX_GRIPPER_WIDTH))

        T_cam_grasp = np.eye(4, dtype=np.float64)
        T_cam_grasp[:3, :3] = R_cam
        T_cam_grasp[:3, 3] = t_cam
        T_base_grasp = T_cam_to_base @ T_cam_grasp
        R_base = T_base_grasp[:3, :3]
        t_base = T_base_grasp[:3, 3]
        tcp_target = graspnet_tcp_target(T_cam_to_base,t_cam,R_cam,float(best.depth),self.geometry.S)
        network_tip = tcp_target[:3,3].copy()
        tcp_target[:3,3] = np.asarray(pick_base,dtype=float)  # 用户要求：TCP对准物体中心，网络仅提供朝向/宽度

        # 10. 列重排: GraspNet [approach,closing,ortho] → 机器人 [opening,ortho,approach]
        #     robot_R[:,0]=opening ← gn_R[:,1]=closing
        #     robot_R[:,1]=ortho   ← gn_R[:,2]=ortho
        #     robot_R[:,2]=approach← gn_R[:,0]=approach
        robot_R = np.column_stack([R_base[:, 1], R_base[:, 2], R_base[:, 0]])

        # 11. 安全检查: approach 在 base 系朝下 (阈值 -0.80 ≈ 37°, 配合垂直度过滤)
        approach_z = float(robot_R[2, 2])
        if approach_z > -0.80:
            return {"success": False,
                    "error": f"approach_not_downward: z={approach_z:.3f}"}

        # 12. robot_R → 四元数
        quat = _rotmat_to_quat(tcp_target[:3,:3])

        # 13. 位置偏移诊断: GraspNet 抓取点 vs YOLO 检测点 (base 系)
        xy_offset = float(np.hypot(t_base[0] - pick_base[0],
                                    t_base[1] - pick_base[1]))
        self.get_logger().info(
            f"✅ 返回位姿: t=({t_base[0]:.3f},{t_base[1]:.3f},{t_base[2]:.3f}) "
            f"approach_z={approach_z:.3f} score={best.score:.3f} width={width:.3f} "
            f"XY偏移={xy_offset*1000:.0f}mm"
        )
        return {
            "success": True,
            "translation": tcp_target[:3,3].tolist(),
            "raw_grasp_translation_base": t_base.tolist(),
            "network_fingertip_position_base": network_tip.tolist(),
            "grasp_point_type": "estimated_centroid",
            "depth": float(best.depth),
            "target_frame": "tcp_link",
            "reference_point": "midpoint_between_fingertips",
            "image_stamp": image_stamp,
            "camera_id": camera,
            "quaternion": quat,
            "width": width,
            "score": float(best.score),
            "n_candidates": int(len(gg)),
        }

    def _publish_result(self, result_dict):
        msg = String()
        msg.data = json.dumps(result_dict, ensure_ascii=False)
        self._result_pub.publish(msg)


def main():
    rclpy.init()
    node = GraspNetServiceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
