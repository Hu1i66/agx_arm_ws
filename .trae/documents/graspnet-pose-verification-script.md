# GraspNet 位姿生成独立验证脚本

## Context

GraspNet-1Billion 框架已完全就绪（`test_graspnet.py` 验证通过：torch 2.11+cu130、checkpoint-rs.tar epoch=18、推理 0.52s、13 候选、显存 0.66GB）。现有 `grasp_pose_node.py` 是 ROS2 service 节点，深度耦合抓取流程，无法单独测试位姿生成质量。

本任务开发一个**独立验证脚本**，不集成到抓取流程，能单独运行：加载深度图 → 运行 GraspNet 推理 → 3D 可视化夹取位姿 + 终端打印位姿详情。用于在接入抓取流程前，先验证 GraspNet 在真实 D455 数据上的位姿生成质量。

## 设计决策（已与用户确认）

- **数据来源**：ROS2 订阅 + 文件双模式（默认 ROS2 订阅一帧并自动保存，支持 `--load` 离线复现）
- **可视化**：3D Open3D 点云 + 夹爪几何体（复用 `demo.py` 的 `vis_grasps` 模式）
- **位姿详情**：终端打印 top-N 候选的 translation/rotation/score/width（满足"位置坐标和姿态信息"要求）

## 脚本位置

`/home/lxf/graspnet/verify_grasp_pose.py`

放在 graspnet 目录下，与 `test_graspnet.py` 并列，复用 graspnet-baseline 的 sys.path 注入模式，独立于 `agx_arm_ws` 抓取流程。

## 实现步骤

### 1. 脚本骨架与 sys.path 注入

复用 `test_graspnet.py` L20-25 的 sys.path 注入（注入 graspnet-baseline 的 models/utils/pointnet2/knn）：

```python
ROOT = "/home/lxf/graspnet/graspnet-baseline"
for sub in ["", "models", "utils", "pointnet2", "knn"]:
    sys.path.insert(0, os.path.join(ROOT, sub) if sub else ROOT)
```

argparse 参数：
- `--source ros2|file`（默认 ros2）
- `--load <dir>`（file 模式加载目录，含 depth.npy/color.npy/intrinsics.json）
- `--save <dir>`（ros2 模式保存目录，默认 `./saved_frames/frame_<timestamp>`）
- `--checkpoint_path`（默认 `/home/lxf/graspnet/checkpoints/checkpoint-rs.tar`）
- `--num_point 20000`、`--num_view 300`
- `--top_n 10`（可视化/打印的候选数）
- `--collision_thresh 0.0`（>0 则启用碰撞检测，默认关闭）
- `--depth_offset 0.04`（D455 近距离修正，与 grasp_pose_node.py 一致）

### 2. 数据获取

**ROS2 模式**（复用 `grasp_pose_node.py` 的订阅模式）：
- 创建临时 rclpy 节点，订阅三个话题（各收一帧后退出 spin）：
  - `/camera/camera/aligned_depth_to_color/image_raw`（Image, 16UC1, mm）
  - `/camera/camera/color/image_raw`（Image, 着色用，可选）
  - `/camera/camera/color/camera_info`（CameraInfo, 内参）
- 内参回退：若 camera_info 未收到，用 D455 硬编码值（`realsense_yolo_node.py:125-128`：fx=378.394659861614, fy=379.366916262423, cx=330.140969430714, cy=246.095530649072, 640×480）
- 收到后保存到 `--save` 目录：`depth.npy`(uint16 H×W)、`color.npy`(uint8 H×W×3)、`intrinsics.json`({fx,fy,cx,cy,width,height})

**文件模式**：
- `np.load(<dir>/depth.npy)`、`np.load(<dir>/color.npy)`、`json.load(<dir>/intrinsics.json)`

### 3. 深度图转点云

复用 `grasp_pose_node.py:471-511` 的 `_depth_to_pointcloud` 逻辑 + `demo.py` 的 `get_and_process_data`：
- `GraspNetCameraInfo(width, height, fx, fy, cx, cy, 1000.0)`（scale=1000, mm→m）
- depth_offset 修正：`depth = np.clip(depth.astype(int32) - offset_mm, 1, 65535).astype(uint16)`
- `cloud = create_point_cloud_from_depth_image(depth_m, cam, organized=True)`
- 构造 `o3d.geometry.PointCloud`（带 RGB 着色）
- 采样到 num_point（不足重复采样，复用 `test_graspnet.py` L160-168 模式）

### 4. GraspNet 推理

复用 `test_graspnet.py:127-145` + `demo.py:get_grasps`：
- 模型加载：`GraspNet(input_feature_dim=0, num_view, num_angle=12, num_depth=4, cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)`
- `torch.load(checkpoint)` → `load_state_dict` → `net.eval()`
- 推理：`end_points = {'point_clouds': tensor, 'cloud_colors': colors}` → `net(end_points)` → `pred_decode` → `GraspGroup(gg_array)`
- 可选碰撞检测：`ModelFreeCollisionDetector`（`--collision_thresh > 0` 时，复用 `demo.py:collision_detection`）

### 5. 3D 可视化（复用 demo.py:vis_grasps）

```python
gg.nms()
gg.sort_by_score()
gg = gg[:50]
grippers = gg.to_open3d_geometry_list()
o3d.visualization.draw_geometries([cloud, *grippers])
```

`to_open3d_geometry_list()` 是 graspnetAPI 内置方法，生成夹爪 TriangleMesh（含位置+朝向）。

### 6. 终端打印位姿详情

打印 top-N 候选的完整位姿信息（满足"位置坐标和姿态信息"）：
```
=== Top-10 Grasp Poses (相机系) ===
[0] score=0.974 width=0.100 depth=0.020
    translation: (-0.059, -0.018, 0.001)
    rotation_matrix:
      [ 0.999  0.000  0.000]
      [ 0.000  0.999  0.000]
      [ 0.000  0.000  0.999]
    approach_dir (R[:,2]): (0.000, 0.000, 1.000)
    ...
```

## 关键复用代码（不重写）

| 功能 | 复用来源 | 位置 |
|------|---------|------|
| sys.path 注入 | test_graspnet.py | L20-25 |
| 模型加载+推理 | test_graspnet.py | L127-145, demo.py:get_net/get_grasps |
| 深度图转点云 | grasp_pose_node.py | `_depth_to_pointcloud` L471-511 |
| CameraInfo 内参 | grasp_pose_node.py | L481-488 (camera_info.k[0/4/2/5]) |
| D455 硬编码内参 | realsense_yolo_node.py | L125-128 |
| 3D 可视化 | demo.py | `vis_grasps` L114-119 |
| 碰撞检测(可选) | demo.py | `collision_detection` L107-112 |
| 采样到 num_point | test_graspnet.py | L160-168 |

## 运行方式

```bash
# ROS2 在线模式 (需相机+realsense节点运行)
source /opt/ros/humble/setup.bash
source /home/lxf/orange_dataset/.venv/bin/activate
cd /home/lxf/graspnet
python3 verify_grasp_pose.py --source ros2

# 文件离线模式 (复现)
python3 verify_grasp_pose.py --source file --load ./saved_frames/frame_001

# 启用碰撞检测
python3 verify_grasp_pose.py --source ros2 --collision_thresh 0.01
```

## 验证方法

1. **ROS2 在线测试**：启动 D455 相机（`align_depth.enable:=true`），运行脚本，确认：
   - 订阅到深度图+RGB+camera_info 并保存到 `saved_frames/`
   - 终端打印 ≥1 个候选位姿（score/translation/rotation）
   - Open3D 窗口弹出：点云 + 夹爪几何体（夹爪位置在物体上，朝向合理）
2. **文件离线复现**：用保存的帧 `--load` 重新运行，确认结果一致
3. **位姿质量检查**：在传送带放一个香蕉，确认夹爪短轴对齐香蕉长边（approach 方向合理）
4. **无物体场景**：空桌面运行，确认候选数较少或 score 低（GraspNet 对无物体场景应少给候选）

## 不做的事

- 不集成到 `auto_sorting_action.py` 抓取流程
- 不调用 `grasp_pose_node.py` 的 ROS2 service（独立直接推理）
- 不做 2D 深度图叠加（用户选择 3D Open3D）
- 不做手眼标定/base 系变换（位姿在相机系，聚焦验证 GraspNet 本身质量）
