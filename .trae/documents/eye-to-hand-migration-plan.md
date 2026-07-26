# 眼在手外 (Eye-to-Hand) 迁移实施计划

## 摘要

将工作空间从眼在手内 (eye-in-hand) 配置全面迁移到眼在手外 (eye-to-hand) 配置。基于新的标定文件 `/home/lxf/handeye/result/2026-07-25_03-57-46_calibration.json`,修改所有涉及相机-机器人坐标变换的代码。

**核心变化**: 相机不再安装在机械臂末端,而是固定在传送带上方。因此:
- 相机→基座变换矩阵 `T_cam_to_base` 是**常量**(不再依赖 TCP 位姿)
- 不再需要订阅 `/feedback/tcp_pose` 用于坐标变换
- D455 工作在可靠深度范围 (0.5-0.8m),移除 Z 平面校正和深度偏移补偿
- 添加 `base_link → camera_color_optical_frame` 的静态 TF 发布器

## 当前状态分析

经探索确认,**所有文件仍处于眼在手内状态**(此前摘要中声称的修改实际未应用):

| 文件 | 状态 | 问题 |
|------|------|------|
| `graspnet_service_node.py` | 未迁移 | 使用旧标定路径、订阅 tcp_pose、应用 Z 校正 |
| `realsense_yolo_node.py` | 未迁移 | 硬编码眼在手内参数、订阅 robot_pose、应用 Z 校正 |
| `auto_sorting_action.py` | 未迁移 | 下游消费者,注释引用眼在手内 |
| `grasp_pose_node.py` | 未迁移 | 已废弃但仍被测试脚本引用 |
| `start_graspnet_pipeline.sh` | 未更新 | 引用旧标定文件路径 |
| TF 发布器 | 不存在 | 相机坐标系未接入 ROS TF 树 |

### 标定数据对比

| 参数 | 旧 (eye-in-hand) | 新 (eye-to-hand) |
|------|------------------|------------------|
| 文件 | `2026-06-06_04-12-35_calibration.json` | `2026-07-25_03-57-46_calibration.json` |
| 平移 (m) | [-0.054, -0.014, 0.050] | [0.673, 0.011, 0.597] |
| 四元数 | [-0.124, 0.127, -0.696, 0.695] | [-0.693, -0.712, 0.088, 0.067] |
| 变换含义 | camera → end_effector | camera → base_link |
| 平移模长 | 0.075m (相机离末端 7.5cm) | 0.900m (相机离基座 0.9m) |

## 实施步骤

### 步骤 1: 修改 `graspnet_service_node.py` (核心迁移)

**文件**: `/home/lxf/agx_arm_ws/graspnet_service_node.py`

**修改内容**:

1. **更新标定文件路径** (L42):
   ```python
   _HANDEYE_JSON_PATH = "/home/lxf/handeye/result/2026-07-25_03-57-46_calibration.json"
   ```

2. **移除 Z 平面校正常量** (L51-57): 删除 `_Z_PLANE_SLOPE` 和 `_Z_PLANE_X_REF` 及其注释

3. **设置深度偏移为 0** (L44):
   ```python
   _DEPTH_OFFSET_M = 0.0  # eye-to-hand: 相机在可靠深度范围,无需近距校正
   ```

4. **重命名变量并移除 TCP 订阅** (`__init__` ~L376, L393-394):
   - `self.T_cam_to_ee` → `self.T_cam_to_base`
   - 删除 `self.create_subscription(PoseStamped, "/feedback/tcp_pose", self._tcp_pose_cb, 10)`
   - 删除 `_tcp_pose_cb` 方法
   - 删除 `self._latest_tcp_pose` 缓存

5. **简化变换链** (~L461-465):
   ```python
   # eye-to-hand: T_cam_to_base 是常量,不再依赖 TCP 位姿
   T_cam_to_base = self.T_cam_to_base
   T_base_to_cam = np.linalg.inv(T_cam_to_base)
   ```
   - 移除 `tcp_pose is None` 检查和 `missing_depth_or_tcp_pose` 错误路径
   - 移除 `_posestamped_to_matrix(tcp_pose)` 调用

6. **移除 Z 平面校正应用** (~L601-607): 删除 `t_base[2] = z_before + _Z_PLANE_SLOPE * (...)` 代码块

7. **更新文件头部文档字符串** (L1-14): 将 "tcp_pose" 描述改为 "固定相机位姿"

8. **更新 `_load_handeye_matrix` 注释** (L77): 改为 "camera_color_optical_frame -> base_link"

### 步骤 2: 修改 `realsense_yolo_node.py` (核心迁移)

**文件**: `/home/lxf/orange_dataset/realsense_yolo_node.py`

**修改内容**:

1. **替换硬编码标定数据** (L153-210):
   ```python
   # ================== 手眼标定数据（相机到机械臂基座,眼在手外）==================
   # 来源: /home/lxf/handeye/result/2026-07-25_03-57-46_calibration.json
   # eye-to-hand: 相机固定在传送带上方, T_cam_to_base 是常量
   self.camera_to_base_translation = np.array([
       0.6732749433043899,   # X
       0.010662122461489898, # Y
       0.597133855461562     # Z
   ])
   self.camera_to_base_quat = np.array([
       -0.6934151560947895,   # qx
       -0.7119098057466932,   # qy
       0.08845804637870601,   # qz
       0.06734258541670725    # qw
   ])
   self.camera_to_base_rpy = np.array([
       -2.9203729063228363,   # roll
       0.026795812790156897,  # pitch
       1.6000918271309976     # yaw
   ])
   ```
   - 删除 `camera_to_end_effector_translation_override` 参数 (不再需要运行时微调)

2. **重命名变量和矩阵** (L193-210):
   - `rotation_matrix_cam_to_ee` → `rotation_matrix_cam_to_base`
   - `rotation_matrix_cam_to_ee_rpy` → `rotation_matrix_cam_to_base_rpy`
   - `transform_cam_to_ee` → `transform_cam_to_base`
   - `transform_ee_to_cam` → `transform_base_to_cam` (注意: 这是逆变换)
   - 更新所有日志输出中的变量名

3. **移除 robot_pose 订阅** (L228-231, L296-301):
   - 删除 `robot_pose_topic` 参数声明
   - 删除 `self.sub_robot_pose` 订阅
   - 删除 `robot_pose_cb` 回调方法
   - 删除 `self.latest_robot_pose` 缓存

4. **移除 Z 平面校正参数** (L267-273):
   - 删除 `z_plane_slope` 和 `z_plane_x_ref` 参数声明
   - 删除相关日志输出

5. **简化 `transform_camera_to_base` 方法** (L667-681):
   ```python
   def transform_camera_to_base(self, camera_coords):
       """相机坐标 → 基座坐标 (eye-to-hand: 直接用常量变换矩阵)"""
       if camera_coords is None:
           return None, None
       point_homogeneous = np.append(camera_coords, 1)
       base_coords = self.transform_cam_to_base @ point_homogeneous
       return base_coords[:3], None  # ee_coords 不再适用,返回 None
   ```
   - 移除对 `transform_camera_to_end_effector` 和 `transform_end_effector_to_base` 的调用
   - 移除 Z 平面校正

6. **简化 `_compute_grasp_orientation` 中的点云变换** (~L955, L964):
   ```python
   # eye-to-hand: 直接变换到基座系
   cam_pts = np.stack([cam_x, cam_y, cam_z, np.ones_like(cam_x)], axis=0)  # 4xN
   base_pts = (self.transform_cam_to_base @ cam_pts)[:3, :]  # 3xN
   ```
   - 移除 `ee_pts = self.transform_cam_to_ee @ cam_pts`
   - 移除 `base_pts[2, :] += self.z_plane_slope * (...)`

7. **更新所有引用旧变量名的地方**:
   - `transform_camera_to_end_effector` 方法可保留(内部计算用)或删除
   - `transform_end_effector_to_base` 方法可删除(不再需要)
   - 更新 `ObjectTrack` 类中 `ee_coords` 相关字段 (保留但始终为 None,或删除)

8. **设置 `depth_offset_m` 默认值为 0.0** (L264)

### 步骤 3: 修改 `auto_sorting_action.py` (下游更新)

**文件**: `/home/lxf/agx_arm_ws/auto_sorting_action.py`

**修改内容**:

1. **更新注释** (~L1397-1400): 将 "D455 深度点云估算" 等注释更新为反映眼在手外配置

2. **验证 `two_stage` 逻辑** (~L1363):
   - 当前 `two_stage = (cmd_type == "sort_verify")` 已对 sort/sort_graspnet 为 False
   - 保留现有逻辑 (眼在手外下两阶段精定位仍可用于 `sort_verify` 命令)

3. **移除 Z 平面校正引用** (注释中): 搜索并更新任何提及 Z 平面校正的注释

4. **更新 `call_graspnet_service` 文档字符串** (~L562): 移除 "tcp_pose" 引用

### 步骤 4: 修改 `grasp_pose_node.py` (废弃节点同步)

**文件**: `/home/lxf/orange_dataset/grasp_pose_node.py`

**修改内容** (与步骤 1 类似):

1. **更新标定路径** (L91):
   ```python
   _HANDEYE_JSON_PATH = "/home/lxf/handeye/result/2026-07-25_03-57-46_calibration.json"
   ```

2. **重命名 `T_cam_to_ee` → `T_cam_to_base`** (L222-231 及所有引用)

3. **移除 `/feedback/tcp_pose` 订阅** (L195-207 中的 robot_pose topic 配置)

4. **简化变换链** (`_transform_grasp_to_base` L655-716):
   ```python
   # eye-to-hand: T_base = T_cam_to_base @ T_cam (常量变换)
   T_base = self.T_cam_to_base @ T_cam
   ```
   - 移除 `T_robot_pose` 相关代码
   - 移除 `_robot_pose_buffer` 和 `_robot_pose_cb`

5. **更新 `_compute_short_axis_base`** (L721-766): 使用 `T_cam_to_base` 替代 `T_robot_pose @ T_cam_to_ee`

6. **移除时间同步缓冲区** (depth_sync_tolerance, robot_pose_sync_tolerance): 眼在手外不需要 TCP 位姿同步

### 步骤 5: 创建 TF 静态变换发布器

**新文件**: `/home/lxf/agx_arm_ws/launch/eye_to_hand_camera_tf.launch.py`

```python
"""眼在手外相机 TF 发布器
发布 base_link → camera_color_optical_frame 静态变换,
使 RViz/MoveIt 等工具能在机器人坐标系中可视化相机数据。

标定来源: /home/lxf/handeye/result/2026-07-25_03-57-46_calibration.json

启动:
  ros2 launch eye_to_hand_camera_tf.launch.py

⚠️ 使用此发布器时, realsense2_camera 需设置 publish_tf:=false 避免冲突:
  ros2 launch realsense2_camera rs_launch.py ... publish_tf:=false
"""
import launch
import launch_ros.actions


def generate_launch_description():
    # 标定数据 (camera_color_optical_frame → base_link)
    # 来源: /home/lxf/handeye/result/2026-07-25_03-57-46_calibration.json
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_color_optical_frame_broadcaster',
            arguments=[
                '--x', '0.6732749433043899',
                '--y', '0.010662122461489898',
                '--z', '0.597133855461562',
                '--qx', '-0.6934151560947895',
                '--qy', '-0.7119098057466932',
                '--qz', '0.08845804637870601',
                '--qw', '0.06734258541670725',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_color_optical_frame',
            ],
        ),
    ])
```

**变换方向分析** (已验证):
- 标定文件的 `position`/`orientation` 表示 camera_color_optical_frame 在 base_link 系中的位姿
- `_load_handeye_matrix` 返回 `T_cam_to_base` (将点从相机系变换到基座系: `p_base = T @ p_cam`)
- ROS TF 中 `parent → child` 的变换矩阵 `T_tf` 满足 `p_parent = T_tf @ p_child`
- 因此发布 `base_link → camera_color_optical_frame` 时,`T_tf = T_cam_to_base`,**直接使用标定值即可,无需取逆**
- 翻译 `[0.673, 0.011, 0.597]` = 相机原点在基座系中的位置 (相机离基座 ~0.9m,在前上方)
- 旋转 `[-0.693, -0.712, 0.088, 0.067]` = 相机坐标系到基座坐标系的旋转

**操作**:
- 创建 `/home/lxf/agx_arm_ws/launch/` 目录 (如不存在)

### 步骤 6: 更新 `start_graspnet_pipeline.sh`

**文件**: `/home/lxf/agx_arm_ws/start_graspnet_pipeline.sh`

**修改内容**:

1. **更新标定文件路径** (L73):
   ```bash
   HANDEYE="/home/lxf/handeye/result/2026-07-25_03-57-46_calibration.json"
   ```

2. **更新 T2 相机启动命令** (~L100-106): 添加 `publish_tf:=false` 参数
   ```bash
   ros2 launch realsense2_camera rs_launch.py \
     rgb_camera.color_profile:=640,480,30 \
     depth_module.profile:=640,480,30 \
     align_depth.enable:=true \
     publish_tf:=false
   ```

3. **添加 TF 发布器启动命令** (在 T2 之后或作为 T2 的一部分):
   ```bash
   # T2b: 相机 TF 发布器 (眼在手外)
   ros2 launch /home/lxf/agx_arm_ws/launch/eye_to_hand_camera_tf.launch.py
   ```

4. **更新文件头部注释**: 反映眼在手外配置

## 假设与决策

### 假设
1. 新标定文件 `2026-07-25_03-57-46_calibration.json` 的 `position`/`orientation` 表示 camera_color_optical_frame 在 base_link 系中的位姿 (与旧文件语义一致,只是参考系从末端变为基座)
2. 相机已物理安装在传送带上方固定位置,深度数据在 0.5-0.8m 可靠范围内
3. `/feedback/tcp_pose` 仍由 `agx_arm_ctrl_single_node.py` 发布 (用于机器人控制),只是不再用于相机坐标变换
4. `sorting_gui_client.py` 无需修改 (纯下游消费者,只显示 `base_position_m`)

### 决策
1. **保留 Python 节点内的矩阵变换** (主路径),TF 发布器作为补充 (用于 RViz/MoveIt 可视化)
2. **完全移除 Z 平面校正** (眼在手外下深度可靠,无需补偿)
3. **设置 `_DEPTH_OFFSET_M = 0.0`** (同上)
4. **同步更新废弃的 `grasp_pose_node.py`** (保持代码库一致性)
5. **相机启动设置 `publish_tf:=false`** (避免与我们的静态 TF 发布器冲突)
6. **`two_stage` 逻辑保持不变** (sort_verify 命令仍可使用两阶段精定位)

## 验证步骤

### 1. 启动验证
```bash
# T1: 启动机械臂 + MoveIt
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper

# T2: 启动相机 (publish_tf:=false)
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=640,480,30 depth_module.profile:=640,480,30 align_depth.enable:=true publish_tf:=false

# T2b: 启动相机 TF 发布器
ros2 launch /home/lxf/agx_arm_ws/launch/eye_to_hand_camera_tf.launch.py

# T3: 启动 YOLO 检测节点
python3 /home/lxf/orange_dataset/realsense_yolo_node.py --ros-args -p show_gui_window:=false -p use_depth_camera:=true

# T4: 启动 GraspNet 服务
python3 /home/lxf/agx_arm_ws/graspnet_service_node.py

# T5: 启动动作节点
python3 /home/lxf/agx_arm_ws/auto_sorting_action.py

# T6: 启动 GUI
python3 /home/lxf/agx_arm_ws/sorting_gui_client.py
```

### 2. TF 树验证
```bash
# 验证 camera_color_optical_frame 已接入 TF 树
ros2 run tf2_tools view_frames
# 检查 base_link → camera_color_optical_frame 变换存在

# 验证变换值正确
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
# 应显示 translation: [0.673, 0.011, 0.597]
```

### 3. YOLO 检测验证
- 在传送带不同 X 位置放置物体 (x=0.2, 0.3, 0.4, 0.5)
- 观察 GUI 显示的 XYZ 坐标
- **验证 Z 值不再随 X 系统性偏移** (眼在手外下深度可靠)
- 验证 `method` 字段显示 `d455_stereo` (非 `monocular_fallback`)

### 4. GraspNet 抓取验证
- 在 x=0.2m 位置放置物体 (此前眼在手内下失败的位置)
- 点击 GUI 的 "GraspNet抓取" 按钮
- **验证 GraspNet 返回有效位姿** (此前因深度失真被误拒)
- 验证机械臂正确下降并抓取物体

### 5. 日志检查
```bash
# 检查 GraspNet 服务日志
cat ~/.ros/log/latest/graspnet_service_node_*.log | grep -E "T_cam_to_base|推理|过滤"

# 检查 YOLO 节点日志
cat ~/.ros/log/latest/object_detector_*.log | grep -E "手眼|变换|深度"

# 验证无 "tcp_pose" 或 "Z 平面校正" 相关日志
```

### 6. 回归测试
- 测试普通抓取 (sort 命令)
- 测试两阶段精定位 (sort_verify 命令)
- 测试自动分拣 (Start Automatic Sorting)
- 验证苹果/草莓/橙子分到料框1,其他分到料框2

## 风险与回退

### 风险
1. **标定方向错误**: 如果标定文件给出的实际是 `T_base_to_cam` 而非 `T_cam_to_base`,所有坐标变换会反向
   - 缓解: 实施时先用 `tf2_echo` 验证 TF 方向,再测试抓取
2. **TF 冲突**: 如果相机驱动仍发布 TF,会与静态发布器冲突
   - 缓解: 确保相机启动时设置 `publish_tf:=false`
3. **深度偏移**: 即使眼在手外,D455 可能仍有小范围深度偏差
   - 缓解: 保留 `depth_offset_m` 参数 (设为 0.0),如需要可在线调整

### 回退方案
如眼在手外配置出现问题,可通过以下方式回退:
1. 恢复 `graspnet_service_node.py` 的 `_HANDEYE_JSON_PATH` 为旧路径
2. 恢复 `realsense_yolo_node.py` 的硬编码值为旧值
3. 重新启用 Z 平面校正参数
4. 使用 git 回滚到迁移前的 commit
