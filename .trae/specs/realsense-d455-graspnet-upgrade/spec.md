# RealSense D455 + GraspNet 视觉系统升级 Spec

## Why

当前系统采用单目位姿估计算法（`depth = fx * real_W / pixel_W`），深度精度受限，导致：
- 需要维护 `PER_OBJECT_Z_CORRECTION` 按物体类别的 z 修正表（柠檬 +2.5cm、青苹果 -1.5cm、草莓 +1cm）
- 需要两阶段精定位 `_two_stage_refine`（机械臂移到物体上方 25cm 重新检测）
- 每个新物体类别都需要手动调参

D455 深度相机已装在机械臂末端（eye-in-hand），但深度流未被消费。升级到 D455 深度 + GraspNet-1Billion 抓取位姿生成器后，可彻底消除单目深度不准带来的补丁，并获得 6DoF 抓取位姿（精确位置 + 最优 yaw 角度）。

## What Changes

### 新增
- **新 ROS2 节点** `grasp_pose_node.py`：订阅 D455 RGBD，加载 GraspNet-1Billion 模型，提供 `/generate_grasp_pose` service
- **新 ROS2 service** `GenerateGraspPose.srv`：输入 bbox+class_name+header_stamp，输出 top-N 抓取候选（PoseStamped + score + approach_angle_deg）
- **新 ROS2 message** `GraspCandidate.msg`：单个抓取候选数据结构
- **新命令** `sort_graspnet`：auto_sorting_action.py 中的分拣命令，调用 GraspNet service 生成抓取位姿
- **新参数** `use_graspnet`、`use_depth_camera`：运行时切换新旧路径

### 修改
- `realsense_yolo_node.py`：新增深度订阅，`depth_from_camera` 替换 `monocular_depth_from_bbox`（保留作为 fallback），`/detection_info` 新增 `header_stamp` 字段
- `auto_sorting_action.py`：新增 `sort_graspnet` 命令分支，异步调用 service（call_async + spin_once），多候选降级尝试，失败回退到 `sort_verify`
- `sorting_gui_client.py`：`_dispatch_sort_cmd` 和手动分拣回调发送 `sort_graspnet` 命令（携带 bbox + detection_stamp），GUI 新增 GraspNet 开关复选框
- `agx_arm_msgs` 包：CMakeLists.txt 和 package.xml 新增 service 和 message 定义

### 保留（不破坏）
- `sort_verify` 命令和 `_two_stage_refine` 完整路径（作为 fallback）
- `PER_OBJECT_Z_CORRECTION` 字典（GraspNet 路径默认不应用，参数可开启）
- `monocular_depth_from_bbox` 方法（参数开关控制）
- EMA 滤波 + ObjectTrack 跟踪
- 现有 `/detection_info` JSON 结构（向后兼容，仅新增字段）
- 自动分拣状态机（IDLE → CHECK_LEFT → CONVEYOR_WAIT → CONVEYOR_STOPPING → SORTING → STOPPING → IDLE）

## Impact

### Affected code
- **新增文件**：
  - `/home/lxf/orange_dataset/grasp_pose_node.py`
  - `/home/lxf/agx_arm_ws/src/agx_arm_ros/src/agx_arm_msgs/srv/GenerateGraspPose.srv`
  - `/home/lxf/agx_arm_ws/src/agx_arm_ros/src/agx_arm_msgs/msg/GraspCandidate.msg`
- **修改文件**：
  - `/home/lxf/orange_dataset/realsense_yolo_node.py`
  - `/home/lxf/agx_arm_ws/auto_sorting_action.py`
  - `/home/lxf/agx_arm_ws/sorting_gui_client.py`
  - `/home/lxf/agx_arm_ws/src/agx_arm_ros/src/agx_arm_msgs/CMakeLists.txt`
  - `/home/lxf/agx_arm_ws/src/agx_arm_ros/src/agx_arm_msgs/package.xml`
- **新增依赖**（graspnet-venv 隔离）：
  - `/home/lxf/graspnet/graspnet-baseline/`（GraspNet-1Billion 官方仓库）
  - `/home/lxf/graspnet/graspnet-venv/`（独立 Python 虚拟环境）
  - `/home/lxf/graspnet/checkpoints/checkpoint-tbl.tar`（预训练模型）

### Affected capabilities
- 物体检测与深度估计（realsense_yolo_node）
- 抓取位姿生成（新增能力）
- 机械臂分拣流程（auto_sorting_action）
- GUI 自动分拣（sorting_gui_client）

## ADDED Requirements

### Requirement: D455 深度流消费
系统 SHALL 订阅 `/camera/camera/aligned_depth_to_color/image_raw` 话题，从 D455 深度图提取物体深度，替代单目深度反推。

#### Scenario: D455 深度有效
- **WHEN** YOLO 检测到物体，且 D455 深度图在 bbox 中心 5x5 区域有 ≥3 个有效像素（值 > 0）
- **THEN** 用中位数作为物体深度（mm → m 转换），并做合理性检查（0.2m ≤ depth ≤ 3.0m）
- **AND** `/detection_info` 的 `method` 字段为 `"depth_camera"`

#### Scenario: D455 深度无效
- **WHEN** D455 深度图在 bbox 中心区域有效像素 < 3，或深度超出合理范围
- **THEN** 自动回退到 `monocular_depth_from_bbox` 单目深度估计
- **AND** `/detection_info` 的 `method` 字段为 `"monocular_rgb"`

#### Scenario: 参数强制切换
- **WHEN** `use_depth_camera` 参数为 `false`
- **THEN** 不订阅深度图，始终使用单目深度估计

### Requirement: GraspNet 抓取位姿生成服务
系统 SHALL 提供 ROS2 service `/generate_grasp_pose`，按需生成 6DoF 抓取位姿候选。

#### Scenario: 成功生成抓取位姿
- **WHEN** 收到 service 请求，包含有效的 bbox、class_name 和 header_stamp
- **AND** 深度缓冲中存在与 header_stamp 偏差 < 50ms 的深度图
- **AND** 机械臂位姿缓冲中存在与 header_stamp 偏差 < 150ms 的位姿
- **THEN** 按 bbox 裁剪点云（带 30% margin），预处理后输入 GraspNet
- **AND** 过滤抓取候选：z-down 接近方向（与 -z 夹角 < 30°）+ score > 0.5 + workspace 范围内
- **AND** 返回 top-N（默认 5）候选，按 score 降序排列
- **AND** `response.success = true`，`response.candidates` 非空

#### Scenario: 深度时间同步失败
- **WHEN** 深度缓冲中不存在与 header_stamp 偏差 < 50ms 的深度图
- **THEN** `response.success = false`，`response.error_msg = "depth_sync_timeout"`

#### Scenario: 点云为空
- **WHEN** bbox 裁剪后点云点数 < 100
- **THEN** `response.success = false`，`response.error_msg = "too few points after crop"`

#### Scenario: GraspNet 无有效抓取
- **WHEN** GraspNet 推理返回空列表，或所有候选被 z-down/score/workspace 过滤
- **THEN** `response.success = false`，`response.error_msg = "no grasps passed filter"`

### Requirement: sort_graspnet 命令
auto_sorting_action.py SHALL 支持新命令 `sort_graspnet`，通过 service 获取抓取位姿并执行抓取。

#### Scenario: GraspNet 成功
- **WHEN** 收到 `sort_graspnet` 命令，且 `use_graspnet=true`，且 service 可用
- **AND** service 返回 `success=true`
- **THEN** 用 `candidates[0].grasp_pose.position` 更新 POSE_PICK 的 (x, y, z)
- **AND** z 值直接用作夹爪夹持点位置（不加 GRIPPER_PICK_Z_OFFSET）
- **AND** 把所有候选的 orientation 插入到 IK 候选姿态列表前面
- **AND** 继续现有抓取流程（第零步~第九步）

#### Scenario: GraspNet 失败回退
- **WHEN** service 返回 `success=false`，或调用超时（10s），或 service 不可用
- **THEN** 日志记录失败原因
- **AND** 自动回退到 `sort_verify` 路径（调用 `_two_stage_refine`）

#### Scenario: 参数禁用 GraspNet
- **WHEN** `use_graspnet=false`
- **THEN** 即使收到 `sort_graspnet` 命令，也走 `sort_verify` 路径

### Requirement: 时间戳同步
系统 SHALL 在 `/detection_info` JSON 中包含 `header_stamp` 字段，用于 GraspNet service 的深度图和机械臂位姿时间同步。

#### Scenario: 检测信息包含时间戳
- **WHEN** realsense_yolo_node 发布 `/detection_info`
- **THEN** JSON 顶层包含 `header_stamp` 字段（浮点秒数，对应 YOLO 检测时刻的 RGB 图像时间戳）

#### Scenario: 命令携带时间戳
- **WHEN** sorting_gui_client 构造 `sort_graspnet` 命令
- **THEN** 命令 JSON 包含 `detection_stamp` 字段（从 `/detection_info.header_stamp` 缓存）
- **AND** 命令 JSON 包含 `bbox` 字段（从 `obj.bbox_pixel` 提取 [x1, y1, x2, y2]）

### Requirement: 异步 service 调用避免阻塞
auto_sorting_action.py SHALL 使用 `call_async + spin_once` 模式调用 GraspNet service，避免阻塞其他回调。

#### Scenario: 等待 service 响应期间继续处理回调
- **WHEN** 调用 `/generate_grasp_pose` service 并等待响应
- **THEN** 在等待循环中调用 `rclpy.spin_once(self, timeout_sec=0.1)`
- **AND** `/detection_info` 订阅回调、状态发布定时器、夹爪反馈等回调继续触发

#### Scenario: service 调用超时
- **WHEN** service 调用超过 `graspnet_timeout_s`（默认 10s）仍未完成
- **THEN** 返回 None，触发回退到 `sort_verify`

## MODIFIED Requirements

### Requirement: 机械臂分拣流程
分拣流程在 `sort_graspnet` 命令下，用 GraspNet 位姿替代两阶段精定位，其余流程（碰撞检测禁用、夹爪控制、料框放置）保持不变。

#### Scenario: GraspNet 路径的碰撞检测禁用
- **WHEN** 走 GraspNet 路径（非回退）
- **THEN** 在下降抓取前仍然调用 `_set_bin_collision_allowed(allowed=True)` 禁用传送带碰撞体
- **AND** 在回观察位后恢复碰撞检测（成功路径和错误路径都恢复）

#### Scenario: GraspNet 路径的夹爪控制
- **WHEN** 走 GraspNet 路径
- **THEN** 夹爪开合目标仍用 `compute_gripper_targets(object_diameter_m)` 计算
- **AND** 一次性闭合，无二次补压

### Requirement: GUI 自动分拣状态机
自动分拣状态机不变，仅 `_dispatch_sort_cmd` 发送的命令类型和字段变化。

#### Scenario: 自动分拣发送 sort_graspnet
- **WHEN** `use_graspnet=true` 且自动分拣状态机进入 SORTING 阶段
- **THEN** `_dispatch_sort_cmd` 构造 `sort_graspnet` 命令
- **AND** 命令包含 `bbox` 和 `detection_stamp` 字段

#### Scenario: GUI 开关切换
- **WHEN** 用户取消勾选 "使用 GraspNet 抓取位姿生成" 复选框
- **THEN** 后续分拣命令变为 `sort_verify`
- **AND** 不需要重启 GUI

## REMOVED Requirements

无移除需求。所有现有功能保留作为 fallback，新功能通过参数开关启用。

## 设计决策摘要

| 决策点 | 选择 | 理由 |
|--------|------|------|
| 升级范围 | 深度替换 + 6DoF 抓取 | 根本性解决单目深度不准问题 |
| 抓取生成器 | GraspNet-1Billion | 开源、基于点云、RTX 4060 可运行 |
| 夹爪配置 | max_gripper_width=0.10 | 用户实测夹爪开合 10cm |
| 触发模式 | 单次按需触发 | 节省 GPU，10 秒级分拣可接受 |
| 姿态约束 | 仅顶部抓取（z-down, <30°） | 保持与现有代码兼容，避免侧向碰撞 |
| 点云裁剪 | bbox 裁剪（带 30% margin） | 精确、快速、天然关联 YOLO 检测 |
| 集成方式 | 独立节点 + ROS2 service | 解耦清晰、按需触发、易调试 |
| 旧代码处理 | 并存 + 参数开关 | 安全回退，便于对比测试 |
| 失败回退 | 回退到单目方案（sort_verify） | 保证系统可用性 |
| 时间同步 | Request 加 Header + 服务端缓冲查找 | 解决 YOLO 检测时刻与服务调用时刻的延迟 |
| 多候选支持 | 返回 top-5 候选 | IK 失败时可尝试下一个，避免重新推理 |
| 阻塞规避 | 客户端 call_async + spin_once | 沿用现有 _two_stage_refine 风格，无需 MultiThreadedExecutor |
| Python 环境 | graspnet-venv 隔离 | 避免 open3d/numpy 版本冲突 |
