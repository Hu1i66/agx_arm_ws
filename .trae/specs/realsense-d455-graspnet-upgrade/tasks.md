# Tasks

## 阶段 0: 环境准备与依赖安装

- [ ] Task 0.1: 创建 graspnet-venv 隔离环境
  - [ ] 在 `/home/lxf/graspnet/` 下创建 Python venv
  - [ ] 安装 PyTorch（与系统 CUDA 13.0 匹配）、open3d、trimesh、numba、scipy、scikit-learn
  - [ ] 验证 `python -c "import torch; print(torch.cuda.is_available())"` 输出 True

- [ ] Task 0.2: 克隆并安装 GraspNet-1Billion
  - [ ] `git clone https://github.com/graspnet/graspnet-baseline.git` 到 `/home/lxf/graspnet/`
  - [ ] `pip install -e graspnet-baseline` 安装 graspnetAPI
  - [ ] 下载 checkpoint-tbl.tar 到 `/home/lxf/graspnet/checkpoints/` 并解压

- [ ] Task 0.3: 编译 pointnet2_ops CUDA 算子
  - [ ] 进入 `graspnet-baseline/pointnet2` 目录
  - [ ] `python setup.py install` 编译
  - [ ] 若失败：检查 CUDA 版本、gcc 版本；尝试降级 PyTorch；或切换不依赖 pointnet2 的变体
  - [ ] 验证 `python -c "import pointnet2_ops"` 无报错

- [ ] Task 0.4: 编写并运行 GraspNet 验证脚本
  - [ ] 创建 `/home/lxf/graspnet/test_graspnet.py`（用随机点云测试推理）
  - [ ] 验证输出至少 1 个候选，推理时间 < 10s
  - [ ] 验证显存占用 < 4GB

## 阶段 1: ROS2 消息接口定义

- [ ] Task 1.1: 新增 GraspCandidate.msg
  - [ ] 在 `agx_arm_msgs/msg/GraspCandidate.msg` 创建消息定义
  - [ ] 字段：`geometry_msgs/PoseStamped grasp_pose`、`float32 score`、`float32 approach_angle_deg`

- [ ] Task 1.2: 新增 GenerateGraspPose.srv
  - [ ] 在 `agx_arm_msgs/srv/GenerateGraspPose.srv` 创建服务定义
  - [ ] Request: `std_msgs/Header header`、`sensor_msgs/RegionOfInterest bbox`、`string class_name`、`uint8 max_candidates`
  - [ ] Response: `bool success`、`string error_msg`、`agx_arm_msgs/GraspCandidate[] candidates`、`float32 best_score`、`builtin_interfaces/Time depth_stamp`

- [ ] Task 1.3: 更新 agx_arm_msgs 包配置
  - [ ] CMakeLists.txt: 添加 `find_package(sensor_msgs REQUIRED)`、`find_package(builtin_interfaces REQUIRED)`
  - [ ] CMakeLists.txt: 在 `rosidl_generate_interfaces` 中添加新 msg 和 srv
  - [ ] package.xml: 添加 sensor_msgs、builtin_interfaces 依赖
  - [ ] `colcon build --packages-select agx_arm_msgs` 编译
  - [ ] 验证 `ros2 interface show agx_arm_msgs/srv/GenerateGraspPose` 正确显示

## 阶段 2: realsense_yolo_node.py 改动

- [ ] Task 2.1: 新增深度订阅与缓冲
  - [ ] 添加参数 `use_depth_camera`（默认 True）
  - [ ] 新增订阅 `/camera/camera/aligned_depth_to_color/image_raw`
  - [ ] 实现 `depth_cb` 回调，缓存 `self.latest_depth`（uint16 ndarray）和 `self.latest_depth_stamp`
  - [ ] 错误处理：解析失败时清空 `latest_depth`

- [ ] Task 2.2: 实现 depth_from_camera 方法
  - [ ] 在 `realsense_yolo_node.py` 中新增 `depth_from_camera(box)` 方法
  - [ ] 取 bbox 中心 5x5 像素区域的中位数（排除 0 值无效像素）
  - [ ] 至少 3 个有效像素才返回深度
  - [ ] 深度范围检查（0.2m ≤ depth ≤ 3.0m）
  - [ ] 返回 depth_m（float）或 None

- [ ] Task 2.3: 替换深度计算调用点
  - [ ] 在 L789 附近，根据 `use_depth_camera` 开关选择 `depth_from_camera` 或 `monocular_depth_from_bbox`
  - [ ] D455 深度无效时自动回退到单目
  - [ ] 记录回退次数（`self._depth_fallback_count`）供日志统计

- [ ] Task 2.4: /detection_info 新增 header_stamp 字段
  - [ ] 在 `info_dict` 中添加 `"header_stamp": self.latest_rgb_stamp`
  - [ ] 更新 `method` 字段：`"depth_camera"` 或 `"monocular_rgb"`（根据实际使用的深度来源）

- [ ] Task 2.5: 验证 realsense_yolo_node 改动
  - [ ] 启动 D455（align_depth=true）+ realsense_yolo_node
  - [ ] `ros2 topic echo /detection_info` 确认包含 `header_stamp` 字段
  - [ ] 确认 `method` 字段为 `"depth_camera"`
  - [ ] 物体放在传送带上，验证 depth_m 数值合理（0.4-1.0m）
  - [ ] `ros2 param set /realsense_yolo_node use_depth_camera false` 切换后验证 `method` 变为 `"monocular_rgb"`

## 阶段 3: grasp_pose_node.py 新节点实现

- [ ] Task 3.1: 节点骨架与参数
  - [ ] 创建 `/home/lxf/orange_dataset/grasp_pose_node.py`
  - [ ] sys.path 注入 graspnet-venv 的 site-packages
  - [ ] 声明所有参数（graspnet_checkpoint、max_gripper_width、score_threshold、approach_angle_max_deg、workspace_min/max、voxel_size、bbox_margin_ratio、depth_sync_tolerance 等）
  - [ ] 创建订阅者（RGB、depth、camera_info、robot_pose）
  - [ ] 创建 service `/generate_grasp_pose`
  - [ ] 创建 publisher `/grasp_pose_marker`

- [ ] Task 3.2: 时间戳缓冲实现
  - [ ] 用 dict + stamp_ns 键存储深度图和机械臂位姿
  - [ ] 实现 `_lookup_depth_by_stamp(stamp)` 二分查找方法（容差 50ms）
  - [ ] 实现 `_lookup_robot_pose_by_stamp(stamp)` 二分查找方法（容差 150ms）
  - [ ] 定时清理超过 5s 的过期条目（每 5s 清理一次）

- [ ] Task 3.3: 点云生成与裁剪
  - [ ] 实现 `_depth_to_pointcloud(depth_img, camera_info)`：用 Open3D RGBDImage.create_from_color_and_depth + PinholeCameraIntrinsic
  - [ ] 实现 `_crop_by_bbox(pcd, bbox, margin)`：按 bbox 像素坐标裁剪点云（带 30% margin 膨胀）
  - [ ] 实现 `_preprocess(pcd)`：voxel_down_sample(0.005) + statistical_outlier_removal(nb=20, std=2.0)

- [ ] Task 3.4: GraspNet 模型加载与推理
  - [ ] 在 `__init__` 中加载 GraspNet 模型（checkpoint-tbl，max_width=0.10）
  - [ ] 实现 `_run_graspnet(pcd)`：调用 graspnet.inference，返回 grasp 列表（rotation, translation, score, width, depth）
  - [ ] 错误处理：推理异常时返回空列表

- [ ] Task 3.5: 抓取过滤与排序
  - [ ] 实现 `_filter_grasps(grasps)`：
    - 计算每个 grasp 的 approach_dir = rotation_matrix[:, 2]
    - 计算与 [0, 0, -1] 的夹角
    - 过滤：夹角 < approach_angle_max_deg（30°）+ score > score_threshold（0.5）
  - [ ] 按 score 降序排序
  - [ ] 取 top-N（max_candidates 参数控制）

- [ ] Task 3.6: 坐标变换
  - [ ] 加载手眼标定（从 `/home/lxf/handeye/result/2026-06-06_04-12-35_calibration.json`）
  - [ ] 构建 `transform_cam_to_ee` 4x4 矩阵
  - [ ] 实现 `_transform_grasp_to_base(grasp, robot_pose)`：
    - T_cam = build_transform(grasp.rotation, grasp.translation)
    - T_ee = transform_cam_to_ee @ T_cam
    - T_base = robot_pose_matrix @ T_ee
    - 提取 base 系下的 (x, y, z, quaternion)
  - [ ] workspace 过滤：变换后的 (x, y, z) 必须在 workspace_min/max 范围内

- [ ] Task 3.7: service 回调实现
  - [ ] 实现 `handle_grasp_request(request, response)`：
    1. 时间同步查找深度图
    2. 生成点云
    3. bbox 裁剪
    4. 预处理
    5. GraspNet 推理
    6. 过滤排序
    7. 坐标变换 + workspace 过滤
    8. 构造 GraspCandidate[] Response
    9. 发布 RViz MarkerArray
  - [ ] 所有失败情况返回明确的 error_msg

- [ ] Task 3.8: RViz 可视化
  - [ ] 实现 `_publish_marker(candidates)`：发布 MarkerArray
  - [ ] 每个候选一个 ARROW，方向 = 接近方向，长度 = 10cm
  - [ ] 颜色按分数：绿(>0.7) / 黄(0.5-0.7) / 红(<0.5)
  - [ ] lifetime = 5s

- [ ] Task 3.9: 验证 grasp_pose_node
  - [ ] 启动 D455 + realsense_yolo_node + grasp_pose_node
  - [ ] 物体放在传送带上
  - [ ] `ros2 service call /generate_grasp_pose ...` 手动测试
  - [ ] RViz 中订阅 `/grasp_pose_marker` 查看可视化
  - [ ] 验证返回的 candidates 中至少 1 个 success=true
  - [ ] 验证 grasp_pose.header.frame_id = "base_link"

## 阶段 4: auto_sorting_action.py 改动

- [ ] Task 4.1: 新增 service client 与参数
  - [ ] import GenerateGraspPose、GraspCandidate、RegionOfInterest、Header
  - [ ] 声明参数：`use_graspnet`（默认 True）、`graspnet_timeout_s`（默认 10.0）、`apply_z_correction_in_graspnet`（默认 False）
  - [ ] 创建 service client `/generate_grasp_pose`
  - [ ] `wait_for_service(timeout_sec=5.0)`，设置 `grasp_service_available` 标志

- [ ] Task 4.2: 实现 _call_grasp_service_async 方法
  - [ ] 构造 GenerateGraspPose.Request（header、bbox、class_name、max_candidates）
  - [ ] `future = self.grasp_client.call_async(request)`
  - [ ] `while not future.done() and time.time() - start < timeout: rclpy.spin_once(self, timeout_sec=0.1)`
  - [ ] 超时返回 None，成功返回 response

- [ ] Task 4.3: 实现 _build_pick_orientations_with_graspnet 方法
  - [ ] 调用现有 `_build_pick_orientations_multi(target)` 生成基础候选
  - [ ] 把 GraspNet 候选的 orientation 插入到列表前面（按 score 降序）
  - [ ] 去重：与 GraspNet 候选夹角 < 5° 的 yaw 采样候选被移除

- [ ] Task 4.4: 新增 sort_graspnet 命令分支
  - [ ] 在 L1226 的命令判断中添加 `"sort_graspnet"`
  - [ ] 提取 bbox、detection_stamp、class_name 等参数
  - [ ] 调用 `_call_grasp_service_async`
  - [ ] 成功：用 candidates[0].grasp_pose.position 更新 POSE_PICK（z 直接用，不加 offset）
  - [ ] 失败：日志记录，回退到 sort_verify 路径

- [ ] Task 4.5: 集成多候选 IK 尝试
  - [ ] 在下降抓取步骤（第二步）中：
    - GraspNet 成功时：把所有候选 orientation 插入 pick_orientations 前面
    - GraspNet 失败/回退时：仅用现有 `_build_pick_orientations_multi`
  - [ ] `move_arm_via_ik` 内部循环尝试所有 orientation（现有逻辑，无需修改）

- [ ] Task 4.6: 验证 auto_sorting_action 改动
  - [ ] `python3 -m py_compile auto_sorting_action.py` 语法检查
  - [ ] 启动 auto_sorting_action（grasp_pose_node 未启动）
  - [ ] 验证日志显示 "服务不可用, 回退到 sort_verify"
  - [ ] 启动 grasp_pose_node 后重启 auto_sorting_action
  - [ ] 验证日志显示 "服务已连接"
  - [ ] 发送 sort_graspnet 命令，验证 GraspNet 被调用

## 阶段 5: sorting_gui_client.py 改动

- [ ] Task 5.1: 新增参数与状态
  - [ ] `self.use_graspnet = True`
  - [ ] `self.latest_detection_stamp = 0.0`
  - [ ] 在 `/detection_info` 回调中缓存 `header_stamp`

- [ ] Task 5.2: 修改 _dispatch_sort_cmd
  - [ ] 根据 `use_graspnet` 选择命令类型（sort_graspnet 或 sort_verify）
  - [ ] sort_graspnet 命令包含 `bbox` 和 `detection_stamp` 字段
  - [ ] 日志区分 GraspNet 路径和两阶段路径

- [ ] Task 5.3: 修改手动分拣回调
  - [ ] L607 附近：根据 `use_graspnet` 选择命令类型
  - [ ] sort_graspnet 命令包含 `bbox` 和 `detection_stamp`

- [ ] Task 5.4: 新增 GUI 开关复选框
  - [ ] 在控制面板添加 `tk.Checkbutton` "使用 GraspNet 抓取位姿生成"
  - [ ] 绑定 `self.graspnet_var`（BooleanVar，默认 True）
  - [ ] 实现 `_on_graspnet_toggle` 回调

- [ ] Task 5.5: 验证 sorting_gui_client 改动
  - [ ] `python3 -m py_compile sorting_gui_client.py` 语法检查
  - [ ] 启动 GUI，确认复选框显示
  - [ ] 取消勾选后手动分拣，验证发送 sort_verify 命令
  - [ ] 勾选后手动分拣，验证发送 sort_graspnet 命令

## 阶段 6: 集成测试

- [ ] Task 6.1: 单元测试 grasp_pose_node 内部方法
  - [ ] 创建 `/home/lxf/orange_dataset/test_grasp_pose_node.py`
  - [ ] 测试 `_depth_to_pointcloud`、`_crop_by_bbox`、`_filter_grasps`、`_transform_grasp_to_base`、`_lookup_depth_by_stamp`
  - [ ] 运行 `python -m pytest test_grasp_pose_node.py`

- [ ] Task 6.2: service 接口测试
  - [ ] 启动 grasp_pose_node
  - [ ] `ros2 service call /generate_grasp_pose ...` 测试空请求、无效 bbox、正常请求
  - [ ] 验证所有 error_msg 场景

- [ ] Task 6.3: 回退逻辑测试
  - [ ] 不启动 grasp_pose_node，启动 auto_sorting_action
  - [ ] 发送 sort_graspnet 命令，验证回退到 sort_verify
  - [ ] 启动 grasp_pose_node 但不加载模型（模拟服务卡死）
  - [ ] 验证 10s 超时后回退

## 阶段 7: 硬件在环测试

- [ ] Task 7.1: 单物体抓取测试（苹果）
  - [ ] 物体放在传送带中心
  - [ ] 机械臂在观察位
  - [ ] 手动触发 sort_graspnet
  - [ ] RViz 观察 /grasp_pose_marker
  - [ ] 验证候选数 ≥ 3、best_score ≥ 0.5、approach_angle < 30°
  - [ ] 验证机械臂成功抓取并放置到正确料框
  - [ ] 重复 10 次，记录成功率

- [ ] Task 7.2: 多物体类别测试
  - [ ] 依次测试 7 类水果：apple、green apple、strawberry、orange、lemon、honey peach、pear
  - [ ] 每类测试 5 次
  - [ ] 记录 GraspNet score、抓取成功率、是否碰台面
  - [ ] 与 sort_verify 路径成功率对比

- [ ] Task 7.3: 边界情况测试
  - [ ] 物体部分超出视野：验证点云裁剪后仍能生成有效抓取
  - [ ] 强光干扰：验证 D455 深度失效时自动回退到单目
  - [ ] 物体反光（橘子皮）：验证深度图空洞的容错
  - [ ] GraspNet 无有效抓取：验证回退到 sort_verify

- [ ] Task 7.4: 端到端自动分拣测试
  - [ ] 启动全部 6 个节点
  - [ ] 传送带上随机放置 10 个不同物体
  - [ ] 点击 "开始自动分拣"
  - [ ] 验证状态机正确流转
  - [ ] 验证 10 个物体中至少 8 个被正确分拣
  - [ ] 验证无碰撞、无 GUI 崩溃、无服务死锁

## 阶段 8: 性能基准与回归测试

- [ ] Task 8.1: GraspNet 推理性能基准
  - [ ] 创建 `/home/lxf/graspnet/benchmark.py`
  - [ ] 测量平均推理时间、P95 推理时间、显存占用
  - [ ] 验证平均推理时间 < 5s

- [ ] Task 8.2: 单目路径回归测试
  - [ ] `use_graspnet=false` 时验证 sort_verify 流程完全正常
  - [ ] `use_depth_camera=false` 时验证单目深度估计正常
  - [ ] 长时间运行（30 分钟）无 GUI 崩溃

# Task Dependencies

- Task 0.x（环境准备）→ 所有后续任务
- Task 1.x（消息接口）→ Task 3.x（grasp_pose_node）→ Task 4.x（auto_sorting_action）
- Task 2.x（realsense_yolo_node）可并行于 Task 1.x
- Task 5.x（sorting_gui_client）依赖 Task 4.x
- Task 6.x（集成测试）依赖 Task 1-5
- Task 7.x（硬件测试）依赖 Task 6.x
- Task 8.x（性能与回归）依赖 Task 7.x
