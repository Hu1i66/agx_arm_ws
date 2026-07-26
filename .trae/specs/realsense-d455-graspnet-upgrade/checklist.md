# Checklist

## 阶段 0: 环境准备
- [ ] graspnet-venv 创建成功，PyTorch + CUDA 可用
- [ ] GraspNet-1Billion 仓库克隆完成，graspnetAPI 可 import
- [ ] checkpoint-tbl.tar 下载并解压到 `/home/lxf/graspnet/checkpoints/`
- [ ] pointnet2_ops 编译成功，`import pointnet2_ops` 无报错
- [ ] `test_graspnet.py` 输出至少 1 个候选，推理时间 < 10s

## 阶段 1: ROS2 消息接口
- [ ] `agx_arm_msgs/msg/GraspCandidate.msg` 文件存在且字段正确
- [ ] `agx_arm_msgs/srv/GenerateGraspPose.srv` 文件存在且字段正确
- [ ] CMakeLists.txt 包含 sensor_msgs、builtin_interfaces 依赖
- [ ] package.xml 包含 sensor_msgs、builtin_interfaces 依赖
- [ ] `colcon build --packages-select agx_arm_msgs` 编译成功
- [ ] `ros2 interface show agx_arm_msgs/srv/GenerateGraspPose` 正确显示

## 阶段 2: realsense_yolo_node.py
- [ ] 参数 `use_depth_camera` 声明成功（默认 True）
- [ ] 订阅 `/camera/camera/aligned_depth_to_color/image_raw` 成功
- [ ] `depth_cb` 回调正确缓存深度图（uint16 ndarray）
- [ ] `depth_from_camera` 方法实现：中心 5x5 中位数，深度范围 0.2-3.0m
- [ ] 调用点 L789 根据参数选择深度来源，D455 失败时自动回退单目
- [ ] `/detection_info` JSON 包含 `header_stamp` 字段
- [ ] `method` 字段根据深度来源动态切换（"depth_camera" / "monocular_rgb"）
- [ ] `ros2 topic echo /detection_info` 验证字段存在
- [ ] `ros2 param set` 切换 `use_depth_camera` 后 `method` 字段变化

## 阶段 3: grasp_pose_node.py
- [ ] 文件 `/home/lxf/orange_dataset/grasp_pose_node.py` 存在
- [ ] sys.path 注入 graspnet-venv 的 site-packages
- [ ] 所有参数声明（graspnet_checkpoint、max_gripper_width、score_threshold 等）
- [ ] 订阅 RGB、depth、camera_info、robot_pose
- [ ] 时间戳缓冲 dict 实现，5s 过期清理
- [ ] `_lookup_depth_by_stamp` 二分查找（50ms 容差）
- [ ] `_lookup_robot_pose_by_stamp` 二分查找（150ms 容差）
- [ ] `_depth_to_pointcloud` 用 Open3D 生成点云
- [ ] `_crop_by_bbox` 按 bbox 裁剪（带 30% margin）
- [ ] `_preprocess` 下采样 + 离群点去除
- [ ] GraspNet 模型加载成功（checkpoint-tbl，max_width=0.10）
- [ ] `_run_graspnet` 推理返回 grasp 列表
- [ ] `_filter_grasps` z-down 过滤（<30°）+ score 过滤（>0.5）
- [ ] 坐标变换 cam_optical → ee → base 正确
- [ ] workspace 过滤生效
- [ ] service `/generate_grasp_pose` 注册并响应
- [ ] 所有失败情况返回明确 error_msg
- [ ] `/grasp_pose_marker` MarkerArray 发布
- [ ] RViz 中可视化箭头颜色按分数区分
- [ ] `ros2 service call` 手动测试成功

## 阶段 4: auto_sorting_action.py
- [ ] import GenerateGraspPose、GraspCandidate、RegionOfInterest、Header
- [ ] 参数 `use_graspnet`、`graspnet_timeout_s`、`apply_z_correction_in_graspnet` 声明
- [ ] service client 创建，`wait_for_service` 设置 `grasp_service_available`
- [ ] `_call_grasp_service_async` 实现：call_async + spin_once 循环
- [ ] `_build_pick_orientations_with_graspnet` 实现：GraspNet orientation 插入 + 去重
- [ ] `sort_graspnet` 命令分支实现
- [ ] GraspNet 成功时 z 直接用作夹爪夹持点位置（不加 GRIPPER_PICK_Z_OFFSET）
- [ ] GraspNet 失败时回退到 sort_verify 路径
- [ ] 多候选 orientation 插入 pick_orientations 前面
- [ ] `python3 -m py_compile auto_sorting_action.py` 语法通过
- [ ] grasp_pose_node 未启动时日志显示 "服务不可用, 回退"
- [ ] grasp_pose_node 启动后日志显示 "服务已连接"

## 阶段 5: sorting_gui_client.py
- [ ] `self.use_graspnet = True` 初始化
- [ ] `self.latest_detection_stamp` 在 /detection_info 回调中更新
- [ ] `_dispatch_sort_cmd` 根据 use_graspnet 选择命令类型
- [ ] sort_graspnet 命令包含 `bbox` 和 `detection_stamp` 字段
- [ ] 手动分拣回调（L607）同样支持 sort_graspnet
- [ ] GUI "使用 GraspNet" 复选框添加
- [ ] `_on_graspnet_toggle` 回调实现
- [ ] `python3 -m py_compile sorting_gui_client.py` 语法通过
- [ ] 取消勾选后发送 sort_verify 命令
- [ ] 勾选后发送 sort_graspnet 命令

## 阶段 6: 集成测试
- [ ] 单元测试 `test_grasp_pose_node.py` 所有用例通过
- [ ] service 空请求返回 success=false + 明确 error_msg
- [ ] service 无效 bbox 返回 success=false
- [ ] service 正常请求返回候选列表
- [ ] grasp_pose_node 未启动时 auto_sorting_action 回退到 sort_verify
- [ ] grasp_pose_node 卡死时 10s 超时后回退

## 阶段 7: 硬件在环测试
- [ ] 单物体（苹果）抓取 10 次成功率 ≥ 80%
- [ ] 7 类水果各测试 5 次，记录成功率
- [ ] GraspNet 路径抓取成功率与 sort_verify 相当或更好
- [ ] 无碰台面现象
- [ ] 回退率 < 20%
- [ ] 边界情况（部分超出视野、强光、反光）正确处理
- [ ] 端到端 10 物体自动分拣，≥8 个正确分拣
- [ ] 无机械臂碰撞、无 GUI 崩溃、无服务死锁

## 阶段 8: 性能与回归
- [ ] GraspNet 平均推理时间 < 5s
- [ ] P95 推理时间 < 10s
- [ ] 显存占用 < 4GB
- [ ] `use_graspnet=false` 时 sort_verify 流程完全正常
- [ ] `use_depth_camera=false` 时单目深度估计正常
- [ ] 30 分钟长时间运行无 GUI 崩溃（Tcl_Release 问题不复发）

## 关键设计约束验证
- [ ] GraspNet 路径下 z 值直接用作夹爪夹持点位置（不加 GRIPPER_PICK_Z_OFFSET）
- [ ] PER_OBJECT_Z_CORRECTION 在 GraspNet 路径下默认不应用（参数可开启）
- [ ] 碰撞检测禁用/恢复策略在 GraspNet 路径下不变
- [ ] 夹爪开合目标仍用 compute_gripper_targets 计算
- [ ] 自动分拣状态机不变（IDLE → CHECK_LEFT → ... → SORTING → ...）
- [ ] 传送带停止后 1s 观察延迟机制不变
- [ ] 分界线 u=380 不变
- [ ] /detection_info JSON 向后兼容（仅新增字段，不删除/修改现有字段）
