# Tasks

## Phase 0: D455 深度优化（前置，不需要后续任务依赖即可启动）

- [ ] Task 0.1: 运行 RealSense Self-Calibration 工具进行相机自校准
  - [ ] 执行 `rs-self-calibration` On-Chip + Tare Calibration
  - [ ] 通过 `realsense-viewer` 验证深度噪声水平改善
  - [ ] 记录校准前后深度噪声对比数据

- [ ] Task 0.2: 配置深度后处理滤波链
  - [ ] 在 `rs_launch.py` 或相机启动脚本中启用 spatial/temporal/hole-filling filter
  - [ ] 验证滤波后深度图空洞减少、边缘更清晰

## Phase 1: 视觉节点开发

- [ ] Task 1.1: 创建 blue_block_detector.py 基础框架
  - [ ] 创建 ROS2 节点 `blue_block_detector`，继承 Node
  - [ ] 订阅 `/camera/camera/color/image_raw`、`/camera/camera/aligned_depth_to_color/image_raw`、`/camera/camera/color/camera_info`
  - [ ] 发布 `/detection_info`（String, JSON）、`/yolo/annotated_image/compressed`（CompressedImage）
  - [ ] 使用 `/home/lxf/orange_dataset/.venv` Python 环境

- [ ] Task 1.2: 实现 HSV 颜色分割 + minAreaRect 检测
  - [ ] RGB → HSV 转换，蓝色掩膜提取（H∈[95,125], S∈[80,255], V∈[50,255]）
  - [ ] 形态学处理（开运算 + 闭运算）+ 面积滤波
  - [ ] `cv2.findContours` + `cv2.minAreaRect` 获取旋转矩形
  - [ ] 多物块筛选（面积最大优先）
  - [ ] EMA 滤波（α=0.25）帧间平滑
  - [ ] 标注图像绘制（旋转矩形 + 中心点），发布到 `/yolo/annotated_image/compressed`

- [ ] Task 1.3: 实现 RANSAC 平面拟合深度估计
  - [ ] 从 minAreaRect 提取收缩后 ROI，载入对齐深度图
  - [ ] 有效深度过滤 → 反投影为 3D 点云（相机坐标系）
  - [ ] Open3D `segment_plane(distance_threshold=0.005)` 拟合平面
  - [ ] 法向量校验（c > 0.7）+ inlier 比例校验（≥ 30%）
  - [ ] inliers Z 中位数作为 surface_z_m
  - [ ] 失败回退到 25% 分位数统计

- [ ] Task 1.4: 实现物块高度估算与三维坐标计算
  - [ ] bbox 外扩环形区域桌面深度中位数提取
  - [ ] `block_height = surface_z_m - table_z` + 范围校验
  - [ ] 像素中心 → 相机坐标系（用 surface_z_m 而非单点深度）
  - [ ] 相机坐标系 → 基坐标系（T_cam_to_base + calibration offsets）
  - [ ] 组装 JSON 并发布到 `/detection_info`

## Phase 2: 抓取集成

- [ ] Task 2.1: auto_sorting_action.py 新增 sort_blue_block 流程
  - [ ] 新增 `sort_blue_block` 命令解析（从 `/sorting_cmds` 提取 field）
  - [ ] 实现顶面中位抓取下降深度计算：`clamping_z = surface_z - height/2`
  - [ ] 实现分层姿态约束：第一层宽松垂直 → 第二层短轴对齐回退
  - [ ] 实现蓝方块抓取序列（step0-step9）
  - [ ] 夹爪开合宽度 = `block_width_m - 0.002m`
  - [ ] 抓取成功判定：闭合宽度 < `block_width_m * 0.7`
  - [ ] 失败重试最多 2 次
  - [ ] 碰撞检测禁用/恢复（复用现有逻辑）

- [ ] Task 2.2: sorting_gui_client.py 新增蓝方块模式
  - [ ] 新增"蓝方块分拣"按钮（蓝色背景 #33CAE8）+ "水果分拣"切换按钮
  - [ ] 模式切换逻辑：停止/启动对应视觉节点（通过 ROS worker 子进程，避免 fork）
  - [ ] 蓝方块检测结果显示面板（XYZ、尺寸、旋转角、深度方法）
  - [ ] 手动抓取触发：发送 `sort_blue_block` 命令到 `/sorting_cmds`
  - [ ] 与水果模式互斥（同一时间只有一个视觉节点运行）

## Phase 3: 精度调优与测试验证

- [ ] Task 3.1: 标定偏移校正
  - [ ] 物块放置在已知位置，对比检测坐标 vs 实际坐标
  - [ ] 反算 `x_offset_m`、`y_offset_m`、`z_offset_m` 修正值
  - [ ] 更新 blue_block_detector.py 中的偏移参数

- [ ] Task 3.2: HSV 阈值 + 深度参数调优
  - [ ] 在不同光照条件下测试 HSV 阈值，确定稳定范围
  - [ ] 调整 RANSAC distance_threshold、inlier 比例阈值
  - [ ] 验证回退路径可靠性

- [ ] Task 3.3: 30 次静态抓取测试
  - [ ] 1cm、3cm、5cm 立方体各 10 次
  - [ ] 记录每次抓取成功/失败状态、深度方法、XY 偏差
  - [ ] 计算抓取成功率（目标 ≥ 90%）

# Task Dependencies

- Task 1.2 依赖 Task 1.1
- Task 1.3 依赖 Task 1.1
- Task 1.4 依赖 Task 1.2 和 Task 1.3
- Task 2.1 依赖 Task 1.4（需要确定数据协议）
- Task 2.2 可与 Task 2.1 并行（通过约定的 JSON 协议解耦）
- Phase 0 可与 Phase 1 并行
- Phase 3 依赖 Phase 1 + Phase 2 全部完成
