# 蓝色长方体物块视觉识别与精准抓取 Spec

**Date**: 2026-07-26  
**Status**: Draft  
**Scope**: 新增视觉节点 `blue_block_detector.py` + 修改 `auto_sorting_action.py` 和 `sorting_gui_client.py`  
**Depends on**: 现有 realsense_yolo_node.py 架构、/detection_info 协议、sort/standby 运动控制流水线

---

## Why

当前水果分拣系统存在两大痛点：深度视觉精度不高、抓取姿态约束在"求解率"和"姿态控制"之间难以平衡。蓝色长方体物块作为规则形状目标，天然适合验证和积累高精度视觉与抓取技术。通过开发独立的蓝方块分拣能力，可以在不破坏现有水果分拣的前提下迭代优化技术，最终将验证通过的方法回灌到水果分拣系统，目标在机械臂工作范围内达到 95% 抓取成功率。

## What Changes

### 新增
- **新 ROS2 节点** `blue_block_detector.py`（`/home/lxf/orange_dataset/`）：HSV 颜色分割（#33CAE8）+ minAreaRect 旋转矩形检测 + RANSAC 平面拟合深度 + `/detection_info` 发布
- **新命令** `sort_blue_block`：auto_sorting_action.py 中的蓝方块抓取命令
- **Phase 0 前置优化**：D455 Self-Calibration + 后处理滤波链（spatial/temporal/hole-filling）

### 修改
- `auto_sorting_action.py`：新增 `sort_blue_block` 命令分支，顶面中位抓取策略，分层姿态约束（宽松垂直 → 严格短轴对齐回退），物块高度感知的下降深度计算
- `sorting_gui_client.py`：新增"蓝方块分拣"模式切换按钮，蓝方块检测结果显示面板，`sort_blue_block` 命令发送

### 保留（不破坏）
- 现有 `sort` / `sort_graspnet` / `sort_verify` 命令全部原样保留
- `/detection_info` JSON 结构向后兼容（现有字段不变，仅新增蓝方块专用字段）
- 水果 YOLO 节点 `realsense_yolo_node.py` 不变
- 所有现有运动控制、碰撞检测、夹爪控制逻辑

## Impact

### Affected code
- **新增文件**：
  - `/home/lxf/orange_dataset/blue_block_detector.py`
- **修改文件**：
  - `/home/lxf/agx_arm_ws/auto_sorting_action.py`
  - `/home/lxf/agx_arm_ws/sorting_gui_client.py`

### Affected capabilities
- 物体检测（新增独立节点，与水果 YOLO 节点互斥运行）
- 深度估计（RANSAC 平面拟合，仅用于蓝方块路径）
- 机械臂抓取流程（新增 sort_blue_block 命令）
- GUI 操作界面（新增模式切换）

---

## Phase 0: D455 深度优化（前置工作）

### Requirement: D455 Self-Calibration
在开发蓝方块节点之前 SHALL 运行 RealSense SDK Self-Calibration 工具，恢复相机出厂深度精度。

#### Scenario: 自校准执行
- **WHEN** 在终端运行 `rs-self-calibration` 工具
- **THEN** 相机完成 On-Chip Self-Calibration（深度噪声优化）和 Tare Calibration（绝对精度优化）
- **AND** 校准后通过 `realsense-viewer` 验证深度图噪声水平降低

### Requirement: 深度后处理滤波链
D455 相机启动参数 SHALL 启用 spatial filter、temporal filter 和 hole-filling filter。

#### Scenario: 滤波链配置
- **WHEN** 启动 realsense2_camera_node
- **THEN** 启用 `spatial_filter.enable:=true`、`temporal_filter.enable:=true`、`hole_filling_filter.enable:=true`
- **AND** spatial filter 参数：`filter_magnitude=2`、`filter_smooth_alpha=0.5`、`filter_smooth_delta=20`
- **AND** temporal filter 参数：`filter_smooth_alpha=0.4`、`filter_smooth_delta=20`

---

## ADDED Requirements

### Requirement: 蓝方块视觉识别节点
系统 SHALL 提供独立的 ROS2 节点 `blue_block_detector`，通过 HSV 颜色分割识别蓝色 (#33CAE8) 长方体物块，并发布检测结果。

#### Scenario: HSV 颜色分割
- **WHEN** 节点订阅 `/camera/camera/color/image_raw` 接收到 RGB 图像
- **THEN** 将 RGB 图像转换为 HSV 颜色空间
- **AND** 使用预设 HSV 阈值（H∈[95,125], S∈[80,255], V∈[50,255]，支持运行时参数调节）生成蓝色掩膜
- **AND** 对掩膜执行开运算（3×3 kernel）去噪 + 闭运算（5×5 kernel）填孔
- **AND** 过滤面积 < 200px² 的噪声区域

#### Scenario: 最小外接矩形检测
- **WHEN** 掩膜中存在有效蓝色区域
- **THEN** 对掩膜执行 `cv2.findContours(RETR_EXTERNAL)` 提取外部轮廓
- **AND** 对有效轮廓执行 `cv2.minAreaRect()` 获取旋转矩形：中心 (cx,cy)、宽高 (w,h)、旋转角 angle
- **AND** 当检测到多个物块时，优先选择面积最大者

#### Scenario: 多帧滤波
- **WHEN** 连续帧检测到物块
- **THEN** 对 bbox 中心坐标、旋转角、面积应用 EMA 滤波（α=0.25）帧间平滑
- **AND** 复用现有 `realsense_yolo_node.py` 中 ObjectTrack 的最近邻像素距离匹配逻辑

#### Scenario: 无物块检测
- **WHEN** 掩膜中无有效蓝色区域
- **THEN** 发布空的 `/detection_info`（`{"detections": []}`）
- **AND** 继续正常循环，不报错

### Requirement: RANSAC 平面拟合深度估计
系统 SHALL 使用 RANSAC 平面拟合算法从物块 bbox 区域深度点云计算精确的顶面高度，失败时回退到 25% 分位数统计。

#### Scenario: RANSAC 主路径成功
- **WHEN** 获取物块的最小外接矩形
- **THEN** 将矩形向内收缩 15% 排除边缘
- **AND** 在对齐深度图中提取收缩后 ROI 内的深度像素
- **AND** 过滤无效深度（depth == 0 或超出 0.15m~2.5m），要求有效像素 ≥ 50
- **AND** 将有效像素反投影为 3D 点云（相机坐标系）
- **AND** 使用 Open3D `segment_plane(distance_threshold=0.005)` 拟合平面
- **AND** 校验法向量朝上（c > 0.7）且 inlier 比例 ≥ 30%
- **AND** 取 inliers Z 坐标中位数作为 `surface_z_m`
- **AND** `/detection_info` 的 `depth_method` 字段为 `"ransac"`

#### Scenario: RANSAC 失败回退
- **WHEN** RANSAC 拟合失败（inlier 不足 / 法向量不满足 / 有效像素不足）
- **THEN** 回退到 ROI 内深度值的 25% 分位数作为 `surface_z_m`
- **AND** `/detection_info` 的 `depth_method` 字段为 `"percentile_25"`

#### Scenario: 桌面高度估算
- **WHEN** 需要计算物块高度
- **THEN** 在物块 bbox 外扩 20px 环形区域内取深度中位数作为桌面高度 `table_z`
- **AND** 物块高度 `block_height = surface_z_m - table_z`
- **AND** 校验 `block_height` 在 0.008m~0.055m 范围内（1cm-5cm 容差）

### Requirement: 三维坐标计算与发布
系统 SHALL 使用眼在手外标定矩阵将相机坐标系下的物块坐标转换为机械臂基坐标系，并通过 `/detection_info` 发布。

#### Scenario: 坐标转换
- **WHEN** 获得物块像素中心 (center_u, center_v) 和顶面高度 surface_z_m
- **THEN** 使用相机内参反投影到相机坐标系：`X_cam = (u-cx)*Z/fx`, `Y_cam = (v-cy)*Z/fy`, `Z_cam = surface_z_m`
- **AND** 通过眼在手外标定矩阵 `T_cam_to_base` 转换到基坐标系
- **AND** 应用标定偏移修正：`+ [x_offset_m, y_offset_m, z_offset_m]`

#### Scenario: /detection_info 发布格式
- **WHEN** 发布检测结果
- **THEN** JSON 包含以下字段：
```json
{
  "detections": [{
    "name": "blue_block (detected)",
    "confidence": 0.95,
    "bbox": {"x1": 100, "y1": 150, "x2": 200, "y2": 250},
    "base_coords": {"x": 0.35, "y": -0.12, "z": 0.03},
    "grasp_orientation": {"qx": 0, "qy": 0, "qz": 0, "qw": 1},
    "centroid_base_z": 0.03,
    "estimated_diameter_m": 0.03,
    "block_rotation_deg": 15.3,
    "block_width_m": 0.03,
    "block_length_m": 0.04,
    "block_height_m": 0.02,
    "surface_z_m": 0.03,
    "depth_method": "ransac"
  }]
}
```

### Requirement: 蓝方块抓取流程
auto_sorting_action.py SHALL 支持新命令 `sort_blue_block`，执行顶面中位抓取。

#### Scenario: 下降深度计算
- **WHEN** 收到 `sort_blue_block` 命令
- **THEN** 从命令参数中获取 `surface_z_m`（物块顶面高度）和 `block_height_m`（物块高度）
- **AND** 计算夹取目标 Z：`clamping_z = surface_z_m - block_height_m / 2`（物块中位高度）
- **AND** 应用安全下限：`clamping_z = max(clamping_z, MIN_GRASP_Z)`
- **AND** POSE_PICK.z = `clamping_z + GRIPPER_PICK_Z_OFFSET`

#### Scenario: 抓取流程
- **WHEN** 执行 `sort_blue_block`
- **THEN** 执行以下步骤：
  - **step0**: 开夹爪
  - **step0.5**: 姿态预规划（分层约束，见下方）
  - **step1**: 关节空间移动到 PICK_ABOVE（`surface_z + 0.12m`），持续模式
  - **step2**: 笛卡尔直线下降到 PICK，禁用传送带碰撞
  - **step3**: 闭合夹爪（宽度 = `block_width_m - 0.002m`）
  - **step4**: 上提至 PICK_ABOVE
  - **step4.5**: 检查抓取成功（闭合宽度 < `block_width_m * 0.7`）
  - **step5-step9**: 关节空间移动到料框放置位置 + 释放 + 回待机位
  - 恢复碰撞检测（成功和错误路径均）
  - 失败时重试最多 2 次

#### Scenario: 抓取成功判定
- **WHEN** 夹爪闭合后读取 GripperStatus 反馈
- **THEN** 如果夹爪实际闭合宽度 < `block_width_m * 0.7` → 抓取成功
- **AND** 否则 → 抓取失败，释放夹爪，重试（最多 2 次）

### Requirement: 分层姿态约束
IK 求解 SHALL 使用分层回退策略，平衡求解率和姿态精度。

#### Scenario: 第一层（宽松垂直约束）
- **WHEN** 进入 step0.5 姿态预规划
- **THEN** 以 RadialDownwardOri 作为目标姿态（夹爪 Z 轴朝下）
- **AND** IK 候选池使用 200 个随机采样 + 50 次随机游走
- **AND** 通过 FK 检查的候选按 `final_ze < 0.52`（Z 轴偏离地心 < 30°）过滤
- **AND** 位置误差 < 0.015m

#### Scenario: 第二层（严格短轴对齐，第一层失败时回退）
- **WHEN** 第一层无有效解
- **THEN** 从 `block_rotation_deg` 构建目标四元数（夹爪 Y 轴/闭合方向对齐物块短边）
- **AND** IK 候选数量减少到 50 个
- **AND** 首位插入 PCA 短轴对齐朝向 + yaw 约束引导
- **AND** 仍要求 `final_ze < 0.52` 和位置误差 < 0.015m

#### Scenario: 两层均失败
- **WHEN** 两个约束层均无有效解
- **THEN** 返回 `CONTROL_FAILED` 错误状态
- **AND** 机械臂回到待机位，等待重试

### Requirement: GUI 蓝方块模式
sorting_gui_client.py SHALL 提供蓝方块分拣模式切换功能。

#### Scenario: 模式切换
- **WHEN** 用户点击"蓝方块分拣"按钮（蓝色背景 #33CAE8）
- **THEN** GUI 切换到蓝方块模式：停止水果 YOLO 节点，启动 `blue_block_detector.py`（通过 ROS worker 子进程）
- **AND** 蓝方块按钮高亮，水果分拣按钮灰显
- **AND** 检测结果显示面板切换到蓝方块专用字段

#### Scenario: 蓝方块显示字段
- **WHEN** GUI 处于蓝方块模式
- **THEN** 检测面板显示：物块中心 XYZ、顶面高度（标注 RANSAC/分位数方法）、尺寸（长×宽×高）、旋转角

#### Scenario: 抓取触发
- **WHEN** 用户在蓝方块模式下点击抓取按钮
- **THEN** 发送 `sort_blue_block` 命令到 `/sorting_cmds`，携带 pick 坐标、surface_z_m、block_width_m、block_length_m、block_height_m、block_rotation_deg

#### Scenario: 与水果模式的互斥
- **WHEN** 蓝方块检测节点运行时
- **THEN** 水果 YOLO 节点必须停止（避免 D455 USB 带宽竞争）
- **AND** 切换回水果模式时，停止 `blue_block_detector.py`，恢复 YOLO 节点

---

## 精度控制措施

| 环节 | 措施 | 预期精度 |
|------|------|----------|
| RANSAC 距离阈值 | 0.005m（5mm），适配 D455 深度噪声 | ±3mm（平面表面） |
| 法向量校验 | 平面法向量与重力夹角 < 45°（c > 0.7） | 排除斜面/侧面误判 |
| Inlier 比例 | ≥ 30% ROI 像素属于平面才接受 | 排除非平面物体 |
| ROI 收缩 | 向内 15% 排除物块边缘深度跳变 | 减少边缘离群点 |
| 回退保障 | 25% 分位数统计兜底 | ±8mm（兜底可用） |
| 最小外接矩形 | cv2.minAreaRect 亚像素精度 | ±1px ≈ ±1mm @ 40cm |
| 旋转角平滑 | 多帧 EMA 滤波 α=0.25 | ±2° |
| 标定偏移 | 通过抓取测试反算偏移修正值 | 累积误差 < 5mm |

---

## 测试验证方案

### Phase 1: 静态测试

| 测试项 | 方法 | 通过标准 |
|--------|------|----------|
| 颜色分割准确性 | 3 种光照条件下各取 20 帧，人工标注 GT | 检出率 > 98%，误检率 < 2% |
| 深度精度 | 物块放置在已知桌面高度位置，测量实际 vs 检测高度 | MAE < 5mm |
| XY 精度 | 物块放置标定网格点，测量实际 vs 检测位置 | MAE < 5mm |
| 单次抓取成功率 | 30 次连续抓取（1cm/3cm/5cm 各 10 次） | 成功率 ≥ 90% |
| 重复定位精度 | 同一位置重复抓取 10 次 | XY 方差 < 3mm |

### Phase 2: 动态测试（传送带，后续阶段）

| 测试项 | 方法 | 通过标准 |
|--------|------|----------|
| 传送带移动中检测 | 传送带低速运行，连续检测位置 | 跟踪误差 < 1cm |
| 动态抓取成功率 | 连续 50 次跑传送带排序 | 成功率 ≥ 95%（最终目标） |

---

## 设计决策摘要

| 决策点 | 选择 | 理由 |
|--------|------|------|
| 视觉算法 | HSV 颜色分割 + cv2.minAreaRect | 行业主流方案（AgileX PIPER cubeAndLineDet、多个 ROS2 开源项目验证），比 YOLO 多出旋转角信息 |
| 深度估计 | RANSAC 平面拟合 → 25% 分位数回退 | 物块顶面为平面，RANSAC 拟合精度天然高于点采样 |
| 抓取策略 | 顶面中位抓取（下降至物块一半高度） | 夹爪两指从侧面夹住物块，自然稳定 |
| 姿态约束 | 分层回退：宽松垂直 → 严格短轴对齐 | 解决现有系统"约束紧无解 vs 约束松姿态差"的矛盾 |
| 架构模式 | 独立节点 + 互斥运行 | 避免 D455 USB 带宽竞争（已知硬约束），两种模式互不干扰 |
| 协议复用 | 扩展现有 /detection_info JSON | 零破坏性，GUI 和 action 节点改造最小 |
| 与水果节点关系 | 独立节点，互斥运行 | 避免 D455 USB 带宽竞争（项目已知硬约束） |
| D455 优化 | Phase 0 前置 Self-Calibration + 后处理滤波链 | 低实施成本、已验证的深度质量提升手段 |

---

## 参考项目与论文

| 来源 | 关键参考点 |
|------|-----------|
| [AgileX cubeAndLineDet](https://github.com/agilexrobotics/Agilex-College/tree/master/piper/cubeAndLineDet) | HSV 色块识别 + 深度相机三维坐标提取 + PIPER 机械臂，与本项目高度相似 |
| [dgkagkan/robotic_arm_ros2](https://github.com/dgkagkan/robotic_arm_ros2) | ROS2 + MoveIt2 + HSV 色块抓取，2026 年活跃 |
| [Jpst01/ur5-pick-and-place-ros2](https://github.com/Jpst01/ur5-pick-and-place-ros2) | 视觉伺服二次精确定位技术 |
| [CloudGrasp](https://github.com/ritwikrohan/CloudGrasp) | RANSAC 平面分割 + Euclidean 聚类 + 长方体拟合 |
| Song et al. 2026 (Sensors) | HSV + heightmap 抓取 Jenga 木块 99.02% 成功率 |
| MVB-Grasp 2026 (arXiv) | PCA OBB 拟合 + 几何过滤抓取候选 |
| RealSense Self-Calibration 白皮书 | D4xx 系列深度噪声优化与绝对精度校准 |
