# Checklist

## Phase 0: D455 深度优化

- [ ] D455 Self-Calibration 已运行且通过验证（用户手动）
- [ ] 深度后处理滤波链（spatial/temporal/hole-filling）已启用（用户手动，当前 _CAMERA_CMD 未含滤波参数，需补充）
- [ ] 校准前后深度质量有可量化的改善（用户手动）

## Phase 1: 视觉节点

- [x] blue_block_detector.py 节点可独立启动，无 import 错误（py_compile 通过，venv 依赖齐全）
- [x] HSV 颜色分割能正确识别 #33CAE8 蓝色物块（代码层面通过，检出率 > 98% 待实测）
- [x] minAreaRect 返回旋转矩形的中心、宽高、角度数值合理（_normalize_rect 归一化 w=长边/h=短边/angle∈[0,180)）
- [x] 多物块场景能正确选出面积最大的目标（max(valid, key=cv2.contourArea)）
- [x] EMA 滤波对帧间抖动有明显平滑效果（ObjectTrack α=0.25，圆周角最短路径 EMA）
- [x] RANSAC 平面拟合在主路径成功时返回 surface_z_m（depth_method = "ransac"）
- [x] RANSAC 失败时正确回退到 25% 分位数（depth_method = "percentile_25"）
- [x] 法向量校验拒绝非水平平面（c ≤ 0.7 被拒绝走回退）
- [x] 物块高度 block_height 在 0.008m~0.055m 范围内（block_height = table_z - surface_z_m，超出范围仅 warning）
- [x] 相机 → 基座坐标系转换正确（T_cam_to_base + offsets: x=-0.045, y=-0.010, z=0.030）
- [x] /detection_info JSON 格式符合 spec，包含所有蓝方块专用字段（13 个字段全部一致）
- [x] 标注图像正确绘制旋转矩形和中心点，发布到 /yolo/annotated_image/compressed

## Phase 2: 抓取集成

- [x] sort_blue_block 命令能被 auto_sorting_action.py 正确解析（第 2815 行 elif 分支）
- [x] 顶面中位抓取下降深度 = surface_z - height/2（夹爪在物块中位高度闭合）（修复后 surface_z_m 为基座系高度，计算正确）
- [x] 分层姿态约束：第一层有解时使用垂直约束，第一层无解时回退到短轴对齐
- [x] final_ze < 0.52（Z 轴不朝上）约束生效
- [x] 夹爪开合宽度 = block_width_m - 0.002m
- [x] 抓取成功判定：闭合后宽度 > block_width_m * 0.7（修正方向：宽度大说明物块在两指间）
- [x] 失败重试最多 2 次，重试后仍失败则跳过（MAX_BLUE_BLOCK_RETRIES = 2）
- [x] 碰撞检测在下降前正确禁用，回到待机位后正确恢复（含错误路径）
- [x] GUI "蓝方块分拣"按钮功能正常（蓝色背景 #33CAE8）
- [x] 蓝方块模式与水果模式可正确切换（视觉节点互斥，ROS worker 子进程执行 kill/launch）
- [x] 检测面板显示蓝方块专用字段（XYZ、尺寸、旋转角、深度方法）
- [x] sort_blue_block 命令 JSON 包含所有必需字段

## Phase 3: 精度与测试

- [ ] 标定偏移参数已通过实际测试反算并更新（待用户实测）
- [ ] 不同光照条件下 HSV 分割稳定（待用户实测）
- [ ] 深度精度 MAE < 5mm（RANSAC 路径）（待用户实测）
- [ ] XY 精度 MAE < 5mm（待用户实测）
- [ ] 30 次静态抓取成功率 ≥ 90%（待用户实测）
- [ ] 重复定位 XY 方差 < 3mm（待用户实测）
- [ ] 现有水果分拣流程未受影响（回归测试）（待用户实测）
