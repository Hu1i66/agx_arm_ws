# D455 双目抓取 + PCA 短轴对齐 — 实施计划

> 战略转向：**完全移除 GraspNet**，改用 D455 立体深度 + PCA 短轴对齐实现高精度顶视抓取，
> 并为后续随动抓取（传送带不停、跟踪运动物体）预留接口。
>
> 本计划由用户明确需求驱动（通过 AskUserQuestion 确认）：
> - GraspNet → **完全移除**
> - 抓取方向 → **顶视抓取 + 短轴对齐**
> - 精度目标 → **±5mm 以内**
> - 随动时间线 → **中期（1-2 月）**，先做好静态抓取，架构留接口

---

## 一、现状分析

### 1.1 精度问题根因（已定位）
| 误差源 | 量级 | 状态 |
|--------|------|------|
| 内参不一致：`realsense_yolo_node.py` 硬编码 fx=378.39 vs D455 出厂 fx≈384.81 | ~1.7% → 0.5m 处 ~8.5mm | 待修复 |
| D455 近距离(<0.4m)深度系统性偏大 | ~40mm | 已修正(0.04m offset) |
| GraspNet grasp.translation 是夹爪基座坐标非物体中心 | ~27mm xy 偏移 | 移除 GraspNet 即消除 |
| 单目回退深度（已知尺寸反推） | ~10% | D455 优先，单目仅兜底 |

→ 修复内参 + 移除 GraspNet 后，剩余误差为 D455 深度噪声(~1-2mm@0.4-0.5m) + 手眼标定残差，**±5mm 可达**。

### 1.2 GraspNet 路线问题（需移除）
- 过度复杂：service 调用 + 两阶段近距离检测 + 6DoF 姿态 + 独立 venv + CUDA 编译
- 脆弱：点云稀疏时抓取背景、D455 深度误差放大
- **安全隐患**：GraspNet 姿态经 IK/MoveIt 时出现向上/水平抓取（已用 FK 检查 + 姿态约束兜底，但根因是 GraspNet 引入任意朝向）
- 精度未提升反下降

### 1.3 标定需求评估结论
- **D455 出厂已标定**：左右 IR 立体 + RGB-Depth 外参，`aligned_depth_to_color` 已像素级对齐
- 用户给的两篇 CSDN 文章讲的是**传统分立双目相机**的内外参标定，**不适用**于 D455（出厂已完成）
- **无需额外 RGB-Depth 联合标定**
- 仅需：① 手眼标定（已有 `2026-06-06_04-12-35_calibration.json`）② 订阅 `camera_info` 获取准确内参（替代硬编码）

### 1.4 关键文件现状（基于 Phase 1 探索）
- **`/home/lxf/orange_dataset/realsense_yolo_node.py`**（工作区外，检测节点）
  - L124-145：硬编码内参 fx/fy/cx/cy/skew + camera_matrix + dist_coeffs
  - L288-297：rgb + depth 订阅块，**无 camera_info 订阅**
  - L494-499：`robot_pose_cb`（更新 robot_current_pose + robot_pose_history）
  - L558-567：`depth_cb`（存 latest_depth + latest_depth_stamp）
  - L569-622：`transform_camera_to_end_effector` / `transform_end_effector_to_base` / `transform_camera_to_base`
  - L657-680：`monocular_depth_from_bbox`（单目回退）
  - L682-740：`depth_from_camera`（5x5 中值 + 0.04m offset + 有效性检查）
  - L811-1054：`detect_and_publish` 主循环，per-object 坐标计算在 L927-960
  - L1076-1107：`/detection_info` 发布，info_dict 构造（含 base_position_m / bbox_pixel / header_stamp）
  - L17-89：`ObjectTrack` 类，已对 bbox/depth/cam/ee/base 做 EMA 滤波（alpha=0.25）

- **`/home/lxf/agx_arm_ws/auto_sorting_action.py`**（动作执行节点）
  - L27-43：agx_arm_msgs imports（GraspNet 接口）+ path injection
  - L149-169：IK `_combined_error`（已含 Z 轴 + yaw 约束，返回 4 元组）
  - L277-299：GraspNet service client 初始化
  - L534-570：`_two_stage_refine`（两阶段精定位，verify_height=0.40）
  - L633-648：`_build_pick_orientation`（径向朝下，quat1 + 反向 quat2）
  - L650-677：`_build_pick_orientations_multi`（径向 + 8 yaw 采样）
  - L679-755：`_call_grasp_service_async`（**删除**）
  - L757-800：`_build_pick_orientations_with_graspnet`（**删除**）
  - L802-843：`_build_goal_constraints`（orientation=None 时不加姿态约束）
  - L1010-1032：`move_arm_pose`（IK 优先 → MoveIt 笛卡尔兜底）
  - L1457-1463：命令分发（sort/sort_verify/sort_graspnet）
  - L1510-1522：GraspNet 路径初始化（**删除**）
  - L1542-1562：两阶段精定位（保留）
  - L1564-1652：GraspNet 近距离检测块（**删除**）
  - L1680-1710：下降抓取朝向使用 GraspNet 候选（**改为 PCA 朝向**）

- **`/home/lxf/agx_arm_ws/sorting_gui_client.py`**（GUI 客户端）
  - L234-239：use_graspnet BooleanVar + latest_detection_stamp
  - L356-366：两阶段按钮 + GraspNet checkbox UI
  - L480-495：检测循环缓存 header_stamp
  - L613-664：`_pick_from_detection_two_stage`（use_gn 分支）
  - L903-938：`_dispatch_sort_cmd`（自动分拣 use_gn 分支）
  - L940-948：`_on_graspnet_toggle`（**删除**）

---

## 二、目标架构

```
YOLO bbox ──► D455 aligned_depth (camera_info 内参) ──► 像素→3D ──► 手眼变换 ──► base 坐标
                 │                                                       │
                 ▼                                                       ▼
        ObjectTrack EMA 滤波 (已有)                          bbox 内局部点云 → PCA 短轴 → 夹爪 yaw 四元数
        (深度/坐标时域平滑, ±5mm)                                          │
                                                                          ▼
                                              /detection_info 新增: grasp_orientation + elongation_ratio + velocity_mps(null)
                                                                          │
                                                                          ▼
                                              auto_sorting_action: 下降抓取用 PCA 朝向 (替代 GraspNet 朝向)
                                                                   IK yaw 约束引导匹配 PCA 朝向
                                                                   position_only 45° 兜底 + FK 检查 (安全防线)
```

**精度来源**：camera_info 准确内参（消除 8.5mm）+ D455 立体深度（消除单目 10% 误差）+ EMA 滤波（消除帧间噪声）+ 移除 GraspNet（消除 27mm xy 偏移）。

**短轴对齐来源**：bbox 内深度图生成 base 系局部点云 → PCA → 短轴 → 夹爪 X 轴（开合方向）对齐短轴，手指从两条长边合拢夹窄腰。

---

## 三、具体修改

### 文件 1：`/home/lxf/orange_dataset/realsense_yolo_node.py`

#### 1.1 订阅 camera_info（替代硬编码内参）
- **import**：顶部加 `from sensor_msgs.msg import CameraInfo`（L5 附近，已有 `from sensor_msgs.msg import CompressedImage, Image`）
- **新增订阅**：在 L288-297 rgb/depth 订阅块之后，加：
  ```python
  self.sub_cam_info = self.create_subscription(
      CameraInfo, "/camera/camera/color/camera_info", self.camera_info_cb, 10)
  ```
  （topic 名与现有 `/camera/camera/color/image_raw` 同命名空间；executor 运行时用 `ros2 topic list` 确认）
- **新增回调 `camera_info_cb`**：
  - 解析 msg.k（3x3 行优先）→ 更新 self.fx/fy/cx/cy/camera_matrix
  - 解析 msg.d → 更新 self.dist_coeffs（若非空）
  - 仅首帧打印一次日志（"📥 camera_info 已更新内参: fx=.., fy=.., cx=.., cy=.."）
  - 用 `self._cam_info_received` 标志避免重复更新
- **保留硬编码作为 fallback**：L124-145 硬编码值保留作为初始值（camera_info 未到达时仍可用），camera_info 到达后覆盖。打印日志标注当前用的是"硬编码(待 camera_info)"还是"camera_info(实时)"。

#### 1.2 PCA 短轴对齐方法
新增方法 `_compute_grasp_orientation(self, bbox_xyxy, depth_mm, object_base_z)`：
- **入参**：bbox [x1,y1,x2,y2]、depth_map(uint16 mm)、物体 base 系 z（用于高度带通）
- **返回**：`(quat_list, elongation_ratio)` 或 `(None, None)`（圆物体/点云不足时）
- **算法**：
  1. bbox 区域 + 30% margin 裁剪 depth；有效深度(>0)像素集合
  2. 向量化反投影：`x = (u - cx) * z / fx`, `y = (v - cy) * z / fy`, `z = depth_m`（用 self.fx/fy/cx/cy）
  3. 变换到 base 系：camera_coords → ee（手眼矩阵 self.transform_cam_to_ee）→ base（robot_current_pose，用 transform_camera_to_base 现有逻辑）
  4. 高度带通：保留 base_z 在 `[object_base_z - 0.03, object_base_z + 0.03]` 的点（剔除传送带 + 背景）
  5. 点数 < 30 → 返回 (None, None)
  6. PCA on base XY 分量（2xN）：协方差矩阵 → 特征值 λ1≥λ2，特征向量 v1, v2
  7. `elongation_ratio = λ1 / λ2`
  8. 若 `elongation_ratio < 1.5` → 近圆形，返回 (None, elongation_ratio)（用径向朝下）
  9. 短轴方向 = v2（最小特征值对应特征向量，XY 平面 2D）
  10. 构造夹爪朝向（base 系）：
      - Z = [0, 0, -1]（朝下）
      - X = 短轴 v2 投影到 ⊥Z 平面并归一化（v2 已在 XY 平面，⊥Z 自动满足，直接归一化）
      - Y = Z × X，归一化
      - 用现有 `_matrix_to_quaternion` 等价逻辑（该节点无此方法，需自实现四元数转换，或用 tf_transformations.quaternion_from_matrix）
  11. 返回 `([qx, qy, qz, qw], elongation_ratio)`
- **健壮性**：depth_offset_m 修正、无效深度跳过、robot_current_pose 为 None 时返回 (None, None)

> 注：realsense_yolo_node.py 已 import `tf_transformations`，可用 `tf_transformations.quaternion_from_matrix(R_4x4)` 把旋转矩阵转四元数。

#### 1.3 在 detect_and_publish 中调用 PCA
- 位置：L927-960 per-object base_coords 计算之后
- 对 primary 物体（即写入 info_dict 顶层字段的物体）调用：
  ```python
  grasp_ori, elong = self._compute_grasp_orientation(bbox_xyxy, self.latest_depth, base_z)
  ```
- 将结果存入 primary_info 字典：`primary_info['grasp_orientation'] = grasp_ori`（list[4] 或 None）、`primary_info['elongation_ratio'] = elong`
- 仅对 primary 物体计算（性能 + 主要抓取目标），objects_list 中各物体不计算（可选，后续随动再扩展）

#### 1.4 发布到 /detection_info
- 位置：L1076-1107 info_dict 构造
- 在 `if primary_info is not None:` 块内，base_position_m 之后新增：
  ```python
  if "grasp_orientation" in primary_info:
      info_dict["grasp_orientation"] = primary_info["grasp_orientation"]  # [qx,qy,qz,qw] 或 None
      info_dict["elongation_ratio"] = primary_info.get("elongation_ratio")
  # 随动接口占位（中期实现）
  info_dict["velocity_mps"] = None
  ```

---

### 文件 2：`/home/lxf/agx_arm_ws/auto_sorting_action.py`

#### 2.1 移除 GraspNet 依赖
- **L27-43**：删除 agx_arm_msgs imports 块 + `_AGX_MSGS_INSTALL_PATHS` path injection（保留 L45-58 Pinocchio path injection，**不要动**）
- **L277-299**：删除整个 `# ========== GraspNet service client ==========` 块（use_graspnet / graspnet_timeout_s / apply_z_correction_in_graspnet / grasp_client 初始化）
- **L679-755**：删除 `_call_grasp_service_async` 方法
- **L757-800**：删除 `_build_pick_orientations_with_graspnet` 方法
- **L1510-1522**：删除 GraspNet 路径初始化块（graspnet_candidates/graspnet_ok/use_graspnet_path 检查）
- **L1564-1652**：删除整个 GraspNet 近距离检测块（移到验证位 → 等近距离检测 → 调 service）
- **L1683-1686**：删除 `if graspnet_ok and graspnet_candidates:` 用 GraspNet 姿态分支
- **L1697-1702**：删除 `if graspnet_ok and graspnet_candidates:` IK 候选分支

#### 2.2 简化命令分发
- **L1457-1463**：`sort_graspnet` 不再单独处理，重定向到 sort_verify：
  ```python
  elif data.get("cmd") in ("sort", "sort_verify", "sort_graspnet"):
      cmd_type = data.get("cmd")
      # sort_graspnet 已废弃, 统一回退到两阶段精定位 (向后兼容旧 GUI 命令)
      two_stage = (cmd_type in ("sort_verify", "sort_graspnet"))
      use_graspnet_path = False  # GraspNet 已移除
  ```
- 删除所有 `graspnet_ok` / `graspnet_candidates` / `use_graspnet_path` 变量引用，将相关条件简化（`if two_stage and not graspnet_ok:` → `if two_stage:`；`if graspnet_ok:` → 删除该分支）

#### 2.3 用 PCA 朝向替代 GraspNet 朝向
- **下降抓取朝向（L1680-1689 区域）**：从 /detection_info 读取 grasp_orientation
  ```python
  pick_fallback_ori = node._build_pick_orientation(POSE_PICK)[0]
  # 从检测信息读取 PCA 短轴对齐朝向 (替代已移除的 GraspNet 朝向)
  pca_ori = None
  if hasattr(node, '_latest_detection') and node._latest_detection:
      pca_ori = node._latest_detection.get('grasp_orientation')
  if pca_ori is not None and len(pca_ori) == 4:
      active_ori = pca_ori
      print("📐 下降抓取使用 PCA 短轴对齐姿态")
  else:
      active_ori = getattr(node, 'last_successful_orientation', pick_fallback_ori)
      print("📐 下降抓取使用径向朝下姿态 (物体近圆形或无 PCA 朝向)")
  ```
  > 注：`node._latest_detection` 在两阶段精定位后会刷新为近距离检测结果，其 grasp_orientation 即近距离 PCA 朝向，精度更高。sort（无两阶段）路径则用观察位检测的朝向。
- **IK 候选朝向（L1697-1706 区域）**：PCA 朝向插入候选首位
  ```python
  pick_orientations = node._build_pick_orientations_multi(POSE_PICK)
  if pca_ori is not None and len(pca_ori) == 4:
      pick_orientations = [pca_ori] + pick_orientations  # PCA 优先
  ik_desc = "下降抓取 (PCA朝向 + IK多姿态)"
  ```
- **保留 IK `_combined_error` 的 yaw 约束（L149-169）**：现在约束 PCA 朝向（X 轴对齐短轴），逻辑不变，仅注释更新（"GraspNet/短轴"→"PCA 短轴"）。success 仍按位置 + Z 轴判定，yaw 为软约束。

#### 2.4 保留安全防线（不动）
- IK `final_ze < 0.52`（拒绝朝上解）— L226
- FK 轨迹终点 Z 轴检查（z 分量 > -0.5 拒绝）— L1160-1162
- position_only_fallback 45° 倾斜约束 + 自由偏航 — L1071-1108
- 这些是 GraspNet 移除后的最终安全防线，**必须保留**

#### 2.5 清理日志/标签
- L1525-1531 mode_label：移除 `[GraspNet 抓取位姿]`，保留 `[两阶段精定位]` / 空
- L1710 cycle_profiles：移除 `'graspnet_ik'`，统一用 `'pca_ik'` / `'ik_solution'`

---

### 文件 3：`/home/lxf/agx_arm_ws/sorting_gui_client.py`

#### 3.1 移除 GraspNet 开关与 UI
- **L234-239**：删除 `self.use_graspnet` BooleanVar；`latest_detection_stamp` 保留（无 GraspNet 也可缓存，无害，且随动可能用）— 或一并删除以保持整洁（推荐删除，因不再有时间同步需求）
- **L360-366**：删除 GraspNet checkbox UI（`tk.Checkbutton(...use_graspnet...)` + `graspnet_status_var` Label）
- **L940-948**：删除 `_on_graspnet_toggle` 方法
- **L359**：按钮说明文案 "两阶段精定位 / GraspNet 抓取" → "两阶段精定位 (D455双目+短轴对齐)"

#### 3.2 简化按钮逻辑
- **`_pick_from_detection_two_stage`（L613-664）**：删除 `use_gn` 分支，统一发 sort_verify：
  ```python
  txt=(f" 两阶段精定位夹取确认\n\n观察位检测坐标:\n  ({pp['x']:.3f}, {pp['y']:.3f}, {pp['z']:.3f})\n\n"
       f"-> 机械臂移动到物体正上方 (40cm)\n-> D455 近距离二次检测\n"
       f"-> PCA 短轴对齐生成夹爪朝向\n-> 以二次结果为准直接抓取\n\n"
       f"Place: 料框{bin_num} (关节空间预设点位)\n{dia_str}\n确定发送？")
  if not messagebox.askyesno("确认两阶段精定位夹取",txt): return
  cmd={"cmd":"sort_verify","pick":pp,"bin":bin_num,
       "pick_name":f"{d.get('object_name','物体')} (two-stage)",
       "place_name":f"料框{bin_num}"}
  ```
- **`_dispatch_sort_cmd`（L903-938）**：删除 `use_gn` 分支，统一发 sort_verify：
  ```python
  cmd = {"cmd": "sort_verify", "pick": pick, "bin": bin_num,
         "pick_name": f"{name} (auto-sort{'-retry' if retry else ''})",
         "place_name": f"料框{bin_num}"}
  method_tag = "two-stage"
  ```
- **L489-495**：latest_detection_stamp 缓存逻辑 — 若删除了该变量则一并删除；保留也无害（推荐删除以彻底清理 GraspNet 痕迹）

---

## 四、不做的部分（明确边界）
- **不做** RGB-Depth 联合标定（D455 出厂已标定）
- **不做** 随动抓取算法（中期 1-2 月，本次仅留 `velocity_mps=null` 接口占位）
- **不做** 额外多帧深度中值滤波（已有 ObjectTrack EMA 足够 ±5mm）
- **不删** `grasp_pose_node.py` 文件本身（GraspNet service 节点，停止运行即可；删除文件可选，本次不删以保留可回溯性，但确保不被启动）
- **不动** Pinocchio path injection / IK 求解器核心逻辑
- **不动** 碰撞检测禁用/恢复逻辑、料框放置逻辑、自动分拣状态机

---

## 五、假设与决策
1. **camera_info topic 名**：`/camera/camera/color/camera_info`（与现有 image_raw 同命名空间，executor 运行时 `ros2 topic list` 确认）
2. **PCA 在检测节点实现**：因 realsense_yolo_node 已持有深度图 + bbox + robot_pose + 手眼矩阵，无需跨节点通信
3. **两阶段精定位保留**：近距离(40cm)检测精度更高，是 ±5mm 的关键
4. **elongation_ratio_threshold = 1.5**：长短轴特征值比 < 1.5 视为圆物体，用径向朝下（可调参数）
5. **高度带通 ±3cm**：剔除传送带(低)和背景(高)，保留物体表面点（可调）
6. **velocity_mps = null**：随动接口占位，中期实现时由检测节点估算（光流/帧间位移）
7. **sort_graspnet 向后兼容**：旧 GUI/脚本可能仍发此命令，服务端重定向到 sort_verify 而非报错
8. **position_only 兜底保留**：GraspNet 移除后仍需应对工作空间边缘无解情况，45° 倾斜约束 + FK 检查是最终安全防线

---

## 六、验证步骤
1. **内参修复验证**：重启 realsense_yolo_node，日志打印 "camera_info 已更新内参: fx≈384.8"（非硬编码 378.39）
2. **PCA 输出验证**：YOLO 检测物体，`ros2 topic echo /detection_info` 含 `grasp_orientation`(非 null) + `elongation_ratio` + `velocity_mps: null`
3. **细长物体测试**（香蕉/黄瓜）：日志 "📐 下降抓取使用 PCA 短轴对齐姿态"，视觉确认夹爪夹住窄腰（非长边）
4. **圆形物体测试**（苹果/橘子）：日志 "物体近圆形"，用径向朝下，无短轴对齐
5. **精度测试**：物体上方放置标定纸，夹爪中心对齐物体中心，目视 ±5mm 内
6. **安全测试**：抓取全过程无向上/水平姿态；工作空间边缘物体 IK 失败而非朝上解
7. **自动分拣回归**：自动分拣流程正常，全程无 GraspNet service 调用，无报错
8. **回归测试**：单目回退路径（遮挡 D455）仍可用（精度下降但功能不崩）

---

## 七、实施顺序（建议）
| 步骤 | 文件 | 内容 | 风险 |
|------|------|------|------|
| 1 | realsense_yolo_node.py | 1.1 camera_info 订阅 | 低（仅新增内参源） |
| 2 | realsense_yolo_node.py | 1.2-1.4 PCA + 发布 | 中（新算法，需调试） |
| 3 | auto_sorting_action.py | 2.1-2.2 移除 GraspNet + 简化分发 | 中（大量删除，需保证不破坏 sort_verify） |
| 4 | auto_sorting_action.py | 2.3 PCA 朝向接入 | 低（接入点明确） |
| 5 | sorting_gui_client.py | 3.1-3.2 移除开关 + 简化按钮 | 低 |
| 6 | 全部 | 重启三节点，按验证步骤测试 | — |

> 每步完成后做语法检查（`python3 -c "import ast; ast.parse(open('文件').read())"`），确保无语法错误再进入下一步。
