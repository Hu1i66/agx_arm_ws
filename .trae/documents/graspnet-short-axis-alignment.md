# GraspNet 抓取方向对齐物体短轴

## Context（为什么做这个改动）

用户需求：**对于有长边/短边的物体（如香蕉、黄瓜、长条形物），夹爪应夹住物体较短的方向**，即夹爪开合方向（X 轴）对齐物体短轴，手指从两条长边合拢夹住窄腰——这是细长物体最稳定的抓取方式。

GraspNet 每个候选的旋转矩阵三列含义（见 [grasp_pose_node.py:616-617](file:///home/lxf/orange_dataset/grasp_pose_node.py#L616-L617)）：
- `R[:,2]` (Z) = 接近方向 approach（顶抓 ≈ [0,0,−1]）
- `R[:,0]` (X) = **夹爪开合方向 = 夹持方向**
- `R[:,1]` (Y) = 手指长度方向

当前实现有两个障碍导致无法满足需求：
1. **GraspNet 自选 yaw**：按学到的模型挑开合方向，不保证对齐短轴。
2. **IK 与 MoveIt 都忽略 yaw**：IK 求解器 [`_combined_error`](file:///home/lxf/agx_arm_ws/auto_sorting_action.py#L149-L158) 仅约束位置+Z 轴方向；MoveIt 姿态约束 `ori_tolerances=(0.2,0.2,π)` 的 Z 容差=π。**即使 GraspNet 返回正确 yaw 也会被丢弃**，下降抓取时夹爪朝向是任意的。

预期结果：GraspNet 返回的 best 候选 X 轴对齐物体短轴，且该方向能一路传递到下降抓取的最终位姿。

## 方案：三处协同修改

### 修改 1 — `grasp_pose_node.py`：PCA 计算短轴 + 重写候选旋转矩阵

**文件**：`/home/lxf/orange_dataset/grasp_pose_node.py`（用 Shell 编辑，不在 workspace 内）

新增一个方法 `_compute_short_axis_base(pcd_crop, T_robot_pose)`：
- 把裁剪点云（相机系）用 `T_base = T_robot_pose @ self.T_cam_to_ee` 变换到 base 系（Open3D `pcd.transform(T)`）
- 按 base 系 z 做 **高度带通滤波**：保留 `|z - median_z| < 0.03` 的点，剔除传送带平面，只留物体
- 投影到 XY 平面，做 2D PCA：`np.linalg.eigh(np.cov(pts_xy.T))`
  - 最大特征值对应特征向量 = 长轴 `long_xy`
  - 垂直方向 = 短轴 `short_xy = [-long_y, long_x, 0]`
- 返回 `(short_axis_3d, long_axis_3d, elongation_ratio)`，其中 `elongation_ratio = eigval_max / eigval_min`

在 [`_handle_grasp_request`](file:///home/lxf/orange_dataset/grasp_pose_node.py#L713) 中，第 5 步坐标变换之后、第 6 步构造 Response 之前插入：
- 调 `_compute_short_axis_base(pcd_crop, T_robot_pose)` 拿到短轴
- 若 `elongation_ratio >= self.elongation_ratio_threshold`（默认 1.5，物体明显细长）且 `self.align_gripper_to_short_axis=True`：对每个 `candidates_base` 候选重写旋转矩阵：
  - `Z = approach_base`（保留 GraspNet 接近方向，已归一化）
  - `X = short_axis - (short_axis·Z)*Z`，归一化（投影到 ⊥Z 平面）
  - `Y = Z × X`，归一化
  - `R_new = np.column_stack([X, Y, Z])`
  - 用 `R_new` 替换 `c["rotation_matrix"]`，`c["approach"]` 保持 Z
- 否则（圆物体或开关关闭）：保持 GraspNet 原姿态不变

新增参数（`__init__` 中 declare_parameter）：
- `align_gripper_to_short_axis`（bool，默认 True）
- `elongation_ratio_threshold`（float，默认 1.5）

诊断日志：打印长轴/短轴方向、elongation_ratio、是否触发重写。

### 修改 2 — `auto_sorting_action.py`：IK 求解器加 yaw 约束

**文件**：`/home/lxf/agx_arm_ws/auto_sorting_action.py`

修改 [`PinocchioIKSolver.get_ik_solution`](file:///home/lxf/agx_arm_ws/auto_sorting_action.py#L124-L219) 的 `_combined_error`（L149-158）：
```python
def _combined_error(q_in):
    _pin.forwardKinematics(self.model, self.data, q_in)
    _pin.updateFramePlacements(self.model, self.data)
    pe = float(np.linalg.norm(
        self.data.oMf[self.ee_frame_id].translation - target_pos_arr))
    R_act = self.data.oMf[self.ee_frame_id].rotation
    # Z 轴方向误差 (approach)
    actual_z = R_act[:, 2]
    cos_angle = float(np.clip(np.dot(actual_z, target_z), -1.0, 1.0))
    ze = float(np.arccos(cos_angle))
    # yaw 误差: 比较夹爪开合方向 X 轴, abs() 容许夹爪 180° 对称
    actual_x = R_act[:, 0]
    target_x = target_se3.rotation[:, 0]
    cos_yaw = float(np.clip(abs(np.dot(actual_x, target_x)), -1.0, 1.0))
    yaw_err = float(np.arccos(cos_yaw))
    return pe + 0.06 * ze + 0.05 * yaw_err, pe, ze
```
- Phase 1/Phase 2 的早停条件（L183, L205）增加 `yaw_err < 0.25`（~14°）
- 成功判定 `success = final_pe < 0.015` **保持不变**（位置为准，yaw 作为软约束引导，避免因 yaw 不可达导致 IK 失败抓取失败）
- 函数返回元组不变（不影响调用方）

效果：IK 在多 yaw 候选中偏好能匹配 GraspNet 短轴方向的那个；`move_arm_via_ik` 已把 GraspNet 候选 orientation 放在候选列表首位（[`_build_pick_orientations_with_graspnet`](file:///home/lxf/agx_arm_ws/auto_sorting_action.py#L745)），yaw 约束使其被真正采用。

### 修改 3 — `auto_sorting_action.py`：下降抓取用 GraspNet 姿态

**文件**：`/home/lxf/agx_arm_ws/auto_sorting_action.py`，下降抓取段 L1623-1626

当前：`active_ori = getattr(node, 'last_successful_orientation', pick_fallback_ori)`（来自 PICK_UP 移动，yaw 任意）。

改为：GraspNet 成功时用 best 候选姿态作为下降目标朝向：
```python
if graspnet_ok and graspnet_candidates:
    bq = graspnet_candidates[0].grasp_pose.pose.orientation
    active_ori = [bq.x, bq.y, bq.z, bq.w]
    print(f"📐 下降抓取使用 GraspNet 短轴对齐姿态")
else:
    active_ori = getattr(node, 'last_successful_orientation', pick_fallback_ori)
```
- 笛卡尔直线下降（首选路径）：`pose_pick_msg` 用 `active_ori`，路径从 PICK_UP 插值到 PICK，最终 yaw = 短轴对齐 ✓
- IK 备用路径（L1641）：`_build_pick_orientations_with_graspnet` 已把 GraspNet 候选放首位 + 修改 2 的 yaw 约束 → 同样对齐短轴 ✓

PICK_UP 移动（L1602）不改：其 yaw 任意，笛卡尔下降会插值旋转到 GraspNet yaw（13cm 短距内小角度旋转，`max_step=0.005` 足够顺滑）。

## 关键复用点
- 点云变换：Open3D `pcd.transform(T)`，`T_base = T_robot_pose @ self.T_cam_to_ee`（与 [`_transform_grasp_to_base`](file:///home/lxf/orange_dataset/grasp_pose_node.py#L654) 同一变换链）
- 旋转矩阵→四元数：[`_rotmat_to_quat`](file:///home/lxf/orange_dataset/grasp_pose_node.py#L989)（已存在，Response 构造已用）
- GraspNet 候选插首位：[`_build_pick_orientations_with_graspnet`](file:///home/lxf/agx_arm_ws/auto_sorting_action.py#L745)（已存在）
- IK 求解入口：[`move_arm_via_ik`](file:///home/lxf/agx_arm_ws/auto_sorting_action.py#L920) → `get_ik_solution`

## 验证

1. **重启节点**：`grasp_pose_node.py`（PID 777157 需 kill 后重启）、`auto_sorting_action.py`（PID 910391 需 kill 后重启）
2. **细长物体测试**：放一根香蕉/黄瓜在传送带，GUI 点"两阶段精定位夹取"
   - 看 `/tmp/grasp_pose_node.log`：应有 `elongation_ratio=X.XX`（应 >1.5）和 `短轴对齐已触发` 日志
   - 看终端：应有 `📐 下降抓取使用 GraspNet 短轴对齐姿态`
   - **目视**：夹爪下降后手指应跨在物体的两条长边上（夹窄腰），而非夹长端
3. **圆物体回归**：放苹果/橙子，确认 `elongation_ratio < 1.5`，姿态保持 GraspNet 原始（不重写），抓取正常
4. **IK 日志**：`IK 求解成功: error=...`，error 应包含 yaw 贡献但 success 仍按位置判定，不出现"IK 无解"
5. **RViz**：`/grasp_pose_marker` 箭头（approach 方向）不变，但配合姿态可观察 X 轴朝向

## 风险与回退
- 若 IK 因 yaw 约束成功率下降：把 `_combined_error` 中 `0.05 * yaw_err` 权重调小至 0.03，或把早停 `yaw_err < 0.25` 放宽到 0.35
- 若 PCA 短轴方向不稳（物体过小/点云太稀）：调高 `elongation_ratio_threshold` 到 2.0，或设 `align_gripper_to_short_axis:=false` 完全关闭退回原行为
- 修改 2 仅是软约束，最坏情况退化为原行为（位置优先），不会让抓取彻底失败
