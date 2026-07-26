# 下降抓取 joint6 姿态预规划 Spec

## Why
当前到达抓取点上方 (第一步) 和下降抓取 (第二步) 使用不同的 target orientation，导致 `execute_cartesian_path` 在 13cm 下降过程中插补旋转，joint6 非预期转动 → 夹爪戳刺/碰撞 → 抓取失败。

## What Changes
- 将 `active_ori` 计算前移到第一步之前
- 第一步 `move_arm_pose(POSE_PICK_UP)` 传入 `preferred_orientation=active_ori`
- 第二步 `execute_cartesian_path` 使用相同 `active_ori`，下降时姿态一致 → joint6 零旋转

## Impact
- Affected specs: 无
- Affected code: `auto_sorting_action.py` 抓取流程 (第一步 + 第二步 区域)

## MODIFIED Requirements

### Requirement: 下降抓取时 joint6 姿态预对齐
系统 SHALL 在移动到抓取点上方 (第一步) 时即完成末端姿态对齐，确保下降过程 (第二步) 中末端姿态不变、joint6 不产生额外旋转。

#### Scenario: 正常下降 (联合分拣 sort)
- **GIVEN** 使用 `sort` 命令 (非 GraspNet) 执行分拣
- **WHEN** 系统计算 PCA 短轴对齐姿态 `active_ori`
- **THEN** 第一步移动到 `POSE_PICK_UP` 时传入 `active_ori` 作为 `preferred_orientation`
- **AND** 第二步 `execute_cartesian_path` 使用相同 `active_ori`
- **AND** joint6 在下降过程中不产生显著旋转

#### Scenario: GraspNet 下降 (sort_graspnet)
- **GIVEN** 使用 `sort_graspnet` 命令执行分拣
- **WHEN** 系统从 GraspNet 结果获取 `graspnet_quat`
- **THEN** 第一步传入 `graspnet_quat` 作为 `preferred_orientation`
- **AND** 第二步使用相同 `graspnet_quat`
- **AND** joint6 在下降过程中不产生显著旋转

#### Scenario: IK 优先回退
- **GIVEN** 指定了 `preferred_orientation` 传给 `move_arm_pose`
- **WHEN** IK 求解成功
- **THEN** 到达 `POSE_PICK_UP` 时末端姿态与 `active_ori` 一致
- **WHEN** IK 求解失败
- **THEN** 回退到 `move_arm_cartesian` 并传入相同 `preferred_orientation`，确保姿态仍一致
