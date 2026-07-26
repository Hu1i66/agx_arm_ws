# Checklist

- [x] `active_ori` 计算代码已从第二步中移除，前移到第一步之前 (L1764-1779)
- [x] 第一步 `move_arm_pose(POSE_PICK_UP, ...)` 传入 `preferred_orientation=active_ori` (L1784-1786)
- [x] 第二步 `execute_cartesian_path` 使用与第一步相同的 `active_ori` (L1842 → L1845)
- [x] 第一步的回退笛卡尔规划 (`move_arm_cartesian`) 也传入相同的 `active_ori` (move_arm_pose L1008-1013 自动传递)
- [x] sort 命令 (PCA + 径向朝下) 预规划 `active_ori` (L1769-1779)
- [x] sort_graspnet 命令预规划 `active_ori = graspnet_quat` (L1767)
- [x] Python 语法检查通过
