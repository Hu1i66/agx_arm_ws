# Tasks

- [x] Task 1: 将 `active_ori` 计算前移到第一步之前
   将当前在【第二步】下降抓取中的 `active_ori` 计算逻辑 (判断 use_graspnet / PCA / 径向朝下) 前移到【第零步】和【第一步】之间。计算后 `active_ori` 可在第一步和第二步共用。

- [x] Task 2: 第一步 `move_arm_pose` 传入 `preferred_orientation=active_ori`
   在 `move_arm_pose(POSE_PICK_UP, ...)` 调用中传入 `preferred_orientation=active_ori`，确保到达抓取点上方时末端姿态已对齐。同步修改 `move_arm_pose` 方法签名以支持 `preferred_orientation` 参数。

- [x] Task 3: 第二步去除重复的 `active_ori` 计算
   删除第二步中重复的 `active_ori` 计算代码 (因为已前移到第一步之前)，直接使用已计算的 `active_ori`。简化 IK 回退中的 pca_ori 分支。

# Task Dependencies
- Task 2 depends on Task 1
- Task 3 depends on Task 1
- Task 2 and Task 3 can be done in parallel after Task 1
