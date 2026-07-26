# Tasks

- [x] Task 1: GRIPPER_SETTLE_SEC 0.15→0.05，planning_scene sleep 0.15→0.05
  - [x] L1618: `GRIPPER_SETTLE_SEC = 0.05`
  - [x] L2023: `time.sleep(0.05)  # planning_scene 更新`
  - [x] 语法检查

- [x] Task 2: step4 抬起改为直接去 BIN_ABOVE
  - [x] step4 的 `move_arm_joint` 改为 `move_arm_joint(BIN_ABOVE_JOINTS, ...)`
  - [x] 去掉 step5（BIN_ABOVE 移动），step4 已直接到达
  - [x] 删除 step5 的 emergency 检查点
  - [x] `_set_bin_collision_allowed(allowed=True)` 在 step4 之前调用
  - [x] 语法检查

- [x] Task 3: 所有途径点步骤改为 continuous=True
  - [x] L1916: step1 `continuous=True`
  - [x] L2089: step5 BIN_PLACE `continuous=True`
  - [x] L2102: step7 BIN_ABOVE `continuous=True`
  - [x] L2108: step8 STANDBY `continuous=True`
  - [x] 语法检查

- [ ] Task 4: 验证（真机）
  - [ ] 单次分拣计时，确认周期 ≤ 13s
  - [ ] 连续 5 次分拣，确认无 -4/-2 错误
  - [ ] 确认 step4→BIN_ABOVE→BIN_PLACE 流程无碰撞
