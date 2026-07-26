# Checklist

- [x] `/emergency_stop` topic 订阅正确设置在 action 节点 `__init__` 中
- [x] `_check_emergency()` 方法正确实现安全停机流程
- [x] estop: cancel goal → open gripper → 停在原地 (保持锁定)
- [x] standby: cancel goal → open gripper → move standby → restore collision → clear flag
- [x] reset: move standby → restore collision → clear flag
- [x] 分拣序列中至少 8 个检查点覆盖所有关键步骤 (实际13个)
- [x] 重试循环内有检查点 (避免重试时无法中断)
- [x] 急停按钮在 GUI 中红色醒目、位于控制区域顶部
- [x] 急停/返回待机位/复位按钮按下即触发 (无确认弹窗)
- [x] 急停后机械臂停在原地, 不清除标志, 等待复位
- [x] 复位按钮清除急停锁, 回待机位
- [x] 急停触发后日志记录时间和当前步骤
- [x] "返回待机位"按钮在 busy 状态下也可触发中断
- [x] Python 语法检查通过 (action + GUI 两个文件)
