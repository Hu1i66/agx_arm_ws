# 急停 + 返回待机位安全机制 Spec

## Why
当前系统在分拣执行期间处于同步阻塞循环，无法响应任何中断命令。一旦机械臂出现异常运动（如 joint6 旋转戳刺），用户没有任何方式立即停止，存在安全隐患。

## What Changes
- 新增独立 `/emergency_stop` topic，使 action 节点在分拣循环内能接收中断信号
- 在分拣序列的 ~10 个检查点插入 `_check_emergency()` 调用
- GUI 新增红色急停按钮 + 增强现有"返回待机位"按钮走同机制
- 安全停机流程：取消轨迹 → 张开夹爪 → 回待机位 → 恢复碰撞 → 通知 GUI

## Impact
- Affected specs: 无
- Affected code: `auto_sorting_action.py`, `sorting_gui_client.py`

## ADDED Requirements

### Requirement: 急停按钮
系统 SHALL 在 GUI 中提供急停按钮，用户按下后立即中断机械臂所有运动并回到安全状态。

#### Scenario: 分拣过程中触发急停
- **GIVEN** 机械臂正在执行 sort 序列 (步骤0-9中任意步骤)
- **WHEN** 用户点击 GUI 中红色急停按钮并确认
- **THEN** 系统在下一个检查点(<3s)检测到急停标志
- **AND** 取消当前 MoveIt 轨迹
- **AND** 张开夹爪
- **AND** 移动机械臂回待机位
- **AND** 恢复所有碰撞检测
- **AND** GUI 状态恢复为 idle
- **AND** 记录急停时间和当前步骤到日志

#### Scenario: 空闲时触发急停
- **GIVEN** 机械臂处于空闲状态
- **WHEN** 用户点击急停按钮
- **THEN** 系统无操作，保持 idle 状态

### Requirement: 返回待机位按钮（增强）
现有的"返回待机位"按钮 SHALL 在分拣执行期间也可用，通过同机制中断当前序列。

#### Scenario: 分拣过程中返回待机位
- **GIVEN** 机械臂正在执行 sort 序列
- **WHEN** 用户点击"返回待机位"按钮并确认
- **THEN** 与急停相同的安全停机流程执行
- **AND** 日志标记为 standby 而非 estop

### Requirement: 独立中断通道
系统 SHALL 使用独立于 `/sorting_cmds` 的 ROS topic `/emergency_stop` 传递中断信号，确保分拣循环内可接收。

### Requirement: 错误检测点
分拣序列 SHALL 在每一步之间调用 `_check_emergency()` 检测中断标志。共约 10 个检查点覆盖完整序列(含重试循环和放置流程)。

### Requirement: 安全确认
急停和返回待机位按钮 SHALL 在触发前显示确认弹窗，防止误操作。
