# Tasks

- [ ] Task 1: action 节点添加中断订阅和 `_check_emergency` 方法
  - `__init__` 中订阅 `/emergency_stop` topic，回调设置 `self._emergency_flag` 和 `req_type`
  - 实现 `_check_emergency()`: 检查 flag → 若为真则执行安全停机 → 返回 True(需中断)
  - 安全停机: `_cancel_moveit_goal()` → `operate_gripper(OPEN)` → `move_arm_joint(STANDBY)` → `_set_bin_collision(False)` → 日志记录

- [ ] Task 2: 在分拣序列中插入 ~10 个检查点
  - 步骤0(张开夹爪)后、步骤1(移动到上方)后、步骤2(下降抓取)后、步骤3(闭合夹爪)后
  - 步骤4(抬起脱离)后、步骤4.5(抓取检测)后
  - 重试循环内(回观察位后、移动到新上方后)
  - 步骤5-9(放置流程)各步之间
  - 每个检查点: `if node._check_emergency(): success=False; break` 模式

- [ ] Task 3: GUI 添加急停按钮
  - 红色大按钮 (`bg="#FF0000"`, `fg="white"`, `font=("Arial",14,"bold")`)，放在 GUI 控制按钮区域顶部
  - 点击 → 弹确认框 → 发 `{"req":"estop"}` 到 `/emergency_stop`
  - 发"返回待机位"命令: 弹确认框 → 发 `{"req":"standby"}` 到 `/emergency_stop`

- [ ] Task 4: 语法检查 + 功能验证
  - Python 语法检查
  - 验证急停按钮在 GUI 中醒目可见
  - 验证行动节点订阅正确

# Task Dependencies
- Task 2 depends on Task 1
- Task 3 is independent
- Task 4 depends on Task 1, 2, 3
