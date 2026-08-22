# Task 6 报告: GUI 动态抓取模式 + 下游漏抓分界线

## 状态

DONE

## 提交

- `6117a854` feat: add dynamic grasping mode and downstream miss-line fallback to GUI
  （仅修改并提交 `sorting_gui_client.py`，1 file changed, 180 insertions(+), 18 deletions(-)）

## 验证

- `python3 -m py_compile sorting_gui_client.py` → 通过
- 符号断言脚本 → 输出 `OK: 动态抓取 GUI 符号齐全`

## 实现内容（按 brief 步骤）

- Step 1: 新增状态字段 `dynamic_mode` / `_dynamic_state`（OFF/RUNNING/MISS_STOP/STOPPING）/ `_dynamic_trigger_line_u=200` / `_dynamic_miss_line_u=560` / `_dynamic_busy`，另加 4 个 canvas 对象 ID（`_dyn_trig_line_id/_dyn_trig_label_id/_dyn_miss_line_id/_dyn_miss_label_id`）用于分界线显隐管理。
- Step 2: 在自动分拣 LabelFrame 之后新增"动态抓取"LabelFrame，含紫色 `dyn_start_btn`（#9C27B0）与橙色 `dyn_stop_btn`（#FF9800，初始 DISABLED）。因 brief 中的 `ctrl_frame` 不存在，遵循现有 `tk.LabelFrame + tk.Button` 模式实现。
- Step 3: 新增 `_dynamic_start` / `_dynamic_stop` / `_dynamic_tick`（状态机主循环）/ `_restore_manual_buttons`。
- Step 3b: `_pick_object_right_of_line` 签名改为 `(self, line_u=None, require_base=True)`，内部引用改用 `line_u`，`require_base=False` 时不做 base_position_m 过滤；默认参数保持自动分拣原行为不变。
- Step 4: 在 `_draw_dividing_line`（实际方法名，brief 写 `_draw_auto_sort_line`）中绘制蓝色虚线漏抓线；另按 brief 可选说明补画了绿色虚线上游触发线。两线均创建一次、默认隐藏，动态模式启动/停止时显隐。
- Step 5: 手动按钮禁用/弹窗屏蔽条件由 `auto_sort_running` 扩展为 `(auto_sort_running or dynamic_mode)`，涉及 5 处：`_update_detection_loop`(pick_btn/pick_graspnet_btn 处)、`_update_detection_loop`(pick_blue_block_btn 处)、`_conveyor_done`、`_update_status_loop`(idle 排队任务)、`_update_status_loop`(error 不弹窗)。
- Step 6: py_compile 与符号断言均通过。
- Step 7: 已提交（见上）。

## 与 brief 的偏差（均已用实际代码为准）

1. **`_dispatch_sort_cmd` 采用扩展既有方法而非新增同名方法**：文件中原已存在 `_dispatch_sort_cmd(self, obj, retry=False)`（自动分拣使用）。brief 要求新增 `_dispatch_sort_cmd(obj, static_mode)` 会与之重名冲突。改为在既有方法上追加 `static_mode=True` 参数：
   - `static_mode=True`（默认）→ `cmd: "sort"`（自动分拣与漏抓静态兜底共用，保持原自动分拣行为不变）；
   - `static_mode=False` → `cmd: "sort_dynamic"` 并附带 `cycle_id: "dyn-<ts>"`（动态拦截）。
   - 同时保留既有 `retry` 参数与 `place_name` 字段、`object_diameter_m` 条件写入逻辑（与既有 auto-sort 派发模式一致）。
2. **`_restore_manual_buttons` 复用 `_set_manual_buttons_state`**：brief 中该方法引用 `conveyor_on_btn/conveyor_off_btn/single_btn/verify_btn`，其中 `single_btn`/`verify_btn` 不存在，实际手动按钮集合由既有 `_set_manual_buttons_state` 管理。改为在 `_restore_manual_buttons` 内调用 `_set_manual_buttons_state(tk.NORMAL)`。对应地，`_dynamic_start` 中调用 `_set_manual_buttons_state(tk.DISABLED)` 保持"动态抓取中手动按钮禁用"约定。
3. **状态机补全了 brief 遗漏的"漏抓兜底后重启传送带"逻辑**：brief 的 `_dynamic_tick` 在 MISS_STOP 静态 sort 派发后置 `RUNNING` 但从未重启传送带（与设计说明"等 idle → 重启带 → RUNNING"矛盾），会导致漏抓兜底后传送带永久停止。新增第 3 步：idle 且传送带停稳时自动 `_conveyor_set(True)` 恢复。
4. **进入 MISS_STOP 增加 `st != 'MISS_STOP'` 守卫**：避免停带过程中每 tick 重复命中漏抓分支、重复停带与重复日志。
5. **STOPPING 增加 `if self._conveyor_busy: return`**：修复 brief 中"停带命令在途但 conveyor_state 仍为旧值时会直接完成退出"的竞态。
6. **上游触发派发增加 `self.conveyor_state == True` 条件**：避免在传送带尚未重启完成（兜底恢复窗口期）时对静止带下发 `sort_dynamic`。
7. **漏抓线绘制采用"创建一次 + 显隐切换"**：brief 的 snippet 在 `_draw_dividing_line` 内 `if self.dynamic_mode: create_line(...)`，但该方法仅在初始化时调用一次，动态模式启动时不可能为 True，无法显示。改为在 `_draw_dividing_line` 中创建线对象并默认隐藏，`_dynamic_start` 显示、STOPPING 完成时隐藏。画布变量用实际名 `self.camera_canvas`。
8. **新增绿色上游触发线**：按 brief 可选说明（"检查动态模式下是否也需要绘制上游触发线，可用绿色虚线"）实现，因显隐机制已存在、扩展成本低。

## 备注

- 未修改其他文件；自动分拣的 `_dispatch_sort_cmd` 调用（`retry=False/True`）行为完全保持（默认 `static_mode=True` → `sort`）。
- `_auto_sort_tick` 与 `stop_auto_sort` 中的 `if not self.auto_sort_running: return` 属自动分拣自身守卫，未改动。
- 未新增对"自动分拣与动态抓取同时运行"的互斥校验（brief 未要求，维持最小改动）。
