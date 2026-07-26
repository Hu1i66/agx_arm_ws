# 自动分拣功能 — 最终修复计划

## 背景与当前状态

用户要求在 GUI 上增加"自动化分拣"按钮，按下后机械臂自动按规则分拣传送带上的物体：
- 在画面中间画一条分界线 (u=320)，先检测分界线左侧物体，有则直接分拣，无则开传送带
- 当物体跨越分界线时停传送带并分拣，分拣完回观察位，循环往复
- 苹果/草莓/橘子→料框1，柠檬/桃/梨→料框2
- 失败重试一次再跳过；越线判定=左边缘 x1≤320；多物体选 center_u 最大者

经多轮实现，自动分拣功能已 99% 完成（`sorting_gui_client.py`）：
- L243-252: 状态机变量 (`auto_sort_running`, `auto_sort_state`, 等)
- L257: `__init__` 末尾调用 `_draw_dividing_line()`
- L400-412: 自动分拣 UI（开始/停止按钮 + 状态标签 + 说明文字）
- L475-478: `_update_detection_loop` 中自动分拣运行时保持夹取按钮禁用
- L764-785: `_conveyor_done` 中保持传送带按钮禁用 + 抑制失败 messagebox
- L803-809: `_update_status_loop` 中 error 状态抑制 messagebox
- L812-1058: 15 个自动分拣方法 (`_auto_sort_log`, `_bin_for_object`, `_pick_object_left_of_line`, `_dispatch_sort_cmd`, `_set_manual_buttons_state`, `_draw_dividing_line`, `start_auto_sort`, `stop_auto_sort`, `_finish_auto_sort`, `_auto_sort_tick`, `_tick_check_left`, `_tick_conveyor_wait`, `_tick_conveyor_stopping`, `_tick_sorting`, `_tick_stopping`)
- 语法验证（AST + 字节码）已通过

服务端 `auto_sorting_action.py` 已支持 `sort_verify` 两阶段精定位命令 (L1225-1226)，error 后自动回观察位并发布 idle (L1419-1443)，GUI 状态机期望的 `error → idle` 转换正确。

## 剩余唯一修复

### 问题
`sorting_gui_client.py` L795 的队列处理逻辑会干扰自动分拣状态机：

```python
# 当前代码 (L792-801):
if s=='idle':
    self.status_var.set("当前服务器状态: 空闲 (Idle)"); self.status_label.configure(fg="green")
    self.error_notified=False
    if getattr(self,'queue_running',False):  # ← 问题点
        if self.task_queue and (time.time()-self.last_dispatch_time>1.5):
            self.cmd_queue.put(self.task_queue.pop(0)); self.refresh_queue_listbox()
            self.last_dispatch_time=time.time()
        elif not self.task_queue and (time.time()-self.last_dispatch_time>1.5):
            self.queue_running=False; self.qrun.config(text=" 开始执行排队任务",bg="lightgreen")
            messagebox.showinfo("完成","排队队列所有任务已执行完毕！")
```

**冲突场景**：用户启动自动分拣前可能已启用排队执行 (`queue_running=True`)。自动分拣运行时，`_tick_sorting` 在 `idle` 状态下完成一次分拣后切回 `CHECK_LEFT`，期间 `_update_status_loop` 每 500ms 也会检测到 `idle + queue_running=True`，于是弹出排队任务到 `cmd_queue`，与服务端正在处理的下一个 sort_verify 冲突，导致机械臂收到非预期指令。

### 修复
在 L795 的 `if getattr(self,'queue_running',False):` 前添加 `not self.auto_sort_running and` 守卫：

```python
# 修复后:
if s=='idle':
    self.status_var.set("当前服务器状态: 空闲 (Idle)"); self.status_label.configure(fg="green")
    self.error_notified=False
    if not self.auto_sort_running and getattr(self,'queue_running',False):  # ← 加守卫
        if self.task_queue and (time.time()-self.last_dispatch_time>1.5):
            self.cmd_queue.put(self.task_queue.pop(0)); self.refresh_queue_listbox()
            self.last_dispatch_time=time.time()
        elif not self.task_queue and (time.time()-self.last_dispatch_time>1.5):
            self.queue_running=False; self.qrun.config(text=" 开始执行排队任务",bg="lightgreen")
            messagebox.showinfo("完成","排队队列所有任务已执行完毕！")
```

**理由**：自动分拣运行时，所有分拣指令由状态机通过 `_dispatch_sort_cmd` 直接 `cmd_queue.put()`，不应被排队队列干扰。排队队列在自动分拣结束后会自然恢复执行（`_finish_auto_sort` 中 `_set_manual_buttons_state(tk.NORMAL)` 解锁 qrun 按钮，用户可手动恢复）。

## 文件变更

| 文件 | 位置 | 修改 |
|------|------|------|
| `sorting_gui_client.py` | L795 | `if getattr(self,'queue_running',False):` → `if not self.auto_sort_running and getattr(self,'queue_running',False):` |

## 假设与决策

1. **不修改服务端**：服务端 `auto_sorting_action.py` 已正确处理 `sort_verify`、error 恢复、回观察位，无需改动。
2. **不修改 YOLO 节点**：`realsense_yolo_node.py` 已正确发布 `objects` 数组（含 `bbox_pixel.x1/x2` 和 `base_position_m`），`_pick_object_left_of_line` 直接消费即可。
3. **不弹窗**：自动分拣运行期间，所有 error/传送带失败/排队完成都通过 `_auto_sort_log` 写日志和更新状态标签，不弹 messagebox，避免打断用户。
4. **停止流程**：用户点"停止自动分拣"后，状态机进入 STOPPING，等当前分拣完成回观察位、传送带停止后才完全结束（`_finish_auto_sort`），避免机械臂停在危险位置。
5. **重试策略**：error 后状态机重试一次，仍失败则跳过该物体回 CHECK_LEFT，避免卡死。

## 验证步骤

1. **语法验证**（执行修改后立即做）：
   ```bash
   python3 -c "import ast; ast.parse(open('/home/lxf/agx_arm_ws/sorting_gui_client.py').read())"
   python3 -c "import py_compile; py_compile.compile('/home/lxf/agx_arm_ws/sorting_gui_client.py', doraise=True)"
   ```

2. **方法存在性检查**（确认 15 个方法齐全）：
   ```bash
   python3 -c "
   import ast
   tree = ast.parse(open('/home/lxf/agx_arm_ws/sorting_gui_client.py').read())
   methods = set()
   for node in ast.walk(tree):
       if isinstance(node, ast.FunctionDef):
           methods.add(node.name)
   required = ['_auto_sort_log','_bin_for_object','_pick_object_left_of_line','_dispatch_sort_cmd',
               '_set_manual_buttons_state','_draw_dividing_line','start_auto_sort','stop_auto_sort',
               '_finish_auto_sort','_auto_sort_tick','_tick_check_left','_tick_conveyor_wait',
               '_tick_conveyor_stopping','_tick_sorting','_tick_stopping']
   missing = [m for m in required if m not in methods]
   print('Missing:', missing if missing else 'None ✓')
   "
   ```

3. **运行时测试**（用户手动执行）：
   - 启动服务端：`python3 /home/lxf/agx_arm_ws/auto_sorting_action.py`
   - 启动 GUI：`python3 /home/lxf/agx_arm_ws/sorting_gui_client.py`
   - 点击"开始自动分拣"，观察：
     - 分界线黄色虚线显示在相机画面中间
     - 状态标签从 `[CHECK_LEFT] ...` 开始流转
     - 手动按钮（传送带/夹取/排队）全部禁用
   - 在传送带上放物体，观察自动分拣流程
   - 点击"停止自动分拣"，观察机械臂回观察位后完全停止，按钮恢复
