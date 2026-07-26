# 自动化分拣功能实现计划

## Context

用户需要在 GUI 控制面板中添加"自动分拣"功能，实现传送带流水线上的物体自动分拣。物体在传送带上从右向左运动，画面中间(u=320)设分界线，当物体跨越分界线时停止传送带进行分拣。苹果/草莓/橘子→料框1，其他→料框2。失败重试一次再跳过。

## 修改文件

**唯一修改文件**: `/home/lxf/agx_arm_ws/sorting_gui_client.py`

参考文件（不修改）:
- `auto_sorting_action.py` — 服务端 sort_verify 命令处理，成功/失败后都自动回观察位
- `realsense_yolo_node.py` — 检测数据格式参考

## 状态机设计

服务端 sort_verify 成功（L1398 第九步回观察位）和失败（L1437 自动回观察位）后都自动回到观察位，所以不需要额外的 OBSERVE_RETURN 状态。

```
IDLE ──[开始]──> CHECK_LEFT
CHECK_LEFT ──[有左侧物体]──> SORTING (发 sort_verify)
CHECK_LEFT ──[无左侧物体]──> CONVEYOR_WAIT (开传送带)
CONVEYOR_WAIT ──[检测到 x1≤320]──> CONVEYOR_STOPPING (停传送带)
CONVEYOR_STOPPING ──[传送带已停]──> SORTING (发 sort_verify)
SORTING ──[idle, 无 error]──> CHECK_LEFT (成功, 臂已在观察位)
SORTING ──[error→idle, retry<1]──> SORTING (重发 sort_verify, retry=1)
SORTING ──[error→idle, retry≥1]──> CHECK_LEFT (跳过)
[停止]──> STOPPING ──[idle]──> IDLE
```

**关键状态变量**:
- `auto_sort_running` — 是否运行中
- `auto_sort_state` — 当前状态
- `_auto_sort_current_obj` — 当前分拣物体（用于重试）
- `_auto_sort_retry_count` — 重试计数（0=首次, 1=重试）
- `_auto_error_seen` — 在 SORTING 中是否看到 error（等待 idle 后决定重试/跳过）
- `_auto_sort_stop_requested` — 停止请求标志
- `_auto_sort_line_u` — 分界线位置（320）

## 实现步骤

### 1. __init__ 中添加状态变量（在 setup_ui() 之前）

```python
self.auto_sort_running = False
self.auto_sort_state = 'IDLE'
self._auto_sort_current_obj = None
self._auto_sort_retry_count = 0
self._auto_error_seen = False
self._auto_sort_stop_requested = False
self._auto_sort_line_u = 320
self._auto_sort_tick_id = None
self._conv_stop_retry = 0
```

### 2. _ui_control 中添加自动分拣 UI（在传送带控制之后、观察/复位/退出之前，L388 之后）

新增 LabelFrame "自动分拣"，包含：
- 状态标签 `auto_sort_state_var`（显示当前状态）
- "开始自动分拣" 按钮（紫色 `#9C27B0`）
- "停止自动分拣" 按钮（橙色 `#FF5722`，初始禁用）
- 说明文字：分界线规则和分拣规则

### 3. 新增方法（全部在 SortingApp 类内）

**核心逻辑**:
- `_pick_object_left_of_line()` — 筛选 x1≤320 且有 base_position_m 的物体，返回 center_u 最大的
- `_bin_for_object(name)` — apple/green apple/strawberry/orange→1, lemon/honey peach/pear→2
- `_dispatch_sort_cmd(obj)` — 构造 sort_verify 命令并发送（复用 _pick_from_detection_two_stage 的命令格式）

**状态机**:
- `start_auto_sort()` — 入口：禁用手动按钮，启动 tick
- `stop_auto_sort()` — 入口：停传送带，设置停止标志
- `_auto_sort_tick()` — 300ms 轮询，按状态分发
- `_tick_check_left()` — 检查左侧物体，有则分拣，无则开传送带
- `_tick_conveyor_wait()` — 等待物体越线
- `_tick_conveyor_stopping()` — 等传送带停止后分拣
- `_tick_sorting()` — 等 idle（处理 error 重试/跳过）
- `_tick_stopping()` — 停止流程
- `_finish_auto_sort()` — 恢复按钮，停止 tick
- `_set_manual_buttons_state(state)` — 批量禁用/启用手动按钮
- `_auto_sort_log(msg)` — 日志到状态标签和文件
- `_draw_dividing_line()` — 在相机画布画黄色虚线

### 4. _tick_sorting 的 error 处理（关键逻辑）

服务端 error 后会自动恢复并回到 idle。所以：
- 看到 'error' → 设置 `_auto_error_seen = True`，继续等待
- 看到 'idle' 且 `_auto_error_seen`:
  - retry_count == 0 → 重发 sort_verify, retry_count = 1
  - retry_count >= 1 → 跳过，回 CHECK_LEFT
- 看到 'idle' 且无 error → 成功，回 CHECK_LEFT

### 5. 修改现有方法

**_update_status_loop (L772-776)** — 自动分拣运行时抑制 error messagebox:
```python
if self.auto_sort_running:
    pass  # 状态机自行处理 error
elif not getattr(self, 'error_notified', False):
    self.error_notified = True; self.clear_queue()
    messagebox.showwarning(...)
```

**_conveyor_done (L739-741)** — 自动分拣运行时保持按钮禁用:
```python
if not self.auto_sort_running:
    self.conveyor_on_btn.config(state=tk.NORMAL)
    self.conveyor_off_btn.config(state=tk.NORMAL)
```
失败时也抑制 messagebox（自动分拣运行时）。

**_update_camera_loop** — 显示/隐藏分界线:
```python
if hasattr(self, '_div_line_id'):
    state = 'normal' if self.auto_sort_running else 'hidden'
    self.camera_canvas.itemconfig(self._div_line_id, state=state)
```

### 6. 分界线可视化

在 `__init__` 末尾调用 `_draw_dividing_line()`（创建后隐藏）。画布 400x300，分界线在 x=200（320/640 * 400 = 200），黄色虚线 + 标签。

### 7. _dispatch_sort_cmd 命令格式

复用 `_pick_from_detection_two_stage` (L580) 的格式：
```python
cmd = {
    "cmd": "sort_verify",
    "pick": {"x":.., "y":.., "z":.., "qx":.., "qy":.., "qz":.., "qw":..},
    "bin": bin_num,  # 由 _bin_for_object 决定
    "pick_name": f"{name} (auto-sort)",
    "place_name": f"料框{bin_num}",
    "object_diameter_m": dia  # if available
}
```
四元数来自 `euler_to_quaternion(default_grasp_roll, pitch, yaw)`。

## 验证方法

1. 启动 GUI: `python3 /home/lxf/agx_arm_ws/sorting_gui_client.py`
2. 确保动作服务端已运行: `python3 /home/lxf/agx_arm_ws/auto_sorting_action.py`
3. 点击"开始自动分拣"，确认：
   - 相机画面出现黄色分界线
   - 手动按钮（传送带、夹取等）被禁用
   - 状态标签显示当前状态
4. 放物体在传送带右侧，确认：
   - 传送带自动开启
   - 物体跨越分界线时传送带停止
   - 机械臂执行两阶段分拣
   - 分拣完成后回到观察位，循环继续
5. 测试停止按钮：点击后传送带停止，等当前分拣完成后恢复 IDLE
6. 测试失败重试：故意让分拣失败，确认重试一次后跳过
