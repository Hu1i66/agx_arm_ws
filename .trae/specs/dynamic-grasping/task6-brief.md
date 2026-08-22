# Task 6 Brief: GUI 动态抓取模式 + 下游漏抓分界线

> 本文件是 Task 6 的完整任务描述，摘自 `/home/lxf/agx_arm_ws/.trae/specs/dynamic-grasping/plan.md`。工作目录：`/home/lxf/agx_arm_ws`。

## 任务目标

在 GUI（`sorting_gui_client.py`）中新增动态抓取模式：传送带持续运行、机械臂移动拦截物体；并在相机画面下游（右侧）绘制一条可调"漏抓分界线"，物体越过分界线立即停带 → 静态兜底抓取 → 重启带。

## 全局约束（本任务相关，逐字摘自 plan.md）

- 传送带线速度范围 0.02–0.15 m/s，来自 STM32 串口（真实值，无需二次校正），保留滑动窗口滤波
- 动态抓取中所有手动按钮禁用、弹窗屏蔽（沿用现有约定）
- 现有 `sort` / `sort_graspnet` / `sort_verify` / `sort_blue_block` 命令全部原样保留
- 所有代码注释使用中文，与现有代码风格一致
- GUI 有紫色 '开始自动分拣' 和橙色 '停止自动分拣' 按钮的配色约定
- 自动分拣状态机约定：`auto_sort_running` 为 True 时禁用手动按钮、屏蔽 messagebox

## 修改文件

- Modify: `/home/lxf/agx_arm_ws/sorting_gui_client.py`

## 接口（Produces）

- GUI 字段：`self.dynamic_mode`（bool）、`self._dynamic_state`（'OFF'/'RUNNING'/'MISS_STOP'/'STOPPING'）、`self._dynamic_trigger_line_u`（int，默认 200）、`self._dynamic_miss_line_u`（int，默认 560，可调）
- 按钮："动态抓取"（紫色 #9C27B0）与"停止动态抓取"（橙色 #FF9800）
- 方法：`_dynamic_start`、`_dynamic_stop`、`_dynamic_tick`（状态机主循环）、`_dispatch_sort_cmd(obj, static_mode)`、`_restore_manual_buttons()`
- 修改 `_pick_object_right_of_line` 签名支持 `line_u` 参数

## 设计说明

动态模式状态机（GUI 主线程，复用现有 `_dispatch_sort_cmd` 模式）：

- `OFF`：默认。点"动态抓取"→ 若传送带未运行则 `_conveyor_set(True)` → `RUNNING`
- `RUNNING`：传送带持续运行。每轮检查（`status==idle` 时）：
  1. 下游漏抓分界线右侧有物体（`_pick_object_right_of_line` 用 `_dynamic_miss_line_u`）→ `MISS_STOP`
  2. 否则上游触发线右侧有物体（用 `_dynamic_trigger_line_u`）→ 下发 `sort_dynamic`
  3. 否则等待
- `MISS_STOP`：`_conveyor_set(False)` → 等传送带停 → 下发静态 `sort` → 等 `idle` → `_conveyor_set(True)` → `RUNNING`
- `STOPPING`：点"停止动态抓取"→ 若忙则等完成 → `_conveyor_set(False)` → 回待机位 → `OFF`，恢复按钮
- 动态模式下：禁用手动按钮、屏蔽 messagebox（沿用 `auto_sort_running` 逻辑，扩展为 `dynamic_mode or auto_sort_running`）

## 实施步骤

### Step 1: 添加状态字段

在 `self._auto_sort_line_u=260`（第 389 行附近）之后添加：

```python
            # ── 动态抓取模式 (传送带持续运行, 移动拦截) ──
            self.dynamic_mode = False
            self._dynamic_state = 'OFF'      # OFF/RUNNING/MISS_STOP/STOPPING
            self._dynamic_trigger_line_u = 200   # 上游触发线: 物体越线进入跟踪区
            self._dynamic_miss_line_u = 560      # 下游漏抓分界线 (可调): 越线即漏抓
            self._dynamic_busy = False
```

### Step 2: 添加按钮

在自动分拣按钮区域（"开始自动分拣"/"停止自动分拣"附近）添加：

```python
            # ── 动态抓取模式按钮 ──
            dyn_f = tk.Frame(ctrl_frame)
            dyn_f.pack(fill=tk.X, padx=8, pady=4)
            self.dyn_start_btn = tk.Button(
                dyn_f, text=" 动态抓取 ", command=self._dynamic_start,
                bg="#9C27B0", fg="white", font=("Arial", 12, "bold"), width=12)
            self.dyn_start_btn.pack(side=tk.LEFT, padx=4)
            self.dyn_stop_btn = tk.Button(
                dyn_f, text=" 停止动态抓取 ", command=self._dynamic_stop,
                bg="#FF9800", fg="white", font=("Arial", 12, "bold"), width=12,
                state=tk.DISABLED)
            self.dyn_stop_btn.pack(side=tk.LEFT, padx=4)
```

注意：若 GUI 使用 ttk 或自定义 Button 工厂，请遵循现有按钮创建模式，确保 `command`、`state=tk.DISABLED` 语义一致。

### Step 3: 添加状态机方法

在 `_pick_object_right_of_line`（第 1478 行附近）之后添加：

```python
        # ═══════════════════════ 动态抓取模式 ═══════════════════════

        def _dynamic_start(self):
            """启动动态抓取: 开启传送带并进入 RUNNING 状态."""
            if self._dynamic_state != 'OFF':
                return
            self.dynamic_mode = True
            self._dynamic_state = 'RUNNING'
            self.dyn_start_btn.config(state=tk.DISABLED)
            self.dyn_stop_btn.config(state=tk.NORMAL)
            self._auto_sort_log(f"动态抓取启动: 传送带持续运行, 移动拦截物体")
            if self.conveyor_state != True and not self._conveyor_busy:
                self._conveyor_set(True)
            self.after(300, self._dynamic_tick)

        def _dynamic_stop(self):
            """停止动态抓取: 完成当前任务后停带并复位."""
            if self._dynamic_state == 'OFF':
                return
            self._dynamic_state = 'STOPPING'
            self.dyn_stop_btn.config(state=tk.DISABLED)
            self._auto_sort_log("动态抓取停止请求: 完成当前任务后停带")
            self.after(200, self._dynamic_tick)

        def _dynamic_tick(self):
            """动态抓取状态机主循环 (GUI 主线程, 复用 _dispatch_sort_cmd)."""
            if not self.dynamic_mode:
                return
            st = self._dynamic_state

            if st == 'STOPPING':
                if self.current_status == 'busy':
                    self.after(200, self._dynamic_tick)
                    return
                if self.conveyor_state != False and not self._conveyor_busy:
                    self._conveyor_set(False)
                    self.after(200, self._dynamic_tick)
                    return
                self.dynamic_mode = False
                self._dynamic_state = 'OFF'
                self.dyn_start_btn.config(state=tk.NORMAL)
                self.dyn_stop_btn.config(state=tk.DISABLED)
                self._restore_manual_buttons()
                self._auto_sort_log("动态抓取已停止")
                return

            if self.current_status == 'busy':
                # 机械臂忙碌(正在执行 sort_dynamic / sort), 等待完成
                self.after(200, self._dynamic_tick)
                return

            # ── idle: 检查漏抓分界线 (优先) 与上游触发线 ──
            # 1) 下游漏抓: 分界线右侧有物体 → 停带静态兜底
            miss_obj = self._pick_object_right_of_line(
                line_u=self._dynamic_miss_line_u, require_base=True)
            if miss_obj is not None and self.conveyor_state == True:
                self._dynamic_state = 'MISS_STOP'
                self._auto_sort_log(f"漏抓检测: 物体越过下游分界线 u={self._dynamic_miss_line_u}, 停带静态抓取")
                self._conveyor_set(False)
                self.after(300, self._dynamic_tick)
                return

            # 2) 上游触发: 触发线右侧有物体 → 下发 sort_dynamic
            if st == 'MISS_STOP':
                if self.conveyor_state != False:
                    self.after(200, self._dynamic_tick)
                    return
                # 传送带已停: 对下游物体执行静态 sort
                self._dispatch_sort_cmd(miss_obj, static_mode=True)
                self._dynamic_state = 'RUNNING'  # 等待 busy→idle 后重启带
                self.after(200, self._dynamic_tick)
                return

            trig_obj = self._pick_object_right_of_line(
                line_u=self._dynamic_trigger_line_u, require_base=True)
            if trig_obj is not None:
                self._dynamic_state = 'RUNNING'
                self._auto_sort_log(f"动态: 目标 {trig_obj.get('object_name','?')} 进入跟踪区, 下发 sort_dynamic")
                self._dispatch_sort_cmd(trig_obj, static_mode=False)
                self.after(300, self._dynamic_tick)
                return

            self.after(200, self._dynamic_tick)

        def _dispatch_sort_cmd(self, obj, static_mode):
            """下发 sort_dynamic (动态) 或 sort (静态兜底) 命令."""
            d = self.latest_detection or {}
            bin_num = self._bin_for_object(obj.get('object_name', ''))
            diameter = obj.get('estimated_diameter_m') or (obj.get('size_m') or {}).get('diameter', 0.06)
            pick = obj.get('base_position_m') or {}
            if not pick or 'x' not in pick:
                self._auto_sort_log("⚠️ 物体缺少 base_position_m, 跳过")
                return
            cmd = {
                'cmd': 'sort' if static_mode else 'sort_dynamic',
                'pick': {'x': pick['x'], 'y': pick['y'], 'z': pick['z']},
                'pick_name': obj.get('object_name', 'object'),
                'bin': bin_num,
                'object_diameter_m': diameter,
                'cycle_id': f'dyn-{int(time.time())}',
            }
            self.cmd_queue.put(cmd)
            self._dynamic_busy = True
            self._auto_sort_log(f"下发 {'静态兜底' if static_mode else '动态抓取'}: {obj.get('object_name','?')} → 料框{bin_num}")

        def _restore_manual_buttons(self):
            """恢复手动按钮可用状态 (退出动态模式时调用)."""
            for b in (self.conveyor_on_btn, self.conveyor_off_btn,
                      self.single_btn, self.verify_btn):
                try: b.config(state=tk.NORMAL)
                except Exception: pass
```

**重要**：方法名如 `_conveyor_set`、`_bin_for_object`、`_auto_sort_log`、`cmd_queue`、`conveyor_state`、`current_status` 等必须先确认在现有代码中的真实名称，如有差异以现有代码为准，并在报告中说明。

### Step 3b: 改造 `_pick_object_right_of_line` 支持 `line_u` 参数

修改现有 `_pick_object_right_of_line`（第 1472-1479 行附近）签名与内部引用：

```python
        def _pick_object_right_of_line(self, line_u=None, require_base=True):
            """筛选 x2 ≥ 分界线 且有 base_position_m 的物体, 返回 center_u 最大的.

            自动分拣用 self._auto_sort_line_u (默认), 动态模式用传入 line_u.
            """
            line_u = self._auto_sort_line_u if line_u is None else line_u
            d = self.latest_detection
            if not d or not d.get('detected'): return None
```

将该函数后续对 `self._auto_sort_line_u` 的引用改为 `line_u`（如 `if x2 >= self._auto_sort_line_u:` → `if x2 >= line_u:`）。`require_base` 为 True 时，跳过无 `base_position_m` 的物体（即该物体没有 `base_position_m` 或其中无 `x` 键则跳过）。

### Step 4: 绘制下游漏抓分界线

在 `_draw_auto_sort_line`（第 1532 行附近）方法中，在绘制自动分拣黄线之后追加（仅动态模式时显示蓝色虚线）：

```python
            if self.dynamic_mode:
                x_miss = int(self._canvas_w * self._dynamic_miss_line_u / 640)
                self.canvas.create_line(
                    x_miss, 0, x_miss, self._canvas_h, fill='blue', dash=(6, 4))
                self.canvas.create_text(
                    x_miss + 5, 26, text=f"漏抓线 u={self._dynamic_miss_line_u}",
                    fill='blue', anchor=tk.NW, font=("Arial", 9, "bold"))
```

注意：画布变量名（`self.canvas`、`self._canvas_w`、`self._canvas_h`）以现有代码为准。同时检查动态模式下是否也需要绘制上游触发线（可用绿色虚线，如果现有绘制逻辑容易扩展）。

### Step 5: 同步手动按钮禁用与弹窗屏蔽

在现有 `_conveyor_done`（第 1357 行）与 `_update_status_loop`（第 1437/1447 行）中，把 `if not self.auto_sort_running:` 扩展为 `if not (self.auto_sort_running or self.dynamic_mode):`，确保动态模式运行时手动按钮保持禁用、错误不弹窗。

### Step 6: 语法检查与符号验证

```bash
cd /home/lxf/agx_arm_ws && python3 -m py_compile sorting_gui_client.py
```

```bash
cd /home/lxf/agx_arm_ws && python3 - <<'EOF'
import ast
src = open('sorting_gui_client.py').read()
for token in ('_dynamic_start', '_dynamic_stop', '_dynamic_tick',
              '_dynamic_trigger_line_u', '_dynamic_miss_line_u',
              'sort_dynamic', 'line_u=None'):
    assert token in src, f'缺失符号: {token}'
print('OK: 动态抓取 GUI 符号齐全')
EOF
```

期望输出 `OK: 动态抓取 GUI 符号齐全`。

### Step 7: 提交

```bash
cd /home/lxf/agx_arm_ws && git add sorting_gui_client.py && git commit -m "feat: add dynamic grasping mode and downstream miss-line fallback to GUI"
```

## 注意事项

- `sorting_gui_client.py` 是大文件（约 104KB），包含 GUI 主类 `SortingGUI` 的多个方法。请先用 Grep/Read 精确定位每个锚点（`_auto_sort_line_u`、`_pick_object_right_of_line`、`_draw_auto_sort_line`、`_conveyor_done`、`_update_status_loop`、按钮区域、`_auto_sort_log`、`_bin_for_object`、`_conveyor_set`、`current_status`、`conveyor_state`）再修改。
- 所有新增/修改代码注释用中文。
- 不要重构本任务之外的既有代码。
- TDD：本任务是 GUI 集成任务，若无法单元测试，至少完成 py_compile 与符号断言验证。
