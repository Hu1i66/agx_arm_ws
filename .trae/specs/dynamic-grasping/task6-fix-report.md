# Task 6 修复报告: 动态抓取模式 2 个 Important 问题

> 工作目录：`/home/lxf/agx_arm_ws`。修改文件：`sorting_gui_client.py`
> 基于提交：`6117a854`（未回退、未 amend）
> 新提交：`b01e32c8` — `fix: gate belt restart on sort completion and guard miss_obj None in dynamic mode`

## 修复内容

### ① 竞态: 漏抓静态 sort 下发后传送带过早重启

**问题**：`_dynamic_tick` 的 MISS_STOP 分支下发静态 `sort` 后即回到 RUNNING，而
`current_status` 由 `_update_status_loop` 每 500ms 轮询刷新、tick 每 200ms 重排，
200ms 时服务端往往还没把状态翻成 `busy`，导致"重启传送带"守卫提前通过，在静态
sort 真正执行前重启了传送带 → 兜底抓取坐标陈旧 → 空夹/失败 → 反复震荡。

**修复**：
- 新增 `_dynamic_wait_inflight()` 门控方法：以 `self._dynamic_busy` 作为
  "sort_dynamic / 静态兜底 sort 在途"标记。派发时置 `True`；仅当观察到 `busy→idle`
  转换后才清零。由于状态刷新滞后，下发后短时间内的 `idle` 可能是派发前的陈旧值，
  故通过 `time.time() - self.last_dispatch_time < 1.0`（覆盖 500ms 轮询周期两轮）
  区分"指令在途未确认"与"任务已完成"，未确认前持续等待重排，不提前清零。
- MISS_STOP 分支下发静态 sort 时显式置 `self._dynamic_busy = True`（静态走
  `_dispatch_sort_cmd(..., static_mode=True)`，原有"已有设置"仅在
  `not static_mode` 分支内，故在此补置）。
- "重启传送带"步骤守卫改为 `not self._dynamic_busy`，即上次兜底 sort 确认完成后
  才重启带。
- 顺带修复动态触发路径的潜在重复下发：sort_dynamic 在途期间（`_dynamic_busy=True`）
  状态机只等待，不会因陈旧 idle 再次命中触发线重复下发。

### ② 冻结: MISS_STOP 分支 miss_obj 为 None 导致状态机静默冻结

**问题**：`miss_obj` 每次 tick 重新计算（`require_base=True`）；停带窗口内某帧
`base_position_m` 缺失或检测抖动时，会以 `None` 调用 `_dispatch_sort_cmd`，其内部
`obj.get('base_position_m')` 抛 `AttributeError`；原 `_dynamic_tick` 无 try/except，
异常发生在 after 回调中导致重排不再执行 → 动态模式静默冻结。

**修复**：
- MISS_STOP 静态 sort 派发前加 `if miss_obj is not None:` 守卫；为 None（物体短暂
  丢失）时不派发，直接走"重启传送带"路径回到 RUNNING，下一轮若物体仍在检测区会
  重新命中 MISS_STOP。
- `_dynamic_tick` 主体用 `try/except` 包裹，捕获 `Exception` 后记录
  `_auto_sort_log("动态抓取 tick 异常: ...")`，并统一 `self.after(200, self._dynamic_tick)`
  续排（参照 `_auto_sort_tick` 的续排写法），任何异常都不会让状态机冻结。
  （原内部 300ms 重排统一改为 200ms 续排，语义一致。）

### Minor: _dynamic_busy 死字段修复 + STOPPING 最小守卫

- `_dynamic_busy` 现已成为真正的读取字段（在途门控、重启带守卫、STOPPING 守卫均读取），
  语义一致：派发时 `True`、观察到 busy→idle 转换时清零、STOPPING 退出时清零。
- STOPPING 分支最前新增 `if self._dynamic_wait_inflight(): return`（在途任务未确认
  完成前不立即停带），解决在途 sort_dynamic 未确认 busy 时过早停带的竞态。

## 验证结果

```bash
cd /home/lxf/agx_arm_ws && python3 -m py_compile sorting_gui_client.py
# → PY_COMPILE_OK (退出码 0)

cd /home/lxf/agx_arm_ws && python3 - <<'EOF'
import ast
src = open('sorting_gui_client.py').read()
for token in ('_dynamic_start', '_dynamic_stop', '_dynamic_tick',
              '_dynamic_trigger_line_u', '_dynamic_miss_line_u',
              'sort_dynamic', 'line_u=None', '_dynamic_busy'):
    assert token in src, f'缺失符号: {token}'
print('OK: 动态抓取 GUI 符号齐全')
EOF
# → OK: 动态抓取 GUI 符号齐全
```

两项验证均通过。

## 提交

```
b01e32c8 fix: gate belt restart on sort completion and guard miss_obj None in dynamic mode
```

仅提交 `sorting_gui_client.py`（1 file changed, 92 insertions(+), 60 deletions(-)），
未改动本任务之外代码。
