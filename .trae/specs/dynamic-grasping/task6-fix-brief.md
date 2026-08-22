# Task 6 修复 Brief: 修复评审发现的 2 个 Important 问题

> 工作目录：`/home/lxf/agx_arm_ws`。修改文件：`sorting_gui_client.py`（当前 HEAD 6117a854，不要回退提交）。

## 背景

Task 6（GUI 动态抓取模式）已实现并提交（6117a854），评审发现 2 个 Important 问题，直接影响 MISS_STOP 静态兜底核心功能。请修复后运行验证并提交新 commit（不要 amend）。

## 修复 ①: 漏抓静态 sort 下发后传送带可能过早重启（竞态）

**问题**：`_dynamic_tick` 的 MISS_STOP 分支在 `_dispatch_sort_cmd(miss_obj, static_mode=True)` 之后置 `_dynamic_state='RUNNING'` 并 200ms 后重排。但 `current_status` 由 `_update_status_loop` 每 500ms 轮询刷新，200ms 时服务端往往还没把状态翻成 `busy`，导致 `current_status=='idle'` 仍成立，于是"重启传送带"守卫提前通过 → 在静态 sort 真正执行前重启了传送带 → 兜底抓取坐标陈旧 → 空夹/失败 → 反复震荡。

**修复方案**（利用现有 `_dynamic_busy` 字段作为"sort 在途"门控，同时消除 Minor ④ 死字段）：
1. MISS_STOP 分支下发静态 sort 时：`self._dynamic_busy = True`。
2. 状态机中所有"等待 busy→idle 转换"逻辑以 `_dynamic_busy` 为门控：tick 中先检查 `if self._dynamic_busy:` → 若 `current_status == 'idle'` 则清零 `_dynamic_busy`（表示上一次 sort_dynamic/sort 已完成），否则等待重排。
3. "重启传送带"步骤的守卫改为 `not self._dynamic_busy`（即上次兜底 sort 确认完成后才重启带）。
4. 保留/完善 `_dispatch_sort_cmd` 内已有的 `self._dynamic_busy = True` 设置。

## 修复 ②: MISS_STOP 分支 miss_obj 为 None 导致状态机冻结

**问题**：`miss_obj` 每次 tick 重新计算（`require_base=True`）；停带窗口内若某帧 `base_position_m` 缺失或检测抖动，branch 2 会以 `miss_obj=None` 调用 `_dispatch_sort_cmd`，其内部 `obj.get('base_position_m')` 对 None 抛 `AttributeError`。`_dynamic_tick` 无 try/except，异常发生在 after 回调中导致重排不再执行 → 动态模式静默冻结（带停、按钮禁用），只能点"停止动态抓取"恢复。

**修复方案**：
1. MISS_STOP 的静态 sort 派发前加 `if miss_obj is not None:` 守卫；若为 None（物体短暂丢失），不做派发，直接走"重启传送带"路径回到 RUNNING（物体已在分界线右侧，若仍在检测区下一轮会重新命中 MISS_STOP）。
2. 给 `_dynamic_tick` 主体加 `try/except`（捕获 Exception，log 错误后 `self.after(200, self._dynamic_tick)` 续排），防止任何异常导致状态机冻结。参照现有 `_auto_sort_tick` 的 try/except 续排写法。

## Minor（一并修复，低成本）

- ④ 上述修复后 `_dynamic_busy` 成为有读取字段，确认其语义一致（派发时 True、busy→idle 转换时清零、STOPPING 退出时清零）。
- ③ STOPPING 下对在途 sort_dynamic 的停带竞态：STOPPING 分支若 `_dynamic_busy` 为 True 且 `current_status != 'busy'`（指令在途未确认），先等待（重排）而不是立即停带。若实现复杂可只做最小守卫。

## 验证（必须执行并贴出结果）

```bash
cd /home/lxf/agx_arm_ws && python3 -m py_compile sorting_gui_client.py
cd /home/lxf/agx_arm_ws && python3 - <<'EOF'
import ast
src = open('sorting_gui_client.py').read()
for token in ('_dynamic_start', '_dynamic_stop', '_dynamic_tick',
              '_dynamic_trigger_line_u', '_dynamic_miss_line_u',
              'sort_dynamic', 'line_u=None', '_dynamic_busy'):
    assert token in src, f'缺失符号: {token}'
print('OK: 动态抓取 GUI 符号齐全')
EOF
```

## 提交

```bash
cd /home/lxf/agx_arm_ws && git add sorting_gui_client.py && git commit -m "fix: gate belt restart on sort completion and guard miss_obj None in dynamic mode"
```

## 报告

将修复说明写入 /home/lxf/agx_arm_ws/.trae/specs/dynamic-grasping/task6-fix-report.md（改了什么、怎么验证、结果）。所有注释中文，不改动本任务之外代码。
