# Task 6 二次修复报告

- 提交: `814063ce` "fix: restore dynamic state machine rescheduling and error timeout" (基于 HEAD b01e32c8, 未回退、未 amend)
- 修改文件: `sorting_gui_client.py` (仅此一个文件, 共 71 增 69 删)
- 验证: `python3 -m py_compile` 通过 + 符号断言脚本通过 (`OK: 动态抓取 GUI 符号齐全`)

## 改动内容

### C1 (Critical): 恢复 `_dynamic_tick` 全路径续排

**根因**: 原 `_dynamic_tick` 把所有"有动作"的分支改成裸 `return`, 而唯一的 `self.after(200, self._dynamic_tick)` 在函数尾部, 只有 fall-through 的"无事等待"路径能到达。任何真实动作(派发 sort_dynamic、进入 MISS_STOP、停带、STOPPING 等待)都会在首轮动作后退出, 状态机永久停机。

**修复**: 仿照 `_auto_sort_tick` 的续排模式, 重写 `_dynamic_tick`。分支只改状态/发指令, 不再 `return`, 所有路径最终汇到函数尾部唯一的 `self.after(200, self._dynamic_tick)`。仅两处允许不续排退出:
1. 函数开头 `if not self.dynamic_mode: return` (动态模式已退出, 仅 STOPPING 完成后可达);
2. STOPPING 完成分支(`dynamic_mode=False` 置位后 `return`)。

### I1 (Important): `_dynamic_wait_inflight` 对 error 状态增加超时宽限

**根因**: 原 `current_status == 'error'` 走 `return True` 无界等待。若在途 sort/sort_dynamic 失败、服务端报 error 且 `_dynamic_busy` 为 True, 门控永远返回 True → 传送带永不重启、动态模式死锁。

**修复**: `'error'` 与 `'idle'` 一起走 1s 空闲宽限逻辑; 超过 1s 仍为 error 时, 通过 `_auto_sort_log` 记录日志后清除 `_dynamic_busy`, 返回 False, 状态机自我恢复(重启带、进入下一次派发)。

### Minor: `_dynamic_busy=True` 收进 `_dispatch_sort_cmd`

原 `_dynamic_tick` 的 MISS_STOP 静态兜底分支在 `_dispatch_sort_cmd` 返回后才 `self._dynamic_busy = True`; 若 `_dispatch_sort_cmd` 因缺 `base_position_m` 提前 return, 会出现"已置位但未派发"的偶发耦合。现改为: 仅在 `_dispatch_sort_cmd` 实际派发处(缺 base_position_m 提前 return 之后)、且 `self.dynamic_mode` 为 True 时才置 `_dynamic_busy = True`; 删除 `_dynamic_tick` 中冗余的置位行。

## 状态机路径推演 (每条路径是否续排)

| # | 路径 | 动作 | 续排 |
|---|------|------|------|
| 1 | STOPPING + 在途任务未完成 (wait_inflight) | 等待, pass | ✅ 尾部 after(200) |
| 2 | STOPPING + 机械臂 busy (未置 busy 的其它任务) | 等待, pass | ✅ 尾部 after(200) |
| 3 | STOPPING + 带未停且无切换在途 | `_conveyor_set(False)` 停带 | ✅ 尾部 after(200) |
| 4 | STOPPING + 停带指令在途 (`_conveyor_busy`) | 等待, pass | ✅ 尾部 after(200) |
| 5 | STOPPING 完成 (带已停稳、无在途) | 复位并置 `dynamic_mode=False` | ❌ 唯一不续排 (状态机退出) |
| 6 | 非 STOPPING + 在途任务未完成 (wait_inflight) | 等待, pass | ✅ 尾部 after(200) |
| 7 | 非 STOPPING + 机械臂 busy | 等待, pass | ✅ 尾部 after(200) |
| 8 | RUNNING + 漏抓检测命中 | 进入 MISS_STOP + 停带 | ✅ 尾部 after(200) |
| 9 | MISS_STOP + 带尚未停稳 | 等待, pass | ✅ 尾部 after(200) |
| 10 | MISS_STOP + 带已停 | 派发静态兜底 sort, 回 RUNNING | ✅ 尾部 after(200) |
| 11 | RUNNING 兜底后恢复 | `_conveyor_set(True)` 重启带 | ✅ 尾部 after(200) |
| 12 | RUNNING + 上游触发命中 | 派发 sort_dynamic, 回 RUNNING | ✅ 尾部 after(200) |
| 13 | 无事等待 (无任何动作) | 无 | ✅ 尾部 after(200) |
| 14 | 任一分支抛出异常 | 记录日志 | ✅ except 后尾部 after(200) |
| 15 | 动态模式已退出 (顶部守卫) | 无 | ❌ 不续排 (仅 STOPPING 完成后可达, 机器已停) |

结论: 除路径 5 (STOPPING 完成) 与路径 15 (机器已停止的顶部守卫, 即 STOPPING 完成后的兜底) 外, 全部路径均续排。

## 验证结果

```bash
$ python3 -m py_compile sorting_gui_client.py      # 通过, 退出码 0
$ python3 - <<'EOF'
import ast
src = open('sorting_gui_client.py').read()
for token in ('_dynamic_start', '_dynamic_stop', '_dynamic_tick',
              '_dynamic_trigger_line_u', '_dynamic_miss_line_u',
              'sort_dynamic', 'line_u=None', '_dynamic_busy'):
    assert token in src, f'缺失符号: {token}'
print('OK: 动态抓取 GUI 符号齐全')
EOF
OK: 动态抓取 GUI 符号齐全
```
