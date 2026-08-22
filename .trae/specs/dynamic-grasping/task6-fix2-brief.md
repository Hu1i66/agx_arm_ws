# Task 6 二次修复 Brief: 修复 C1 状态机续排回归 + I1 error 无界等待

> 工作目录：`/home/lxf/agx_arm_ws`。修改文件：`sorting_gui_client.py`（当前 HEAD b01e32c8，不要回退提交，不要 amend，新建 commit）。

## 背景

第一次修复（b01e32c8）解决了门控与 None 守卫问题，但复审发现一个 Critical 回归和 1 个 Important，需修复。

## Critical C1: 状态机在首个动作分支后永久停机（续排被误删）

**问题**：当前 `_dynamic_tick` 把所有分支改成了裸 `return`，而唯一的 `self.after(200, self._dynamic_tick)` 在函数尾部，**只有"什么都不做"的 fall-through 路径能到达**。任何走分支的 tick 都直接退出函数，再无续排。后果：
- 派发 sort_dynamic 后 → 循环死亡，不再检测 MISS_STOP、不再管理传送带；
- 进入 MISS_STOP → 带能停但静态 sort 永远不派发；
- STOPPING → 停止流程永不完成，dynamic_mode 卡死为 True。

**参照**：现有 `_auto_sort_tick`（约 1810-1822 行）的模式是——分支只改状态/发指令然后 **fall-through 到尾部唯一的 `self.after(...)`**，每轮必执行。

**修复**：重写 `_dynamic_tick`，保证**每个可能路径最终都会重新调度**。推荐两种方式（选一，保持简洁）：
1. 仿照 `_auto_sort_tick`：分支内不 return，只设置状态/发指令，走到尾部统一的 `self.after(200, self._dynamic_tick)`；
2. 或每个 `return` 前都显式 `self.after(200, self._dynamic_tick)`。

要求：用状态机推演验证每个路径——STOPPING 等 busy / STOPPING 停带 / STOPPING 完成退出（此路径是唯一可以不再续排的）/ busy 等待 / MISS_STOP 进入 / MISS_STOP 等带停 / MISS_STOP 静态派发 / RUNNING 触发派发 / 无事等待——每一条都必须有续排（除 STOPPING 完成退出）。

## Important I1: `_dynamic_wait_inflight` 在 current_status == 'error' 时永久等待

**问题**：`_dynamic_wait_inflight()` 约 1586 行 `return True  # 状态未知/error`。若在途 sort/sort_dynamic 失败、服务端报 error 而 `_dynamic_busy` 为 True，门控永远返回 True → 传送带永不重启、动态模式永久卡死。'error' 是真实状态（见 `_update_status_loop` 约 1482 行）。

**修复**：'error' 状态同样走超时宽限（复用现有 1s 空闲宽限逻辑）后清 `_dynamic_busy` 并记录日志（`_auto_sort_log`），避免无界等待，让状态机自我恢复。

## Minor（若顺手可处理，否则留注释说明）

- 静态兜底分支的 `_dynamic_busy=True` 置位若发生在 `_dispatch_sort_cmd` 因缺 base_position_m 提前 return 之后，会造成"已置位但未实际派发"的偶发耦合。建议把置位逻辑收进 `_dispatch_sort_cmd` 内部（仅实际派发时才置 True），消除偶然耦合。

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
cd /home/lxf/agx_arm_ws && git add sorting_gui_client.py && git commit -m "fix: restore dynamic state machine rescheduling and error timeout"
```

## 报告

将修复说明写入 /home/lxf/agx_arm_ws/.trae/specs/dynamic-grasping/task6-fix2-report.md（改了什么、状态机路径推演、验证结果）。所有注释中文，不改动本任务之外代码。
