# Grasp Cycle Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce single sort cycle time from ~17s to ~14.3s by eliminating redundant fixed wait times in `auto_sorting_action.py`.

**Architecture:** Three targeted changes in one file — shorten constant wait values, and leverage the lift movement's natural duration to parallelize grasp detection (by the time lift completes, gripper width is already stable, eliminating the separate 1s polling wait).

**Tech Stack:** Python 3, ROS2 Humble, MoveIt2

**Source Spec:** `.trae/specs/grasp-cycle-optimization/spec.md`

## Global Constraints

- Modify only `auto_sorting_action.py`
- All collision detection, IK, planning logic unchanged
- Gripper action fallback chain unchanged
- Emergency stop checkpoints unchanged
- No new dependencies

---

### Task 1: Shorten GRIPPER_SETTLE_SEC and continuous=False sleep

**Files:**
- Modify: `/home/lxf/agx_arm_ws/auto_sorting_action.py:1618`
- Modify: `/home/lxf/agx_arm_ws/auto_sorting_action.py:689`
- Modify: `/home/lxf/agx_arm_ws/auto_sorting_action.py:1353`

**Interfaces:**
- Produces: `GRIPPER_SETTLE_SEC` = 0.15 (was 0.30)
- Produces: `continuous=False` post-execution sleep = 0.15s (was 0.5s)
- Produces: `continuous=False` post-execution sleep in move_arm_cartesian = 0.15s (was 0.5s)

- [ ] **Step 1: Change GRIPPER_SETTLE_SEC**

```python
# Line 1618: change from 0.30 to 0.15
GRIPPER_SETTLE_SEC = 0.15
```

- [ ] **Step 2: Change continuous=False sleep in send_goal (MoveGroup path)**

```python
# Line 686-689: change 0.5 to 0.15
if continuous:
    time.sleep(0.05) # 连贯动作减少停顿
else:
    time.sleep(0.15)
```

- [ ] **Step 3: Change continuous=False sleep in move_arm_cartesian**

```python
# Line 1350-1353: change 0.5 to 0.15
if continuous:
    time.sleep(0.05)
else:
    time.sleep(0.15)
```

- [ ] **Step 4: Update stale comment at L2095**

The comment "continuous=False: 上一步执行后需 0.5s 稳定时间" is now stale:

```python
# Old (L2095):
# ⚠️ continuous=False: 上一步执行后需 0.5s 稳定时间，否则起点偏差 > 0.01 触发 -4 错误

# Replace with:
# ⚠️ continuous=False: 上一步执行后需 0.15s 稳定时间，否则起点偏差 > 0.01 触发 -4 错误
```

- [ ] **Step 5: Syntax check**

Run: `cd /home/lxf/agx_arm_ws && python3 -c "import py_compile; py_compile.compile('auto_sorting_action.py', doraise=True); print('OK')"`
Expected: `OK`

- [ ] **Step 6: Commit**

```bash
git add auto_sorting_action.py
git commit -m "perf: shorten GRIPPER_SETTLE_SEC (0.30→0.15) and continuous=False sleep (0.5→0.15)"
```

---

### Task 2: Parallelize grasp detection with lift movement

**Files:**
- Modify: `/home/lxf/agx_arm_ws/auto_sorting_action.py` — step4 lift area (L2027-2070)

**Interfaces:**
- Consumes: `sorted_joints` from `pick_up_joints` dict, `check_grasp_success()` method
- Produces: `grasp_ok, gw, gf, reason` — same variables used by downstream retry logic

**Key insight:** The gripper closed at step3, then after `GRIPPER_SETTLE + planning_scene` (0.30s total), step4 lift takes ~1.5s. By the time lift completes, gripper has been closed for > 1.8s — width is absolutely stable. The `check_grasp_success(timeout_s=0.3)` will return on the first or second poll cycle (~0.05-0.10s) instead of waiting a full 1s.

- [ ] **Step 1: Replace step4 + step4.5 with combined flow**

Current code (L2027-2064):
```python
# 【第四步】 抬起脱离
_lift_ok = False
if pick_up_joints:
    sorted_joints = [pick_up_joints[f'joint{i}'] for i in range(1, 7)]
    if node.move_arm_joint(sorted_joints, "抬起脱离 (直接原路逆向)", continuous=True):
        _lift_ok = True
    else:
        node.get_logger().warn("⚠️ 抬起(关节空间)失败(-2), 回退到笛卡尔规划")
if not _lift_ok:
    POSE_LIFT = POSE_PICK.copy()
    POSE_LIFT['z'] = POSE_PICK_UP['z']
    _lift_pose_msg = node._create_pose(POSE_LIFT, active_ori)
    if node.execute_cartesian_path([_lift_pose_msg], "抬起脱离 (直线插补)", fraction_threshold=0.50):
        _lift_ok = True
    elif not node.move_arm_cartesian(POSE_LIFT, "抬起脱离 (退避规划)", continuous=True, preferred_orientation=active_ori, allow_position_only_fallback=True, planning_mode='retreat'):
        continue  # 抬起失败跳过本次重试
    if node.last_planning_strategy:
        cycle_strategies.append(node.last_planning_strategy)
    if node.last_planning_profile_name:
        cycle_profiles.append(node.last_planning_profile_name)
grasp_reached_ok = True
if node._check_emergency(f"step4-抬起脱离,retry{retry+1}"):
    success = False; break

# 【第四步半】 抓取成功检测 — 抬起后执行
grasp_ok, gw, gf, reason = node.check_grasp_success(timeout_s=1.0)
node.get_logger().info(
    f"🔍 抓取检测 (retry {retry+1}/{MAX_GRASP_RETRIES}): "
    f"success={grasp_ok} width={gw*1000:.1f}mm "
    f"force={gf:.2f}N reason={reason}")
node._publish_grasp_status(grasp_ok, gw, gf, reason)
if node._check_emergency(f"step4.5-抓取检测,retry{retry+1}"):
    success = False; break

if grasp_ok:
    node.get_logger().info(
        f"✅ 抓取成功 (retry {retry+1}/{MAX_GRASP_RETRIES})")
    break  # 成功! 跳出重试循环
```

Replace with:
```python
# 【第四步】 抬起脱离 + 内嵌抓取检测 (抬起耗时 ~1.5s, 夹爪早已稳定)
_lift_ok = False
if pick_up_joints:
    sorted_joints = [pick_up_joints[f'joint{i}'] for i in range(1, 7)]
    if node.move_arm_joint(sorted_joints, "抬起脱离 (直接原路逆向)", continuous=True):
        _lift_ok = True
    else:
        node.get_logger().warn("⚠️ 抬起(关节空间)失败(-2), 回退到笛卡尔规划")
if not _lift_ok:
    POSE_LIFT = POSE_PICK.copy()
    POSE_LIFT['z'] = POSE_PICK_UP['z']
    _lift_pose_msg = node._create_pose(POSE_LIFT, active_ori)
    if node.execute_cartesian_path([_lift_pose_msg], "抬起脱离 (直线插补)", fraction_threshold=0.50):
        _lift_ok = True
    elif not node.move_arm_cartesian(POSE_LIFT, "抬起脱离 (退避规划)", continuous=True, preferred_orientation=active_ori, allow_position_only_fallback=True, planning_mode='retreat'):
        continue  # 抬起失败跳过本次重试
    if node.last_planning_strategy:
        cycle_strategies.append(node.last_planning_strategy)
    if node.last_planning_profile_name:
        cycle_profiles.append(node.last_planning_profile_name)
grasp_reached_ok = True
if node._check_emergency(f"step4-抬起脱离,retry{retry+1}"):
    success = False; break

# 【第四步半】 抓取成功检测 — 抬起后立即评估
# 夹爪在 step3 已闭合, 经 GRIPPER_SETTLE(0.15s) + lift(~1.5s) 已稳定 >1.6s,
# check_grasp_success 的 0.3s 超时在第一轮轮询即返回 (width 已稳定).
grasp_ok, gw, gf, reason = node.check_grasp_success(timeout_s=0.3)
node.get_logger().info(
    f"🔍 抓取检测 (retry {retry+1}/{MAX_GRASP_RETRIES}): "
    f"success={grasp_ok} width={gw*1000:.1f}mm "
    f"force={gf:.2f}N reason={reason}")
node._publish_grasp_status(grasp_ok, gw, gf, reason)
if node._check_emergency(f"step4.5-抓取检测,retry{retry+1}"):
    success = False; break

if grasp_ok:
    node.get_logger().info(
        f"✅ 抓取成功 (retry {retry+1}/{MAX_GRASP_RETRIES})")
    break  # 成功! 跳出重试循环
```

The only change is `timeout_s=1.0` → `timeout_s=0.3` and updated comments. All logic flow is preserved.

- [ ] **Step 2: Syntax check**

Run: `cd /home/lxf/agx_arm_ws && python3 -c "import py_compile; py_compile.compile('auto_sorting_action.py', doraise=True); print('OK')"`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
git add auto_sorting_action.py
git commit -m "perf: parallelize grasp detection with lift, reduce check_grasp timeout (1.0→0.3s)"
```

---

### Task 3: Verification — full cycle timing comparison

**Files:**
- Read: `/home/lxf/agx_arm_ws/auto_sorting_action.py` (log output)

- [ ] **Step 1: Run single sort cycle and check timing**

Run a single sort operation via GUI. From logs, calculate cycle duration:
```
grep -E "(第.*次分拣|✅ 抓取成功|回到待机位)" /tmp/sort_log.txt
```
Expected: cycle time reduced from ~17s to ~14-15s range.

- [ ] **Step 2: Run 10 consecutive sort cycles**

Run auto-sort for 10 objects, verify:
- No `-4` (CONTROL_FAILED) errors from stability issues
- No `-2` (PLANNING_FAILED) errors from premature moves
- Cycle time consistently < 15s
- All objects successfully picked and placed

- [ ] **Step 3: Verify grasp detection accuracy**

From logs, check that `check_grasp_success` always returns a definitive result (not `no_feedback` or `ambiguous`):
```
grep "抓取检测" /tmp/sort_log.txt
```
Expected: all entries show `success=True/False` with `width=XX.Xmm` (not `no_feedback`).

- [ ] **Step 4: Final commit with test results**

```bash
git add auto_sorting_action.py
git commit --amend -m "perf: cycle optimization — GRIPPER_SETTLE 0.30→0.15, continuous=False sleep 0.5→0.15, check_grasp timeout 1.0→0.3"
```
