# 夹爪力矩检测抓取成功判断系统 + GUI 力矩调节器

## Context

当前抓取流程 (`auto_sorting_action.py` L1622) 闭合夹爪后直接 `sleep(0.3)` 然后抬起, **完全不检测是否抓到物体**。空抓、物体滑脱等情况无法识别, 导致机械臂带着空夹爪走到料框"放置"。

系统**已具备力反馈基础设施**但未被使用:
- `/feedback/gripper_status` (GripperStatus msg) 实时发布夹爪 `width`(m) 和 `force`(N)
- `/control/joint_states` 的 `effort` 字段控制夹持力 (当前硬编码 1.5N)
- `AgxGripperWrapper.move(width, force)` 支持力控范围 0.5-3.0N
- GripperStatus 含 `driver_overcurrent` 等安全标志

本方案利用现有基础设施, 实现: ① 闭合后读取实际 width/force 判断抓取成功; ② GUI 力矩调节器; ③ 安全保护。

## 修改文件

1. `/home/lxf/agx_arm_ws/auto_sorting_action.py` — 核心逻辑
2. `/home/lxf/agx_arm_ws/sorting_gui_client.py` — GUI 力矩调节器 + 状态显示

## 实现步骤

### 步骤 1: auto_sorting_action.py — 订阅 GripperStatus

**位置**: `MoveItActionClient.__init__` (L224 附近)

```python
from agx_arm_msgs.msg import GripperStatus  # 顶部 import

# __init__ 中新增:
self._latest_gripper_status = None
self._gripper_status_sub = self.create_subscription(
    GripperStatus, "/feedback/gripper_status", self._gripper_status_cb, 10)

# 力矩参数 (GUI 可调)
self._gripper_force = 1.5        # 当前夹持力 (N), GUI 可调
self._GRIPPER_FORCE_MIN = 0.5    # 安全下限
self._GRIPPER_FORCE_MAX = 3.0    # 安全上限 (AgxGripperWrapper.FORCE_MAX)
self._GRIPPER_FORCE_DANGER = 3.5 # 危险阈值 (超过即停止)

def _gripper_status_cb(self, msg):
    self._latest_gripper_status = msg
```

### 步骤 2: auto_sorting_action.py — 修改 operate_gripper 支持力控

**位置**: L1183-1220 (`_publish_gripper_joint_state` 和 `operate_gripper`)

关键改动: `operate_gripper` 新增 `force` 参数, 当指定 force 时跳过 FollowJointTrajectory (不支持力控), 直接用 `/control/joint_states` 路径:

```python
def _publish_gripper_joint_state(self, target_pos, force=None):
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = ['gripper']
    msg.position = [float(target_pos)]
    msg.velocity = []
    msg.effort = [force if force is not None else self._gripper_force]
    self._joint_states_pub.publish(msg)

def operate_gripper(self, target_pos, desc, force=None, use_force_control=False):
    # force 指定时 (抓取闭合): 直接走 /control/joint_states 路径 (支持力控)
    if use_force_control or force is not None:
        f = force if force is not None else self._gripper_force
        f = max(self._GRIPPER_FORCE_MIN, min(self._GRIPPER_FORCE_MAX, f))
        self.get_logger().info(f"✊ {desc} (target={target_pos}, force={f}N) [力控模式]")
        self._publish_gripper_joint_state(target_pos, f)
        time.sleep(0.3)
        return True
    # 无 force 参数 (张开夹爪): 保持原有三级回退
    # ... 原有代码不变 ...
```

### 步骤 3: auto_sorting_action.py — 抓取成功检测函数

**位置**: `MoveItActionClient` 类内新增方法

```python
def check_grasp_success(self, settle_s=0.3):
    """闭合后检测抓取是否成功.
    
    判断逻辑:
      - width ≈ 0 且 force ≈ 0 → 空抓 (未夹到物体)
      - width > 阈值 且 force ≥ 阈值 → 成功抓取
      - driver_overcurrent → 安全保护触发
      - force > danger 阈值 → 危险, 立即停止
    
    Returns: (success, width, force, reason)
    """
    time.sleep(settle_s)  # 等待夹爪稳定
    
    if self._latest_gripper_status is None:
        return False, 0.0, 0.0, "no_feedback"
    
    gs = self._latest_gripper_status
    width, force = gs.width, gs.force
    
    # 安全检查
    if gs.driver_overcurrent:
        return False, width, force, "overcurrent"
    if force > self._GRIPPER_FORCE_DANGER:
        return False, width, force, "force_danger"
    
    # 成功判断
    WIDTH_EMPTY = 0.005   # 5mm: width ≤ 此值 = 完全闭合 (无物体)
    FORCE_CONTACT = 0.3   # 0.3N: force ≥ 此值 = 有接触力
    
    if width <= WIDTH_EMPTY and force <= FORCE_CONTACT:
        return False, width, force, "empty_grasp"
    if width > WIDTH_EMPTY and force >= FORCE_CONTACT:
        return True, width, force, "success"
    return False, width, force, "ambiguous"
```

### 步骤 4: auto_sorting_action.py — 插入检测到抓取流程

**位置**: L1621-1627 (夹爪闭合后, 抬起前)

```python
# 【第三步】 一次性闭合夹爪 (力控模式, 使用 GUI 设定的力矩)
node.operate_gripper(dynamic_close, "闭合夹爪(拿取)", 
                     force=node._gripper_force, use_force_control=True)
time.sleep(0.15)  # planning_scene 更新等待

# 【第三步半】 抓取成功检测
grasp_ok, gw, gf, reason = node.check_grasp_success(settle_s=0.3)
node.get_logger().info(
    f"🔍 抓取检测: success={grasp_ok} width={gw*1000:.1f}mm force={gf:.2f}N reason={reason}")
node._publish_grasp_status(grasp_ok, gw, gf, reason)

if not grasp_ok:
    node.get_logger().warn(f"⚠️ 抓取失败 ({reason}), 跳过本轮")
    # 张开夹爪释放 (可能有部分接触)
    node.operate_gripper(GRIPPER_OPEN, "抓取失败-张开夹爪")
    success = False
    break
```

### 步骤 5: auto_sorting_action.py — 状态发布 + 力矩设置命令

**位置**: `MoveItActionClient` 类内新增方法 + 命令处理循环

```python
def _publish_grasp_status(self, success, width, force, reason):
    """发布抓取结果到 GUI."""
    msg = String()
    msg.data = json.dumps({
        "type": "grasp_result",
        "success": success,
        "width": round(width, 4),
        "force": round(force, 2),
        "reason": reason,
    })
    self.status_pub.publish(msg)
```

**命令处理循环** (已有 `cmd_callback`, 在处理 cmd 的地方加):
```python
elif data.get("cmd") == "set_gripper_force":
    force = float(data.get("force", 1.5))
    force = max(self._GRIPPER_FORCE_MIN, min(self._GRIPPER_FORCE_MAX, force))
    self._gripper_force = force
    self.get_logger().info(f"🔧 夹爪力矩已设置为 {force:.2f}N")
```

### 步骤 6: sorting_gui_client.py — 力矩调节器 UI

**位置**: 曝光控制 LabelFrame 之后 (~L314), 新增 LabelFrame

```python
# ── 夹爪力矩控制 (通过 /sorting_cmds 发给 auto_sorting_action.py) ──
gf_frame = tk.LabelFrame(f, text="夹爪力矩控制", padx=5, pady=5)
gf_frame.pack(fill=tk.X, pady=(5, 0))

self.gripper_force_var = tk.DoubleVar(value=1.5)
self.gripper_force_label = tk.StringVar(value="当前: 1.50N")
tk.Label(gf_frame, textvariable=self.gripper_force_label,
         font=("Arial", 10, "bold"), fg="blue").pack(anchor=tk.W, pady=(0, 3))

# 力矩滑块 (0.5-3.0N)
sf = tk.Frame(gf_frame); sf.pack(fill=tk.X)
tk.Scale(sf, from_=0.5, to=3.0, resolution=0.1, orient=tk.HORIZONTAL,
         variable=self.gripper_force_var, command=self._on_force_change,
         length=160, font=("Arial", 8)).pack(side=tk.LEFT, padx=(0, 4))

# 快速预设按钮
for label, val in [("软", 0.8), ("中", 1.5), ("硬", 2.5)]:
    tk.Button(sf, text=label, command=lambda v=val: self._set_force(v),
              font=("Arial", 8), width=3).pack(side=tk.LEFT, padx=1)

# 抓取状态显示
ttk.Separator(gf_frame, orient='horizontal').pack(fill=tk.X, pady=3)
self.grasp_status_var = tk.StringVar(value="抓取状态: --")
tk.Label(gf_frame, textvariable=self.grasp_status_var,
         font=("Arial", 10), fg="gray").pack(anchor=tk.W)
self.grasp_detail_var = tk.StringVar(value="")
tk.Label(gf_frame, textvariable=self.grasp_detail_var,
         font=("Arial", 8), fg="gray").pack(anchor=tk.W)
```

### 步骤 7: sorting_gui_client.py — 力矩调节回调

```python
def _on_force_change(self, val):
    force = round(float(val), 2)
    self.gripper_force_label.set(f"当前: {force:.2f}N")
    self.cmd_queue.put({"cmd": "set_gripper_force", "force": force})

def _set_force(self, val):
    self.gripper_force_var.set(val)
    self._on_force_change(val)
```

### 步骤 8: sorting_gui_client.py — 抓取状态回调

**位置**: `/sorting_status` 订阅回调中, 新增 `grasp_result` 类型处理

```python
if status_data.get("type") == "grasp_result":
    success = status_data.get("success", False)
    width = status_data.get("width", 0)
    force = status_data.get("force", 0)
    reason = status_data.get("reason", "")
    if success:
        self.grasp_status_var.set("抓取状态: ✅ 成功")
        # 可选: 改颜色为绿色
    else:
        self.grasp_status_var.set(f"抓取状态: ❌ 失败 ({reason})")
    self.grasp_detail_var.set(
        f"宽度={width*1000:.1f}mm  力矩={force:.2f}N")
```

## 数据流

```
GUI 滑块 → /sorting_cmds (set_gripper_force) → action 节点更新 _gripper_force
                                                        ↓
action 节点闭合夹爪 (force=_gripper_force) → /control/joint_states (effort=force)
                                                        ↓
                                              agx_ctrl_single_node → gripper.move(width, force)
                                                        ↓
夹爪传感器 → /feedback/gripper_status (width, force) → action 节点 check_grasp_success()
                                                        ↓
action 节点 → /sorting_status (grasp_result) → GUI 状态显示
```

## 安全保护机制

| 机制 | 触发条件 | 动作 |
|------|----------|------|
| 力矩上限 | GUI 设置 > 3.0N | 钳位到 3.0N (AgxGripperWrapper.FORCE_MAX) |
| 过流保护 | GripperStatus.driver_overcurrent=True | 判定抓取失败, 张开夹爪, 跳过 |
| 危险力矩 | force > 3.5N | 判定抓取失败, 张开夹爪, 跳过 |
| 空抓检测 | width ≤ 5mm 且 force ≤ 0.3N | 判定未抓到, 张开夹爪, 跳过 |

## 验证方法

1. **力矩调节**: 启动系统后在 GUI 拖动力矩滑块, T5 日志应显示 `🔧 夹爪力矩已设置为 X.XXN`
2. **成功检测**: 放置物体点击 GraspNet抓取, T5 日志应显示 `🔍 抓取检测: success=True width=XXmm force=X.XXN`
3. **空抓检测**: 移开物体让夹爪空闭合, 应显示 `success=False reason=empty_grasp`
4. **GUI 状态**: GUI 应实时显示 `✅ 成功` 或 `❌ 失败 (empty_grasp)` + 宽度/力矩数值
5. **力矩预设**: 点击"软/中/硬"按钮, 滑块应跳到对应值并发送命令
6. **安全保护**: 设置力矩为 3.0N 抓取硬物, 不应损坏物体 (driver 内部限流)
