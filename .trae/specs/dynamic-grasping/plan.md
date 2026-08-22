# 传送带动态抓取 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现传送带匀速运行时的动态抓取——机械臂通过 STM32 线速度前馈 + 视觉融合预测物体位置，执行"影子对齐 → 斜线随动下降 → 夹取 → 放置"序列，配合下游视觉分界线漏抓兜底。

**Architecture:** 新增三个可独立测试的组件（方向标定 `conveyor_calib.py`、速度预测 `velocity_predictor.py`、`/conveyor_status` 话题）；`auto_sorting_action.py` 新增 `sort_dynamic` 命令（时序关键逻辑，自治订阅 `/detection_info` + `/conveyor_status`）；GUI 作为编排者新增动态模式状态机 + 下游漏抓分界线（复用现有 `_pick_object_right_of_line` 模式）。

**Tech Stack:** Python 3.10, ROS2 Humble (rclpy), numpy, pytest, MoveIt2 GetCartesianPath/ExecuteTrajectory, STM32 串口 (stm32_conveyor.py), Tkinter GUI

## Global Constraints

- 传送带线速度范围 0.02–0.15 m/s，来自 STM32 串口（真实值，无需二次校正），保留滑动窗口滤波
- 相机为眼在手外、俯视观察传送带，画面横轴与传送带不平行（存在夹角）——必须用标定的 `d̂_base` 而非图像横轴
- 抓取区为固定区域；机械臂下降/抓取时会遮挡俯视相机视野，抓取瞬间不能依赖视觉闭环
- 串口 `exclusive=True`：同一时刻仅一个进程可打开串口（当前为 GUI 侧传送带 worker 进程）
- 物体连续送入、间距不定；漏抓时停带 → 静态抓取 → 重启带
- 动态抓取中所有手动按钮禁用、弹窗屏蔽（沿用现有约定）
- 传送带速度显示四位小数
- 现有 `sort` / `sort_graspnet` / `sort_verify` / `sort_blue_block` 命令全部原样保留
- 碰撞策略沿用现有：下降时禁用传送带碰撞体，回 P_HOVER 后恢复
- 所有代码注释使用中文，与现有代码风格一致

---

### Task 1: 传送带方向标定工具 (conveyor_calib.py)

**Files:**
- Create: `/home/lxf/agx_arm_ws/conveyor_calib.py`
- Test: `/home/lxf/agx_arm_ws/tests/test_conveyor_calib.py`

**Interfaces:**
- Produces: 模块级函数 `fit_belt_direction(xys: list[tuple[float, float]]) -> tuple[float, float, dict]`，返回 `(dx, dy, info)`，其中 `(dx, dy)` 为归一化单位向量（z 分量≈0），`info` 含 `angle_rad`、`n_points`、`residual`；函数 `estimate_belt_speed_vision(samples: list[tuple[float, float, float]]) -> float`（`(t, x, y)` 列表，返回 m/s）；JSON 文件 `conveyor_calib.json`（键：`dx`, `dy`, `speed_scale`, `angle_rad`, `source`）
- Consumes: `/detection_info` 的 `objects[].base_position_m`（`x`,`y`）与 `objects[].header_stamp`（ISO 时间字符串，含微秒），以及 `/conveyor_status` 的 `speed`

#### 设计说明

拟合方向用 PCA 主轴（轨迹点 x/y 的 2D 协方差矩阵最大特征值方向），与物体在匀速传送带上的直线轨迹一致。视觉速度用轨迹点的时间-位移线性拟合斜率。角度计算：`angle = atan2(dy, dx)`，即相机画面横轴与传送带方向的夹角（记录用于诊断）。

- [ ] **Step 1: 写失败测试**

创建 `/home/lxf/agx_arm_ws/tests/test_conveyor_calib.py`：

```python
import math
import os
import sys
import numpy as np
import pytest

sys.path.insert(0, '/home/lxf/agx_arm_ws')
from conveyor_calib import fit_belt_direction, estimate_belt_speed_vision


def _gen_traj(angle_deg, v, t_start=0.0, n=30, dt=0.1, noise=0.001):
    a = math.radians(angle_deg)
    d = np.array([math.cos(a), math.sin(a)])
    pts = []
    for i in range(n):
        t = t_start + i * dt
        p = d * v * (t - t_start)
        p = p + np.random.normal(0, noise, 2)
        pts.append((float(p[0]), float(p[1]), t))
    return pts


def test_fit_belt_direction_recovering_angle():
    np.random.seed(42)
    pts = _gen_traj(angle_deg=37.0, v=0.10)
    dx, dy, info = fit_belt_direction([(x, y) for x, y, _ in pts])
    got = math.degrees(math.atan2(dy, dx)) % 180.0
    assert abs(got - 37.0) < 2.0
    assert abs(math.hypot(dx, dy) - 1.0) < 1e-9
    assert info['n_points'] == 30


def test_fit_belt_direction_double_angle_ambiguity():
    # 直线拟合存在 180° 二义性: 0° 与 180° 等价, 归一化时取 dx>=0 分支
    np.random.seed(1)
    pts = _gen_traj(angle_deg=180.0 + 10.0, v=0.08)
    dx, dy, _ = fit_belt_direction([(x, y) for x, y, _ in pts])
    assert dx >= 0.0  # 归一化保证 dx>=0
    got = math.degrees(math.atan2(dy, dx)) % 180.0
    assert abs(got - 10.0) < 2.0


def test_fit_belt_direction_insufficient_points():
    with pytest.raises(ValueError):
        fit_belt_direction([(0.0, 0.0), (0.01, 0.0)])


def test_estimate_belt_speed_vision_positive():
    np.random.seed(7)
    pts = _gen_traj(angle_deg=0.0, v=0.15)
    v = estimate_belt_speed_vision(pts)
    assert abs(v - 0.15) < 0.01
```

- [ ] **Step 2: 运行测试验证失败**

Run: `cd /home/lxf/agx_arm_ws && python3 -m pytest tests/test_conveyor_calib.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'conveyor_calib'`

- [ ] **Step 3: 写最小实现**

创建 `/home/lxf/agx_arm_ws/conveyor_calib.py`：

```python
#!/usr/bin/env python3
"""传送带方向标定: 用俯视相机采集的 base_position_m 轨迹拟合传送带方向单位向量.

相机画面横轴与传送带不平行(存在夹角), 因此方向必须通过实际运动轨迹拟合,
而不能直接用图像坐标轴. 本模块同时提供视觉速度估计, 用于与 STM32 上报速度交叉校验.
"""
import math
import numpy as np


def fit_belt_direction(xys):
    """对 (x, y) 轨迹点做 PCA 主轴拟合, 返回 (dx, dy, info).

    Args:
        xys: [(x, y), ...] 基坐标系下轨迹点, 至少 3 个且非零跨度.
    Returns:
        (dx, dy, info): 归一化单位向量 (dx>=0 消除 180° 二义性),
                        info 含 angle_rad / n_points / residual.
    Raises:
        ValueError: 点数不足或轨迹退化(零跨度).
    """
    if len(xys) < 3:
        raise ValueError("轨迹点不足: 至少需要 3 个采样点")
    arr = np.asarray(xys, dtype=float).reshape(-1, 2)
    if arr.shape[0] < 3:
        raise ValueError("轨迹点不足: 至少需要 3 个采样点")
    cov = np.cov(arr, rowvar=False)
    if np.linalg.matrix_rank(cov) < 2:
        raise ValueError("轨迹退化: 点几乎共线, 无法确定方向(检查物体是否在动)")
    eigvals, eigvecs = np.linalg.eigh(cov)
    principal = eigvecs[:, int(np.argmax(eigvals))]
    dx, dy = float(principal[0]), float(principal[1])
    # 消除 180° 二义性: 统一取 dx>=0 分支
    if dx < 0.0:
        dx, dy = -dx, -dy
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        raise ValueError("轨迹退化: 主方向模为零")
    dx, dy = dx / norm, dy / norm
    # 残差: 点到拟合直线的平均垂直距离
    proj = arr @ np.array([dx, dy])
    residual = float(np.mean(np.abs(arr - np.outer(proj, [dx, dy])), axis=0).sum() / 2.0)
    info = {
        'angle_rad': math.atan2(dy, dx),
        'n_points': int(arr.shape[0]),
        'residual': residual,
    }
    return dx, dy, info


def estimate_belt_speed_vision(samples):
    """用 (t, x, y) 采样点线性拟合位移速度 (m/s).

    Args:
        samples: [(t, x, y), ...] 时间(秒)与基坐标系位置.
    Returns:
        视觉估计的传送带线速度 (m/s, 非负). 点数不足时返回 0.0.
    """
    if len(samples) < 3:
        return 0.0
    t = np.asarray([s[0] for s in samples], dtype=float)
    p = np.asarray([(s[1], s[2]) for s in samples], dtype=float)
    t0 = t - t[0]
    denom = float(np.sum(t0 * t0))
    if denom < 1e-12:
        return 0.0
    # 位移向量对时间做最小二乘斜率 (向量点积/标量)
    v_vec = np.sum((p - p[0]) * t0[:, None], axis=0) / denom
    return float(np.hypot(v_vec[0], v_vec[1]))
```

- [ ] **Step 4: 运行测试验证通过**

Run: `cd /home/lxf/agx_arm_ws && python3 -m pytest tests/test_conveyor_calib.py -v`
Expected: 4 passed

- [ ] **Step 5: 提交**

```bash
cd /home/lxf/agx_arm_ws && git add conveyor_calib.py tests/test_conveyor_calib.py && git commit -m "feat: add conveyor belt direction calibration fitting"
```

---

### Task 2: /conveyor_status 话题发布（GUI 侧传送带 worker）

**Files:**
- Modify: `/home/lxf/agx_arm_ws/sorting_gui_client.py:134-157`（node `N.__init__`，添加发布器与订阅）
- Modify: `/home/lxf/agx_arm_ws/sorting_gui_client.py:70-91`（`_conv_monitor` 线程，发布状态）

**Interfaces:**
- Consumes: 现有 `_conv.get_status()` 返回的 `speed` / `object_detected` / `motor_running`
- Produces: ROS2 话题 `/conveyor_status`（std_msgs/String，JSON），10Hz，键：`type='conveyor_status'`, `speed`(m/s), `object_detected`(bool, 已反转), `motor_running`(bool), `motor_direction`(int), `connected`(bool)

#### 设计说明

串口 `exclusive=True`，只有 GUI 侧 worker 进程能读 STM32。该 worker 进程内已运行 ROS 节点 `N`，因此在 `_conv_monitor` 线程内用 `N` 的发布器直接 publish（rclpy publish 线程安全）。速度沿用现有滑动窗口滤波（`STM32Conveyor.get_status()` 内部）。

- [ ] **Step 1: 在节点 `N.__init__` 中添加发布器**

修改 `/home/lxf/agx_arm_ws/sorting_gui_client.py` 第 144 行附近，在 `self._emergency_pub` 之后追加：

```python
            # 传送带实时状态发布 (auto_sorting_action.py sort_dynamic 订阅): 速度/物体/电机
            self._conv_status_pub = self.create_publisher(String, '/conveyor_status', 10)
```

- [ ] **Step 2: 在 `_conv_monitor` 线程中发布状态**

修改 `_conv_monitor`（第 71-91 行），在每次 `conv_status_q.put_nowait(...)` 后同步发布到 ROS。替换第 73-87 行的两个分支，改为：

```python
    def _conv_monitor():
        while True:
            try:
                if _conv_ok:
                    s = _conv.get_status()
                    status_msg = {'type': 'conveyor_status',
                                  'speed': s['speed'],
                                  'object_detected': s['object_detected'],
                                  'motor_running': s['motor_running'],
                                  'motor_direction': s.get('motor_direction', 1),
                                  'connected': True}
                    try:
                        conv_status_q.put_nowait({'type': 'status', 'speed': s['speed'],
                                       'object_detected': s['object_detected'],
                                       'motor_running': s['motor_running']})
                    except Exception:
                        pass
                else:
                    status_msg = {'type': 'conveyor_status', 'speed': 0.0,
                                  'object_detected': False, 'motor_running': False,
                                  'motor_direction': 1, 'connected': False}
                    try:
                        conv_status_q.put_nowait({'type': 'status', 'speed': 0.0,
                                       'object_detected': False, 'motor_running': False})
                    except Exception:
                        pass
                try:
                    sm = String(); sm.data = json.dumps(status_msg)
                    node._conv_status_pub.publish(sm)
                except Exception:
                    pass
            except Exception:
                pass
            time.sleep(0.1)
```

注意：`_conv_monitor` 在 `N` 定义之前启动，但其内部只在循环中引用 `node`（运行时已定义）。将发布逻辑放入 `try/except` 防止 `N` 尚未构造时报错。

- [ ] **Step 3: 运行 GUI 验证话题发布**

启动 GUI（确保相机/YOLO/STM32 正常），新开终端运行：

```bash
source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /conveyor_status --once
```

Expected: 输出形如 `data: '{"type": "conveyor_status", "speed": 0.12, ...}'`，speed 为四位小数精度。若 STM32 未连接则 `"connected": false, "speed": 0.0`。

- [ ] **Step 4: 提交**

```bash
cd /home/lxf/agx_arm_ws && git add sorting_gui_client.py && git commit -m "feat: publish /conveyor_status topic from conveyor worker"
```

---

### Task 3: 速度预测模块 velocity_predictor.py

**Files:**
- Create: `/home/lxf/agx_arm_ws/velocity_predictor.py`
- Test: `/home/lxf/agx_arm_ws/tests/test_velocity_predictor.py`

**Interfaces:**
- Consumes: 无（纯逻辑，依赖传入数据）
- Produces: 类 `VelocityPredictor`，方法：
  - `__init__(self, dx: float = 1.0, dy: float = 0.0)`
  - `set_belt_speed(self, v: float)`：设置 STM32 滤波后线速度
  - `set_direction(self, dx: float, dy: float)`：设置标定方向
  - `add_measurement(self, t: float, x: float, y: float)`：记录视觉测量（t 为 `header_stamp` 时刻）
  - `visual_speed(self) -> Optional[tuple[float, float]]`：窗口线性拟合的视觉速度向量，样本<3 返回 None
  - `velocity_vector(self, blend: float = 0.85) -> tuple[float, float]`：融合后的速度向量 `(vx, vy)`
  - `predict(self, t_meas: float, x_meas: float, y_meas: float, t_target: float) -> tuple[float, float]`：常速度模型外推

#### 设计说明

速度向量 = 前馈（`v_stm32 × d̂_base`）为主 + 视觉估计修正（blend 权重）。预测用 `p(t) = p_meas + v×(t_target − t_meas)`，`t_meas` 必须是检测时刻 `header_stamp` 而非接收时刻，以补偿视觉管线延迟。视觉估计对 STM32 速度做交叉验证（诊断用 `visual_speed()`）。

- [ ] **Step 1: 写失败测试**

创建 `/home/lxf/agx_arm_ws/tests/test_velocity_predictor.py`：

```python
import sys
import math
import numpy as np

sys.path.insert(0, '/home/lxf/agx_arm_ws')
from velocity_predictor import VelocityPredictor


def test_velocity_vector_from_direction_and_belt_speed():
    a = math.radians(37.0)
    dx, dy = math.cos(a), math.sin(a)
    p = VelocityPredictor(dx=dx, dy=dy)
    p.set_belt_speed(0.12)
    vx, vy = p.velocity_vector()
    assert abs(vx - dx * 0.12) < 1e-9
    assert abs(vy - dy * 0.12) < 1e-9


def test_visual_speed_returns_none_with_few_samples():
    p = VelocityPredictor()
    p.add_measurement(0.0, 0.0, 0.0)
    p.add_measurement(0.1, 0.0, 0.0)
    assert p.visual_speed() is None


def test_predict_constant_velocity_with_latency_compensation():
    a = math.radians(0.0)  # 传送带沿 x 方向
    p = VelocityPredictor(dx=1.0, dy=0.0)
    p.set_belt_speed(0.10)
    # 测量时刻 t=1.0 时物体在 (0.2, 0.0); 预测到 t=2.0 (间隔 1.0s)
    x, y = p.predict(t_meas=1.0, x_meas=0.2, y_meas=0.0, t_target=2.0)
    assert abs(x - 0.30) < 1e-9
    assert abs(y - 0.0) < 1e-9


def test_visual_speed_recovery_on_moving_object():
    p = VelocityPredictor(dx=1.0, dy=0.0)
    for i in range(10):
        t = i * 0.1
        p.add_measurement(t, 0.05 * t, 0.0)
    vs = p.visual_speed()
    assert vs is not None
    assert abs(vs[0] - 0.05) < 0.01


def test_visual_speed_fuses_into_velocity_vector():
    p = VelocityPredictor(dx=1.0, dy=0.0)
    p.set_belt_speed(0.10)
    for i in range(10):
        p.add_measurement(i * 0.1, 0.08 * i * 0.1, 0.0)  # 视觉估计 ~0.08
    vx, vy = p.velocity_vector(blend=0.85)
    # 前馈 0.10 为主, 视觉 0.08 修正 → 0.85*0.10 + 0.15*0.08 = 0.097
    assert abs(vx - 0.097) < 1e-6
```

- [ ] **Step 2: 运行测试验证失败**

Run: `cd /home/lxf/agx_arm_ws && python3 -m pytest tests/test_velocity_predictor.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'velocity_predictor'`

- [ ] **Step 3: 写最小实现**

创建 `/home/lxf/agx_arm_ws/velocity_predictor.py`：

```python
#!/usr/bin/env python3
"""动态抓取速度预测模块.

融合 STM32 线速度前馈(主)与视觉帧间位移估计(修正), 用常速度模型预测物体
未来位置. 关键: 测量时刻 t_meas 必须是检测的 header_stamp(图像时刻)而非接收
时刻, 以此显式补偿视觉管线延迟(YOLO+深度+EMA+坐标变换).
"""
import math
from typing import Optional, Tuple


class VelocityPredictor:
    """常速度模型预测器. 线程不安全, 由单一线程(动作节点主循环)调用."""

    MAX_SAMPLES = 20

    def __init__(self, dx: float = 1.0, dy: float = 0.0):
        self.dx = dx
        self.dy = dy
        self.v_stm32 = 0.0
        self._measurements = []  # [(t, x, y)]

    def set_belt_speed(self, v: float) -> None:
        self.v_stm32 = max(0.0, float(v))

    def set_direction(self, dx: float, dy: float) -> None:
        n = math.hypot(dx, dy)
        if n > 1e-9:
            self.dx, self.dy = dx / n, dy / n

    def add_measurement(self, t: float, x: float, y: float) -> None:
        self._measurements.append((float(t), float(x), float(y)))
        if len(self._measurements) > self.MAX_SAMPLES:
            self._measurements = self._measurements[-self.MAX_SAMPLES:]

    def visual_speed(self) -> Optional[Tuple[float, float]]:
        """对 (t,x,y) 做窗口线性拟合, 返回视觉估计速度向量; 样本不足返回 None."""
        m = self._measurements
        if len(m) < 3:
            return None
        t0 = m[0][0]
        dt = [t - t0 for t, _, _ in m]
        denom = sum(d * d for d in dt)
        if denom < 1e-12:
            return None
        p0x, p0y = m[0][1], m[0][2]
        vx = sum((x - p0x) * d for (_, x, _), d in zip(m, dt)) / denom
        vy = sum((y - p0y) * d for (_, _, y), d in zip(m, dt)) / denom
        return (vx, vy)

    def velocity_vector(self, blend: float = 0.85) -> Tuple[float, float]:
        """融合速度向量: blend*前馈 + (1-blend)*视觉估计 (视觉不可用时纯前馈)."""
        fx, fy = self.v_stm32 * self.dx, self.v_stm32 * self.dy
        vs = self.visual_speed()
        if vs is None:
            return (fx, fy)
        return (blend * fx + (1.0 - blend) * vs[0],
                blend * fy + (1.0 - blend) * vs[1])

    def predict(self, t_meas: float, x_meas: float, y_meas: float,
                t_target: float) -> Tuple[float, float]:
        """常速度外推: p(t_target) = p_meas + v*(t_target - t_meas)."""
        dt = float(t_target) - float(t_meas)
        vx, vy = self.velocity_vector()
        return (float(x_meas) + vx * dt, float(y_meas) + vy * dt)
```

- [ ] **Step 4: 运行测试验证通过**

Run: `cd /home/lxf/agx_arm_ws && python3 -m pytest tests/test_velocity_predictor.py -v`
Expected: 5 passed

- [ ] **Step 5: 提交**

```bash
cd /home/lxf/agx_arm_ws && git add velocity_predictor.py tests/test_velocity_predictor.py && git commit -m "feat: add velocity predictor for dynamic grasping"
```

---

### Task 4: auto_sorting_action.py 集成预测与 /conveyor_status

**Files:**
- Modify: `/home/lxf/agx_arm_ws/auto_sorting_action.py`
  - `__init__` 内（第 316 行 `det_sub` 之后）：添加 `/conveyor_status` 订阅
  - `_detection_cb`（第 461 行）附近：添加动态抓取辅助方法
  - 常量区（`GRIPPER_PICK_Z_OFFSET` 之后，第 2528 行附近）：添加动态抓取常量

**Interfaces:**
- Consumes: `/conveyor_status`（Task 2）、`conveyor_calib.json`（Task 1）、`VelocityPredictor`（Task 3）、现有 `_latest_detection` / `_create_pose` / `_build_pick_orientations_multi` / `move_arm_pose` / `move_arm_cartesian` / `execute_cartesian_path` / `operate_gripper`
- Produces: 节点属性 `self._belt_speed`（m/s）、`self._belt_direction`（int）、`self._predictor`（VelocityPredictor 实例）、`self._dynamic_calib`（dict 或 None）；方法 `_load_dynamic_calib()`、`_dynamic_find_object(objects, target_name)`、`_dynamic_current_xy(target_name)`、`_run_dynamic_grasp(pick_id, bin_num, object_diameter_m)`（供 Task 5 命令分支调用）

- [ ] **Step 1: 导入并订阅 /conveyor_status**

在文件顶部 import 区（`from std_msgs.msg import String` 之后）添加：

```python
# ── 动态抓取: 传送带状态订阅 + 速度预测 ──
from velocity_predictor import VelocityPredictor
```

在 `__init__` 中 `self.det_sub = ...`（第 316 行）之后添加：

```python
        # ── 动态抓取: 订阅传送带状态 (速度/方向/物体检测) ──
        self._belt_speed = 0.0
        self._belt_direction = 1
        self._predictor = VelocityPredictor()
        self._dynamic_calib = None
        self._conveyor_status_sub = self.create_subscription(
            String, '/conveyor_status', self._conveyor_status_cb, 10)
        self._load_dynamic_calib()
```

在 `_detection_cb`（第 461 行）之前添加回调：

```python
    def _conveyor_status_cb(self, msg):
        """缓存传送带状态 (STM32 滤波后速度), 供动态抓取预测."""
        try:
            d = json.loads(msg.data)
            if d.get('type') == 'conveyor_status':
                self._belt_speed = float(d.get('speed', 0.0))
                self._belt_direction = int(d.get('motor_direction', 1))
                self._predictor.set_belt_speed(self._belt_speed)
        except Exception:
            pass

    def _load_dynamic_calib(self):
        """加载传送带方向标定 conveyor_calib.json, 初始化预测器方向."""
        import os
        path = '/home/lxf/agx_arm_ws/conveyor_calib.json'
        if not os.path.exists(path):
            self.get_logger().warn(f"⚠️ 未找到传送带标定文件 {path}, 动态抓取将退化为静态抓取")
            return
        try:
            with open(path, 'r') as f:
                calib = json.load(f)
            self._dynamic_calib = calib
            self._predictor.set_direction(float(calib['dx']), float(calib['dy']))
            self.get_logger().info(
                f"✅ 传送带方向标定已加载: d̂=({calib['dx']:.4f},{calib['dy']:.4f}) "
                f"angle={calib.get('angle_rad', 0.0):.3f}rad")
        except Exception as e:
            self.get_logger().warn(f"⚠️ 传送带标定加载失败: {e}")
```

- [ ] **Step 2: 添加动态抓取辅助方法**

在 `_get_fresh_detection`（第 509 行）之后添加：

```python
    def _dynamic_find_object(self, objects, target_name):
        """从检测列表中找到目标物体(名称匹配), 返回该物体的 dict."""
        target_key = (target_name or '').split(' (')[0].strip().lower()
        for obj in objects or []:
            name = (obj.get('name', '') or '').lower().strip()
            if target_key and (target_key in name or name in target_key):
                return obj
        return None

    def _dynamic_current_xy(self, target_name, max_age_s=2.0):
        """返回目标物体最新基坐标系位置 (x, y) 与测量时刻 (秒).

        使用 header_stamp(图像时刻) 而非接收时刻, 供 VelocityPredictor 延迟补偿.
        返回 (x, y, t_meas) 或 None.
        """
        det = self._latest_detection
        if not det or not det.get('detected'):
            return None
        obj = self._dynamic_find_object(det.get('objects', []), target_name)
        if obj is None:
            return None
        bp = obj.get('base_position_m')
        if not bp or 'x' not in bp or 'y' not in bp:
            return None
        hs = obj.get('header_stamp')
        t_meas = time.time()
        if isinstance(hs, str) and hs:
            try:
                # ISO 格式 "2026-08-22 12:34:56.789123" (可含 T/Z)
                hs_clean = hs.replace('T', ' ').replace('Z', '')
                t_meas = time.mktime(time.strptime(hs_clean[:19], '%Y-%m-%d %H:%M:%S'))
                if len(hs_clean) > 20:
                    t_meas += float('0.' + hs_clean[20:])
            except Exception:
                t_meas = time.time()
        if time.time() - t_meas > max_age_s:
            return None
        return (float(bp['x']), float(bp['y']), t_meas)
```

- [ ] **Step 3: 添加动态抓取常量**

在 `MIN_GRASP_Z = 0.045`（第 2536 行）之后添加：

```python
    # ── 动态抓取 (sort_dynamic) ──
    # P_HOVER: 抓取区上方悬停位 (base 系, link6 末端位置, 待用户标定后替换示例值)
    # 要求: 位于抓取区正上方、安全高度、远离传送带碰撞体
    P_HOVER_POSE = {'x': 0.45, 'y': 0.0, 'z': 0.32}      # 示例值, 需标定
    P_HOVER_ORIENTATION = [0.0, 0.0, -1.0, 0.0]          # 简化示例, 需按末端实际姿态标定
    # 动态抓取前瞻名义耗时 (秒): 移到影子点 / 下降 / 抬起 的名义用时,
    # 用于计算目标前瞻量 (物体位移 = 速度 × 名义耗时). 空跑测试后细调.
    SHADOW_LEAD_S = 1.0
    DESCENT_NOMINAL_S = 0.6
    LIFT_LEAD_S = 0.3
    # 动态抓取最低下降高度 (base 系, 米); 与静态 MIN_GRASP_Z 含义相同, 兜底防碰
    DYNAMIC_MIN_GRASP_Z = 0.045
```

- [ ] **Step 4: 运行语法检查与单元验证**

Run: `cd /home/lxf/agx_arm_ws && python3 -m py_compile auto_sorting_action.py velocity_predictor.py conveyor_calib.py`
Expected: 无输出（编译通过）

Run: `cd /home/lxf/agx_arm_ws && python3 -m pytest tests/test_velocity_predictor.py tests/test_conveyor_calib.py -q`
Expected: 9 passed（确认未破坏既有模块）

- [ ] **Step 5: 提交**

```bash
cd /home/lxf/agx_arm_ws && git add auto_sorting_action.py && git commit -m "feat: integrate conveyor status and velocity predictor into action node"
```

---

### Task 5: sort_dynamic 命令：斜线随动下降执行序列

**Files:**
- Modify: `/home/lxf/agx_arm_ws/auto_sorting_action.py`
  - `main()` 命令分发（`elif data.get("cmd") in ("sort", ...)` 之后，`sort_blue_block` 之前，第 3160 行附近）：添加 `sort_dynamic` 分支
  - 新增方法 `_run_dynamic_grasp(...)`（放在 `move_arm_cartesian` 定义之后）
  - 新增方法 `_dynamic_disable_conveyor_collision()` 与 `_dynamic_restore_conveyor_collision()`（复用现有碰撞禁用/恢复逻辑）

**Interfaces:**
- Consumes: Task 4 的 `self._predictor` / `self._belt_speed` / `_dynamic_current_xy` / `_load_dynamic_calib`；现有 `_create_pose` / `move_arm_pose` / `execute_cartesian_path` / `operate_gripper` / 碰撞 ACM 逻辑；`compute_gripper_targets`
- Produces: `sort_dynamic` 命令完整执行：预测影子点 → 斜线随动下降 → 夹取 → 带前向分量抬起 → 放料框 → 回 P_HOVER；返回 bool 成功与否

#### 设计说明

命令入参（GUI 下发 JSON）：`pick`（初始位置 dict `{'x','y','z'}`）、`pick_name`（物体名，如 "apple (detected)"）、`bin`（1/2）、`object_diameter_m`。动作节点在**每次运动前**用最新检测 + 预测器刷新目标：
1. 无标定文件 → 直接返回 False（GUI 回退静态抓取）
2. 横向影子对齐：目标 = 最新预测位置 + `v×SHADOW_LEAD_S` 前瞻，末端降到影子高度 `z_shadow`，用 `move_arm_pose`（笛卡尔规划）
3. 斜线随动下降：目标 = 预测位置 + `v×DESCENT_NOMINAL_S` 前瞻，z = 物体表面 z + `GRIPPER_PICK_Z_OFFSET`（夹取高度），用 `execute_cartesian_path` 走一条斜线（从影子点到抓取点），水平位移 ≈ `v×DESCENT_NOMINAL_S`。下降前禁用传送带碰撞体
4. 夹取：`operate_gripper(GRIPPER_GRAB)` + 力矩判定（复用静态逻辑的判定函数）
5. 抬起带前向分量：先水平平移 `v×LIFT_LEAD_S` 同时抬升到安全高度（一次笛卡尔路径），避免夹爪拖拽物体
6. 放料框：`JOINT_BIN{bin}_ABOVE → JOINT_BIN{bin}_PLACE → 开夹爪 → JOINT_BIN{bin}_ABOVE`
7. 回 P_HOVER，恢复碰撞体
8. 任一运动失败 → 回退到观察位恢复碰撞，返回 False（GUI 触发静态兜底）

- [ ] **Step 1: 添加碰撞体禁用/恢复辅助方法**

在 `move_arm_cartesian` 定义结束（第 1560 行附近，下一个 `def` 之前）添加：

```python
    def _dynamic_disable_conveyor_collision(self):
        """动态抓取: 临时禁用传送带碰撞体与机械臂的碰撞检测 (供斜线下降).
        复用现有 PlanningScene ACM diff 机制. 调用前需 sleep 0.15s 供 move_group 处理."""
        try:
            allowed = [
                ('arm', 'belt_deck'), ('arm', 'conveyor_belt_col'),
                ('arm', 'rail_ny'), ('arm', 'control_box'),
                ('arm', 'conveyor_control_box'),
            ]
            acm = AllowedCollisionEntry()
            for link_a, link_b in allowed:
                acm.model_a_name = link_a
                acm.model_b_name = link_b
                acm.entry_names.append(link_b)
                acm.enabled.append(True)
            diff = PlanningSceneMsg()
            diff.is_diff = True
            diff.allowed_collision_matrix.entry_names = [link_a]
            diff.allowed_collision_matrix.entry_values = [acm]
            self._planning_scene_pub.publish(diff)
            time.sleep(0.15)
            self.get_logger().info("🧩 动态抓取: 已禁用传送带碰撞体碰撞检测")
        except Exception as e:
            self.get_logger().warn(f"⚠️ 禁用碰撞体失败: {e}")

    def _dynamic_restore_conveyor_collision(self):
        """动态抓取: 恢复传送带碰撞检测 (回 P_HOVER 后调用)."""
        try:
            acm = AllowedCollisionEntry()
            acm.model_a_name = 'arm'
            acm.entry_names.append('__all__')
            acm.enabled.append(False)
            diff = PlanningSceneMsg()
            diff.is_diff = True
            diff.allowed_collision_matrix.entry_names = ['arm']
            diff.allowed_collision_matrix.entry_values = [acm]
            self._planning_scene_pub.publish(diff)
            time.sleep(0.15)
            self.get_logger().info("🧩 动态抓取: 已恢复传送带碰撞检测")
        except Exception as e:
            self.get_logger().warn(f"⚠️ 恢复碰撞体失败: {e}")
```

- [ ] **Step 2: 添加 _run_dynamic_grasp 方法**

在碰撞辅助方法之后添加：

```python
    def _run_dynamic_grasp(self, pick_id, bin_num, object_diameter_m, start_pose):
        """动态抓取核心序列: 影子对齐 → 斜线随动下降 → 夹取 → 抬起 → 放置.

        Args:
            pick_id: 物体名称 (如 "apple (detected)")
            bin_num: 料框编号 1/2
            object_diameter_m: 物体直径 (m), 用于夹爪开度
            start_pose: {'x','y','z'} 命令下发时的检测位置 (备用)
        Returns:
            bool: 本次动态抓取是否成功
        """
        if not self._dynamic_calib:
            self.get_logger().warn("⚠️ 无传送带标定, 动态抓取不可用")
            return False

        v = self._belt_speed
        if v <= 0.0:
            self.get_logger().warn("⚠️ 传送带速度不可用(v<=0), 动态抓取不可用")
            return False

        # ── 1. 读取目标最新位置 (含延迟补偿的测量时刻) ──
        xy = self._dynamic_current_xy(pick_id)
        if xy is None:
            self.get_logger().warn("⚠️ 动态抓取: 未获取到目标最新位置, 回退静态")
            return False
        x_now, y_now, t_meas = xy

        # 物体表面 z (base 系), 决定夹取高度
        det = self._latest_detection
        obj = self._dynamic_find_object(det.get('objects', []), pick_id)
        surface_z = float(obj.get('base_position_m', {}).get('z', 0.0)) if obj else float(start_pose.get('z', 0.0))
        grasp_z = max(surface_z + GRIPPER_PICK_Z_OFFSET, MIN_GRASP_Z)

        # ── 2. 影子点 (横向对齐): 目标 = 预测位置 + v*SHADOW_LEAD_S 前瞻 ──
        t_shadow = time.time()
        self._predictor.add_measurement(t_meas, x_now, y_now)
        sx, sy = self._predictor.predict(t_meas, x_now, y_now, t_shadow + SHADOW_LEAD_S)
        z_shadow = P_HOVER_POSE['z']
        self.get_logger().info(
            f"🎯 动态: 影子点=({sx:.3f},{sy:.3f},z={z_shadow:.3f}) 速度={v:.4f} m/s")
        shadow_ok = self.move_arm_pose(
            {'x': sx, 'y': sy, 'z': z_shadow},
            "动态-影子对齐", continuous=False, planning_mode='normal',
            preferred_orientation=self._build_pick_orientation({'x': sx, 'y': sy}))
        if not shadow_ok:
            self.get_logger().warn("⚠️ 动态: 影子点运动失败")
            return False

        # ── 3. 斜线随动下降: 目标 = 预测位置 + v*DESCENT_NOMINAL_S 前瞻 ──
        xy2 = self._dynamic_current_xy(pick_id)
        if xy2 is not None:
            x_now, y_now, t_meas = xy2
            self._predictor.add_measurement(t_meas, x_now, y_now)
        t_descent = time.time()
        gx, gy = self._predictor.predict(t_meas, x_now, y_now, t_descent + DESCENT_NOMINAL_S)
        # 目标姿态: 用抓取点生成的 PCA 短轴候选 (外部已传入 start_pose 无姿态, 用径向候选)
        orientations = self._build_pick_orientations_multi({'x': gx, 'y': gy})
        self._dynamic_disable_conveyor_collision()
        waypoints = [self._create_pose({'x': gx, 'y': gy, 'z': grasp_z}, orientations[0])]
        self.get_logger().info(
            f"🎯 动态: 下降目标=({gx:.3f},{gy:.3f},z={grasp_z:.3f}) 前瞻={DESCENT_NOMINAL_S}s")
        descent_ok = self.execute_cartesian_path(
            waypoints, "动态-斜线随动下降", fraction_threshold=0.80, jump_threshold=0.8)
        if not descent_ok:
            self._dynamic_restore_conveyor_collision()
            self.get_logger().warn("⚠️ 动态: 斜线下降失败")
            return False

        # ── 4. 夹取: 一次性闭合 + 力矩判定 ──
        if object_diameter_m:
            _, close_target = compute_gripper_targets(object_diameter_m)
        else:
            close_target = GRIPPER_CLOSE
        self._pre_grasp_width = self._latest_gripper_status.width if self._latest_gripper_status else None
        grasp_ok = self.operate_gripper(close_target, "动态-闭合夹爪")
        if not grasp_ok:
            self._dynamic_restore_conveyor_collision()
            self.get_logger().warn("⚠️ 动态: 夹爪闭合失败")
            return False

        # ── 5. 带前向分量抬起 (水平平移 + 抬升, 一次笛卡尔路径) ──
        xy3 = self._dynamic_current_xy(pick_id)
        lift_dx, lift_dy = 0.0, 0.0
        if xy3 is not None:
            x3, y3, t3 = xy3
            self._predictor.add_measurement(t3, x3, y3)
            lx, ly = self._predictor.predict(t3, x3, y3, time.time() + LIFT_LEAD_S)
            lift_dx, lift_dy = lx - gx, ly - gy
        lift_z = z_shadow + 0.05
        lift_ok = self.execute_cartesian_path(
            [self._create_pose({'x': gx + lift_dx, 'y': gy + lift_dy, 'z': lift_z},
                               self._build_pick_orientation({'x': gx + lift_dx, 'y': gy + lift_dy})[0])],
            "动态-带前向抬起", fraction_threshold=0.80)
        if not lift_ok:
            self._dynamic_restore_conveyor_collision()
            self.get_logger().warn("⚠️ 动态: 抬起失败, 尝试原地抬升")
            self.move_arm_pose({'x': gx, 'y': gy, 'z': lift_z}, "动态-原地抬升")

        # ── 6. 放料框 ──
        bin_above, bin_place = BIN_JOINTS[bin_num]
        self.move_arm_joint(bin_above, f"动态-料框{bin_num}上方")
        self.move_arm_joint(bin_place, f"动态-料框{bin_num}放置")
        self.operate_gripper(GRIPPER_OPEN, f"动态-料框{bin_num}释放")
        self.move_arm_joint(bin_above, f"动态-料框{bin_num}上方")

        # ── 7. 回 P_HOVER 并恢复碰撞 ──
        self.move_arm_pose(P_HOVER_POSE, "动态-回悬停位", continuous=False,
                           planning_mode='normal',
                           preferred_orientation=P_HOVER_ORIENTATION)
        self._dynamic_restore_conveyor_collision()
        self.get_logger().info("✅ 动态抓取完成")
        return True
```

注意：`compute_gripper_targets` 定义在 `main()` 内（局部函数），`_run_dynamic_grasp` 是节点方法，需通过闭包/参数传递。将 `compute_gripper_targets` 作为参数传入，或在调用处用局部函数包装。**本步骤在第 6 步的命令分发里以闭包方式调用**（见 Step 5 说明）。

- [ ] **Step 3: 运行语法检查**

Run: `cd /home/lxf/agx_arm_ws && python3 -m py_compile auto_sorting_action.py`
Expected: 无输出（编译通过）。若 `compute_gripper_targets` 引用报错，属预期——在 Step 5 通过闭包参数传递解决。

- [ ] **Step 4: 添加 sort_dynamic 命令分发**

在 `main()` 命令分发中，`elif data.get("cmd") in ("sort", "sort_verify", "sort_graspnet"):` 分支结束后、`elif data.get("cmd") == "sort_blue_block":`（第 3162 行）之前，添加：

```python
            elif data.get("cmd") == "sort_dynamic":
                # 动态抓取: 传送带持续运行, 机械臂预测并拦截移动物体
                # 依赖: conveyor_calib.json (方向标定) + /conveyor_status (STM32 速度)
                node.is_busy = True
                status_msg.data = 'busy'
                node.status_pub.publish(status_msg)
                node.last_planning_profile_name = ''
                node.last_planning_strategy = ''
                try:
                    pick_id = str(data.get("pick_name", "Pick"))
                    bin_num = int(data.get("bin", 1))
                    if bin_num not in BIN_JOINTS:
                        bin_num = 1
                    object_diameter_m = data.get("object_diameter_m", None)
                    start_pose = data.get("pick", {})
                    ok = node._run_dynamic_grasp(
                        pick_id, bin_num, object_diameter_m, start_pose,
                        compute_gripper_targets)
                except Exception as e:
                    print(f"❌ 动态抓取异常: {e}")
                    import traceback; traceback.print_exc()
                    ok = False
                if not ok:
                    # 动态抓取失败 → 回退静态抓取 (传送带已由 GUI 停止/或将停止)
                    node.get_logger().warn("⚠️ 动态抓取失败, 回退静态抓取流程")
                    node.move_arm_joint(JOINT_STANDBY, "动态失败-回待机位")
                    node.operate_gripper(GRIPPER_OPEN, "动态失败-张开夹爪")
                    result = {'ok': False, 'cmd': 'sort_dynamic',
                              'msg': '动态抓取失败(已回退)'}
                else:
                    result = {'ok': True, 'cmd': 'sort_dynamic',
                              'msg': '动态抓取成功'}
                cycle_result_msg.data = json.dumps(result)
                node.cycle_result_pub.publish(cycle_result_msg)
                node.is_busy = False
                status_msg.data = 'idle'
                node.status_pub.publish(status_msg)
```

同时修改 `_run_dynamic_grasp` 签名，将 `compute_gripper_targets` 作为第 5 个参数（在 Step 2 代码基础上修改方法签名行）：

```python
    def _run_dynamic_grasp(self, pick_id, bin_num, object_diameter_m, start_pose, compute_gripper_targets):
```

- [ ] **Step 5: 运行语法与逻辑检查**

Run: `cd /home/lxf/agx_arm_ws && python3 -m py_compile auto_sorting_action.py`
Expected: 无输出（编译通过）

Run: `cd /home/lxf/agx_arm_ws && python3 - <<'EOF'
import ast, sys
src = open('auto_sorting_action.py').read()
tree = ast.parse(src)
assert 'sort_dynamic' in src, 'sort_dynamic 分支缺失'
assert '_run_dynamic_grasp' in src, '_run_dynamic_grasp 方法缺失'
assert '_dynamic_disable_conveyor_collision' in src, '碰撞禁用方法缺失'
assert '_dynamic_restore_conveyor_collision' in src, '碰撞恢复方法缺失'
assert 'P_HOVER_POSE' in src, 'P_HOVER 常量缺失'
print('OK: 动态抓取相关符号齐全')
EOF`
Expected: `OK: 动态抓取相关符号齐全`

- [ ] **Step 6: 提交**

```bash
cd /home/lxf/agx_arm_ws && git add auto_sorting_action.py && git commit -m "feat: add sort_dynamic slanted tracking descent command"
```

---

### Task 6: GUI 动态抓取模式 + 下游漏抓分界线

**Files:**
- Modify: `/home/lxf/agx_arm_ws/sorting_gui_client.py`
  - 初始化区（`self._auto_sort_line_u=260` 附近，第 389 行）：添加动态模式状态字段与分界线参数
  - GUI 按钮区：添加"动态抓取"启动/停止按钮（沿用自动分拣的紫/橙配色约定）
  - 状态机区域（`_pick_object_right_of_line` 之后）：添加动态模式状态机方法
  - 相机画布绘制（`_draw_auto_sort_line` 附近）：绘制下游漏抓分界线
  - `_update_status_loop` / 手动按钮禁用逻辑：动态模式运行时保持手动按钮禁用

**Interfaces:**
- Consumes: 现有 `_conveyor_set(run)`、`_dispatch_sort_cmd`、`latest_detection`、`conveyor_status_queue`、`_pick_object_right_of_line`；Task 2 的 `/conveyor_status`
- Produces: GUI 字段 `self.dynamic_mode`（bool）、`self._dynamic_state`（'OFF'/'RUNNING'/'MISS_STOP'/'STOPPING'）、`self._dynamic_trigger_line_u`（int，默认 200）、`self._dynamic_miss_line_u`（int，默认 560，可调）；按钮"动态抓取"/"停止动态抓取"

#### 设计说明

动态模式状态机（GUI 主线程，复用现有 `_dispatch_sort_cmd` 模式）：
- `OFF`：默认。点"动态抓取"→ 若传送带未运行则 `_conveyor_set(True)` → `RUNNING`
- `RUNNING`：传送带持续运行。每轮检查（`status==idle` 时）：
  1. 下游漏抓分界线右侧有物体（`_pick_object_right_of_line` 用 `_dynamic_miss_line_u`）→ `MISS_STOP`
  2. 否则上游触发线右侧有物体（`_pick_object_right_of_line` 用 `_dynamic_trigger_line_u`）→ 下发 `sort_dynamic`
  3. 否则等待
- `MISS_STOP`：`_conveyor_set(False)` → 等传送带停 → 下发静态 `sort` → 等 `idle` → `_conveyor_set(True)` → `RUNNING`
- `STOPPING`：点"停止动态抓取"→ 若忙则等完成 → `_conveyor_set(False)` → 回待机位 → `OFF`，恢复按钮
- 动态模式下：禁用手动按钮、屏蔽 messagebox（沿用 `auto_sort_running` 逻辑，扩展为 `dynamic_mode or auto_sort_running`）

- [ ] **Step 1: 添加状态字段**

在 `self._auto_sort_line_u=260`（第 389 行）之后添加：

```python
            # ── 动态抓取模式 (传送带持续运行, 移动拦截) ──
            self.dynamic_mode = False
            self._dynamic_state = 'OFF'      # OFF/RUNNING/MISS_STOP/STOPPING
            self._dynamic_trigger_line_u = 200   # 上游触发线: 物体越线进入跟踪区
            self._dynamic_miss_line_u = 560      # 下游漏抓分界线 (可调): 越线即漏抓
            self._dynamic_busy = False
```

- [ ] **Step 2: 添加按钮**

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

- [ ] **Step 3: 添加状态机方法**

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

注意：`_pick_object_right_of_line` 当前签名使用 `self._auto_sort_line_u` 硬编码，需改造为可传 `line_u` 参数。修改第 1472-1479 行签名为：

```python
        def _pick_object_right_of_line(self, line_u=None, require_base=True):
            """筛选 x2 ≥ 分界线 且有 base_position_m 的物体, 返回 center_u 最大的.

            自动分拣用 self._auto_sort_line_u (默认), 动态模式用传入 line_u.
            """
            line_u = self._auto_sort_line_u if line_u is None else line_u
            d = self.latest_detection
            if not d or not d.get('detected'): return None
```

并检查该函数后续对 `self._auto_sort_line_u` 的引用改为 `line_u`（第 1488 行 `if x2 >= self._auto_sort_line_u:` → `if x2 >= line_u:`），以及 base_position_m 存在性过滤（`require_base` 为 True 时跳过无 base_position_m 的物体）。

- [ ] **Step 4: 绘制下游漏抓分界线**

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

- [ ] **Step 5: 同步手动按钮禁用与弹窗屏蔽**

在现有 `_conveyor_done`（第 1357 行）与 `_update_status_loop`（第 1437/1447 行）中，把 `if not self.auto_sort_running:` 扩展为 `if not (self.auto_sort_running or self.dynamic_mode):`，确保动态模式运行时手动按钮保持禁用、错误不弹窗。

- [ ] **Step 6: 语法检查与提交**

Run: `cd /home/lxf/agx_arm_ws && python3 -m py_compile sorting_gui_client.py`
Expected: 无输出（编译通过）

Run: `cd /home/lxf/agx_arm_ws && python3 - <<'EOF'
import ast
src = open('sorting_gui_client.py').read()
for token in ('_dynamic_start', '_dynamic_stop', '_dynamic_tick',
              '_dynamic_trigger_line_u', '_dynamic_miss_line_u',
              'sort_dynamic', 'line_u=None'):
    assert token in src, f'缺失符号: {token}'
print('OK: 动态抓取 GUI 符号齐全')
EOF`
Expected: `OK: 动态抓取 GUI 符号齐全`

- [ ] **Step 7: 提交**

```bash
cd /home/lxf/agx_arm_ws && git add sorting_gui_client.py && git commit -m "feat: add dynamic grasping mode and downstream miss-line fallback to GUI"
```

---

### Task 7: 端到端验证（空跑 + 递进速度）

**Files:**
- 无新增代码（验证流程），若需微调参数直接修改 `auto_sorting_action.py` 中的动态常量或 GUI 中的分界线

**Interfaces:**
- Consumes: Task 1-6 全部产物

#### 设计说明

先空跑验证预测轨迹（不夹取），再逐级提速实测。打点对比预测与实际到达位置/时刻。所有验证需在真实硬件上完成。

- [ ] **Step 1: 标定传送带方向**

```bash
cd /home/lxf/agx_arm_ws && source /opt/ros/humble/setup.bash && \
python3 - <<'EOF'
# 手动标定脚本: 需 GUI + STM32 worker 运行以提供 /conveyor_status
# 步骤: 放置特征物体在传送带上 → 以固定速度运行 2-3 秒
import rclpy, json, math, time
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
rclpy.init()
class Calib(Node):
    def __init__(self):
        super().__init__('belt_calib_cli')
        self.samples = []
        self.speed_samples = []
        self.create_subscription(String, '/detection_info', self._det, 10)
        self.create_subscription(String, '/conveyor_status', self._spd, 10)
    def _det(self, m):
        try:
            d = json.loads(m.data)
            for o in d.get('objects', []):
                bp = o.get('base_position_m')
                hs = o.get('header_stamp', '')
                if bp and 'x' in bp:
                    t = time.time()
                    try:
                        c = hs.replace('T',' ').replace('Z','')
                        t = time.mktime(time.strptime(c[:19], '%Y-%m-%d %H:%M:%S'))
                        if len(c) > 20: t += float('0.'+c[20:])
                    except Exception: pass
                    self.samples.append((t, bp['x'], bp['y']))
        except Exception: pass
    def _spd(self, m):
        try:
            d = json.loads(m.data)
            if d.get('type') == 'conveyor_status':
                self.speed_samples.append((time.time(), d.get('speed', 0.0)))
        except Exception: pass
node = Calib()
print("⏳ 正在采集 3 秒轨迹... 请确保传送带以固定速度运行且物体在视野内")
end = time.time() + 3.0
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
from conveyor_calib import fit_belt_direction, estimate_belt_speed_vision
samples = node.samples
if len(samples) < 3:
    print("❌ 采集点不足, 请重试"); rclpy.shutdown(); raise SystemExit(1)
xys = [(s[1], s[2]) for s in samples]
dx, dy, info = fit_belt_direction(xys)
v_vis = estimate_belt_speed_vision(samples)
v_stm = max([s[1] for s in node.speed_samples], default=0.0)
scale = (v_vis / v_stm) if v_stm > 1e-6 else 1.0
out = {'dx': dx, 'dy': dy, 'speed_scale': scale,
       'angle_rad': info['angle_rad'], 'source': 'belt_calib_cli',
       'v_visual_mps': v_vis, 'v_stm32_mps': v_stm, 'n_points': len(samples)}
with open('/home/lxf/agx_arm_ws/conveyor_calib.json', 'w') as f:
    json.dump(out, f, indent=2)
print(f"✅ 标定完成: d̂=({dx:.4f},{dy:.4f}) angle={math.degrees(info['angle_rad']):.2f}°")
print(f"   视觉速度={v_vis:.4f} m/s, STM32速度={v_stm:.4f} m/s, 比例={scale:.3f}")
rclpy.shutdown()
EOF`
```

Expected: 输出 `d̂=(dx,dy)` 与角度；`conveyor_calib.json` 生成。若 `scale` 明显偏离 1.0（>5%），记录并在 `VelocityPredictor` 中乘入 `speed_scale`（后续微调任务）。

- [ ] **Step 2: 空跑验证（不夹取）**

修改 `_run_dynamic_grasp` 第 4 步前临时加 `if True: return True`（空跑模式），运行 GUI 动态模式并记录日志：
- Expected: 日志显示"影子点=... 速度=..."与"下降目标=..."，机械臂沿斜线下降到抓取高度后回 P_HOVER，无碰撞、无 -4 错误
- 对照相机画面：下降目标点应落在物体实际到达位置附近（误差 < 5cm）
- 空跑验证通过后移除空跑临时开关

- [ ] **Step 3: 低速实测 0.02 m/s**

传送带设 0.02 m/s，GUI 启动动态抓取，逐个物体验证：连续抓取 ≥ 5 个物体，成功率 100%，无漏抓分界线误触发。

- [ ] **Step 4: 中速实测 0.08 m/s**

同 Step 3，连续抓取 ≥ 5 个，成功率 ≥ 80%。若失败：调大/调小 `SHADOW_LEAD_S` 与 `DESCENT_NOMINAL_S`（步长 0.1s），记录对预测误差的影响。

- [ ] **Step 5: 高速实测 0.15 m/s**

同 Step 3，连续抓取 ≥ 5 个。漏抓时验证漏抓分界线兜底：物体越过 `_dynamic_miss_line_u` → 传送带停 → 静态抓取 → 重启带。

- [ ] **Step 6: 参数固化与提交**

将验证中确定的 `SHADOW_LEAD_S` / `DESCENT_NOMINAL_S` / `_dynamic_miss_line_u` 等最终值写回代码并提交：

```bash
cd /home/lxf/agx_arm_ws && git add auto_sorting_action.py sorting_gui_client.py && git commit -m "tune: finalize dynamic grasping timing constants after field validation"
```

---

## Self-Review

**1. Spec coverage:**
- 一、传送带方向标定 → Task 1（`conveyor_calib.py` + 标定 CLI 在 Task 7 Step 1）✅
- 二、`/conveyor_status` 话题 + VelocityPredictor → Task 2、Task 3 ✅
- 三、斜线随动下降序列（P_HOVER→影子→斜线下降→夹取→抬起→放置）→ Task 5 `_run_dynamic_grasp` ✅
- 四、GUI 动态抓取模式状态机 + 按钮 + 停止 → Task 6 ✅
- 五、视觉分界线漏抓兜底（下游可调线，IR 辅助）→ Task 6 `_dynamic_tick` 的 MISS_STOP 分支（IR 降级为参考，不在本计划实现判定主逻辑）✅
- 六、错误回退 + 递进测试 → Task 5（失败回退静态）+ Task 7（空跑/0.02/0.08/0.15）✅
- 眼在手相机决策（第一版不启用）→ 无需任务 ✅

**2. Placeholder scan:** 全部步骤含具体代码与命令；无 "TBD/TODO/适当处理"。P_HOVER 示例值标注"待标定"但提供替换入口，属参数而非占位符。

**3. Type consistency:** `VelocityPredictor`（Task 3）方法名/签名在 Task 4/5 中的调用一致；`fit_belt_direction` 返回 `(dx, dy, info)` 在 Task 1 测试与 Task 7 CLI 中一致；`_pick_object_right_of_line` 的 `line_u` 参数在 Task 6 中一致；`_run_dynamic_grasp` 第 5 参数 `compute_gripper_targets` 在 Task 5 Step 4 的闭包调用中一致。
