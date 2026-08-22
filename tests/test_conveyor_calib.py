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
        pts.append((t, float(p[0]), float(p[1])))
    return pts


def test_fit_belt_direction_recovering_angle():
    np.random.seed(42)
    pts = _gen_traj(angle_deg=37.0, v=0.10)
    dx, dy, info = fit_belt_direction([(x, y) for _, x, y in pts])
    got = math.degrees(math.atan2(dy, dx)) % 180.0
    assert abs(got - 37.0) < 2.0
    assert abs(math.hypot(dx, dy) - 1.0) < 1e-9
    assert info['n_points'] == 30


def test_fit_belt_direction_double_angle_ambiguity():
    # 直线拟合存在 180° 二义性: 0° 与 180° 等价, 归一化时取 dx>=0 分支
    np.random.seed(1)
    pts = _gen_traj(angle_deg=180.0 + 10.0, v=0.08)
    dx, dy, _ = fit_belt_direction([(x, y) for _, x, y in pts])
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
