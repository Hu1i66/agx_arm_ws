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
