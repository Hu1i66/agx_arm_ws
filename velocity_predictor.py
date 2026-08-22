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
