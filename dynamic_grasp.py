"""动态抓取的观测和相对运动几何。导入不访问 ROS 或硬件。"""
from collections import deque

import numpy as np
from scipy.interpolate import CubicSpline


def conveyor_velocity(calibration, speed, motor_direction):
    """历史方向向量已经是实测运动方向；缺失M2参考位时不能擅自翻转。"""
    direction = np.array([float(calibration['dx']), float(calibration['dy']), 0.])
    norm = np.linalg.norm(direction)
    speed = float(speed)
    if not np.isfinite(direction).all() or norm < 1e-9 or not np.isfinite(speed) or speed <= 0:
        raise ValueError('传送带方向/速度无效')
    reference = calibration.get('motor_direction')
    sign = -1. if reference is not None and int(motor_direction) != int(reference) else 1.
    return speed*sign*direction/norm


class MotionTrack:
    """只收同一相机、会话、track 的当前帧质心；不用静态 EMA 坐标。"""

    def __init__(self, track_id, session):
        if track_id is None or not session:
            raise ValueError('动态目标缺少 track_id/session，请重启新版视觉节点')
        self.track_id, self.session = track_id, session
        self.samples = deque(maxlen=30)
        self.geometry = None

    def observe(self, detection):
        if detection.get('camera') != 'd455' or detection.get('motion_session') != self.session:
            return
        obj = next((o for o in detection.get('objects', []) if o.get('track_id') == self.track_id), None)
        observation = (obj or {}).get('motion_observation')
        if not observation:
            return
        try:
            stamp = float(observation['stamp_s'])
            xyz = np.array([observation['position_m'][k] for k in ('x', 'y', 'z')], dtype=float)
            if not np.isfinite(stamp) or stamp <= 0 or not np.isfinite(xyz).all():
                return
        except (KeyError, TypeError, ValueError):
            return
        if self.samples and stamp <= self.samples[-1][0]:
            return
        self.samples.append((stamp, xyz))
        self.geometry = obj.get('grasp_geometry')

    def snapshot(self, now, max_age):
        if not self.samples:
            raise ValueError('尚无本目标的当前帧观测')
        stamp, xyz = self.samples[-1]
        if not 0 <= now - stamp <= max_age:
            raise ValueError(f'目标观测过期或时钟不一致，年龄={now-stamp:.3f}s')
        return stamp, xyz.copy()

    def longitudinal_speed(self, direction, reference_z):
        """正常深度帧的沿带位移速度；多帧斜率取中位数，少量抖动不拉偏预测。"""
        if not self.samples:
            return None
        last = self.samples[-1][0]
        clean = [(t, float(p @ direction)) for t, p in self.samples
                 if last-t <= 1.2 and abs(p[2]-reference_z) <= .015]
        if len(clean) < 4 or clean[-1][0]-clean[0][0] < .4:
            return None
        slopes = [(b-a)/(tb-ta) for i, (ta, a) in enumerate(clean)
                  for tb, b in clean[i+1:] if tb-ta >= .3]
        return float(np.median(slopes)) if slopes else None

    def visual_velocity(self):
        """仅用于报告核对，不自动修正 STM32。带截距的窗口最小二乘。"""
        if len(self.samples) < 3:
            return None
        last = self.samples[-1][0]
        samples = [(t, p) for t, p in self.samples if last-t <= 0.8]
        if len(samples) < 3 or samples[-1][0]-samples[0][0] < 0.1:
            return None
        times = np.array([t-last for t, _ in samples])
        times -= times.mean()
        return (times[:, None] * np.array([p for _, p in samples])).sum(axis=0) / (times @ times)


def smoothstep(s):
    s = np.clip(s, 0., 1.)
    return s*s*s*(10.+s*(-15.+6.*s))


def moving_tcp_path(anchor, velocity, start_s, start_xyz, tool_z, settings, sample_times=None):
    """在物体随动坐标系内：高位对齐→工具轴接近→闭合随动→抬起。

    开始点使用真实 TCP。先在预抓取高度平滑消除到位误差，再沿工具 Z 接近。
    轨迹末端平滑减去随动速度，保证搬运前停稳。所有时间保持原值，不重新均分。
    """
    stamp, xyz = anchor
    velocity, axis = np.asarray(velocity), np.asarray(tool_z)
    align, approach = settings['align_s'], settings['approach_s']
    close_at = align + approach
    lift_at = close_at + settings['follow_s']
    total = lift_at + settings['lift_s']
    boundaries = (0., align, close_at, lift_at, total)
    # 各阶段单独均匀采样，避免把边界硬插入网格形成1~10ms的极短段。
    times = (np.unique(np.concatenate([np.linspace(a, b, int(np.ceil((b-a)/settings['sample_s']))+1)
             for a, b in zip(boundaries[:-1], boundaries[1:])]))
             if sample_times is None else np.asarray(sample_times, dtype=float))
    pre = -settings['pregrasp_m'] * axis
    depth = settings['extra_depth_m'] * axis
    initial_offset = np.asarray(start_xyz) - (xyz + velocity*(start_s-stamp))
    positions = []
    for t in times:
        obj = xyz + velocity*(start_s+t-stamp)
        if t <= align:
            u = t/align
            offset = initial_offset + smoothstep(u)*(pre-initial_offset)
            # 起步真实速度为零，平滑接入皮带速度；不能令第一个采样瞬间跳到v。
            offset -= velocity*align*(u-6.*u**3+8.*u**4-3.*u**5)
        elif t <= close_at:
            offset = pre + smoothstep((t-align)/approach)*(depth-pre)
        elif t <= lift_at:
            offset = depth
        else:
            u = (t-lift_at)/settings['lift_s']
            offset = depth + np.array([0., 0., settings['lift_m']])*smoothstep(u)
            # smoothstep 的积分：抬起过程中物体已夹住，让水平速度连续降至零。
            integral = 2.5*u**4 - 3.*u**5 + u**6
            obj -= velocity*settings['lift_s']*integral
        positions.append(obj+offset)
    return times, np.array(positions), close_at


def joint_spline(times, joints, velocity_limits, acceleration_limits):
    """三次样条提供连续关节速度；解析检查各段的速度/加速度极值。"""
    spline = CubicSpline(times, joints, axis=0, bc_type=((1, np.zeros(6)), (1, np.zeros(6))))
    candidates = list(times)
    for i, width in enumerate(np.diff(times)):
        # 每个关节的速度二次式在加速度零点达到极值。
        a, b = spline.c[0, i], spline.c[1, i]
        for aa, bb in zip(a, b):
            if abs(aa) > 1e-12:
                root = -bb/(3.*aa)
                if 0 < root < width:
                    candidates.append(times[i]+root)
    candidates = np.array(candidates)
    peak_v = np.max(np.abs(spline(candidates, 1)), axis=0)
    peak_a = np.max(np.abs(spline(times, 2)), axis=0)
    scale = max(1., float(np.max(peak_v/velocity_limits)), float(np.sqrt(np.max(peak_a/acceleration_limits))))
    return spline, scale, peak_v, peak_a
