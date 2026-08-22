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
