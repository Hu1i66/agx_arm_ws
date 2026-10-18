"""手眼标定数学真值测试（计划 5.8）：双模式合成真值、完整逆、q/-q 等价。

纯 numpy（axxb），不开相机、不启动节点、不用硬件。
"""
import math
import pathlib
import sys

import numpy as np
import pytest

WS = pathlib.Path(__file__).resolve().parent.parent
PKG = WS / "src" / "handeye_calibration_ros" / "handeye_calibration_ros"
sys.path.insert(0, str(PKG))
import axxb  # noqa: E402

TOL = 1e-4


def rot_axis_angle(axis, angle_rad):
    """Rodrigues 旋转（numpy-only，避免测试依赖 scipy）。"""
    a = np.asarray(axis, dtype=np.float64)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]],
                  [a[2], 0, -a[0]],
                  [-a[1], a[0], 0]], dtype=np.float64)
    return np.eye(3) + math.sin(angle_rad) * K + (1 - math.cos(angle_rad)) * (K @ K)


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64)
    return T


def _poses(rng, n=18, t_center=(0.3, 0.0, 0.2)):
    """生成 n 个位姿, 旋转轴随机(非平行)且跨度足够。"""
    out = []
    for _ in range(n):
        axis = rng.normal(size=3)
        ang = rng.uniform(20.0, 120.0) * math.pi / 180.0
        t = np.asarray(t_center) + rng.normal(size=3) * 0.15
        out.append(make_T(rot_axis_angle(axis, ang), t))
    return out


# ---------------- 腕载 (eye_in_hand) ----------------
def test_eye_in_hand_synthetic_truth():
    rng = np.random.default_rng(20261)
    X_true = make_T(rot_axis_angle([0.3, 0.7, 0.2], 1.1), [0.05, -0.08, 0.12])   # ^tcp T_cam
    Z_true = make_T(rot_axis_angle([-0.2, 0.4, 0.9], 0.6), [0.3, 0.1, 0.45])     # ^base T_board

    G_list, C_list = [], []
    for G in _poses(rng):
        # 计划 5.8: C_i = inv(X) @ inv(G_i) @ Z
        C = np.linalg.inv(X_true) @ np.linalg.inv(G) @ Z_true
        G_list.append(G)
        C_list.append(C)

    res = axxb.solve_axxb(G_list, C_list)
    assert np.allclose(res["T"], X_true, atol=TOL), \
        f"腕载真值恢复失败: {np.abs(res['T'] - X_true).max()}"
    assert res["consistency_deg"] < 0.02
    assert res["median_rel_angle_deg"] >= 25.0
    assert res["n_pairs_ge25deg"] >= 10
    holdout = axxb.holdout_validate(G_list, C_list, 4)
    assert holdout is not None and holdout["median_mm"] < 1.0


# ---------------- 外置 (eye_to_hand) ----------------
def test_eye_to_hand_synthetic_truth():
    rng = np.random.default_rng(777)
    # 真值即活动基线量级
    Y_true = make_T(rot_axis_angle([-0.6924, -0.7044, 0.0849], 2.0),
                    [0.6490691434509585, -0.05381643112008146, 0.5963265498171803])  # ^base T_cam
    S_true = make_T(rot_axis_angle([0.1, -0.5, 0.3], 0.8), [0.02, 0.01, 0.05])      # ^tcp T_board

    G_list, C_list = [], []
    for G in _poses(rng, t_center=(0.25, -0.1, 0.15)):
        # 计划 5.8: C_i = inv(Y) @ G_i @ S
        C_list.append(np.linalg.inv(Y_true) @ G @ S_true)
        G_list.append(G)

    # 生产预处理: 传 inv(G_i) / C_i, 求解应返回 Y
    res = axxb.solve_axxb([np.linalg.inv(G) for G in G_list], C_list)
    assert np.allclose(res["T"], Y_true, atol=TOL), \
        f"外置真值恢复失败: {np.abs(res['T'] - Y_true).max()}"
    assert res["consistency_deg"] < 0.02


# ---------------- 完整逆 [Rᵀ, -Rᵀt] ----------------
def test_full_inverse_requires_translation_rotation_change():
    R = rot_axis_angle([0.2, 0.9, -0.4], 1.3)
    t = np.array([0.31, -0.27, 0.55])
    T = make_T(R, t)
    T_inv = np.linalg.inv(T)
    # 完整逆 = [Rᵀ, -Rᵀt]
    assert np.allclose(T_inv[:3, :3], R.T, atol=1e-12)
    assert np.allclose(T_inv[:3, 3], -R.T @ t, atol=1e-12)

    # 旧教程"平移原样, 仅四元数取共轭"必须失败（t 非零）
    wrong_t = t.copy()
    assert not np.allclose(wrong_t, T_inv[:3, 3], atol=TOL), \
        "'平移原样' 在非零平移下必须判为错误"

    # 验证组合还原
    assert np.allclose(T @ T_inv, np.eye(4), atol=1e-12)


def test_quaternion_double_cover_equivalence():
    R = rot_axis_angle([0.5, -0.3, 0.8], 0.9)
    q = axxb.quat_from_R(R)
    R1 = axxb.R_from_quat(q)
    R2 = axxb.R_from_quat(-q)          # q 与 -q 表示同一旋转
    assert np.allclose(R1, R, atol=1e-12)
    assert np.allclose(R2, R, atol=1e-12)
    # 不按四元数分量比较（q 与 -q 分量相反但等价）
    assert not np.allclose(q, -q)


def test_axxb_self_test_runs():
    """axxb 自带纯 NumPy self-test 必须通过。"""
    assert axxb._self_test() is not False
