#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AX=XB 手眼标定求解器 (纯 numpy)。

实现文档《标定板参数与手眼标定要点》§6 要求的双算法互校:
    算法1: Kronecker 9x9 零空间 ([(I⊗R_A) − (R_Bᵀ⊗I)]·vec(R_X) = 0, 首选, 无符号歧义)
    算法2: 四元数零空间 (独立实现, 用于互校)

约定 (与 cv2.calibrateHandEye 保持一致, 从而不改变下游语义):
    输入 G = T_gripper2base, C = T_target2cam;
    求解 A·X = X·B, 其中 A 侧 = inv(G_j)·G_i, B 侧 = C_j·inv(C_i) (两侧取法不同);
    返回 X 与 cv2.calibrateHandEye 在同样输入下的输出同义。
"""

import numpy as np

# ── 判据阈值 (文档 §6.4) ──
CONSISTENCY_MAX_DEG = 0.02     # 双算法一致性上限, 超过说明方位跨度不足
MEDIAN_REL_ANGLE_MIN_DEG = 20.0  # 相对转角中位下限
USABLE_PAIR_ANGLE_DEG = 25.0   # n_pairs_ge25deg字段语义保持不变
MIN_USABLE_PAIRS = 10          # 可用运动对数下限
HOLDOUT_MAX_MM = 15.0           # 留出位姿验证上限

_ANGLE_EPS = 1e-12


# ────────────────────────── 基础工具 ──────────────────────────

def rt_to_T(rotation, translation):
    """(R, t) -> 4x4 齐次变换。"""
    T = np.eye(4)
    T[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return T


def T_to_rt(T):
    """4x4 -> (R, t)。"""
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    return T[:3, :3].copy(), T[:3, 3].copy()


def project_to_SO3(matrix):
    """把接近旋转的矩阵投影到 SO(3) (SVD 正交化 + 手性修正)。"""
    U, _, Vt = np.linalg.svd(np.asarray(matrix, dtype=np.float64).reshape(3, 3))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U = U.copy()
        U[:, -1] *= -1.0
        R = U @ Vt
    return R


def rotation_angle_deg(R1, R2):
    """两个旋转之间的测地夹角 (度)。"""
    R = np.asarray(R1, dtype=np.float64).T @ np.asarray(R2, dtype=np.float64)
    cos_angle = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_angle)))


def quat_from_R(R):
    """旋转矩阵 -> 四元数 (x, y, z, w), 规范到 w >= 0 (双覆盖定号)。"""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    q /= np.linalg.norm(q)
    if q[3] < 0.0:  # 统一规范到 w >= 0
        q = -q
    return q


def R_from_quat(q):
    """四元数 (x, y, z, w) -> 旋转矩阵。"""
    q = np.asarray(q, dtype=np.float64).reshape(4)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def _left_matrix(q):
    """左乘矩阵 L(q), 满足 L(q)·p = q ⊗ p (Hamilton, 标量在后)。"""
    x, y, z, w = q
    return np.array([
        [w, -z, y, x],
        [z, w, -x, y],
        [-y, x, w, z],
        [-x, -y, -z, w],
    ], dtype=np.float64)


def _right_matrix(q):
    """右乘矩阵 R(q), 满足 R(q)·p = p ⊗ q (Hamilton, 标量在后)。"""
    x, y, z, w = q
    return np.array([
        [w, z, -y, x],
        [-z, w, x, y],
        [y, -x, w, z],
        [-x, -y, -z, w],
    ], dtype=np.float64)


# ────────────────────────── 相对运动 ──────────────────────────

def relative_motions(T_g2b_list, T_t2c_list):
    """构造 AX=XB 的 A/B 两侧相对运动。

    A 侧 = inv(G_j)·G_i;  B 侧 = C_j·inv(C_i)  —— 两侧取法**不同**, 写反会导致旋转错约 117°。
    """
    n = len(T_g2b_list)
    RA, RB, tA, tB, angles = [], [], [], [], []
    for i in range(n):
        for j in range(i + 1, n):
            A = np.linalg.inv(T_g2b_list[j]) @ T_g2b_list[i]
            B = T_t2c_list[j] @ np.linalg.inv(T_t2c_list[i])
            RA.append(A[:3, :3])
            tA.append(A[:3, 3])
            RB.append(B[:3, :3])
            tB.append(B[:3, 3])
            angles.append(rotation_angle_deg(np.eye(3), A[:3, :3]))
    if not RA:
        raise ValueError("样本不足, 无法构造相对运动对 (至少需要 3 组位姿)")
    return (np.asarray(RA), np.asarray(RB), np.asarray(tA), np.asarray(tB),
            np.asarray(angles))


# ────────────────────────── 双算法 ──────────────────────────

def solve_rotation_kronecker(RA, RB):
    """算法1: Kronecker 9x9 零空间 (首选, 无符号歧义)。"""
    I3 = np.eye(3)
    blocks = [np.kron(I3, RA[k]) - np.kron(RB[k].T, I3) for k in range(len(RA))]
    M = np.vstack(blocks)
    _, _, Vt = np.linalg.svd(M)
    vec = Vt[-1]
    RX = vec.reshape(3, 3, order='F')   # vec 为列优先
    if np.linalg.det(RX) < 0.0:         # 零向量符号未定: 先定号再投影
        RX = -RX
    return project_to_SO3(RX)


def solve_rotation_quaternion(RA, RB):
    """算法2: 四元数零空间 (独立实现, 用于与算法1互校)。"""
    blocks = []
    for k in range(len(RA)):
        qA = quat_from_R(RA[k])
        qB = quat_from_R(RB[k])
        blocks.append(_left_matrix(qA) - _right_matrix(qB))
    M = np.vstack(blocks)
    _, _, Vt = np.linalg.svd(M)
    q = Vt[-1]
    if q[3] < 0.0:                      # 四元数双覆盖: 统一规范到 w >= 0
        q = -q
    q /= np.linalg.norm(q)
    return project_to_SO3(R_from_quat(q))


def solve_translation(RA, RB, tA, tB, RX):
    """由 (R_A − I)·t_X = R_X·t_B − t_A 的最小二乘解求平移。"""
    lhs = np.vstack([RA[k] - np.eye(3) for k in range(len(RA))])
    rhs = np.concatenate([RX @ tB[k] - tA[k] for k in range(len(RA))])
    t, _, _, _ = np.linalg.lstsq(lhs, rhs, rcond=None)
    return t


def solve_axxb(T_g2b_list, T_t2c_list):
    """求解 AX=XB。

    返回 dict:
        T                  X (4x4), 与 cv2.calibrateHandEye 同义
        R, t
        consistency_deg    双算法角度差 —— 病态报警器 (>0.02° 回去补方位跨度)
        eq_residual_mm     方程残差 ‖A·X−X·B‖ 的平移分量中位 (仅参考, 不作为验收)
        n_pairs            运动对数
        n_pairs_ge25deg    相对转角 >= 25° 的运动对数
        median_rel_angle_deg 相对转角中位 (度)
    """
    T_g2b_list = np.asarray(T_g2b_list, dtype=np.float64)
    T_t2c_list = np.asarray(T_t2c_list, dtype=np.float64)
    RA, RB, tA, tB, angles = relative_motions(T_g2b_list, T_t2c_list)

    RX_kron = solve_rotation_kronecker(RA, RB)
    RX_quat = solve_rotation_quaternion(RA, RB)
    consistency_deg = rotation_angle_deg(RX_kron, RX_quat)

    RX = RX_kron
    tX = solve_translation(RA, RB, tA, tB, RX)
    T = rt_to_T(RX, tX)

    residual_mm = []
    for k in range(len(RA)):
        A = rt_to_T(RA[k], tA[k])
        B = rt_to_T(RB[k], tB[k])
        residual_mm.append(float(np.linalg.norm((A @ T - T @ B)[:3, 3])) * 1000.0)

    return {
        "T": T,
        "R": RX,
        "t": tX,
        "consistency_deg": consistency_deg,
        "eq_residual_mm": float(np.median(residual_mm)) if residual_mm else 0.0,
        "n_pairs": int(len(RA)),
        "n_pairs_ge25deg": int(np.sum(angles >= USABLE_PAIR_ANGLE_DEG)),
        "median_rel_angle_deg": float(np.median(angles)),
    }


def holdout_validate(T_g2b_list, T_t2c_list, n_holdout=4):
    """留出位姿验证 (文档 §6.4 唯一硬判据)。

    留出若干位姿不参与求解, 用训练集解得 X 后预测留出位姿的不变量
    T_b2t = G_i·X·C_i, 与训练集均值比较, 平移中位误差按HOLDOUT_MAX_MM核验，原始误差仍完整报告。
    """
    T_g2b_list = np.asarray(T_g2b_list, dtype=np.float64)
    T_t2c_list = np.asarray(T_t2c_list, dtype=np.float64)
    n = len(T_g2b_list)
    if n < 5:
        return None
    n_holdout = int(max(1, min(n_holdout, n - 3)))
    hold_idx = sorted(set(int(v) for v in
                          np.linspace(0, n - 1, n_holdout).round().astype(int).tolist()))
    train_idx = [i for i in range(n) if i not in hold_idx]

    solved = solve_axxb(T_g2b_list[train_idx], T_t2c_list[train_idx])
    X = solved["T"]

    train_points = np.array([(T_g2b_list[i] @ X @ T_t2c_list[i])[:3, 3] for i in train_idx])
    train_mean = train_points.mean(axis=0)

    errors_mm = []
    for i in hold_idx:
        predicted = (T_g2b_list[i] @ X @ T_t2c_list[i])[:3, 3]
        errors_mm.append(float(np.linalg.norm(predicted - train_mean)) * 1000.0)
    errors_mm = np.asarray(errors_mm)

    return {
        "holdout_idx": hold_idx,
        "n": int(len(hold_idx)),
        "median_mm": float(np.median(errors_mm)),
        "p95_mm": float(np.percentile(errors_mm, 95)),
        "max_mm": float(np.max(errors_mm)),
        "train_consistency_deg": solved["consistency_deg"],
    }


def format_criteria_report(result, holdout=None):
    """把判据整理成可直接打印的多行文本 (按可信度排序)。"""
    lines = []
    lines.append("========== 标定判据 (按可信度排序) ==========")
    if holdout is not None:
        flag = "通过" if holdout["median_mm"] < HOLDOUT_MAX_MM else "不通过"
        lines.append(
            f"[{flag}] 留出位姿验证 (最硬判据, 留 {holdout['n']} 组): "
            f"中位={holdout['median_mm']:.2f}mm p95={holdout['p95_mm']:.2f}mm "
            f"max={holdout['max_mm']:.2f}mm (阈值 <{HOLDOUT_MAX_MM:.0f}mm)")
    else:
        lines.append("[--] 留出位姿验证: 位姿数不足, 未执行")

    flag = "通过" if result["consistency_deg"] < CONSISTENCY_MAX_DEG else "不通过"
    lines.append(
        f"[{flag}] 双算法一致性 (病态报警器): {result['consistency_deg']:.4f}° "
        f"(阈值 <{CONSISTENCY_MAX_DEG}°)"
        + ("" if flag == "通过" else "  -> 方位跨度不足, 请回去补 ±25~35° 倾斜与绕光轴自转"))

    flag = "通过" if result["median_rel_angle_deg"] >= MEDIAN_REL_ANGLE_MIN_DEG else "不通过"
    lines.append(
        f"[{flag}] 相对转角中位: {result['median_rel_angle_deg']:.1f}° "
        f"(阈值 >={MEDIAN_REL_ANGLE_MIN_DEG:.0f}°, 理想 30~40°)")

    flag = "通过" if result["n_pairs_ge25deg"] >= MIN_USABLE_PAIRS else "不通过"
    lines.append(
        f"[{flag}] 可用运动对数 (>=25°): {result['n_pairs_ge25deg']} "
        f"(总对数 {result['n_pairs']}, 建议 >={MIN_USABLE_PAIRS})")

    lines.append(
        f"[仅参考] 方程残差 ||A·X−X·B|| 平移分量中位: {result['eq_residual_mm']:.4f} mm "
        f"—— 实测对病态不敏感, 不作为验收")
    return "\n".join(lines)


# ────────────────────────── 独立自测 ──────────────────────────

def _self_test():
    """合成真值自测: 精确位姿下双算法必须一致, 且能恢复 X。"""
    rng = np.random.default_rng(20261007)

    def random_rotation(max_deg=180.0):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = np.radians(rng.uniform(-max_deg, max_deg))
        return R_from_quat(np.concatenate([axis * np.sin(angle / 2.0), [np.cos(angle / 2.0)]]))

    T_b2t = rt_to_T(random_rotation(), rng.normal(scale=0.3, size=3))
    T_X_true = rt_to_T(random_rotation(), rng.normal(scale=0.1, size=3))

    T_g2b_list, T_t2c_list = [], []
    for _ in range(15):
        G = rt_to_T(random_rotation(), rng.normal(scale=0.4, size=3))
        C = np.linalg.inv(T_X_true) @ np.linalg.inv(G) @ T_b2t
        T_g2b_list.append(G)
        T_t2c_list.append(C)

    solved = solve_axxb(T_g2b_list, T_t2c_list)
    rot_err = rotation_angle_deg(T_X_true[:3, :3], solved["R"])
    t_err_mm = float(np.linalg.norm(T_X_true[:3, 3] - solved["t"])) * 1000.0

    print(format_criteria_report(solved, holdout_validate(T_g2b_list, T_t2c_list)))
    print(f"恢复误差: 旋转 {rot_err:.3e}°, 平移 {t_err_mm:.3e} mm")

    # 四元数工具自检
    Rq = random_rotation()
    assert rotation_angle_deg(Rq, R_from_quat(quat_from_R(Rq))) < 1e-9
    assert quat_from_R(Rq)[3] >= 0.0
    # 同步旋转自检: R_X·R_B·R_Xᵀ 与 R_B 同角
    assert solved["consistency_deg"] < 1e-4, f"双算法不一致: {solved['consistency_deg']}"
    assert rot_err < 1e-4, f"旋转恢复失败: {rot_err}"
    assert t_err_mm < 1e-4, f"平移恢复失败: {t_err_mm}"
    print("自测通过: 双算法一致性 <1e-4°, 真值恢复误差 <1e-4")


if __name__ == "__main__":
    _self_test()