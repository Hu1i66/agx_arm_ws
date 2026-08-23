#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线验证"方案 X: 关节空间连续多航点轨迹"的核心前提 (仅离线, 不启 ROS).

背景:
  日志4 显示"连续笛卡尔下降"(GetCartesianPath, 10 航点) 完整度 0.0%-0.2% 全失败.
  根因: 笛卡尔刚性直线在 Piper 悬停(z=0.32)->近桌面(z≈0.109) 扩展位形下产生大量
        奇异性不可达中间点, 而非目标点本身不可达 (日志3 同区关节IK 每段成功).
  方案 X: 不再用 GetCartesianPath, 改为"按时间前移的多航点, 每点用 Pinocchio IK
        解关节角 → 一次整条 JointTrajectory 下发".

本脚本验证:
  1) 可达率: 沿下降轨迹的 CARTO_N 个航点逐点 Pinocchio IK (生产多种子) 是否全部可达
  2) 关节连续性: 相邻航点关节角跳变幅度, 是否满足平滑插值可行性 (过大跳变→需过滤)
  3) 对比: 同点 y 固定 vs y 随时间前移 (验证"按时间前移"思想本身不破坏可达性)

用法:
  python3 offline_verify_jointspace_multiwaypoint.py [--seed N] [--n N] \
      [--z0 0.32] [--z1 0.109] [--speed 0.03] [--lead 3.0]
"""
import argparse
import json
import os
import sys

# ── 复用生产 Pinocchio 路径注入 (与 auto_sorting_action 顶部一致) ──
_venv_ik_path = "/home/lxf/agx_arm_ws/.venv_ik/lib/python3.10/site-packages"
if os.path.exists(_venv_ik_path) and _venv_ik_path not in sys.path:
    sys.path.insert(0, _venv_ik_path)
_cmeel_path = "/home/lxf/.local/lib/python3.10/site-packages/cmeel.prefix/lib/python3.10/site-packages"
if os.path.exists(_cmeel_path):
    if _cmeel_path in sys.path:
        sys.path.remove(_cmeel_path)
    sys.path.insert(0, _cmeel_path)
_ws_root = '/home/lxf/agx_arm_ws'
if _ws_root not in sys.path:
    sys.path.insert(0, _ws_root)

import numpy as np
import auto_sorting_action as asa

URDF_PATH = '/home/lxf/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/test2.urdf'
CALIB_FILE = '/home/lxf/agx_arm_ws/conveyor_calib.json'


def build_orientation(x, y, z_axis=(0., 0., -1.)):
    """复刻生产 _build_pick_orientations_multi 的径向直下姿态 (主候选)."""
    r = np.hypot(x, y)
    radial = np.array([x, y, 0.0]) / r if r > 1e-9 else np.array([1., 0., 0.])
    z_axis = np.array(z_axis)
    y_radial = np.cross(z_axis, radial)
    y_radial = y_radial / np.linalg.norm(y_radial)
    m00, m01, m02 = radial[0], y_radial[0], z_axis[0]
    m10, m11, m12 = radial[1], y_radial[1], z_axis[1]
    m20, m21, m22 = radial[2], y_radial[2], z_axis[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = np.array([(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s])
    elif (m00 > m11) and (m00 > m22):
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        q = np.array([0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s])
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        q = np.array([(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s])
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        q = np.array([(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s])
    q = q / np.linalg.norm(q)
    return q


def solve_all_orientations(solver, pos, initial_guess, standby, err_thresh=0.015):
    """镜像生产 move_arm_via_ik 多种子: 姿态(径向+反向+8yaw)×种子(当前/待机/大偏移/全随机)."""
    target_quats = [build_orientation(pos[0], pos[1])]
    radial = np.array([pos[0], pos[1], 0.0])
    r = np.linalg.norm(radial[:2])
    z = np.array([0., 0., -1.])
    if r > 1e-9:
        x_rad = radial / r
        y_rad = np.cross(z, x_rad)
        y_rad = y_rad / np.linalg.norm(y_rad)
        # 反向180° (复用 orb 构造)
        m00, m01, m02 = -x_rad[0], -y_rad[0], z[0]
        m10, m11, m12 = -x_rad[1], -y_rad[1], z[1]
        m20, m21, m22 = -x_rad[2], -y_rad[2], z[2]
        trace = m00 + m11 + m22
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            q = np.array([(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s])
        elif (m00 > m11) and (m00 > m22):
            s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
            q = np.array([0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s])
        elif m11 > m22:
            s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
            q = np.array([(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s])
        else:
            s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
            q = np.array([(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s])
        target_quats.append(q / np.linalg.norm(q))
    for i in range(8):
        ang = 2.0 * np.pi * i / 8
        xa = np.array([np.cos(ang), np.sin(ang), 0.0])
        ya = np.cross(z, xa)
        ya = ya / np.linalg.norm(ya)
        target_quats.append(build_orientation_axes(xa, ya, z))

    # 种子候选 (生产 move_arm_via_ik)
    seeds = []
    if initial_guess is not None:
        seeds.append(initial_guess.astype(float))
        for _ in range(3):
            seeds.append((initial_guess + np.random.uniform(-0.5, 0.5, 6)).astype(float))
    seeds.append(standby.astype(float))
    for _ in range(2):
        seeds.append(np.random.uniform(-np.pi, np.pi, 6).astype(float))

    best_q, best_err = None, float('inf')
    for tq in target_quats:
        for seed in seeds:
            ok, q_sol, err, _ = solver.get_ik_solution(
                pos, tq, initial_guess=seed)
            if ok and err < best_err:
                best_err = err
                best_q = np.array(q_sol)
                if err < 0.010:
                    break
        if best_err < 0.010:
            break
    return best_q, best_err


def build_orientation_axes(x_axis, y_axis, z_axis):
    m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
    m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
    m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = np.array([(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s])
    elif (m00 > m11) and (m00 > m22):
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        q = np.array([0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s])
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        q = np.array([(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s])
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        q = np.array([(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s])
    return q / np.linalg.norm(q)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--n', type=int, default=10, help='航点数')
    ap.add_argument('--z0', type=float, default=0.32, help='起始高度(悬停)')
    ap.add_argument('--z1', type=float, default=0.109, help='结束高度(抓取 grasp_z)')
    ap.add_argument('--speed', type=float, default=0.03, help='物体速度 m/s')
    ap.add_argument('--lead', type=float, default=3.0, help='下降总量时间 s (DESCENT_TOTAL)')
    ap.add_argument('--x0', type=float, default=0.33, help='起始 x')
    ap.add_argument('--y0', type=float, default=-0.15, help='起始 y (物体位置)')
    ap.add_argument('--yadv', dest='yadv', action='store_true', default=True,
                    help='航点 y 按时间前移 (模拟真实跟踪, 默认开启)')
    ap.add_argument('--no-yadv', dest='yadv', action='store_false',
                    help='关闭 y 前移, y 固定 (对照, 验证跟踪思想不影响可达性)')
    args = ap.parse_args()

    np.random.seed(args.seed)

    if not os.path.exists(CALIB_FILE):
        print(f"❌ 未找到 {CALIB_FILE}")
        return
    with open(CALIB_FILE) as f:
        calib = json.load(f)

    solver = asa.PinocchioIKSolver(URDF_PATH, 'link6')
    standby = np.zeros(6)
    current = standby.copy()

    dy = calib['dy']
    # 弓形前移方向: 近似沿传送带方向 (标定 d̂ 投影)
    dir_x, dir_y = calib.get('dx', calib.get('dy', 1.0)), dy
    norm = np.hypot(dir_x, dir_y) or 1.0
    ux, uy = dir_x / norm, dir_y / norm

    # 生成下降轨迹航点 (z 线性, xy 沿传送带方向随时间前移)
    zz = np.linspace(args.z0, args.z1, args.n)
    waypoint_qs = []
    print(f"{'='*84}")
    print(f"方案X验证: {args.n} 航点, z: {args.z0:.3f}→{args.z1:.3f}, "
          f"速度 {args.speed} m/s, 总量 {args.lead}s")
    print(f"传送带方向 u=({ux:.3f},{uy:.3f}), 起点 ({args.x0},{args.y0})")
    print(f"{'='*84}")

    recover_to = current  # warm start: 用上一航点关节解做下一航点初始猜测, 保证连续性
    for i in range(args.n):
        z = zz[i]
        t_frac = i / max(args.n - 1, 1)
        # 物体随时间前移 (沿传送带方向)
        if args.yadv:
            spos = args.speed * args.lead * t_frac
            px = args.x0 + ux * spos
            py = args.y0 + uy * spos
        else:
            px, py = args.x0, args.y0
        pos = np.array([px, py, z])
        R = np.hypot(px, py)
        q, err = solve_all_orientations(solver, pos, recover_to, standby)
        status = "✅" if q is not None else "❌"
        note = ""
        if q is not None and i > 0:
            # 关节跳变: 与上一解的最大分量差
            wprev = waypoint_qs[i - 1][1]
            if wprev is not None:
                maxjump = float(np.abs(np.asarray(q) - wprev).max())
                note = f"  Δq_max≈{maxjump:.3f}rad  (>0.6可能不连续→需过滤)"
        waypoint_qs.append((pos.copy(), q.copy() if q is not None else None))
        print(f"  #{i} z={z:.3f} ({px:.3f},{py:.3f}) R={R:.3f} {status} err={err:.4f}"
              + note)
        if q is not None:
            recover_to = q  # warm start 下一航点, 归一化舵位差异
        else:
            recover_to = standby

    # 汇总
    ok_n = sum(1 for _, q in waypoint_qs if q is not None)
    print(f"\n{'='*84}")
    print(f"可达率: {ok_n}/{args.n} ({ok_n/args.n*100:.1f}%)   {'✅ 全部可达 → 方案X前提成立' if ok_n==args.n else '❌ 有航点不可达'}")
    # 连续性: 相邻关节解 max 跳变
    jumps = []
    prev = None
    for _, q in waypoint_qs:
        if q is not None and prev is not None:
            jumps.append(float(np.abs(q - prev).max()))
        prev = q
    if jumps:
        print(f"相邻航点 Δq_max: min={min(jumps):.3f} max={max(jumps):.3f} "
              f"mean={np.mean(jumps):.3f} rad")
        print(f"  {'✅ 平滑(全部<0.6rad)' if max(jumps) < 0.6 else '⚠️ 存在>0.6rad跳变, 需中间插值'}")
    print(f"{'='*84}")


if __name__ == '__main__':
    main()