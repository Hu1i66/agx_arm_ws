#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线验证 Pinocchio IK 多种子（multi-seed）对动态抓取目标点的求解率影响。

用途（仅离线，不启动 ROS，不驱动机械臂）：
  1. 复用 auto_sorting_action 内嵌的 PinocchioIKSolver 与姿态生成逻辑。
  2. 从 conveyor_calib.json 读取方向/速度，回放一段沿皮带方向的物体轨迹。
  3. 对比两种策略的求解成功率/精度/耗时：
       - single-seed : 仅用"当前关节角"(待机位) 做初始猜测  (现状/RC-4)
       - multi-seed  : 当前关节 + 待机位 + 大偏移 + 全随机 (拟改进)
  4. 同时输出若干不可达点的径向距离诊断 (RC-1)。

用法:
  python3 offline_ik_multi_seed_test.py [--seed N] [--n-points N] [--points "x1,y1;x2,y2;..."]
"""
import argparse
import json
import os
import sys
import time

# ── 复用生产代码的 Pinocchio 路径注入 (与 auto_sorting_action 顶部一致) ──
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

# 复用生产 IK 求解器 (import 该模块不触发 rclpy 节点, 因受 __main__ 保护)
import auto_sorting_action as asa

URDF_PATH = '/home/lxf/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/test2.urdf'
CALIB_FILE = '/home/lxf/agx_arm_ws/conveyor_calib.json'

# ── 与生产代码一致: 动态抓取关键高度/速度常量 (影子 z 取 P_HOVER 高度) ──
SHADOW_Z = 0.32          # 影子对齐高度 (main() P_HOVER_POSE['z'] 示例值)
GRASP_Z_OFFSET = 0.064   # 抓取时 link6 下探高度 (main() GRIPPER_PICK_Z_OFFSET 示例值)
DYNAMIC_MIN_GRASP_Z = 0.045
LEAD_S = 0.6             # 下降前瞻 (DESCENT_NOMINAL_S 示例值 0.6)


def orb_from_axes(x_axis, y_axis, z_axis, pos):
    """由三轴/四元数构造器 (等价生产 _matrix_to_quaternion). 仅用于离线镜像."""
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
    q = q / np.linalg.norm(q)
    return q


def build_orientation(x, y, z_axis=(0., 0., -1.)):
    """复刻生产代码 _build_pick_orientations_multi 的径向直下姿态 + yaw 采样候选."""
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


def multi_seed_candidates(initial_guess, standby):
    """拟新增的多种子策略 (对应 RC-4 修复方案) — 从 TRAC-IK 8 种子策略移植."""
    seeds = []
    if initial_guess is not None:
        seeds.append(('当前关节', list(initial_guess)))
        # 大偏移 (±0.5rad)
        for _ in range(3):
            noisy = initial_guess + np.random.uniform(-0.5, 0.5, 6)
            seeds.append(('大偏移', list(noisy)))
    seeds.append(('待机位', list(standby)))
    # 全空间均匀随机 (2 个)
    for _ in range(2):
        seeds.append(('全随机', list(np.random.uniform(-np.pi, np.pi, 6))))
    return seeds


def production_multi_seed_solve(solver, pos, orientations, initial_guess, standby):
    """镜像生产 move_arm_via_ik 的多种子逻辑 (姿态×种子双层遍历, 提前终止).

    与生产代码保持一致的遍历顺序与终止条件, 用于离线验证生产逻辑效果.
    返回 (ok, best_err).
    """
    seed_candidates = multi_seed_candidates(initial_guess, standby)
    best_err = float('inf')
    best_q = None
    for target_quat in orientations:
        for seed_name, seed_val in seed_candidates:
            ok, q_sol, err, _ = solver.get_ik_solution(
                pos, target_quat, initial_guess=np.asarray(seed_val, dtype=float))
            if ok and err < best_err:
                best_err = err
                best_q = q_sol
                if err < 0.001:
                    break
        if best_err < 0.001:
            break
    return best_q is not None, best_err


def try_solver(solver, pos, seed):
    target_quat = build_orientation(pos[0], pos[1])
    ok, q_sol, err, comp_t = solver.get_ik_solution(pos, target_quat, initial_guess=seed)
    return ok, err, comp_t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=None, help='随机种子 (固定可复现)')
    ap.add_argument('--n-points', type=int, default=8, help='沿传送带方向采样点数')
    ap.add_argument('--points', type=str, default=None,
                    help='手动指定测试点 "x1,y1;x2,y2;..." (优先级最高, 覆盖回放)')
    ap.add_argument('--z', type=str, default='both',
                    choices=['both', 'shadow', 'grasp'], help='测试高度: both=影子+抓取, shadow=仅悬停, grasp=仅抓取')
    ap.add_argument('--zval', type=float, default=None,
                    help='显式指定抓取高度 (m), 覆盖 --z 的 grasp 计算值, 用于验证实际下降点可达性')
    args = ap.parse_args()

    if args.seed is not None:
        np.random.seed(args.seed)

    # ── 加载标定 ──
    if not os.path.exists(CALIB_FILE):
        print(f"❌ 未找到 {CALIB_FILE}, 请先运行标定 CLI")
        return
    with open(CALIB_FILE) as f:
        calib = json.load(f)
    dx, dy = calib['dx'], calib['dy']
    print(f"📐 方向向量: d̂=({dx:.4f},{dy:.4f}) angle={calib.get('angle_rad',0):.3f}rad "
          f"({np.degrees(calib.get('angle_rad',0)):.1f}°)")
    print(f"🐍 视觉速度≈{calib.get('v_visual_mps',0):.4f}, STM32≈{calib.get('v_stm32_mps',0):.4f}, "
          f"比例={calib.get('speed_scale',0):.3f}")

    # ── 初始化求解器 ──
    solver = asa.PinocchioIKSolver(URDF_PATH, 'link6')
    standby = np.zeros(6)
    # 生产代码 m 状态: java_something... 这里以"初始在待机位"模拟实际启动状态
    current = standby.copy()   # 单种子现状 = 只有待机位

    # ── 生成测试点 ──
    points = []
    if args.points:
        for token in args.points.split(';'):
            x, y = token.split(',')
            points.append((float(x), float(y)))
    else:
        # 沿传送带方向构造轨迹: 皮带经过视野中心一带, 沿 d̂ 匀速移动
        center_x, center_y = 0.32, 0.05
        span = 0.25
        for i in range(args.n_points):
            t = i / max(args.n_points - 1, 1)
            x = center_x + dx * (t - 0.5) * 2.0 * span
            y = center_y + dy * (t - 0.5) * 2.0 * span
            points.append((x, y))

    # 目标 x 边界过滤诊断 (RC-1): 径向距离超过 $R_MAX$ 视为工作空间之外
    R_MAX = 0.55
    print(f"\n{'='*78}")
    print(f"共 {len(points)} 个目标点, 工作空间判定阈值 R_MAX={R_MAX}m")
    print(f"{'='*78}")

    summary_single = {'ok': 0, 'fail': 0}
    summary_multi = {'ok': 0, 'fail': 0}
    wall_t0 = time.time()
    for (gx, gy) in points:
        R = np.hypot(gx, gy)
        bench = "🚫不可达诊断(RC-1)" if R > R_MAX else ""
        print(f"\n▶ 目标 (x={gx:.3f}, y={gy:.3f})  R=√(x²+y²)={R:.3f} {bench}")

        targets = []
        if args.z in ('both', 'shadow'):
            targets.append((f"影子Z={SHADOW_Z}", np.array([gx, gy, SHADOW_Z])))
        if args.z in ('both', 'grasp'):
            gz = args.zval if args.zval is not None else max(SHADOW_Z - GRASP_Z_OFFSET, DYNAMIC_MIN_GRASP_Z)
            targets.append((f"抓取Z={gz:.3f}", np.array([gx, gy, gz])))

        for label, pos in targets:
            # ── 单种子 (现状): 只用当前关节角作初始猜测 ──
            ok1, err1, t1 = try_solver(solver, pos, current)
            # ── 生产多种子逻辑镜像 (与 move_arm_via_ik 新实现一致): 姿态×8种子 ──
            # 生产实际姿态列表 = 径向 + 反向(2) + 8 yaw 均匀 = 10 姿态
            oris = [build_orientation(pos[0], pos[1])]
            radial = np.array([pos[0], pos[1], 0.0])
            r = np.linalg.norm(radial[:2])
            if r > 1e-9:
                x_rad = radial / np.linalg.norm(radial)   # 3维径向单位向量
                z = np.array([0., 0., -1.])
                y_rad = np.cross(z, x_rad)
                y_rad = y_rad / np.linalg.norm(y_rad)
                oris.append(orb_from_axes(-x_rad, -y_rad, z, pos))  # 反向180°
            for i in range(8):
                ang = 2.0 * np.pi * i / 8
                xa = np.array([np.cos(ang), np.sin(ang), 0.0])
                ya = np.cross(np.array([0.,0.,-1.]), xa)
                ya = ya / np.linalg.norm(ya)
                oris.append(orb_from_axes(xa, ya, np.array([0.,0.,-1.]), pos))
            ok2, err2 = production_multi_seed_solve(solver, pos, oris, current, standby)

            summary_single['ok' if ok1 else 'fail'] += 1
            summary_multi['ok' if ok2 else 'fail'] += 1
            status = lambda ok: "✅" if ok else "❌"
            print(f"    {label}: 单种子{status(ok1)} (err={err1:.4f}, {t1*1000:.0f}ms) | "
                  f"生产多种子{status(ok2)} (err={err2:.4f})")

    wall_end = time.time()
    print(f"\n{'='*78}")
    print(f"汇总 (共 {summary_single['ok']+summary_single['fail']} 项评估):")
    print(f"  单种子(现状): 成功 {summary_single['ok']}, 失败 {summary_single['fail']}, "
          f"成功率 {summary_single['ok']/(summary_single['ok']+summary_single['fail'])*100:.1f}%")
    print(f"  多种子(拟改): 成功 {summary_multi['ok']}, 失败 {summary_multi['fail']}, "
          f"成功率 {summary_multi['ok']/(summary_multi['ok']+summary_multi['fail'])*100:.1f}%")
    print(f"  总耗时: {wall_end-wall_t0:.2f}s")
    print(f"{'='*78}")
    print("\n说明: 工作中 pinocchio 求解为随机优化, 单次结果有方差; 若多种子成功率明显更高, "
          "支持 RC-4 结论(单种子不可靠).")


if __name__ == '__main__':
    main()