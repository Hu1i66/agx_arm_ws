#!/usr/bin/env python3
"""离线显示动态接近时间表；不导入ROS、不访问CAN/相机、不发送运动。"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dynamic_grasp import moving_tcp_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', type=float, nargs=3, default=[.34, -.06, .03], metavar=('X', 'Y', 'Z'))
    parser.add_argument('--velocity', type=float, nargs=2, default=[0., .04], metavar=('VX', 'VY'))
    parser.add_argument('--tilt-deg', type=float, default=10.)
    parser.add_argument('--yaw-deg', type=float, default=0.)
    parser.add_argument('--output', type=Path, default=ROOT/'data/diagnostics/dynamic_preview.json')
    args = parser.parse_args()
    values = args.target+args.velocity+[args.tilt_deg, args.yaw_deg]
    if not np.isfinite(values).all():
        parser.error('坐标、速度、角度必须有限')
    cfg = json.loads((ROOT/'config/dynamic_grasp.json').read_text())
    target, velocity = np.array(args.target), np.r_[args.velocity, 0.]
    orientation = Rotation.from_euler('xyz', [180., args.tilt_deg, args.yaw_deg], degrees=True)
    axis = orientation.as_matrix()[:, 2]
    start = target-cfg['pregrasp_m']*axis
    times, positions, close_at = moving_tcp_path((0., target), velocity, 0., start, axis, cfg)
    close_i = int(np.argmin(np.abs(times-close_at)))
    hold = (times >= close_at) & (times <= close_at+cfg['follow_s'])
    relative = positions-target-times[:, None]*velocity
    data = dict(mode='offline geometry preview only; no IK/collision/hardware validation',
        settings=cfg, velocity_mps=velocity.tolist(), close_at_s=close_at,
        duration_s=float(times[-1]), times_s=times.tolist(), tcp_positions_m=positions.tolist(),
        relative_positions_m=relative.tolist(),
        tool_axis_approach_m=float(np.linalg.norm(relative[close_i]+cfg['pregrasp_m']*axis)),
        closure_relative_drift_mm=float(np.max(np.linalg.norm(relative[hold]-cfg['extra_depth_m']*axis, axis=1)))*1000,
        belt_distance_during_closure_m=float(np.linalg.norm(velocity)*cfg['close_s']))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    print(f'纯离线几何预览：{args.output.resolve()}')
    print(f'闭合派发时刻 {close_at:.2f}s；完整随动/抬起 {times[-1]:.2f}s（不含预定位和计算等待）')
    print(f'闭合期间皮带位移 {data["belt_distance_during_closure_m"]*1000:.1f}mm；'
          f'理想随动模型相对漂移 {data["closure_relative_drift_mm"]:.6f}mm')
    print('这是理想模型的时间/位置计算，不代表实机误差或成功率。')


if __name__ == '__main__':
    main()
