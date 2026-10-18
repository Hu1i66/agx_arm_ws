#!/usr/bin/env python3
"""Fixed-point TCP XYZ calibration. Pure NumPy solver; ROS is imported only for capture.

R_base_flange[i] @ tcp_xyz + p_base_flange[i] = fixed_point_base.
No motion publisher, enable client, parameter writer, or automatic promotion.
"""
import argparse
from collections import deque
from datetime import datetime
import json
import hashlib
import math
import os
from pathlib import Path
import shlex
import sys
import threading
import time
import uuid

import numpy as np


def finite_array(value, shape, name):
    raw = np.asarray(value, dtype=object)
    if raw.shape != shape or any(isinstance(x, (bool, np.bool_)) for x in raw.flat):
        raise ValueError(f'{name}: shape must be {shape}, numeric values required')
    try:
        out = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name}: invalid numeric values') from exc
    if not np.isfinite(out).all():
        raise ValueError(f'{name}: NaN/Inf is not allowed')
    return out


def offset_transform(offset):
    x, y, z, r, p, w = finite_array(offset, (6,), 'tcp_offset')
    cr, sr, cp, sp, cw, sw = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(w), math.sin(w)
    result = np.eye(4)
    result[:3, :3] = [[cw*cp, cw*sp*sr-sw*cr, cw*sp*cr+sw*sr],
                     [sw*cp, sw*sp*sr+cw*cr, sw*sp*cr-cw*sr],
                     [-sp, cp*sr, cp*cr]]
    result[:3, 3] = [x, y, z]
    return result


def tcp_to_flange(position, quaternion, current_offset):
    p = finite_array(position, (3,), 'position_m')
    q = finite_array(quaternion, (4,), 'quaternion_xyzw')
    norm = np.linalg.norm(q)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('quaternion must be nonzero and normalizable')
    x, y, z, w = q / norm
    reported = np.eye(4)
    reported[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    reported[:3, 3] = p
    offset = offset_transform(current_offset)
    inverse = np.eye(4)
    inverse[:3, :3] = offset[:3, :3].T
    inverse[:3, 3] = -offset[:3, :3].T @ offset[:3, 3]
    return reported @ inverse


def solve_pivot(flange_poses, preserved_rpy=(0., 0., 0.), *, min_samples=6,
                max_rms_mm=2., max_residual_mm=5., max_condition=100.):
    """Fit all supplied observations. Never hide outliers or invent orientation."""
    rpy = finite_array(preserved_rpy, (3,), 'preserved_rpy')
    if type(min_samples) is not int or min_samples < 6:
        raise ValueError('min_samples must be an integer >= 6')
    gates = finite_array([max_rms_mm, max_residual_mm, max_condition], (3,), 'quality gates')
    if (gates <= 0).any() or max_condition < 1:
        raise ValueError('quality gates must be positive; condition limit >= 1')
    poses = [finite_array(t, (4, 4), 'flange_pose') for t in flange_poses]
    if not poses:
        raise ValueError('no observations')
    for t in poses:
        r = t[:3, :3]
        if (not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8)
                or not np.allclose(r.T @ r, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(r), 1., atol=1e-6)):
            raise ValueError('flange_pose must be a rigid transform')
    a = np.vstack([np.hstack([t[:3, :3], -np.eye(3)]) for t in poses])
    b = np.concatenate([-t[:3, 3] for t in poses])
    solution, _, rank, singular = np.linalg.lstsq(a, b, rcond=None)
    condition = float(singular[0] / singular[-1]) if rank == 6 else None
    errors = np.array([np.linalg.norm(t[:3, :3] @ solution[:3] + t[:3, 3] - solution[3:]) * 1000
                       for t in poses])
    rms, maximum = float(np.sqrt(np.mean(errors**2))), float(errors.max())
    checks = [{'check': 'samples', 'ok': len(poses) >= min_samples},
              {'check': 'rank_6', 'ok': rank == 6},
              {'check': 'condition', 'ok': condition is not None and condition <= max_condition},
              {'check': 'rms_mm', 'ok': rms <= max_rms_mm},
              {'check': 'max_residual_mm', 'ok': maximum <= max_residual_mm}]
    for check in checks:
        check['ok'] = bool(check['ok'])
    passed = all(c['ok'] for c in checks)
    return {'kind': 'tcp_pivot_candidate', 'schema_version': 1,
            'quality_status': 'passed' if passed else 'failed', 'quality_checks': checks,
            'sample_count': len(poses), 'rank': int(rank), 'condition_number': condition,
            'rms_mm': rms, 'max_residual_mm': maximum, 'residuals_mm': errors.tolist(),
            'thresholds': dict(min_samples=min_samples, max_rms_mm=max_rms_mm,
                               max_residual_mm=max_residual_mm, max_condition=max_condition),
            'tcp_offset': solution[:3].tolist() + rpy.tolist() if passed else None,
            'pivot_base_m': solution[3:].tolist() if rank == 6 else None,
            'orientation_calibrated': False, 'preserved_rpy_rad': rpy.tolist(),
            'units': 'm', 'angle_units': 'rad', 'parent': 'link6', 'child': 'tcp_link',
            'applied': False}


def solve_session(payload, **gates):
    if (payload.get('kind') != 'tcp_pivot_session' or payload.get('schema_version') != 1
            or payload.get('units') != 'm' or payload.get('quaternion_order') != 'xyzw'):
        raise ValueError('unsupported TCP session schema/units/quaternion convention')
    offset = finite_array(payload['current_tcp_offset'], (6,), 'current_tcp_offset')
    samples = payload['samples']
    if not isinstance(samples, list):
        raise ValueError('samples must be a list')
    poses = [tcp_to_flange(row['position_m'], row['quaternion_xyzw'], offset) for row in samples]
    options = dict(payload.get("solver_options", {}))
    options.update(gates)
    result = solve_pivot(poses, offset[3:], **options)
    result['current_tcp_offset'] = offset.tolist()
    return result


def stable_sample(rows, *, now, after, min_count=10, max_age_s=1.,
                  max_translation_mm=1., max_angle_deg=.3):
    """Check a recent stationary window after Enter, then use its newest pose."""
    recent = [dict(row) for row in rows if row['received_monotonic'] >= after]
    if len(recent) < min_count or now - recent[-1]['received_monotonic'] > max_age_s:
        raise ValueError('机器人位姿缺失/过期；请确认T1并等待新反馈')
    transforms = []
    for row in recent:
        if row.get('frame_id') not in ('', 'base_link'):
            raise ValueError('位姿必须相对base_link')
        transforms.append(tcp_to_flange(row['position_m'], row['quaternion_xyzw'], [0.] * 6))
    reference = transforms[-1]
    displacement = max(np.linalg.norm(t[:3, 3] - reference[:3, 3]) * 1000 for t in transforms)
    angle = max(math.degrees(math.acos(float(np.clip(
        (np.trace(reference[:3, :3].T @ t[:3, :3]) - 1) / 2, -1, 1)))) for t in transforms)
    if displacement > max_translation_mm or angle > max_angle_deg:
        raise ValueError(f'尚未停稳：位移 {displacement:.3f}mm，转角 {angle:.3f}deg')
    row = recent[-1]
    row.update(assumed_base_frame=not bool(row.get('frame_id')),
               window_samples=len(recent), stability_mm=displacement, stability_deg=angle)
    return row


def save_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        os.replace(temporary, path)
    finally:
        if temporary.exists(): temporary.unlink()


def save_solution(payload, working_file, target, **gates):
    """Bind each result to its own immutable input, even if capture continues."""
    working_file, target = Path(working_file).expanduser().resolve(), Path(target).expanduser().resolve()
    if target == working_file:
        raise ValueError('输出不能覆盖原始采样文件')
    result = solve_session(payload, **gates)
    snapshot = target.parent / ('solve_input_' + uuid.uuid4().hex + '.json')
    saved_input = dict(payload, solver_options=result["thresholds"])
    save_json(snapshot, saved_input)
    result.update(session_file=str(snapshot), working_session_file=str(working_file),
                  input_sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest())
    save_json(target, result)
    return result


def describe_result(result):
    print(f"质量状态 {result['quality_status']}；样本 {result['sample_count']}；rank={result['rank']}；"
          f"RMS={result['rms_mm']:.3f}mm；max={result['max_residual_mm']:.3f}mm")
    print('未通过项:', [c['check'] for c in result['quality_checks'] if not c['ok']])
    if result['tcp_offset'] is not None:
        argument = 'tcp_offset:=' + json.dumps(result['tcp_offset'])
        print('XYZ已标定，RPY沿用当前值（未标定朝向）。现场核验后，停止T1并用以下命令重启：')
        print('./scripts/run_component.sh arm -- ' + shlex.quote(argument))
        print('本程序没有修改驱动参数或生产配置。勿用ros2 param set在线应用TCP。')


def capture_live(ros_args, output=None):
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters
    from geometry_msgs.msg import PoseStamped

    class CaptureNode(Node):
        def __init__(self):
            super().__init__('tcp_pivot_calibration')
            self.declare_parameter('pose_topic', '/feedback/tcp_pose')
            self.declare_parameter('result_save_path', str(Path(__file__).resolve().parents[3] / 'data/calibration/tcp'))
            self.declare_parameter('max_rms_mm', 2.)
            self.declare_parameter('max_residual_mm', 5.)
            self.declare_parameter('max_condition', 100.)
            self.rows, self.lock, self.sequence = deque(maxlen=400), threading.Lock(), 0
            self.create_subscription(PoseStamped, self.get_parameter('pose_topic').value, self.receive, 10)

        def read_offset(self):
            client = self.create_client(GetParameters, '/agx_arm_ctrl_single_node/get_parameters')
            try:
                if not client.wait_for_service(timeout_sec=5.):
                    raise ValueError('无法读取T1 tcp_offset；请先启动T1')
                request = GetParameters.Request(); request.names = ['tcp_offset']
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.)
                if not future.done() or future.exception() is not None:
                    raise ValueError('tcp_offset读取失败/超时')
                result = future.result()
                if result is None or len(result.values) != 1 or result.values[0].type != 8:
                    raise ValueError('tcp_offset必须是6维double数组')
                return finite_array(list(result.values[0].double_array_value), (6,), 'tcp_offset').tolist()
            finally:
                self.destroy_client(client)

        def receive(self, message):
            p, q = message.pose.position, message.pose.orientation
            with self.lock:
                self.sequence += 1
                self.rows.append(dict(position_m=[p.x, p.y, p.z], quaternion_xyzw=[q.x, q.y, q.z, q.w],
                                      frame_id=message.header.frame_id, sequence=self.sequence,
                                      received_monotonic=time.monotonic(),
                                      header_stamp=dict(sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec)))

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node, thread = None, None
    stop, spin_error = threading.Event(), []
    try:
        node = CaptureNode()
        offset = node.read_offset()  # The only spin point until the receiver thread starts.
        session = Path(node.get_parameter('result_save_path').value).expanduser().resolve() / (
            datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '_' + uuid.uuid4().hex[:8])
        payload = dict(kind='tcp_pivot_session', schema_version=1, units='m', quaternion_order='xyzw',
                       current_tcp_offset=offset, robot_base_frame='base_link', flange_frame='link6',
                       pose_source=node.get_parameter('pose_topic').value, samples=[],
                       current_tcp_offset_source='/agx_arm_ctrl_single_node/tcp_offset')
        def spin():
            try:
                while not stop.is_set() and rclpy.ok(): rclpy.spin_once(node, timeout_sec=.05)
            except Exception as exc:
                spin_error.append(str(exc))
        thread = threading.Thread(target=spin, daemon=True); thread.start()
        print('TCP固定点法：只接收位姿、不控制运动。工具同一点保持接触固定点，改变至少两个轴的姿态。')
        print('建议10~15组，至少6组。Enter采样 / d回退 / q求解保存（可继续补采） / c退出。')
        print('当前TCP参数:', offset, '\n会话:', session)
        save_json(session / 'frames.json', payload)
        while rclpy.ok():
            command = input(f"[{len(payload['samples'])}组] Enter/d/q/c: ").strip().lower()
            if spin_error: raise ValueError('接收线程退出: ' + spin_error[0])
            if command == 'c': break
            if command == 'd':
                if payload['samples']: payload['samples'].pop()
            elif command == 'q':
                gates = {key: node.get_parameter(key).value for key in ('max_rms_mm', 'max_residual_mm', 'max_condition')}
                try:
                    result = save_solution(payload, session / 'frames.json',
                                           output or session / 'candidate.json', **gates)
                    describe_result(result)
                except ValueError as exc:
                    print('无法求解:', exc)
            elif command == '':
                pressed = time.monotonic()
                time.sleep(.8)
                with node.lock: rows = list(node.rows)
                try:
                    sample = stable_sample(rows, now=time.monotonic(), after=max(pressed, time.monotonic()-.25))
                    new = tcp_to_flange(sample['position_m'], sample['quaternion_xyzw'], offset)
                    for old in payload['samples']:
                        previous = tcp_to_flange(old['position_m'], old['quaternion_xyzw'], offset)
                        if (np.linalg.norm(previous[:3, 3]-new[:3, 3]) < .002
                                and np.linalg.norm(previous[:3, :3]-new[:3, :3]) < .025):
                            raise ValueError('与已有采样姿态重复，请改变工具朝向')
                    payload['samples'].append(sample)
                    print('已采集；空header按当前Piper驱动约定解释为base_link。' if sample['assumed_base_frame'] else '已采集。')
                except ValueError as exc:
                    print('拒绝采样:', exc)
            else:
                print('无效输入'); continue
            save_json(session / 'frames.json', payload)
        return 0
    finally:
        stop.set()
        if thread is not None: thread.join(timeout=2.)
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description='固定点法TCP XYZ标定；不控制机械臂，不自动应用参数')
    parser.add_argument('--input', help='离线复算frames.json，不导入ROS、不连接设备')
    parser.add_argument('--output', help='候选JSON路径；默认保存到会话内')
    options, ros_args = parser.parse_known_args(sys.argv[1:] if args is None else args)
    try:
        if options.input:
            if ros_args: parser.error('离线复算不接受ROS参数')
            source = Path(options.input).expanduser().resolve()
            payload = json.loads(source.read_text())
            target = Path(options.output).expanduser() if options.output else source.with_name('candidate.json')
            result = save_solution(payload, source, target)
            describe_result(result)
            return 0 if result['quality_status'] == 'passed' else 2
        return capture_live(ros_args, options.output)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        print('TCP标定失败:', exc, file=sys.stderr); return 2
    except (KeyboardInterrupt, EOFError):
        print('\n已退出，原始采样已保留。'); return 0


if __name__ == '__main__':
    sys.exit(main())
