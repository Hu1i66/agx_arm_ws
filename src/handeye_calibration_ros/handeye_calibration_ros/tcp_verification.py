#!/usr/bin/env python3
"""TCP fixed-point attitude verification. Default math mode; ROS is lazy loaded."""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import signal
import select
import sys
import threading
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

try:
    from . import tcp_calibration as cal
except ImportError:
    import tcp_calibration as cal

JOINTS = [f'joint{i}' for i in range(1, 7)]
STRICT_GATES = dict(min_samples=6, max_rms_mm=2., max_residual_mm=5., max_condition=100.)


def load_candidate(path, *, allow_unqualified=False, waiver_reason=''):
    path = Path(path).expanduser().resolve()
    raw = path.read_bytes(); candidate = json.loads(raw)
    if (candidate.get('kind') != 'tcp_pivot_candidate' or candidate.get('schema_version') != 1
            or candidate.get('units') != 'm' or candidate.get('angle_units') != 'rad'
            or candidate.get('parent') != 'link6' or candidate.get('child') != 'tcp_link'
            or candidate.get('orientation_calibrated') is not False):
        raise ValueError('TCP候选格式/坐标/单位不完整或不支持')
    snapshot = Path(candidate['session_file']).expanduser()
    if not snapshot.is_absolute(): snapshot = path.parent / snapshot
    snapshot = snapshot.resolve(); data = snapshot.read_bytes()
    if hashlib.sha256(data).hexdigest() != candidate.get('input_sha256'):
        raise ValueError('输入快照SHA256不一致')
    payload = json.loads(data)
    if payload.get('robot_base_frame') != 'base_link' or payload.get('flange_frame') != 'link6':
        raise ValueError('采样坐标不完整或不是Piper base_link/link6')
    source = cal.solve_session(payload, **candidate['thresholds'])
    for key in ('quality_status', 'sample_count', 'rank'):
        if source[key] != candidate.get(key): raise ValueError('候选元数据与输入复算不一致: ' + key)
    for key in ('condition_number', 'rms_mm', 'max_residual_mm', 'residuals_mm',
                'current_tcp_offset', 'preserved_rpy_rad'):
        if source.get(key) is None or candidate.get(key) is None or not np.allclose(
                source[key], candidate[key], atol=1e-8, rtol=1e-8):
            raise ValueError('候选数值与输入复算不一致: ' + key)
    if source['quality_checks'] != candidate.get('quality_checks'):
        raise ValueError('候选质量检查与复算不一致')
    if source['quality_status'] == 'passed':
        if candidate.get('tcp_offset') is None or not np.allclose(
                cal.finite_array(candidate['tcp_offset'], (6,), 'tcp_offset'), source['tcp_offset'], atol=1e-9):
            raise ValueError('候选TCP为空或与复算不一致')
    elif candidate.get('tcp_offset') is not None:
        raise ValueError('failed候选含非空TCP，格式不一致')
    strict = cal.solve_session(payload, **STRICT_GATES)
    accepted = cal.solve_session(payload, **cal.DEFAULT_GATES)
    qualified = source['quality_status'] == 'passed' and accepted['quality_status'] == 'passed'
    waiver = None
    chosen = accepted['tcp_offset'] if qualified else None
    if not qualified:
        failures = {c['check'] for c in strict['quality_checks'] if not c['ok']}
        if (not allow_unqualified or len(waiver_reason.strip()) < 5
                or not failures.issubset({'rms_mm', 'max_residual_mm'})):
            raise ValueError('候选未按当前粗标定标准通过（RMS≤5mm、max≤10mm）；仅残差不合格可经明确--allow-unqualified及--waiver-reason用于验证')
        old = cal.finite_array(payload['current_tcp_offset'], (6,), '旧TCP')
        poses = [cal.tcp_to_flange(r['position_m'], r['quaternion_xyzw'], old) for r in payload['samples']]
        a = np.vstack([np.hstack([t[:3, :3], -np.eye(3)]) for t in poses])
        b = np.concatenate([-t[:3, 3] for t in poses])
        estimate = np.linalg.lstsq(a, b, rcond=None)[0]
        chosen = estimate[:3].tolist() + old[3:].tolist()
        waiver = dict(scope='verification_only', reason=waiver_reason.strip(),
                      source_quality_status=candidate['quality_status'],
                      original_gates=STRICT_GATES, failed_checks=sorted(failures),
                      production_qualified=False)
    cal.finite_array(chosen, (6,), '验证TCP')
    if abs(chosen[3]) > math.pi or abs(chosen[4]) > math.pi/2 or abs(chosen[5]) > math.pi:
        raise ValueError('RPY超出当前SDK支持范围，不能与启动配置准确对应')
    return dict(candidate_file=str(path), candidate_sha256=hashlib.sha256(raw).hexdigest(),
                input_file=str(snapshot), input_sha256=hashlib.sha256(data).hexdigest(),
                source_quality_status=candidate['quality_status'], qualified=qualified,
                acceptance_gates=dict(cal.DEFAULT_GATES), strict_quality_status=strict['quality_status'], strict_gates=dict(STRICT_GATES),
                quality_waiver=waiver, tcp_offset=chosen, original_metrics=dict(
                    sample_count=strict['sample_count'], rank=strict['rank'],
                    condition=strict['condition_number'], rms_mm=strict['rms_mm'],
                    max_residual_mm=strict['max_residual_mm']))


def verify_input_hashes(assessment):
    for prefix in ('candidate', 'input'):
        path = Path(assessment[prefix+'_file'])
        if hashlib.sha256(path.read_bytes()).hexdigest() != assessment[prefix+'_sha256']:
            raise ValueError('验证期间候选/输入快照发生变化: '+str(path))


def make_targets(start_pose, *, angle_deg=5., steps=5, axes=('x', 'y'), cycles=1):
    pose = cal.finite_array(start_pose, (7,), 'start_pose')
    if (not math.isfinite(angle_deg) or not 0 < angle_deg <= 15 or type(steps) is not int
            or not 1 <= steps <= 10 or angle_deg/steps > 2.5 or type(cycles) is not int
            or not 1 <= cycles <= 3 or len(axes) < 2 or len(set(axes)) != len(axes)
            or any(a not in ('x', 'y', 'z') for a in axes)):
        raise ValueError('角度须(0,15]deg，steps为1..10且每步<=2.5deg，cycles为1..3，至少两个不同轴')
    base = Rotation.from_quat(pose[3:])
    targets = []
    for cycle in range(cycles):
        for axis in axes:
            for left, right in ((0, angle_deg), (angle_deg, 0), (0, -angle_deg), (-angle_deg, 0)):
                for angle in np.linspace(left, right, steps+1)[1:]:
                    direction = np.zeros(3); direction['xyz'.index(axis)] = math.radians(float(angle))
                    q = (base * Rotation.from_rotvec(direction)).as_quat()
                    targets.append(dict(step=len(targets)+1, cycle=cycle+1, axis=axis,
                                        angle_deg=float(angle), pose=pose[:3].tolist()+q.tolist()))
    if len(targets) > 240: raise ValueError('最多240个有限姿态步骤')
    return targets


def group_sweeps(targets, steps):
    if not targets or len(targets)%steps: raise ValueError('连续摆动目标数量不完整')
    strokes=[]
    for index in range(0,len(targets),steps):
        rows=targets[index:index+steps]
        if len({(r['axis'],r['cycle']) for r in rows})!=1: raise ValueError('一程摆动不能跨轴/跨轮')
        stroke=dict(rows[-1],step=len(strokes)+1,waypoints=[r['pose'] for r in rows])
        strokes.append(stroke)
    return strokes


def group_round(targets,start_pose,amplitude):
    return dict(step=1,axis='/'.join(dict.fromkeys(t['axis'] for t in targets)),angle_deg=0.,amplitude_deg=float(amplitude),
                reference_pose=list(start_pose),pose=list(start_pose),waypoints=[t['pose'] for t in targets])


def retime_round(positions,limits,speed,duration,workspace=None):
    import subprocess
    if not math.isfinite(speed) or not .005<=speed<=.5: raise ValueError('整轮模式speed须0.005..0.5')
    if not math.isfinite(duration) or not 0<duration<=18: raise ValueError('整轮轨迹须在18秒内，为20秒运动阶段留余量')
    q=[cal.finite_array(row,(6,),'整轮关节') for row in positions]
    root=Path(workspace) if workspace else Path(__file__).resolve().parents[3]
    binary=root/'install/tcp_trajectory_tools/lib/tcp_trajectory_tools/tcp_time_parameterize'
    if not binary.is_file(): raise ValueError('请先在本worktree构建tcp_trajectory_tools，缺少: '+str(binary))
    payload=dict(positions=[row.tolist() for row in q],velocity=[limits[n]['velocity'] for n in JOINTS],
                 acceleration=[limits[n]['acceleration'] for n in JOINTS],speed=speed,duration_s=duration)
    result=subprocess.run([str(binary)],input=json.dumps(payload,allow_nan=False),text=True,capture_output=True,timeout=15)
    if result.returncode: raise ValueError('20秒方案不可行，保留幅度并拒绝执行: '+result.stderr.strip()[-600:])
    response=json.loads(result.stdout);points=response['points']
    if not 2<=len(points)<=2000: raise ValueError('整轮采样点数量异常')
    scaled={n:dict(b,velocity=b['velocity']*speed,acceleration=b['acceleration']*speed) for n,b in limits.items()}
    check_trajectory(JOINTS,points,scaled)
    if abs(points[-1]['time_s']-duration)>1e-6 or not np.allclose(points[0]['positions'],q[0],atol=1e-7,rtol=0) or not np.allclose(points[-1]['positions'],q[-1],atol=1e-7,rtol=0):
        raise ValueError('时间参数化改变了起终点或时长')
    if max(abs(x) for x in points[0]['velocities']+points[-1]['velocities'])>1e-7: raise ValueError('整轮起终点不为零速')
    return points


def cubic_segment(left,right):
    from scipy.interpolate import CubicHermiteSpline
    dt=right['time_s']-left['time_s']
    if not math.isfinite(dt) or dt<=0: raise ValueError('三次插值时间不递增')
    return CubicHermiteSpline([0.,dt],[left['positions'],right['positions']],
                             [left['velocities'],right['velocities']],axis=0)


def cubic_extrema(left,right):
    curve=cubic_segment(left,right);dt=right['time_s']-left['time_s']
    lower=[];upper=[];vmax=[];amax=[]
    for i in range(6):
        a,b,c,d=curve.c[:,0,i]
        stationary=[float(x.real) for x in np.roots([3*a,2*b,c]) if abs(x.imag)<1e-10 and 0<x.real<dt]
        values=[float(curve(t)[i]) for t in [0.,dt]+stationary]
        vtimes=[0.,dt]+([-b/(3*a)] if abs(a)>1e-15 and 0<-b/(3*a)<dt else [])
        lower.append(min(values));upper.append(max(values))
        vmax.append(max(abs(float(curve(t,1)[i])) for t in vtimes))
        amax.append(max(abs(float(curve(t,2)[i])) for t in [0.,dt]))
    return dict(lower=np.array(lower),upper=np.array(upper),velocity=np.array(vmax),acceleration=np.array(amax))


def smooth_retime(q,names,limits,speed):
    from scipy.interpolate import PchipInterpolator
    from scipy.optimize import brentq
    unique=[q[0]]
    for row in q[1:]:
        if np.max(np.abs(row-unique[-1]))>1e-10: unique.append(row)
    if len(unique)==1: unique.append(unique[0].copy())
    q=np.asarray(unique)
    distance=np.concatenate([[0.],np.cumsum(np.maximum(np.max(np.abs(np.diff(q,axis=0)),axis=1),1e-6))])
    progress=distance/distance[-1]
    # Give the stroke long start/end intervals rather than compressing acceleration into one tiny edge.
    grid=np.array([brentq(lambda u:10*u**3-15*u**4+6*u**5-float(s),0.,1.) for s in progress])
    velocity=PchipInterpolator(grid,q,axis=0).derivative()(grid)
    velocity[0]=0.;velocity[-1]=0.
    points=[dict(positions=row.tolist(),velocities=v.tolist(),accelerations=[],time_s=float(t)) for row,v,t in zip(q,velocity,grid)]
    factor=1.
    for left,right in zip(points,points[1:]):
        extreme=cubic_extrema(left,right)
        for i,n in enumerate(names):
            factor=max(factor,extreme['velocity'][i]/(limits[n]['velocity']*speed),
                       math.sqrt(extreme['acceleration'][i]/(limits[n]['acceleration']*speed)))
    factor*=1.01  # Numerical margin; uniform stretch preserves the spatial cubic curve.
    for point in points:
        point['time_s']*=factor
        point['velocities']=(np.array(point['velocities'])/factor).tolist()
    check_trajectory(names,points,limits)
    return points


def path_samples(points,spacing=.01):
    yield np.asarray(points[0]['positions'])
    for left,right in zip(points,points[1:]):
        if left['accelerations']==[] and right['accelerations']==[]:
            curve=cubic_segment(left,right);dt=right['time_s']-left['time_s']
            count=max(2,int(math.ceil(float(np.max(cubic_extrema(left,right)['velocity']))*dt/spacing)))
            for t in np.linspace(0.,dt,count+1)[1:]: yield curve(t)
        else:
            q0=np.asarray(left['positions']);q1=np.asarray(right['positions'])
            count=max(1,int(math.ceil(float(np.max(np.abs(q1-q0)))/spacing)))
            yield from np.linspace(q0,q1,count+1)[1:]


def flange_target(tcp_pose, offset):
    pose = cal.finite_array(tcp_pose, (7,), 'tcp_pose')
    return cal.tcp_to_flange(pose[:3], pose[3:], offset)


def pose_error(target, actual):
    target = cal.finite_array(target, (7,), 'target')
    actual = cal.finite_array(actual, (7,), 'feedback')
    return dict(position_mm=float(np.linalg.norm(target[:3]-actual[:3])*1000),
                orientation_deg=float(np.degrees((Rotation.from_quat(target[3:]).inv()
                                                 * Rotation.from_quat(actual[3:])).magnitude())))


def retime(positions, names, limits, speed, *, smooth=False):
    if not math.isfinite(speed) or not .005 <= speed <= .1:
        raise ValueError('速度缩放仅允许0.005..0.1，默认0.03')
    if names != JOINTS: raise ValueError('仅支持Piper六轴轨迹，禁止带夹爪运动')
    q = [cal.finite_array(p, (6,), 'joint positions') for p in positions]
    if len(q) < 2: raise ValueError('轨迹至少两个点')
    velocity = np.array([limits[n]['velocity'] for n in names])*speed
    acceleration = np.array([limits[n]['acceleration'] for n in names])*speed
    if not np.isfinite(velocity).all() or not np.isfinite(acceleration).all() or min(*velocity,*acceleration) <= 0:
        raise ValueError('速度/加速度限位缺失或非法')
    if smooth: return smooth_retime(q,names,limits,speed)
    elapsed = 0.; points = []
    for i, position in enumerate(q):
        if i:
            delta = np.abs(position-q[i-1])
            # Zero endpoint v/a quintic: vmax=1.875*d/dt, amax<6*d/dt^2.
            elapsed += max(.25, float(np.max(2*delta/velocity)),
                           float(np.max(np.sqrt(6*delta/acceleration))))
        points.append(dict(positions=position.tolist(), velocities=[0.]*6,
                           accelerations=[0.]*6, time_s=elapsed))
    check_trajectory(names, points, limits)
    return points


def check_trajectory(names, points, limits):
    if names != JOINTS or len(points) < 2: raise ValueError('轨迹关节或数量非法')
    previous = None
    for point in points:
        q = cal.finite_array(point['positions'], (6,), '轨迹位置')
        v = cal.finite_array(point['velocities'], (6,), '轨迹速度')
        a = np.zeros(6) if point['accelerations']==[] else cal.finite_array(point['accelerations'], (6,), '轨迹加速度')
        t = point['time_s']
        if not isinstance(t, (int,float)) or not math.isfinite(t) or t < 0:
            raise ValueError('轨迹时间非法')
        for i,n in enumerate(names):
            bound=limits[n]
            if not bound['lower']-1e-8 <= q[i] <= bound['upper']+1e-8:
                raise ValueError('关节越限: '+n)
            if abs(v[i]) > bound['velocity'] or abs(a[i]) > bound['acceleration']:
                raise ValueError('关节速度/加速度越限: '+n)
        if previous is not None:
            if t <= previous['time_s']: raise ValueError('轨迹时间不递增')
            if (point['accelerations']==[]) != (previous['accelerations']==[]): raise ValueError('混合插值阶数不支持')
            if point['accelerations']==[]:
                extreme=cubic_extrema(previous,point)
                for i,n in enumerate(names):
                    bound=limits[n]
                    if (extreme['lower'][i]<bound['lower']-1e-8 or extreme['upper'][i]>bound['upper']+1e-8
                            or extreme['velocity'][i]>bound['velocity']+1e-8
                            or extreme['acceleration'][i]>bound['acceleration']+1e-8):
                        raise ValueError('三次插值内部位置/速度/加速度越限: '+n)
            if np.max(np.abs(q-np.asarray(previous['positions']))) > .12:
                raise ValueError('关节轨迹跳跃超过0.12rad')
        previous=point


def empty_report(mode):
    return dict(kind='tcp_fixed_point_verification', schema_version=1, mode=mode,
                execution_status='not_started', executed_steps=[], plans=[],
                internal_feedback_note='机器人内部模型/运动跟踪证据，不证明真实尖端漂移',
                external_observation=dict(status='未完成物理精度验证', max_drift_mm=None))


def record_observation(report, drift_mm, method):
    if report.get('mode') != 'execute' or not (report.get('executed_steps') or report.get('inflight_step',{}).get('goal_accepted')):
        raise ValueError('只有实际执行或已接受的中断执行记录才能关联外部物理观察')
    if not math.isfinite(drift_mm) or drift_mm < 0 or len(method.strip()) < 2:
        raise ValueError('必须提供非负有限漂移mm和测量方法')
    report['external_observation'] = dict(status='操作者外部测量已记录', max_drift_mm=drift_mm,
        method=method.strip(), source='operator_manual', independently_verified=False,
        recorded_at=datetime.now().astimezone().isoformat())


def execute_prepared(plans, backend, stop, report):
    try:
        for plan in plans:
            if stop.is_set(): raise InterruptedError('中断：不再派发后续动作')
            backend.guard(plan)
            if stop.is_set(): raise InterruptedError('派发前收到中断')
            target=plan.get('target',{})
            label=f"±{target['amplitude_deg']}deg整轮" if 'amplitude_deg' in target else f"{target.get('angle_deg','')}deg"
            print(f"开始执行 {len(report['executed_steps'])+1}/{len(plans)}: {target.get('axis','')} {label}",flush=True)
            row = backend.execute(plan, stop)
            report['executed_steps'].append(row)
            print(f"已完成 {len(report['executed_steps'])}/{len(plans)}",flush=True)
            if row.get('status') != 'succeeded': raise ValueError('执行结果失败')
        report['execution_status'] = 'completed'
    except BaseException:
        report['execution_status'] = 'aborted'
        if 'inflight_step' in report: report['inflight_step']['status']='aborted'
        backend.cancel_active()
        raise


def confirm_execution(stop):
    print('现场确认执行，请输入 EXECUTE TCP: ',end='',flush=True)
    while True:
        if stop.is_set(): raise InterruptedError('现场确认时收到中断')
        ready,_,_=select.select([sys.stdin],[],[],.1)
        if ready:
            if sys.stdin.readline().strip()!='EXECUTE TCP': raise InterruptedError('操作者未确认执行')
            if stop.is_set(): raise InterruptedError('现场确认后收到中断')
            return


def create_ros_backend(*args, **kwargs):
    try:
        from .tcp_verification_ros import RosBackend
    except ImportError:
        from tcp_verification_ros import RosBackend
    return RosBackend(*args, **kwargs)


def main(args=None):
    parser=argparse.ArgumentParser(description='TCP定点姿态验证：默认离线数学；现场execute需再次输入确认')
    parser.add_argument('--mode', choices=['math','plan','execute'], default='math')
    parser.add_argument('--dry-run', action='store_true', help='离线预览，不导入ROS、不写文件')
    parser.add_argument('--candidate')
    parser.add_argument('--allow-unqualified', action='store_true')
    parser.add_argument('--waiver-reason', default='')
    parser.add_argument('--tasks-stopped', action='store_true')
    parser.add_argument('--point-aligned-with-gap', action='store_true')
    parser.add_argument('--angle-deg', type=float, default=5.)
    parser.add_argument('--steps', type=int, default=5)
    parser.add_argument('--axes', nargs='+', default=['x','y'])
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--continuous-round',action='store_true',help='全程单轨迹，无分段停顿，保留指定幅度')
    parser.add_argument('--motion-seconds',type=float,default=18.,help='整轮模式轨迹秒数，最大18；运动阶段20秒超时')
    parser.add_argument('--arm-workspace',help='运行中T1的工作空间；仅只读核对该树的进程记录')
    parser.add_argument('--sweep',action='store_true',help='每条摆动边合并为一程，三次插值连续经过中间点')
    parser.add_argument('--speed', type=float, default=.03)
    parser.add_argument('--position-tolerance-mm', type=float, default=5.)
    parser.add_argument('--start-pose', nargs=7, type=float, default=[.3,0,.2,0,0,0,1])
    parser.add_argument('--output')
    parser.add_argument('--record-observation', metavar='REPORT_JSON')
    parser.add_argument('--max-drift-mm', type=float)
    parser.add_argument('--measurement-method', default='')
    options=parser.parse_args(args)
    if options.dry_run and options.mode != 'math': parser.error('--dry-run仅允许math模式')
    report=empty_report(options.mode); backend=None; stop=threading.Event()
    root=Path(__file__).resolve().parents[3]
    output=Path(options.output).expanduser() if options.output else root/'data/calibration/tcp_verification'/(
        datetime.now().strftime('%Y-%m-%d_%H-%M-%S')+'_'+uuid.uuid4().hex[:8])/'report.json'
    old_handlers={}
    report['report_file']=str(output.resolve())
    def interrupt(signum,frame): stop.set()
    try:
        if options.record_observation:
            if options.dry_run or options.max_drift_mm is None: raise ValueError('外部观察需要--max-drift-mm且不能dry-run')
            source=Path(options.record_observation).expanduser().resolve()
            report=json.loads(source.read_text()); record_observation(report,options.max_drift_mm,options.measurement_method)
            output=Path(options.output).expanduser() if options.output else source
            report['report_file']=str(output.resolve())
            cal.save_json(output,report); print('人工物理观测已记录:',output); return 0
        if not math.isfinite(options.position_tolerance_mm) or not 1 <= options.position_tolerance_mm <= 10:
            raise ValueError('内部位置跟踪容差仅允许1..10mm')
        speed_max=.5 if options.continuous_round else .1
        if not math.isfinite(options.speed) or not .005 <= options.speed <= speed_max:
            raise ValueError('speed超出当前模式允许范围')
        if options.continuous_round:
            if options.cycles!=1 or not math.isfinite(options.motion_seconds) or not 0<options.motion_seconds<=18:
                raise ValueError('整轮模式仅1轮，motion-seconds须(0,18]')
            options.sweep=True
        assessment = load_candidate(options.candidate, allow_unqualified=options.allow_unqualified,
                                    waiver_reason=options.waiver_reason) if options.candidate else None
        report['candidate_assessment']=assessment
        report['settings']=dict(angle_deg=options.angle_deg,steps=options.steps,axes=options.axes,
                                cycles=options.cycles,sweep=options.sweep,continuous_round=options.continuous_round,
                                motion_seconds=options.motion_seconds if options.continuous_round else None,speed=options.speed,position_tolerance_mm=options.position_tolerance_mm)
        if options.mode == 'math':
            report['start_source']='offline_demo_non_feedback'
            report['targets']=make_targets(options.start_pose, angle_deg=options.angle_deg,steps=options.steps,
                                            axes=options.axes,cycles=options.cycles)
            if options.continuous_round: report['targets']=[group_round(report['targets'],options.start_pose,options.angle_deg)]
            elif options.sweep: report['targets']=group_sweeps(report['targets'],options.steps)
            offset=assessment['tcp_offset'] if assessment else [.02,-.01,.12,.2,-.3,.4]
            report['math_flange_targets']=[flange_target(t['pose'],offset).tolist() for t in report['targets']]
            report['execution_status']='math_only_no_motion'
            print(json.dumps({k:report[k] for k in ('mode','execution_status','start_source','candidate_assessment','settings')} |
                             dict(target_count=len(report['targets']),fixed_xyz=options.start_pose[:3],
                                  note='offline_demo；仅数学示例，无真实反馈、无运动'),
                             ensure_ascii=False,indent=2,allow_nan=False))
        else:
            if assessment is None or not options.tasks_stopped:
                raise ValueError('现场规划/执行需要--candidate与--tasks-stopped，并关闭T5/T6')
            if options.mode == 'execute' and (not options.point_aligned_with_gap or not sys.stdin.isatty()):
                raise ValueError('执行需要真实TTY、--point-aligned-with-gap及现场确认')
            for signum in (signal.SIGINT,signal.SIGTERM):
                old_handlers[signum]=signal.signal(signum,interrupt)
            backend=create_ros_backend(assessment,options,stop,report)
            start=backend.preflight()
            report['start_source']='fresh_real_feedback'; report['start']=start
            planning_pose=backend.compute_fk(start['joints']) if options.continuous_round else start['pose']
            if options.continuous_round:
                report['planning_reference']=dict(source='FK_of_fresh_actual_joint_feedback',model_pose=planning_pose,sdk_feedback_reference=start['pose'])
            targets=make_targets(planning_pose,angle_deg=options.angle_deg,steps=options.steps,
                                  axes=options.axes,cycles=options.cycles)
            if options.continuous_round:
                targets=[group_round(targets,planning_pose,options.angle_deg)]
                targets[0]['feedback_reference_pose']=start['pose']
            elif options.sweep: targets=group_sweeps(targets,options.steps)
            report['targets']=targets
            seed=start['joints']
            for target in targets:
                if stop.is_set(): raise InterruptedError('规划被中断')
                plan=backend.plan(seed,target); report['plans'].append(plan)
                seed=plan['end_state']
            report['execution_status']='planned_only'
            cal.save_json(output,report)
            if options.mode == 'execute':
                print('原质量及豁免:',json.dumps(assessment,ensure_ascii=False))
                print(f"已完成{len(targets)}段规划，轨迹时长合计{sum(p['points'][-1]['time_s'] for p in report['plans']):.1f}秒（另有核查时间）。确认T5/T6已关闭、工具点已对准固定参考点且留有间隙，T1启动参数匹配。")
                print('本过程不会回零。中断只取消本工具的执行goal，不追加运动。')
                confirm_execution(stop)
                execute_prepared(report['plans'],backend,stop,report)
        if not options.dry_run:
            cal.save_json(output,report); print('验证报告:',output)
        return 0
    except Exception as exc:
        report['execution_status']='aborted';report['error']=str(exc)
        if backend is not None: backend.cancel_active()
        if not options.dry_run: cal.save_json(output,report)
        print('TCP验证已停止:',exc,file=sys.stderr); return 2
    finally:
        if backend is not None:
            backend.close()
            cal.save_json(output,report)
        for signum, handler in old_handlers.items(): signal.signal(signum,handler)


if __name__=='__main__': sys.exit(main())
