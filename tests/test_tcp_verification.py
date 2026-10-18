"""Offline verification tests. No ROS node or real motion is created."""
import hashlib
import json
from pathlib import Path
import sys
import threading

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/handeye_calibration_ros'))
from handeye_calibration_ros import tcp_calibration as cal
from handeye_calibration_ros import tcp_verification as verify


def candidate(tmp_path, noisy=False):
    rng = np.random.default_rng(6017)
    current = [.02, -.01, .04, .1, -.2, .3]
    xyz = np.array([.015, -.005, .145])
    samples = []
    for _ in range(12):
        flange = np.eye(4); flange[:3, :3] = Rotation.random(random_state=rng).as_matrix()
        flange[:3, 3] = [.4, -.1, .2] - flange[:3, :3] @ xyz
        if noisy: flange[:3, 3] += rng.normal(0, .006, 3)
        tcp = flange @ cal.offset_transform(current)
        samples.append(dict(position_m=tcp[:3, 3].tolist(),
                            quaternion_xyzw=Rotation.from_matrix(tcp[:3, :3]).as_quat().tolist()))
    payload = dict(kind='tcp_pivot_session', schema_version=1, units='m', quaternion_order='xyzw',
                   current_tcp_offset=current, robot_base_frame='base_link', flange_frame='link6', samples=samples)
    path = tmp_path / 'candidate.json'
    cal.save_solution(payload, tmp_path / 'frames.json', path)
    return path


def test_nonzero_xyz_rpy_full_inverse():
    offset = [.03, -.02, .13, .3, -.4, .7]
    start = [.4, -.1, .25, *Rotation.from_euler('xyz', [.2, .5, -.1]).as_quat()]
    targets = verify.make_targets(start, angle_deg=5, steps=5)
    for target in targets:
        flange = verify.flange_target(target['pose'], offset)
        reconstructed = flange @ cal.offset_transform(offset)
        np.testing.assert_allclose(reconstructed[:3, 3], start[:3], atol=1e-12)
        np.testing.assert_allclose(reconstructed[:3, :3], Rotation.from_quat(target['pose'][3:]).as_matrix(), atol=1e-12)
    assert not np.allclose(verify.flange_target(targets[0]['pose'], offset)[:3, 3],
                           verify.flange_target(targets[4]['pose'], offset)[:3, 3])


def test_fixed_point_sequence_uses_two_axes_and_returns_orientation():
    start = [.3, 0., .2, 0., 0., 0., 1.]
    targets = verify.make_targets(start)
    assert len(targets) == 40
    assert {x['axis'] for x in targets} == {'x', 'y'}
    assert max(abs(x['angle_deg']) for x in targets) == 5
    for t in targets: np.testing.assert_allclose(t['pose'][:3], start[:3], atol=0)
    np.testing.assert_allclose(targets[-1]['pose'], start, atol=1e-12)


def test_valid_candidate_recomputed(tmp_path):
    assessment = verify.load_candidate(candidate(tmp_path))
    assert assessment['qualified'] is True
    assert assessment['quality_waiver'] is None
    np.testing.assert_allclose(assessment['tcp_offset'], [.015, -.005, .145, .1, -.2, .3], atol=1e-9)


@pytest.mark.parametrize('kind', ['hash', 'value', 'null', 'fake_passed'])
def test_invalid_candidate_rejected(tmp_path, kind):
    path = candidate(tmp_path, noisy=(kind == 'fake_passed'))
    value = json.loads(path.read_text())
    if kind == 'hash': value['input_sha256'] = '0' * 64
    if kind == 'value': value['tcp_offset'][0] += .01
    if kind == 'null': value['tcp_offset'] = None
    if kind == 'fake_passed': value['quality_status'] = 'passed'
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError): verify.load_candidate(path)


def test_failed_requires_explicit_reasoned_verification_only_waiver(tmp_path):
    path = candidate(tmp_path, noisy=True)
    before = path.read_bytes()
    with pytest.raises(ValueError): verify.load_candidate(path)
    with pytest.raises(ValueError): verify.load_candidate(path, allow_unqualified=True)
    assessment = verify.load_candidate(path, allow_unqualified=True, waiver_reason='用户接受粗标定，只用于观察验证')
    assert assessment['qualified'] is False
    assert assessment['quality_waiver']['scope'] == 'verification_only'
    assert assessment['source_quality_status'] == 'failed'
    assert path.read_bytes() == before
    assert len(assessment['tcp_offset']) == 6


def limits():
    return {f'joint{i}': dict(lower=-3., upper=3., velocity=1., acceleration=2.) for i in range(1, 7)}


def test_conservative_retiming_and_joint_limits():
    names = list(limits())
    points = verify.retime([[0.] * 6, [.01] * 6, [.02] * 6], names, limits(), .03)
    assert points[0]['time_s'] == 0
    assert points[-1]['time_s'] > points[1]['time_s'] > 0
    assert all(p['velocities'] == [0.] * 6 for p in points)
    verify.check_trajectory(names, points, limits())
    with pytest.raises(ValueError): verify.retime([[0.] * 6, [4.] * 6], names, limits(), .03)
    with pytest.raises(ValueError): verify.retime([[0.] * 6, [1.] * 6], names, limits(), .03)


def test_dry_run_never_constructs_ros_backend_or_writes(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(verify, 'create_ros_backend', lambda *a, **k: pytest.fail('ROS in dry-run'))
    out = tmp_path / 'forbidden.json'
    assert verify.main(['--dry-run', '--output', str(out)]) == 0
    assert not out.exists()
    assert 'offline_demo' in capsys.readouterr().out


class FakeBackend:
    def __init__(self, stop, failure=None):
        self.stop, self.failure, self.sent, self.cancelled = stop, failure, [], 0
    def guard(self, plan=None):
        if self.failure == 'stale': raise ValueError('feedback stale')
    def execute(self, plan, stop):
        self.sent.append(plan)
        if self.failure == 'timeout': raise TimeoutError('timeout')
        self.stop.set()
        return dict(status='succeeded', target_pose=[.3,0,.2,0,0,0,1], feedback_pose=[.3,0,.2,0,0,0,1])
    def cancel_active(self): self.cancelled += 1


@pytest.mark.parametrize('failure', [None, 'timeout', 'stale'])
def test_interruption_or_failure_stops_future_dispatch_and_cancels(failure):
    stop = threading.Event(); backend = FakeBackend(stop, failure)
    report = dict(executed_steps=[], execution_status='not_started')
    with pytest.raises((ValueError, TimeoutError, InterruptedError)):
        verify.execute_prepared([{'step': 1}, {'step': 2}], backend, stop, report)
    assert len(backend.sent) <= 1
    assert backend.cancelled == 1
    assert report['execution_status'] == 'aborted'


def test_internal_feedback_cannot_be_external_measurement():
    report = verify.empty_report('plan')
    assert report['external_observation']['status'] == '未完成物理精度验证'
    with pytest.raises(ValueError): verify.record_observation(report, .2, '量表')

from types import SimpleNamespace as NS
from concurrent.futures import Future
from handeye_calibration_ros import tcp_verification_ros as adapter


def bare_backend():
    b=adapter.RosBackend.__new__(adapter.RosBackend)
    b.report={}; b.stop=threading.Event(); b.parameter_changed=False
    b.lock=threading.Lock(); b.latest={}; b.limits=limits()
    b.options=NS(speed=.03,position_tolerance_mm=5.)
    return b


def test_input_mutation_detected_after_initial_assessment(tmp_path):
    path=candidate(tmp_path); assessment=verify.load_candidate(path)
    verify.verify_input_hashes(assessment)
    snapshot=Path(assessment['input_file']);snapshot.write_bytes(snapshot.read_bytes()+b' ')
    with pytest.raises(ValueError,match='发生变化'): verify.verify_input_hashes(assessment)


@pytest.mark.parametrize('fraction', [.99, float('nan'), 0.])
def test_real_adapter_rejects_partial_cartesian_path(fraction):
    b=bare_backend(); b.fresh_feedback=lambda: None
    b.Cart=NS(Request=lambda:NS(header=NS())); b.state=lambda q:q; b.pose_message=lambda p:p; b.cart=object()
    b.call=lambda *a,**kw:NS(error_code=NS(val=1),fraction=fraction)
    with pytest.raises(ValueError,match='路径不完整'): b.plan(dict.fromkeys(verify.JOINTS,0.),{'pose':[0,0,0,0,0,0,1]})


def test_planner_seed_must_match_first_point():
    b=bare_backend(); b.fresh_feedback=lambda: None
    b.Cart=NS(Request=lambda:NS(header=NS()));b.state=lambda q:q;b.pose_message=lambda p:p;b.cart=object()
    b.call=lambda *a,**kw:NS(error_code=NS(val=1),fraction=1.,solution=NS(joint_trajectory=NS(
        joint_names=verify.JOINTS,points=[NS(positions=[.001]*6),NS(positions=[.002]*6)])))
    with pytest.raises(ValueError,match='起点不一致'): b.plan(dict.fromkeys(verify.JOINTS,0.),{'pose':[0,0,0,0,0,0,1]})


@pytest.mark.parametrize('collision,drift', [(True,0.),(False,.002)])
def test_adapter_checks_collision_and_fixed_xyz_along_path(collision,drift):
    b=bare_backend();b.Valid=NS(Request=lambda:NS());b.valid=object();b.state=lambda q:q
    b.call=lambda *a,**k:NS(valid=not collision)
    b.compute_fk=lambda q:[drift,0,0,0,0,0,1]
    plan={'points':[{'positions':[0.]*6},{'positions':[.01]*6}], 'target':{'pose':[0,0,0,0,0,0,1]}}
    with pytest.raises(ValueError): b.validate_path(plan,dict.fromkeys(verify.JOINTS,0.))


def test_guard_adds_timed_and_validated_actual_start_bridge(tmp_path):
    b=bare_backend();b.assessment=verify.load_candidate(candidate(tmp_path));b.node=NS(get_node_names=lambda:[])
    state=dict(joints=dict.fromkeys(verify.JOINTS,.003),control=dict.fromkeys(verify.JOINTS,.003))
    b.fresh_feedback=lambda:state;b.wait_for_stationary=lambda:state;b.check_scene=lambda:None;checked=[]
    b.validate_path=lambda p,s:checked.append(p['points'])
    plan=dict(start_state=dict.fromkeys(verify.JOINTS,0.),points=verify.retime([[0.]*6,[.01]*6],verify.JOINTS,limits(),.03))
    b.guard(plan)
    assert checked and plan['points'][0]['positions']==[.003]*6
    assert plan['points'][1]['time_s']>0
    assert plan['dispatch_start']==state['joints']
    count=len(plan['points'])
    b.guard(plan)
    assert len(plan['points'])==count and len(plan['planned_positions'])==2


@pytest.mark.parametrize('fault', ['joint_angle_limit','communication_status_joint','missing'])
def test_hardware_fault_arrays_rejected_even_when_err_status_zero(fault):
    import time
    b=bare_backend(); stamp=NS(sec=1,nanosec=0)
    pose=NS(header=NS(frame_id='base_link',stamp=stamp),pose=NS(position=NS(x=0,y=0,z=0),orientation=NS(x=0,y=0,z=0,w=1)))
    joints=NS(name=verify.JOINTS,position=[0.]*6)
    arm=NS(ctrl_mode=1,arm_status=0,err_status=0,joint_angle_limit=[False]*6,communication_status_joint=[False]*6)
    if fault=='missing': arm.communication_status_joint=[]
    else: getattr(arm,fault)[2]=True
    now=time.monotonic()
    b.latest={k:(v,now,now,1) for k,v in dict(pose=pose,joints=joints,control=joints,arm=arm).items()}
    with pytest.raises(ValueError,match='故障/反馈不完整'):b.fresh_feedback()


def test_stamped_feedback_replay_does_not_refresh_age(monkeypatch):
    b=bare_backend();times=iter([10.,12.]);monkeypatch.setattr(adapter.time,'monotonic',lambda:next(times))
    msg=NS(header=NS(stamp=NS(sec=1,nanosec=0)))
    b.receive('pose',msg);b.receive('pose',msg)
    assert b.latest['pose'][1]==12. and b.latest['pose'][2]==10.


def test_plan_mode_never_calls_execute(tmp_path,monkeypatch):
    path=candidate(tmp_path);out=tmp_path/'plan.json'
    class PlanBackend:
        def preflight(self):return dict(pose=[.3,0,.2,0,0,0,1],joints=dict.fromkeys(verify.JOINTS,0.))
        def plan(self,seed,target):return dict(end_state=seed,target=target)
        def execute(self,*a):pytest.fail('plan mode sent motion')
        def close(self):pass
        def cancel_active(self):pass
    monkeypatch.setattr(verify,'create_ros_backend',lambda *a:PlanBackend())
    assert verify.main(['--mode','plan','--candidate',str(path),'--tasks-stopped','--output',str(out)])==0
    report=json.loads(out.read_text());assert report['execution_status']=='planned_only'
    assert report['executed_steps']==[] and len(report['plans'])==40


def test_failed_default_cli_refuses_before_ros(tmp_path,monkeypatch):
    path=candidate(tmp_path,noisy=True)
    monkeypatch.setattr(verify,'create_ros_backend',lambda *a:pytest.fail('invalid candidate created ROS'))
    assert verify.main(['--mode','plan','--candidate',str(path),'--tasks-stopped','--output',str(tmp_path/'rejected.json')])==2


def test_startup_offset_reads_python_console_script_argv(tmp_path,monkeypatch):
    import runtime_process_manager as process
    params=tmp_path/'params.yaml';params.write_text('/**:\n  ros__parameters:\n    tcp_offset: !!python/tuple [0.01, 0.02, 0.13, 0.1, -0.2, 0.3]\n')
    proc=tmp_path/'cmdline';proc.write_bytes(('\0'.join(['/usr/bin/python3','/ws/install/lib/agx_arm_ctrl_single','--params-file',str(params),''])).encode())
    real_path=Path
    monkeypatch.setattr(adapter,'Path',lambda value:proc if str(value)=='/proc/123/cmdline' else real_path(value))
    monkeypatch.setattr(process,'ProcessManager',lambda w:NS(get_record=lambda k:{'pgid':9},verify_identity=lambda r:(True,'')))
    monkeypatch.setattr(process,'_pgid_members',lambda p:[123])
    assert adapter.startup_offset(tmp_path)['offset']==[.01,.02,.13,.1,-.2,.3]


def test_late_acceptance_cancel_owner_survives_until_terminal(tmp_path,monkeypatch):
    import types
    module=types.ModuleType('action_msgs.msg');module.GoalStatus=NS(STATUS_SUCCEEDED=4)
    monkeypatch.setitem(sys.modules,'action_msgs.msg',module)
    b=bare_backend();b.assessment=verify.load_candidate(candidate(tmp_path))
    joints=dict.fromkeys(verify.JOINTS,0.);b.fresh_feedback=lambda:dict(joints=joints)
    accepted=Future();terminal=Future();cancelled=[]
    def cancel():
        cancelled.append(True);done=Future();done.set_result(NS(return_code=0,goals_canceling=[1]));return done
    handle=NS(accepted=True,get_result_async=lambda:terminal,cancel_goal_async=cancel)
    b.execute_client=NS(wait_for_server=lambda **k:True,send_goal_async=lambda goal:accepted)
    b.RobotTrajectory=lambda:NS(joint_trajectory=NS(points=[]));b.Point=lambda:NS(time_from_start=NS())
    b.Execute=NS(Goal=lambda:NS())
    b.pending_goal=None;b.active_goal=None;b.result_future=None;b.cancel_future=None;b.cancel_requested=False
    b.wait=lambda *a,**k:(_ for _ in ()).throw(TimeoutError('delayed acceptance'))
    plan=dict(dispatch_start=joints,target=dict(step=1,pose=[0,0,0,0,0,0,1]),
              points=verify.retime([[0.]*6,[.01]*6],verify.JOINTS,limits(),.03))
    with pytest.raises(TimeoutError):b.execute(plan,b.stop)
    assert 'inflight_step' in b.report
    b.cancel_requested=True;accepted.set_result(handle)
    assert cancelled and b.result_future is terminal and not b.cancellation_resolved()
    terminal.set_result(NS(status=5))
    assert b.cancellation_resolved()


def test_external_observation_requires_real_execution_record():
    report=verify.empty_report('execute');report['executed_steps']=[{'status':'succeeded'}]
    verify.record_observation(report,1.5,'固定尖点侧视标尺测量')
    assert report['external_observation']['source']=='operator_manual'
    assert report['external_observation']['independently_verified'] is False

@pytest.mark.parametrize('status', [0,None,'exception',4,5,6])
def test_cancel_requires_confirmed_terminal_result(status):
    b=bare_backend();b.pending_goal=Future();b.pending_goal.set_result(NS(accepted=True))
    b.result_future=Future()
    if status=='exception':b.result_future.set_exception(RuntimeError('transport error'))
    else:b.result_future.set_result(None if status is None else NS(status=status))
    assert b.cancellation_resolved()==(status in (4,5,6))


def test_parameter_change_after_guard_prevents_goal_dispatch(tmp_path,monkeypatch):
    import types
    module=types.ModuleType('action_msgs.msg');module.GoalStatus=NS(STATUS_SUCCEEDED=4)
    monkeypatch.setitem(sys.modules,'action_msgs.msg',module)
    b=bare_backend();b.assessment=verify.load_candidate(candidate(tmp_path));b.parameter_changed=True
    joints=dict.fromkeys(verify.JOINTS,0.);b.fresh_feedback=lambda:dict(joints=joints)
    b.execute_client=NS(wait_for_server=lambda **k:True,send_goal_async=lambda goal:pytest.fail('parameter changed but sent'))
    b.RobotTrajectory=lambda:NS(joint_trajectory=NS(points=[]));b.Point=lambda:NS(time_from_start=NS());b.Execute=NS(Goal=lambda:NS())
    plan=dict(dispatch_start=joints,target=dict(step=1,pose=[0,0,0,0,0,0,1]),
              points=verify.retime([[0.]*6,[.01]*6],verify.JOINTS,limits(),.03))
    with pytest.raises(ValueError,match='派发前TCP参数改变'):b.execute(plan,b.stop)


def test_cancel_owner_survives_report_disk_failure(monkeypatch):
    b=bare_backend();b.report={'report_file':'/unused/report.json'}
    b.pending_goal=Future();b.pending_goal.set_result(NS(accepted=True));b.active_goal=NS()
    b.result_future=Future();b.thread=None;b.executor=None;b.node=None;b.rclpy=NS(ok=lambda:False)
    b.shutdown_event=threading.Event();calls=[]
    def cancel():
        calls.append(True)
        if len(calls)==2:b.result_future.set_result(NS(status=5))
    b.cancel_active=cancel
    monkeypatch.setattr(adapter.cal,'save_json',lambda *a:(_ for _ in ()).throw(OSError('disk full')))
    monkeypatch.setattr(adapter.time,'sleep',lambda *a:None)
    b.close()
    assert len(calls)==2 and b.shutdown_event.is_set()
    assert b.report['cancellation_state']=='no_owned_nonterminal_goal'


def test_confirmation_interrupt_exits_without_newline(monkeypatch):
    stop=threading.Event()
    def readiness(*args):stop.set();return [],[],[]
    monkeypatch.setattr(verify.select,'select',readiness)
    with pytest.raises(InterruptedError,match='现场确认时'):verify.confirm_execution(stop)


def test_quintic_retime_obeys_scaled_interior_derivative_limits():
    points=verify.retime([[0.]*6,[.02]*6,[.03]*6],verify.JOINTS,limits(),.03)
    for left,right in zip(points,points[1:]):
        dt=right['time_s']-left['time_s'];delta=np.asarray(right['positions'])-left['positions']
        for u in np.linspace(0,1,101):
            velocity=np.abs(delta*(30*u**2-60*u**3+30*u**4)/dt)
            acceleration=np.abs(delta*(60*u-180*u**2+120*u**3)/dt**2)
            assert np.max(velocity)<=1.*.03 and np.max(acceleration)<=2.*.03


def test_aborted_accepted_motion_can_keep_external_observation():
    report=verify.empty_report('execute');report['inflight_step']={'status':'aborted','goal_accepted':True}
    verify.record_observation(report,4.,'侧视相机标尺人工测量')
    assert report['external_observation']['max_drift_mm']==4.


def test_small_start_shift_revalidates_bridge_before_dispatch(tmp_path,monkeypatch):
    """A settled 0.0002rad change must rebuild the checked start, never jump or give up immediately."""
    import types
    module=types.ModuleType('action_msgs.msg');module.GoalStatus=NS(STATUS_SUCCEEDED=4)
    monkeypatch.setitem(sys.modules,'action_msgs.msg',module)
    b=bare_backend();b.assessment=verify.load_candidate(candidate(tmp_path))
    zero=dict.fromkeys(verify.JOINTS,0.);actual=dict.fromkeys(verify.JOINTS,.0002)
    b.fresh_feedback=lambda:dict(joints=actual)
    sent=[];rebuilt=[];pending=Future()
    b.execute_client=NS(wait_for_server=lambda **k:True,send_goal_async=lambda goal:(sent.append(goal) or pending))
    b.RobotTrajectory=lambda:NS(joint_trajectory=NS(points=[]));b.Point=lambda:NS(time_from_start=NS());b.Execute=NS(Goal=lambda:NS())
    b.pending_goal=None;b.active_goal=None;b.result_future=None;b.cancel_future=None;b.cancel_requested=False
    def guard(plan):
        rebuilt.append(True);plan['dispatch_start']=actual.copy()
        plan['points']=verify.retime([[.0002]*6,[.01]*6],verify.JOINTS,limits(),.03)
    b.guard=guard
    b.wait=lambda *a,**k:(_ for _ in ()).throw(TimeoutError('fake acceptance boundary reached'))
    plan=dict(dispatch_start=zero,target=dict(step=1,pose=[0,0,0,0,0,0,1]),
              points=verify.retime([[0.]*6,[.01]*6],verify.JOINTS,limits(),.03))
    with pytest.raises(TimeoutError,match='fake acceptance boundary reached'):b.execute(plan,b.stop)
    assert rebuilt==[True] and len(sent)==1
    assert sent[0].trajectory.joint_trajectory.points[0].positions==[.0002]*6


def test_stationary_window_waits_for_settling_with_new_feedback(monkeypatch):
    b=bare_backend();clock=[0.];index=[0]
    monkeypatch.setattr(adapter.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(adapter.time,'sleep',lambda t:clock.__setitem__(0,clock[0]+t))
    def feedback():
        index[0]+=1
        q=.002 if clock[0]<.15 else .003
        return dict(joints=dict.fromkeys(verify.JOINTS,q),joint_source_stamp_ns=index[0])
    b.fresh_feedback=feedback
    state=b.wait_for_stationary()
    assert clock[0]>=.45 and state['joints']['joint1']==.003


def test_persistent_start_shift_is_bounded_and_sends_no_goal(tmp_path,monkeypatch):
    import types
    module=types.ModuleType('action_msgs.msg');module.GoalStatus=NS(STATUS_SUCCEEDED=4)
    monkeypatch.setitem(sys.modules,'action_msgs.msg',module)
    b=bare_backend();b.assessment=verify.load_candidate(candidate(tmp_path))
    b.fresh_feedback=lambda:dict(joints=dict.fromkeys(verify.JOINTS,.002))
    b.execute_client=NS(wait_for_server=lambda **k:True,send_goal_async=lambda goal:pytest.fail('unstable start dispatched'))
    rebuilds=[];b.guard=lambda plan:rebuilds.append(True)
    plan=dict(dispatch_start=dict.fromkeys(verify.JOINTS,0.),target=dict(step=1))
    with pytest.raises(ValueError,match='复查3次后'):b.execute(plan,b.stop)
    assert len(rebuilds)==3 and len(b.report['start_revalidation'])==4


@pytest.mark.parametrize('moving', [False,True])
def test_stationary_window_rejects_frozen_or_continuously_moving_feedback(monkeypatch,moving):
    b=bare_backend();clock=[0.];index=[0]
    monkeypatch.setattr(adapter.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(adapter.time,'sleep',lambda t:clock.__setitem__(0,clock[0]+t))
    def feedback():
        index[0]+=1
        return dict(joints=dict.fromkeys(verify.JOINTS,clock[0]*.02 if moving else 0.),
                    joint_source_stamp_ns=index[0] if moving else 1)
    b.fresh_feedback=feedback
    with pytest.raises(ValueError,match='未停稳或未收到足够新的关节反馈'):b.wait_for_stationary()
    assert clock[0]<=3.05


def test_sweep_groups_dense_waypoints_into_eight_strokes():
    dense=verify.make_targets([.3,0,.2,0,0,0,1],angle_deg=15,steps=6)
    strokes=verify.group_sweeps(dense,6)
    assert len(strokes)==8
    assert [t['angle_deg'] for t in strokes]==[15,0,-15,0,15,0,-15,0]
    assert all(len(s['waypoints'])==6 for s in strokes)
    assert all(p[:3]==[.3,0,.2] for s in strokes for p in s['waypoints'])


def test_smooth_retime_faster_with_nonzero_interior_velocity():
    q=np.linspace(0,.3,31)
    positions=[[float(x),float(x/2),0.,0.,0.,0.] for x in q]
    old=verify.retime(positions,verify.JOINTS,limits(),.1)
    smooth=verify.retime(positions,verify.JOINTS,limits(),.1,smooth=True)
    assert smooth[-1]['time_s']<old[-1]['time_s']*.7
    assert smooth[0]['velocities']==smooth[-1]['velocities']==[0.]*6
    assert any(abs(p['velocities'][0])>0 for p in smooth[1:-1])
    assert all(p['accelerations']==[] for p in smooth)
    verify.check_trajectory(verify.JOINTS,smooth,limits())


def test_cubic_interior_velocity_or_acceleration_violation_rejected():
    points=[dict(positions=[0.]*6,velocities=[0.]*6,accelerations=[],time_s=0.),
            dict(positions=[.05]*6,velocities=[0.]*6,accelerations=[],time_s=.01)]
    with pytest.raises(ValueError,match='三次插值'):verify.check_trajectory(verify.JOINTS,points,limits())


def test_cubic_path_samples_follow_curve_instead_of_chord():
    points=[dict(positions=[0.]*6,velocities=[0.]*6,accelerations=[],time_s=0.),
            dict(positions=[.1,.1,0,0,0,0],velocities=[.1,0,0,0,0,0],accelerations=[],time_s=2.)]
    rows=list(verify.path_samples(points,spacing=.005))
    assert any(abs(row[0]-row[1])>.005 for row in rows[1:-1])
    assert max(np.max(np.abs(b-a)) for a,b in zip(rows,rows[1:]))<=.005+1e-10


def test_smooth_retime_dense_curve_respects_scaled_limits_and_endpoints():
    positions=[[float(x),float(x*x),float(np.sin(x)),0.,0.,0.] for x in np.linspace(0,.2,25)]
    points=verify.retime(positions,verify.JOINTS,limits(),.08,smooth=True)
    for a,b in zip(points,points[1:]):
        ext=verify.cubic_extrema(a,b)
        assert max(ext['velocity'])<=.08+1e-9
        assert max(ext['acceleration'])<=.16+1e-9
        curve=verify.cubic_segment(a,b);dt=b['time_s']-a['time_s']
        dense=np.array([curve(t) for t in np.linspace(0,dt,101)])
        assert np.all(dense>=ext['lower']-1e-10) and np.all(dense<=ext['upper']+1e-10)
    np.testing.assert_allclose(points[0]['positions'],positions[0])
    np.testing.assert_allclose(points[-1]['positions'],positions[-1])


def test_sweep_dry_run_is_eight_strokes_and_zero_ros(tmp_path,monkeypatch,capsys):
    monkeypatch.setattr(verify,'create_ros_backend',lambda *a:pytest.fail('sweep dry-run created ROS'))
    out=tmp_path/'forbidden.json'
    assert verify.main(['--dry-run','--sweep','--angle-deg','15','--steps','6','--speed','.1','--output',str(out)])==0
    assert not out.exists()
    assert json.loads(capsys.readouterr().out)['target_count']==8


def test_smooth_repeated_points_are_deduplicated_without_dwell():
    points=verify.retime([[0.]*6,[0.]*6,[.05]*6,[.05]*6,[.1]*6],verify.JOINTS,limits(),.1,smooth=True)
    assert len(points)==3 and all(p['accelerations']==[] for p in points)


def test_cubic_position_overshoot_rejected_even_with_endpoints_in_limits():
    bounds=limits()
    for b in bounds.values():b.update(upper=.001,lower=-.001,velocity=100,acceleration=100)
    points=[dict(positions=[0.]*6,velocities=[.1]*6,accelerations=[],time_s=0.),
            dict(positions=[0.]*6,velocities=[-.1]*6,accelerations=[],time_s=1.)]
    with pytest.raises(ValueError,match='三次插值内部'):verify.check_trajectory(verify.JOINTS,points,bounds)


def test_ros_plan_sweep_retains_all_waypoints_and_cubic_timing():
    b=bare_backend();b.options.sweep=True;b.fresh_feedback=lambda:None
    b.Cart=NS(Request=lambda:NS(header=NS()));b.state=lambda q:q;b.pose_message=lambda p:p;b.cart=object()
    requests=[]
    def call(client,request,**kw):
        requests.append(request)
        positions=[[float(x),0,0,0,0,0] for x in np.linspace(0,.08,12)]
        return NS(error_code=NS(val=1),fraction=1.,solution=NS(joint_trajectory=NS(
            joint_names=verify.JOINTS,points=[NS(positions=q) for q in positions])))
    b.call=call;b.validate_path=lambda *a:None
    target=verify.group_sweeps(verify.make_targets([.3,0,.2,0,0,0,1],angle_deg=15,steps=6),6)[0]
    plan=b.plan(dict.fromkeys(verify.JOINTS,0.),target)
    assert len(requests)==1 and len(requests[0].waypoints)==6 and requests[0].avoid_collisions is True
    assert plan['interpolation']=='cubic'
    assert any(abs(p['velocities'][0])>0 for p in plan['points'][1:-1])
    assert all(p['accelerations']==[] for p in plan['points'])
