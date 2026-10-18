"""Real protocol boundaries found in the final review; no hardware starts."""
import copy,json,time,types
import pytest
from workbench.service import Workbench
from workbench.tasks import Tasks
from workbench.calibration import Calibration

def fake_service():
    calls=[]
    def belt(op):calls.append(('belt',op));raise RuntimeError('serial disconnected')
    obj=types.SimpleNamespace(tasks=types.SimpleNamespace(pause=False,continuous=True,queue_running=True),conveyor=belt,
        send=lambda *args:calls.append(('send',*args)),calibration=types.SimpleNamespace(stop_motion_verifications=lambda:calls.append(('verification','stop'))),logs=types.SimpleNamespace(add=lambda *x:calls.append(('log',))))
    return obj,calls

def test_emergency_publishes_before_broken_belt_and_stops_owned_verification():
    obj,calls=fake_service()
    try:Workbench.dispatch(obj,'motion.send',dict(cmd='emergency_stop'))
    except RuntimeError:pass
    assert calls[0][0:2]==('send','/emergency_stop')
    assert ('verification','stop') in calls

def tasks():
    calls=[];t=Tasks(lambda *a:calls.append(a),lambda a:calls.append(a),lambda a:calls.append(a),lambda *a:calls.append(a));return t,calls

def test_only_matching_cycle_result_and_static_failure_pause():
    t,c=tasks();t.inflight='new';t.continuous=True;t.pause=False
    t.result({'cycle_id':'','cycle_success':1});assert t.inflight=='new'
    t.result({'cycle_id':'old','cycle_success':1});assert t.inflight=='new'
    t.result({'cycle_id':'new','cycle_success':0});assert t.pause and not t.continuous and t.inflight is None

def test_failed_track_does_not_hide_new_vision_session():
    t,c=tasks();t.detection('d455',{'motion_session':'old'});t.failed.add(1)
    t.detection('d455',{'motion_session':'new'})
    assert not t.failed

def test_unacknowledged_or_disappeared_receiver_does_not_leave_belt_running():
    t,c=tasks();t.pause=False;t.continuous=True;t.inflight='sent';t.sent_at=time.monotonic()-5;t.acknowledged=False;t.receiver_alive=lambda:False
    t.tick();assert t.pause and t.inflight is None and 'stop' in c

def test_owned_verification_emergency_reuses_stop(tmp_path):
    c=Calibration(tmp_path);calls=[]
    for kind in ('tcp','verify-tcp','verify-handeye'):
        c.sessions[kind]={'kind':kind,'proc':types.SimpleNamespace(stop=lambda k=kind:calls.append(k))}
    c.stop_motion_verifications();assert calls==['verify-tcp']

def test_dabai_onsite_tcp_parent_and_independent_restore(tmp_path):
    folder=tmp_path/'config/calibration';folder.mkdir(parents=True)
    manifest={'cameras':{'dabai':{'mode':'eye_in_hand','parent':'link6','child':'dabai_color_optical_frame','active_file':'dabai_eye_in_hand.json'}}}
    (folder/'manifest.json').write_text(json.dumps(manifest))
    he={'position':[0,0,.04],'orientation':[0,0,0,1],'metadata':{'camera_id':'dabai','mode':'eye_in_hand','robot_pose_frame':'tcp_link','transform_parent':'tcp_link','transform_child':'dabai_color_optical_frame','units':'m','quaternion_order':'xyzw','tcp_offset':[0,0,.15,0,0,0]}}
    path=tmp_path/'dabai.json';path.write_text(json.dumps(he));c=Calibration(tmp_path)
    r=c.apply_calibration('handeye','dabai',path);assert (folder/'dabai_eye_in_hand.json').read_bytes()==path.read_bytes()
    # Applying and restoring another kind must retain this selection.
    tcp=tmp_path/'tcp.json';tcp.write_text(json.dumps({'tcp_offset':[0,0,.15,0,0,0]}));(folder/'tcp.json').write_text(tcp.read_text())
    backup=c.apply_calibration('tcp','dabai',tcp)['backup'];c.restore(backup)
    assert 'handeye:dabai' in json.loads((folder/'manifest.json').read_text())['workbench_selections']

def test_static_pregrasp_setting_reaches_executed_tool_axis_path():
    import ast,pathlib,math
    source=ast.parse((pathlib.Path(__file__).resolve().parents[1]/'auto_sorting_action.py').read_text())
    method=next(x for c in source.body if isinstance(c,ast.ClassDef) for x in c.body if isinstance(x,ast.FunctionDef) and x.name=='move_arm_pose')
    class Quaternion:pass
    namespace={'Quaternion':Quaternion,'math':math,'np':__import__('numpy')}
    exec(compile(ast.Module(body=[method],type_ignores=[]),'<actual T5 move_arm_pose>','exec'),namespace)
    calls=[];q=[0,0,0,1]
    obj=types.SimpleNamespace(last_grasp_extra_depth=0.,ik_solver=object(),current_joints={'joint'+str(i):0 for i in range(1,7)},_static_cycle_settings={'approach_height_m':.09},_active_camera='d455',
        _plan_tool_axis_approach=lambda *a,**kw:calls.append(kw) or ({'x':.1,'y':.2,'z':.3},q,[0]*6,0),move_arm_joint=lambda *a,**kw:True,get_logger=lambda:types.SimpleNamespace(info=lambda *a:None))
    namespace['move_arm_pose'](obj,{'x':.1,'y':.2,'z':.3},'test',fixed_orientation=True,preferred_orientation=q,approach_target_z=.3)
    assert calls[0]['pregrasp_m']==.09

def test_capture_missing_camera_is_in_emitted_progress(tmp_path):
    from workbench.capture import Capture
    events=[];cap=Capture(tmp_path,lambda event,data:events.append(copy.deepcopy(data)))
    result=cap.next_frame(['dabai'],timeout=.01)
    assert result['errors']['dabai'] and events[-1]['errors']['dabai']

def test_t5_rejection_returns_original_cycle_id():
    import ast,pathlib
    tree=ast.parse((pathlib.Path(__file__).resolve().parents[1]/'auto_sorting_action.py').read_text())
    method=next(x for c in tree.body if isinstance(c,ast.ClassDef) for x in c.body if isinstance(x,ast.FunctionDef) and x.name=='cmd_callback')
    def stale(command):raise ValueError('expired target')
    scope={'json':json,'assert_visual_target_fresh':stale};exec(compile(ast.Module(body=[method],type_ignores=[]),'<actual T5 callback>','exec'),scope)
    calls=[];obj=types.SimpleNamespace(validate_workbench_command=lambda c:None,get_logger=lambda:types.SimpleNamespace(error=lambda x:None),workbench_ack=lambda *a:calls.append(a))
    scope['cmd_callback'](obj,types.SimpleNamespace(data=json.dumps({'workbench':True,'cycle_id':'cycle-2','camera_id':'d455'})))
    assert calls[0][0]['cycle_id']=='cycle-2' and calls[0][1] is False

def test_tcp_relative_dabai_mount_converts_to_flange(monkeypatch):
    import numpy as np,xml.etree.ElementTree as ET,runtime_config as rc
    from runtime_geometry import add_dabai_mount
    monkeypatch.setattr(rc,'load_handeye',lambda camera:dict(parent='tcp_link',position=[0,0,.04],orientation=[0,0,0,1],raw={'metadata':{'tcp_offset':[0,0,.15,0,0,0]}}))
    model,_=add_dabai_mount('<robot><link name="link6"/></robot>','<robot/>')
    origin=ET.fromstring(model).find("link[@name='dabai_mount_body']/collision[@name='dabai_body']/origin")
    assert np.fromstring(origin.get('xyz'),sep=' ')[2]==pytest.approx(.165)

def test_restore_one_item_preserves_later_other_selection(tmp_path):
    folder=tmp_path/'config/calibration';folder.mkdir(parents=True);(folder/'manifest.json').write_text('{}')
    old={'position':[0,0,0],'orientation':[0,0,0,1]};(folder/'d455_eye_to_hand.json').write_text(json.dumps(old))
    he=tmp_path/'he.json';he.write_text(json.dumps(dict(old,position=[.1,0,0])))
    c=Calibration(tmp_path);first=c.apply_calibration('handeye','d455',he)
    tcp=tmp_path/'tcp.json';tcp.write_text(json.dumps({'tcp_offset':[0,0,.15,0,0,0]}));c.apply_calibration('tcp','d455',tcp)
    c.restore(first['backup'])
    assert 'tcp:d455' in json.loads((folder/'manifest.json').read_text())['workbench_selections']
    assert json.loads((folder/'d455_eye_to_hand.json').read_text())==old

def test_old_session_result_does_not_blacklist_new_session():
    t,c=tasks();t.detection('d455',{'motion_session':'old','objects':[{'track_id':1,'base_position_m':{'x':.1,'y':.2,'z':.3}}]})
    cmd=t.dispatch('d455',1,'old');t.detection('d455',{'motion_session':'new'})
    t.result({'cycle_id':cmd['cycle_id'],'ok':True,'failed_track_ids':[1]})
    assert not t.failed
