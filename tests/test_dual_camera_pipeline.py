"""Execute production snapshot/protocol methods offline; never construct ROS nodes."""
import ast
from collections import deque
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def methods(file, cls, names, **globals_):
    tree = ast.parse((ROOT / file).read_text())
    source = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
    source.bases = []
    source.body = [m for m in source.body if isinstance(m, ast.FunctionDef) and m.name in names]
    from runtime_geometry import assert_visual_target_fresh
    env = dict(np=np, time=time, json=json, assert_visual_target_fresh=assert_visual_target_fresh, **globals_)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[source], type_ignores=[])), str(ROOT/file), 'exec'), env)
    return env[cls]()


def snapshot_node():
    n = methods('graspnet_service_node.py', 'GraspNetServiceNode', ['_snapshot', '_req_cb'])
    n._lock = threading.Lock()
    ci = NS(height=2, width=3, k=[100.,0,1.,0,100.,1.,0,0,1])
    row = lambda arr: deque([(10.,arr,time.monotonic())])
    n._frames = {camera:dict(color=row(np.zeros((2,3,3))), depth=row(np.ones((2,3))), info=(ci,time.monotonic())) for camera in ('d455','dabai')}
    n.get_clock = lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000))
    n.geometry = NS(camera=lambda camera,stamp:np.eye(4)*(1 if camera=='d455' else 2))
    n.get_logger = lambda:NS(info=lambda *a:None,error=lambda *a:None)
    n.default_camera = 'd455'
    return n


def test_snapshot_camera_identity_and_copies():
    n = snapshot_node()
    depth,color,ci,T,stamp=n._snapshot('dabai')
    assert stamp==10 and T[0,0]==2
    depth[:]=9
    assert n._frames['dabai']['depth'][0][1][0,0]==1
    assert n._snapshot('d455')[3][0,0]==1


@pytest.mark.parametrize('bad', ['stale','unsynchronized','wrong_size','bad_intrinsics','missing_depth'])
def test_bad_snapshot_rejected(bad):
    n=snapshot_node(); frames=n._frames['dabai']
    if bad=='stale':frames['info']=(frames['info'][0],time.monotonic()-2)
    if bad=='unsynchronized':frames['depth']=deque([(9.,np.ones((2,3)),time.monotonic())])
    if bad=='wrong_size':frames['depth']=deque([(10.,np.ones((3,3)),time.monotonic())])
    if bad=='bad_intrinsics':frames['info'][0].k[0]=float('nan')
    if bad=='missing_depth':frames['depth'].clear()
    with pytest.raises(ValueError):n._snapshot('dabai')


@pytest.mark.parametrize('change', ['inference','missing_pick','unknown_camera','nan_pick'])
def test_error_result_keeps_request_camera(change):
    n=snapshot_node();results=[];n._publish_result=results.append
    def fail(*args):raise ValueError('inference_error')
    n._handle_get_grasp=fail
    req=dict(cmd='get_grasp',pick_semantics='estimated_centroid',camera_id='dabai',request_id='new',pick_x=.1,pick_y=.2,pick_z=.3)
    if change=='missing_pick':del req['pick_x']
    if change=='unknown_camera':req['camera_id']='bad'
    if change=='nan_pick':req['pick_z']=float('nan')
    n._req_cb(NS(data=json.dumps(req)))
    assert results[0]['success'] is False
    assert results[0]['camera_id']==req['camera_id'] and results[0]['request_id']=='new'


def test_late_grasp_result_is_ignored():
    n=methods('auto_sorting_action.py','MoveItActionClient',['_graspnet_result_cb'])
    n._pending_grasp_request=dict(camera_id='dabai',request_id='new')
    n._graspnet_event=threading.Event()
    n._graspnet_result_cb(NS(data=json.dumps(dict(camera_id='d455',request_id='new',success=True))))
    assert not n._graspnet_event.is_set()
    n._graspnet_result_cb(NS(data=json.dumps(dict(camera_id='dabai',request_id='old',success=True))))
    assert not n._graspnet_event.is_set()
    n._graspnet_result_cb(NS(data=json.dumps(dict(camera_id='dabai',request_id='new',success=True))))
    assert n._graspnet_event.is_set()


def test_conveyor_collision_cannot_be_disabled():
    n=methods('auto_sorting_action.py','MoveItActionClient',['_set_conveyor_collision_allowed'])
    warnings=[];n.get_logger=lambda:NS(warning=warnings.append)
    n._set_conveyor_collision_allowed(allowed=True)
    assert warnings  # no scene publisher/client needed, no ACM relaxation occurs


def test_dabai_automatic_is_rejected_before_dispatch():
    import queue
    n=methods('auto_sorting_action.py','MoveItActionClient',['cmd_callback'])
    n.cmd_queue=queue.Queue();n.get_logger=lambda:NS(error=lambda *a:None)
    n.cmd_callback(NS(data=json.dumps(dict(cmd='sort_dynamic',camera_id='dabai'))))
    assert n.cmd_queue.empty()


def test_launcher_dual_camera_and_active_tcp(monkeypatch):
    import runtime_launcher as launch
    from runtime_geometry import load_tcp
    monkeypatch.setenv('AGX_ARM_WS',str(ROOT))
    cmd=launch.build_command('arm')
    assert json.loads(next(a.split(':=',1)[1] for a in cmd if a.startswith('tcp_offset:=')))==load_tcp()
    for camera in ('d455','dabai'):
        assert 'camera:='+camera in launch.build_command('camera_tf',camera=camera)
        assert 'publish_tf:=false' in launch.build_command('camera',camera=camera)
    detector=launch.build_command('detector',camera='dabai')
    assert 'dabai.enabled:=true' in detector and 'd455.enabled:=false' in detector


def test_dabai_feedback_interpolation_undoes_tcp_once(monkeypatch):
    import sys
    from scipy.spatial.transform import Rotation
    from camera_geometry import CameraGeometry
    from runtime_geometry import transform
    S=transform([.01,-.02,.15],Rotation.from_euler('xyz',[.1,.2,.3]).as_quat())
    F=transform([.4,.1,.3],Rotation.from_euler('xyz',[.2,-.1,.5]).as_quat())
    C=transform([-.067,.004,.046],[0,0,0,1])
    n=CameraGeometry.__new__(CameraGeometry)
    n.S=S;n.calibrations={'dabai':({'parent':'link6'},C)}
    n.lock=threading.Lock();n.poses=deque([(9.99,F@S,time.monotonic()),(10.01,F@S,time.monotonic())])
    n._driver_values=[NS(type=8,double_array_value=[.01,-.02,.15,.1,.2,.3])]
    n._driver_checked_at=time.monotonic()
    def missing(*args):raise LookupError('TF trails image')
    n.buffer=NS(lookup_transform=missing)
    monkeypatch.setitem(sys.modules,'rclpy',NS(time=NS(Time=lambda **kw:None)))
    monkeypatch.setitem(sys.modules,'rclpy.time',NS(Time=lambda **kw:None))
    np.testing.assert_allclose(n.camera('dabai',10),F@C,atol=1e-12)
    # At the observation pose, a near, stationary pose may bridge a short missing bracket.
    n.poses=deque([(9.95,F@S,time.monotonic()),(9.90,F@S,time.monotonic())])
    np.testing.assert_allclose(n.camera('dabai',10),F@C,atol=1e-12)
    n._driver_checked_at-=2
    with pytest.raises(ValueError,match='Fresh driver'):n.camera('dabai',10)
    n._driver_checked_at=time.monotonic()
    with pytest.raises(ValueError,match='not bracketed'):n.camera('dabai',11)


def test_gui_dabai_manual_grasp_does_not_require_conveyor():
    # SortingApp is deliberately nested inside main; reuse the existing AST fixture helper.
    from test_gui_review_regressions import app_methods
    n=app_methods('_visual_action_allowed')
    n._closing=n._mode_switching=False;n._detector_ready=True
    n.active_camera='dabai';n.detection_mode='fruit'
    n.latest_detection={'header_stamp':time.time(),'camera':'dabai'}
    n._auto_sort_log=lambda *a:None
    for state in (True, False, None):
        n.conveyor_state=state
        assert n._visual_action_allowed() is True
    assert n._visual_action_allowed(automatic=True) is False


def test_gui_expired_detection_cannot_dispatch():
    from test_gui_review_regressions import app_methods
    n=app_methods('_visual_action_allowed')
    n._closing=n._mode_switching=False;n._detector_ready=True
    n.active_camera='d455';n.detection_mode='fruit';n._auto_sort_log=lambda *a:None
    n.latest_detection={'header_stamp':time.time()-5,'camera':'d455','detected':True}
    assert n._visual_action_allowed() is False


def test_grasp_centroid_recomputed_from_same_snapshot():
    from scipy.spatial.transform import Rotation
    from runtime_geometry import transform,object_centroid_from_rgbd
    n=methods('graspnet_service_node.py','GraspNetServiceNode',['_centroid_snapshot'],object_centroid_from_rgbd=object_centroid_from_rgbd)
    n._lock=threading.Lock()
    n._detections={'dabai':dict(header_stamp=10.,objects=[dict(track_id=7,bbox_pixel=dict(x1=35,y1=26,x2=65,y2=54))])}
    depth=np.full((80,100),1000,dtype=np.uint16);depth[28:52,37:63]=940
    C=transform([0,0,1],Rotation.from_euler('x',np.pi).as_quat())
    ci=NS(k=[200,0,50,0,200,40,0,0,1])
    centre=n._centroid_snapshot(dict(target_track_id=7),'dabai',depth,ci,C,10.)
    assert centre[2]==pytest.approx(.03,abs=.001)
    n._detections['dabai']['header_stamp']=8.
    with pytest.raises(ValueError):n._centroid_snapshot(dict(target_track_id=7),'dabai',depth,ci,C,10.)


def test_action_rejects_expired_visual_command():
    import queue
    n=methods('auto_sorting_action.py','MoveItActionClient',['cmd_callback'])
    n.cmd_queue=queue.Queue();n.get_logger=lambda:NS(error=lambda *a:None)
    cmd=dict(cmd='sort_graspnet',camera_id='dabai',vision_target=True,detection_stamp=time.time()-5)
    n.cmd_callback(NS(data=json.dumps(cmd)))
    assert n.cmd_queue.empty()
    cmd['detection_stamp']=time.time()
    n.cmd_callback(NS(data=json.dumps(cmd)))
    assert not n.cmd_queue.empty()


def test_gui_tracks_selected_object_not_primary_after_confirmation():
    from test_gui_review_regressions import app_methods
    n=app_methods('_tag_visual_command')
    n.latest_detection=dict(header_stamp=time.time(),track_id=1,objects=[
        dict(track_id=1,base_position_m=dict(x=.1,y=.2,z=.03)),
        dict(track_id=7,base_position_m=dict(x=.4,y=.1,z=.04))])
    tagged=n._tag_visual_command(dict(cmd='sort_graspnet',target_track_id=7,pick=dict(x=.3,y=.1,z=.04)))
    assert tagged['target_track_id']==7 and tagged['pick']['x']==.4
    assert tagged['vision_target'] is True


def test_action_rechecks_visual_age_when_dequeued():
    from runtime_geometry import assert_visual_target_fresh
    command=dict(vision_target=True,detection_stamp=100.)
    assert_visual_target_fresh(command,now=100.)
    with pytest.raises(ValueError):assert_visual_target_fresh(command,now=102.)
    tree=ast.parse((ROOT/'auto_sorting_action.py').read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    assert any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='assert_visual_target_fresh' for n in ast.walk(main))


def test_gui_can_start_with_only_dabai(monkeypatch):
    import sorting_gui_client as gui
    import runtime_launcher as launch
    calls=[]
    monkeypatch.setattr(gui,'ensure_component',lambda component,**kw:calls.append((component,kw)) or dict(ready=True,result='reused'))
    monkeypatch.setattr(gui,'_online_detector_mode',lambda:'fruit')
    monkeypatch.setattr(gui,'_log',lambda *a:None)
    gui.auto_start_pipeline(lambda *a:None,'test',camera='dabai')
    assert [c for c,_ in calls]==['camera','camera_tf','detector']
    assert all(kw['camera']=='dabai' for _,kw in calls)
    assert launch.build_command('gui',camera='dabai')[-2:]==['--camera','dabai']


def test_worker_camera_state_matches_initial_camera_argument():
    tree=ast.parse((ROOT/'sorting_gui_client.py').read_text())
    worker=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='ros_process_worker')
    assignment=next(n for n in worker.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='camera_state' for t in n.targets))
    position=next(i for i,key in enumerate(assignment.value.keys) if key.value=='active')
    assert isinstance(assignment.value.values[position],ast.Name)
    assert assignment.value.values[position].id=='initial_camera'
