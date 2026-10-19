import copy,json,zipfile
import numpy as np
import pytest
from workbench.placement import PlacementStore, resolve_placement
from workbench.capture import Capture
from workbench.logs import Logs, ReconnectNotice

def test_placement_snapshot_mapping_and_waypoints(tmp_path):
    store=PlacementStore(tmp_path/'placements.json'); data=store.get()
    plan=copy.deepcopy(data['plans'][0]); plan.update(id='new',name='新终点',via_joints_rad=[[0.1]*6]*5)
    data['plans'].append(plan); data['selection'].update(mode='class',class_to_plan_id={'apple':'new'})
    store.save(data)
    snap=resolve_placement('apple','dabai',store.get())
    data['plans'][1]['name']='改名';store.save(data)
    assert snap['name']=='新终点' and len(snap['via_joints_rad'])==5
    assert resolve_placement('orange','d455',data)['id']=='legacy-default'
    data['plans'][1]['via_joints_rad'].append([0]*6)
    with pytest.raises(ValueError):store.save(data)
    assert len(store.get()['plans'][1]['via_joints_rad'])==5

def test_capture_fresh_pixels_and_export(tmp_path):
    cap=Capture(tmp_path); pixels=np.full((3,4,3),27,np.uint8)
    cap.update('d455',pixels,1.); cap.update('dabai',pixels,2.)
    result=cap.once(['d455','dabai']); assert cap.count==2 and result['state']=='saved'
    manifest=json.loads((cap.session/'session.json').read_text())
    assert manifest['state']=='saved' and manifest['requested_cameras']==['d455','dabai']
    assert manifest['saved_cameras']==['d455','dabai'] and not manifest['errors']
    cap.once(['d455','dabai']); assert cap.count==2 # no duplicate old frame
    cap.stop();cap.update('d455',pixels,3.);assert cap.count==2
    path=cap.export(); z=zipfile.ZipFile(path)
    assert len([x for x in z.namelist() if x.endswith('.png')])==2
    import cv2
    assert (cv2.imread(str(next(cap.session.rglob('*.png'))))==27).all()

def test_dual_capture_timeout_never_saves_one_camera_only(tmp_path):
    import threading,time
    cap=Capture(tmp_path);pixels=np.zeros((2,2,3),np.uint8);cap.update('dabai',pixels,1.)
    result=[];thread=threading.Thread(target=lambda:result.append(cap.next_frame(['d455','dabai'],timeout=.3)))
    thread.start();time.sleep(.03);cap.update('dabai',pixels+1,2.);thread.join(1)
    assert result[0]['state']=='error' and result[0]['saved_count']==0 and cap.count==0
    assert 'd455' in result[0]['errors'] and 'dabai' not in result[0]['errors']
    assert not list(cap.session.rglob('*.png'))
    manifest=json.loads((cap.session/'session.json').read_text())
    assert manifest['state']=='error' and manifest['requested_cameras']==['d455','dabai']
    assert manifest['saved_cameras']==[] and 'd455' in manifest['errors']

def test_single_capture_missing_frame_is_explicit_failure(tmp_path):
    cap=Capture(tmp_path);result=cap.once(['d455'])
    assert result['state']=='error' and result['saved_count']==0 and result['errors']['d455']
    manifest=json.loads((cap.session/'session.json').read_text())
    assert manifest['state']=='error' and manifest['requested_cameras']==['d455']
    assert not list(cap.session.rglob('*.png'))

def test_timed_dual_capture_skips_attempts_when_one_camera_is_stale(tmp_path):
    import time
    cap=Capture(tmp_path);pixels=np.zeros((2,2,3),np.uint8);cap.update('dabai',pixels,1.)
    cap.start(['d455','dabai'],interval=.08)
    try:
        for stamp in range(2,6):
            time.sleep(.03);cap.update('dabai',pixels+stamp,stamp)
    finally:
        cap.stop()
    assert cap.count==0 and not list(cap.session.rglob('*.png'))
    manifest=json.loads((cap.session/'session.json').read_text())
    assert manifest['state']=='error' and manifest['requested_cameras']==['d455','dabai']

def test_warning_current_instance_only_and_log_export(tmp_path):
    notice=ReconnectNotice();notice.set_instance('new',10.)
    warning='Agx_arm is not connected, cannot control'
    assert not notice.feed(warning,'old',11.)
    assert not notice.feed(warning,'new',9.)
    assert notice.feed(warning,'new',11.)['count']==1
    assert notice.feed(warning,'new',12.)['count']==2
    logs=Logs(tmp_path);logs.add('T1','中文\nTraceback:\n  File x\nError')
    assert len(logs.query(keyword='中文'))==1
    out=logs.export();assert 'Traceback' in out.read_text()

def test_capture_request_waits_for_next_frame(tmp_path):
    import threading,time
    cap=Capture(tmp_path);cap.update('d455',np.zeros((2,2,3),np.uint8),1.)
    result=[];thread=threading.Thread(target=lambda:result.append(cap.next_frame(['d455'],timeout=.5)))
    thread.start();time.sleep(.03);assert cap.count==0
    cap.update('d455',np.full((2,2,3),55,np.uint8),2.);thread.join(1)
    assert cap.count==1 and result[0]['count']==1

def test_invalid_external_edit_keeps_last_valid_snapshot(tmp_path):
    store=PlacementStore(tmp_path/'placements.json');data=store.get();data['plans'][0]['name']='已示教'
    store.save(data);store.path.write_text('{ broken')
    assert store.get()['plans'][0]['name']=='已示教'
    assert store.error and store.path.read_text()=='{ broken'
