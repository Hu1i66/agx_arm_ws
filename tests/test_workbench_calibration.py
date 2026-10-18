import json,sys,time
from workbench.calibration import PtySession, Calibration

def test_pty_lines_and_output(tmp_path):
    script=tmp_path/'interactive.py'
    script.write_text("for i in range(4):\n x=input('ready:');print('got:'+repr(x),flush=True)\n")
    output=[];p=PtySession([sys.executable,str(script)],tmp_path,dict(__import__('os').environ),output.append)
    for text in ('\n','d\n','q\n','c\n'):
        p.write(text);time.sleep(.06)
    p.proc.wait(timeout=3);p.thread.join(timeout=2)
    joined=''.join(output)
    assert "got:''" in joined and "got:'d'" in joined and "got:'q'" in joined and "got:'c'" in joined

def test_apply_preserves_quality_and_source(tmp_path):
    folder=tmp_path/'config/calibration';folder.mkdir(parents=True)
    (folder/'manifest.json').write_text(json.dumps({'cameras':{'d455':{'parent':'base_link','child':'camera_color_optical_frame','mode':'eye_to_hand'}}}))
    candidate=tmp_path/'candidate.json';data={'position':[.1,.2,.3],'orientation':[0,0,0,1],'metadata':{'quality_status':'failed','camera_id':'d455'}}
    candidate.write_text(json.dumps(data));raw=candidate.read_bytes()
    c=Calibration(tmp_path);r=c.apply_calibration('handeye','d455',str(candidate))
    assert r['state']=='pending_restart' and candidate.read_bytes()==raw
    active=json.loads((folder/'d455_eye_to_hand.json').read_text())
    assert active['metadata']['quality_status']=='failed'
    assert 'scene_verified' not in active

def test_calibration_rejects_wrong_coordinate_units_and_state_needs_evidence(tmp_path):
    import pytest
    folder=tmp_path/'config/calibration';folder.mkdir(parents=True)
    (folder/'manifest.json').write_text('{}')
    path=tmp_path/'new.json';path.write_text(json.dumps({'position':[0,0,0],'orientation':[0,0,0,1],'metadata':{'camera_id':'d455','mode':'eye_in_hand','units':'mm'}}))
    c=Calibration(tmp_path)
    with pytest.raises(ValueError):c.apply_calibration('handeye','d455',path)
    path.write_text(json.dumps({'position':[.1,.2,.3],'orientation':[0,0,0,1]}))
    c.apply_calibration('handeye','d455',path)
    assert c.selection_state({})['handeye:d455']['state']=='pending_restart'
    sample={'stamp':time.time(),'position':[.1,.2,.3],'orientation':[0,0,0,-1],'tf_position':[.1,.2,.3],'tf_orientation':[0,0,0,1]}
    assert c.selection_state({'d455':sample})['handeye:d455']['state']=='effective'
    sample['stamp']=time.time()-10
    assert c.selection_state({'d455':sample})['handeye:d455']['state']=='pending_restart'
    assert Calibration(tmp_path).selection_state({})['handeye:d455']['backup']
