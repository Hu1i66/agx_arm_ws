from pathlib import Path
from workbench.components import Components

class Launcher:
    def __init__(self): self.calls=[]; self.running=set()
    def ensure_background(self, name, **kw):
        key=(name,kw.get('camera','d455')); self.calls.append(('start',key))
        reused=key in self.running; self.running.add(key)
        if key==('camera','dabai'): raise RuntimeError('no camera')
        return {'result':'reused' if reused else 'spawned'}
    def stop_component(self,name,**kw): self.calls.append(('stop',name)); return {'result':'stopped'}

def test_partial_start_continues_and_reuses(tmp_path):
    l=Launcher(); c=Components(tmp_path,launcher=l,run=lambda x: {'returncode':0},control=lambda x:None)
    c.start('arm'); assert c.start('arm')['result']=='reused'
    results=c.ensure_missing_components()
    assert any(x.get('error') for x in results)
    assert ('start',('action','d455')) in l.calls

def test_recovery_forces_sequence_without_camera_or_resume(tmp_path):
    l=Launcher(); calls=[]
    c=Components(tmp_path,launcher=l,run=lambda argv: calls.append(argv) or {'returncode':0},control=lambda op:calls.append(op))
    result=c.recover_arm_connection()
    assert calls[0]=='recovery_prepare' and calls[-1]=='recovery_complete'
    assert len([x for x in calls if isinstance(x,list)])==2
    assert l.calls==[('stop','arm'),('start',('arm','d455'))]
    assert result['state']=='awaiting_feedback'

def test_failed_can_does_not_enable_or_restart(tmp_path):
    l=Launcher(); calls=[]
    c=Components(tmp_path,launcher=l,run=lambda x: {'returncode':1,'output':'failure'},control=calls.append)
    import pytest
    with pytest.raises(RuntimeError): c.recover_arm_connection()
    assert l.calls==[('stop','arm')] and calls==['recovery_prepare']
