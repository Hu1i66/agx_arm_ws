import io,json,os,pathlib,signal,time,types
import pytest
from workbench.tasks import Tasks
from workbench.components import Components
from runtime_process_manager import ProcessManager,_alive

def test_recovery_cancel_is_independent_of_disconnected_belt():
    calls=[]
    def belt(op):raise RuntimeError('USB serial disconnected')
    t=Tasks(lambda *x:None,calls.append,belt,lambda *x:None)
    result=t.recovery('recovery_prepare')
    assert calls==['recovery_prepare'] and result['warnings'] and t.pause

def test_sudo_password_uses_stdin_not_argv_or_log(tmp_path):
    secret='test-value-only';(tmp_path/'.runtime').mkdir();credentials=tmp_path/'.runtime/workbench-credentials.json';credentials.write_text(json.dumps({'sudo_password':secret}));credentials.chmod(0o600)
    fake=tmp_path/'sudo';captured=tmp_path/'received';fake.write_text('#!/usr/bin/python3\nimport sys,pathlib\npathlib.Path('+repr(str(captured))+').write_bytes(sys.stdin.buffer.readline())\nprint("authorized")\n');fake.chmod(0o755)
    events=[];env=dict(os.environ,PATH=str(tmp_path)+':'+os.environ['PATH']);c=Components(tmp_path,launcher=types.SimpleNamespace(build_env=lambda c:env),emit=lambda *x:events.append(x))
    assert c._run(['sudo','-S','-p','','true'])['returncode']==0
    assert captured.read_bytes()==(secret+'\n').encode()
    assert secret not in json.dumps(events)

def test_stop_all_attempts_all_components_without_homing(tmp_path):
    class FakeLauncher:
        def stop_component(self,name,**kw):
            calls.append((name,kw.get('camera')))
            if (name,kw.get('camera'))==('camera','dabai'):raise RuntimeError('camera exit error')
            return {'result':'stopped'}
    calls=[];commands=[];control=[]
    c=Components(tmp_path,launcher=FakeLauncher(),control=control.append,run=lambda argv:commands.append(argv) or {'returncode':0},conveyor=lambda op:calls.append(('belt',op)) or {})
    result=c.close_all_connections()
    assert control[0]=='recovery_prepare' and ('action','d455') in calls and ('arm','d455') in calls
    assert ('camera','d455') in calls and ('detector','dual') in calls and ('belt','disconnect') in calls
    assert 'workbench' not in [x[0] for x in calls]
    assert commands and 'down' in ' '.join(commands[-1]) and any('error' in r for r in result)

def test_owned_orphan_driver_is_reaped_without_touching_other_group(tmp_path):
    manager=ProcessManager(tmp_path)
    env=dict(os.environ,AGX_ARM_WS=str(tmp_path))
    rec=manager.spawn('arm',['/bin/bash','-c',"trap '' INT; sleep 30 & exit 0"],initiator='workbench',env=env)
    other=manager.spawn('other',['/bin/sleep','30'],initiator='manual')
    time.sleep(.2)
    try:
        result=manager.stop_owned_group('arm',timeout_sigint=.1,timeout_sigterm=.1)
        assert result['result'] in ('stopped','already_exited')
        assert manager.get_record('arm') is None and _alive(other['pid'])
    finally:
        for r in (rec,other):
            try:os.killpg(r['pgid'],signal.SIGTERM)
            except ProcessLookupError:pass


def test_log_burst_is_streamed_in_bounded_chunks_without_dropping_lines(tmp_path):
    from workbench.logs import Logs
    logs=Logs(tmp_path/'logs');path=tmp_path/'driver.log'
    expected=['相机 '+str(i)+' x'*20 for i in range(5000)]
    path.write_text('\n'.join(expected)+'\n')
    actual=[];first=logs.tail(path)
    assert len(first)<len(expected)
    actual.extend(first)
    while logs.offsets[str(path)][0]<path.stat().st_size:actual.extend(logs.tail(path))
    assert actual==expected


def test_stop_all_tool_failure_does_not_block_component_shutdown():
    from workbench.service import Workbench
    calls=[]
    def broken():raise RuntimeError('tool exited')
    obj=types.SimpleNamespace(capture=types.SimpleNamespace(stop=broken),calibration=types.SimpleNamespace(sessions={'one':{'proc':types.SimpleNamespace(stop=broken)}}),components=types.SimpleNamespace(close_all_connections=lambda:calls.append('shutdown') or []))
    result=Workbench.dispatch(obj,'system.stop_all',{})
    assert calls==['shutdown'] and len(result)==2 and all('error' in row for row in result)


def test_workbench_arm_uses_embedded_rviz_only(tmp_path):
    calls=[]
    c=Components(tmp_path,launcher=types.SimpleNamespace(ensure_background=lambda *a,**kw:calls.append(kw) or {'result':'spawned'}))
    c.start('arm')
    assert calls[0]['ros_args']==['use_rviz:=false']


def test_explicit_activation_click_is_not_reused_after_usb_reset():
    from workbench.service import Workbench
    calls=[];obj=types.SimpleNamespace(components=types.SimpleNamespace(start=lambda *a,**kw:calls.append((a,kw))))
    Workbench.dispatch(obj,'component.start',{'component':'can'})
    assert calls[0][1]['force'] is True


def test_capture_status_tracks_real_worker_state(tmp_path):
    from workbench.capture import Capture
    capture=Capture(tmp_path)
    assert capture.status()['running'] is False
    capture.start(['d455'],interval=.1)
    assert capture.status()['running'] is True
    capture.stop()
    assert capture.status()['running'] is False
