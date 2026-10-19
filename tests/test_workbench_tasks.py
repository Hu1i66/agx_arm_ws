import json,time,threading
from workbench.tasks import Tasks

def make():
    calls=[];t=Tasks(lambda *x:calls.append(x),lambda x:calls.append(x),lambda x:calls.append(x),lambda *x:None)
    return t,calls

def test_selected_target_preserves_source_session():
    t,calls=make()
    for c in ('d455','dabai'):
        t.detection(c,{'camera':c,'header_stamp':time.time(),'motion_session':c,'objects':[{'track_id':1,'object_name':'apple','base_position_m':{'x':.1,'y':.2,'z':.3}}]})
    cmd=t.start('dabai-static',track_id=1,session='dabai')
    assert cmd['camera_id']=='dabai' and cmd['motion_session']=='dabai'
    t.request_pause_after_cycle();assert calls[-1]=='pause_after_cycle'
    assert t.phase=='pause_pending'
    t.result({'cycle_id':cmd['cycle_id'],'cycle_success':1});assert calls[-1]=='stop' and t.phase=='paused'


def test_reacquires_target_when_track_id_changes_at_same_position():
    t,calls=make()
    old={'track_id':4,'object_name':'green_pear','base_position_m':{'x':.33,'y':.05,'z':.01}}
    t.detection('dabai',{'camera':'dabai','header_stamp':time.time(),'motion_session':'dabai',
        'objects':[dict(old,track_id=9)]})
    cmd=t.start('dabai-static',track_id=4,session='dabai',target_snapshot=old)
    assert cmd['target_track_id']==9 and cmd['pick']['x']==.33


def test_dynamic_boundary_values_are_loaded_into_tick():
    t,_=make()
    t.set_dynamic_lines({'trigger_line_u':170,'stop_line_u':470,'abandon_line_u':520})
    assert t.dynamic_lines=={
        'trigger_line_u':170,'stop_line_u':470,'abandon_line_u':520,
    }

def test_ui_close_idle_pauses_without_stopping_processes():
    t,calls=make();t.request_pause_after_cycle()
    assert calls==['pause_after_cycle','stop']

def test_queued_tasks_pause_between_objects_and_recovery_clears():
    t,calls=make();t.queued=[dict(cmd='sort',pick={'x':.1,'y':.2,'z':.3})]*2
    t.pause=False;t.queue_running=True;t.tick();assert len(t.queued)==1
    t.request_pause_after_cycle();t.result({'cycle_id':t.inflight,'cycle_success':1});t.tick()
    assert len(t.queued)==1
    t.recovery('recovery_prepare');assert t.queued==[] and not t.queue_running
