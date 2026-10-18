import json,queue,types
from workbench.action_control import WorkbenchControl

class Fake(WorkbenchControl):
    def __init__(self):
        self.cmd_queue=queue.Queue();self.cmd_queue.put('old');self._active_goal_handle=None
        self._emergency_flag=True;self.is_busy=True;self._latest_detection=None
    def get_logger(self):return types.SimpleNamespace(error=lambda x:None)

def test_recovery_cancel_only_and_pause():
    n=Fake();n._workbench_control(types.SimpleNamespace(data=json.dumps({'op':'recovery_prepare'})))
    assert n._workbench_recover and n._pause_after_cycle and n.cmd_queue.empty()
    n._workbench_control(types.SimpleNamespace(data=json.dumps({'op':'recovery_complete'})))
    assert not n._workbench_recover and n._pause_after_cycle and not n._emergency_flag

def test_camera_cache_does_not_mix_equal_ids():
    n=Fake();n._active_camera='d455';n._dynamic_tracker=None
    for camera in ('d455','dabai'):
        n.cache_detection(camera,{'camera':camera,'motion_session':camera,'objects':[{'track_id':1}]})
    assert n._latest_detection['camera']=='d455'
    assert n._detections_by_camera['dabai']['motion_session']=='dabai'

def test_recovery_abandons_wait_and_cancels_late_goal():
    n=Fake();n._workbench_recover=True
    class Future:
        def done(self):return False
        def add_done_callback(self,cb):self.cb=cb
        def cancelled(self):return False
        def result(self):return types.SimpleNamespace(cancel_goal_async=lambda:calls.append('cancel'))
    calls=[];future=Future()
    assert n._wait_control_future(future) is False
    future.cb(future);assert calls==['cancel']
