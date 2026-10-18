"""Execute real GUI methods without constructing Tk or opening a serial device."""
import ast
import json
from pathlib import Path
import queue
import time
from types import SimpleNamespace

import pytest

import sorting_gui_client as gui

SOURCE = Path(gui.__file__)


def app_methods(*names, messagebox=None):
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)
               and n.name == "SortingApp")
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    cls.body = methods
    cls.bases = []
    mod = ast.Module(body=[cls], type_ignores=[])
    namespace = {"time": time, "json": json,
                 "tk": SimpleNamespace(NORMAL="normal", DISABLED="disabled"),
                 "messagebox": messagebox or SimpleNamespace(showwarning=lambda *a: None),
                 "_log": lambda *a: None}
    exec(compile(ast.fix_missing_locations(mod), str(SOURCE), "exec"), namespace)
    return namespace["SortingApp"]()


class Button:
    def __init__(self):
        self.state = "disabled"
    def config(self, **kwargs):
        self.state = kwargs.get("state", self.state)


def test_switching_ignores_old_fruit_frame_and_keeps_buttons_disabled():
    app = app_methods("_update_detection_loop")
    app._mode_switching, app._closing = True, False
    app.latest_detection = None
    app.detection_mode = "fruit"
    app.auto_sort_running, app.dynamic_mode = False, False
    app.detection_queue = queue.Queue()
    app.detection_queue.put(json.dumps({"detected": True, "objects": []}))
    app.det_status_var = SimpleNamespace(set=lambda *a: None)
    app.pick_btn = Button()
    app.pick_two_btn = Button()
    app.pick_graspnet_btn = Button()
    app.after = lambda *a: None
    app._update_detection_loop()
    assert app.latest_detection is None
    assert app.pick_btn.state == app.pick_graspnet_btn.state == "disabled"


@pytest.mark.parametrize("name", [
    "_auto_sort_tick", "_dynamic_tick", "_update_status_loop",
    "start_auto_sort", "_dynamic_start",
])
def test_closing_never_dispatches_or_accesses_task_state(name):
    app = app_methods(name, "_visual_action_allowed")
    app._closing = True
    getattr(app, name)()  # The closing gate must run before touching task state.


def test_late_mode_result_cannot_commit_to_new_request():
    app = app_methods("_on_mode_result")
    app._mode_switching = True
    app._mode_request_id = "new-request"
    app.requested_mode = "blue_block"
    app.detection_mode = "fruit"
    finished = []
    app._finish_mode_switch = lambda: finished.append(True)
    app._on_mode_result("fruit", True, request_id="old-request")
    assert app.detection_mode == "fruit"
    assert finished == []


def test_offline_current_mode_can_be_restarted():
    dialogs = []
    box = SimpleNamespace(askyesno=lambda *a: dialogs.append(True) or False)
    app = app_methods("_switch_to_fruit_mode", messagebox=box)
    app._mode_switching = False
    app.detection_mode = "fruit"
    app._detector_ready = False
    app._mode_online = "offline"
    app._visual_switch_blocked = lambda: False
    app._switch_to_fruit_mode()
    assert dialogs == [True]


def test_blue_mode_cannot_start_fruit_automatic_cycle():
    app = app_methods("start_auto_sort", "_visual_action_allowed")
    app._closing = app._mode_switching = False
    app._detector_ready = True
    app.detection_mode = "blue_block"
    app.auto_sort_running = app.dynamic_mode = False
    app._auto_sort_log = lambda *a: None
    app.start_auto_sort()
    assert app.auto_sort_running is False


def test_initial_pipeline_reuses_blue_instead_of_replacing_it(monkeypatch):
    calls = []
    monkeypatch.setattr(gui, "_online_detector_mode", lambda: "blue_block")
    def ensure(component, **kwargs):
        calls.append((component, kwargs))
        return {"result": "reused", "ready": True}
    monkeypatch.setattr(gui, "ensure_component", ensure)
    monkeypatch.setattr(gui, "_log", lambda *a: None)
    gui.auto_start_pipeline(lambda *a: None, "gui:test")
    detector = next(kw for component, kw in calls if component == "detector")
    assert detector["mode"] == "blue_block"
    assert detector.get("allow_cross_stop") is False


def test_dead_parent_exits_before_connecting_any_device():
    queues = [queue.Queue() for _ in range(9)]
    gui.ros_process_worker(*queues, "gui:test", parent_pid=-1)


def _large_telemetry_worker(telemetry, ack):
    gui._cancel_telemetry_join(telemetry)
    telemetry.put(b'x' * (2 * 1024 * 1024))  # Exceeds the pipe capacity; nobody consumes it.
    ack.put('__SHUTDOWN_ACK__')


def test_full_image_pipe_does_not_block_exit_and_ack_is_delivered():
    import multiprocessing
    ctx = multiprocessing.get_context('spawn')
    telemetry, ack = ctx.Queue(maxsize=1), ctx.Queue()
    child = ctx.Process(target=_large_telemetry_worker, args=(telemetry, ack))
    child.start()
    try:
        assert ack.get(timeout=5) == '__SHUTDOWN_ACK__'
        child.join(timeout=5)
        assert not child.is_alive()
        assert child.exitcode == 0
    finally:
        if child.is_alive():
            child.terminate(); child.join(timeout=3)
        telemetry.close(); ack.close()


@pytest.mark.parametrize('occupied', ['queue_running', '_dispatch_pending'])
def test_mode_switch_refuses_queue_and_unacknowledged_motion(occupied):
    app = app_methods('_visual_switch_blocked')
    app._closing = app._mode_switching = False
    app.auto_sort_running = app.dynamic_mode = app._dynamic_busy = False
    app.current_status, app._conveyor_busy = 'idle', False
    setattr(app, occupied, True)
    assert app._visual_switch_blocked() is True


@pytest.mark.parametrize('method', ['_switch_to_fruit_mode', '_switch_to_blue_block_mode'])
def test_switch_rechecks_busy_after_modal_confirmation(method):
    app = None
    def confirm(*args):
        app.current_status = 'busy'  # A Tk after callback ran inside the dialog.
        return True
    app = app_methods(method, '_visual_switch_blocked',
                      messagebox=SimpleNamespace(askyesno=confirm, showwarning=lambda *a: None))
    app._closing = app._mode_switching = False
    app._detector_ready = False
    app.detection_mode = 'offline'
    app.auto_sort_running = app.dynamic_mode = app._dynamic_busy = False
    app.current_status, app._conveyor_busy = 'idle', False
    getattr(app, method)()
    assert app._mode_switching is False


def test_queue_start_is_blocked_during_mode_switch():
    app = app_methods('toggle_queue_execution', '_visual_action_allowed')
    app.queue_running = False
    app._closing = False
    app._mode_switching = True
    app._auto_sort_log = lambda *a: None
    app.toggle_queue_execution()
    assert app.queue_running is False


@pytest.mark.parametrize('switching,ready,pending', [(True, True, False), (False, False, False), (False, True, True)])
def test_queue_dispatch_pauses_for_switch_offline_or_unacknowledged_command(switching, ready, pending):
    app = app_methods('_update_status_loop')
    app._closing = False
    app.status_queue = queue.Queue()
    app.current_status = 'idle'
    app.status_var = SimpleNamespace(set=lambda *a: None)
    app.status_label = SimpleNamespace(configure=lambda *a, **kw: None)
    app.auto_sort_running = app.dynamic_mode = False
    app._mode_switching, app._detector_ready = switching, ready
    app._dispatch_pending, app.queue_running = pending, True
    app.task_queue = [{'cmd': 'sort'}]
    app.last_dispatch_time = 0
    app._send_motion_command = lambda cmd: pytest.fail('unsafe dispatch')
    app.after = lambda *a: None
    app._update_status_loop()
    assert len(app.task_queue) == 1


def test_motion_pending_covers_acknowledgement_gap():
    app = app_methods('_send_motion_command', '_update_status_loop')
    app.cmd_queue = queue.Queue()
    app._send_motion_command({'cmd': 'sort_graspnet'})
    assert app._dispatch_pending is True
    assert app.cmd_queue.get_nowait()['cmd'] == 'sort_graspnet'
    app._closing = False
    app.status_queue = queue.Queue()
    app.status_queue.put('busy')
    app.status_var = SimpleNamespace(set=lambda *a: None)
    app.status_label = SimpleNamespace(configure=lambda *a, **kw: None)
    app.after = lambda *a: None
    app._update_status_loop()
    assert app.current_status == 'busy'
    assert app._dispatch_pending is False


def test_standby_does_not_wait_for_nonexistent_busy_ack():
    app = app_methods('send_observe')
    app.cmd_queue = queue.Queue()
    app._dispatch_pending = False
    app.clear_queue = lambda: None
    app.send_observe()
    assert app.cmd_queue.get_nowait()['cmd'] == 'standby'
    assert app._dispatch_pending is False


def test_mode_replacement_launch_failure_is_reported(monkeypatch):
    calls = []
    def ensure(*args, **kwargs):
        calls.append(True)
        if len(calls) == 1:
            return {'result': 'mode_conflict'}
        raise OSError('launch failed')
    monkeypatch.setattr(gui._runtime, 'ensure_background', ensure)
    monkeypatch.setattr(gui, 'stop_component', lambda *a, **kw: {'result': 'stopped'})
    result = gui.ensure_component('detector', session_id='gui:test', allow_cross_stop=True)
    assert result['ready'] is False
    assert result['detail'] == 'launch failed'


@pytest.mark.parametrize('cleanup_fails', [False, True])
def test_worker_cleanup_is_idempotent_and_disconnects_on_failure(cleanup_fails):
    import threading
    tree = ast.parse(SOURCE.read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_shutdown_worker')
    calls, ack = [], queue.Queue()
    def cleanup(session):
        calls.append(('cleanup', session))
        if cleanup_fails:
            raise OSError('dummy cleanup failure')
    namespace = {'closing': threading.Event(), 'pipeline_lock': threading.Lock(),
                 'shutdown_lock': threading.Lock(), 'shutdown_done': threading.Event(),
                 'cleanup_session': cleanup, 'session_id': 'gui:test', 'shutdown_q': ack,
                 '_conv': SimpleNamespace(disconnect=lambda: calls.append(('disconnect',))),
                 'rclpy': SimpleNamespace(shutdown=lambda: calls.append(('ros_shutdown',))),
                 '_log': lambda *a: None}
    module = ast.Module(body=[method], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), 'exec'), namespace)
    namespace['_shutdown_worker']()
    namespace['_shutdown_worker']()
    assert calls == [('cleanup', 'gui:test'), ('disconnect',), ('ros_shutdown',)]
    assert ack.get_nowait() == '__SHUTDOWN_ACK__'
    assert ack.empty()
    assert namespace['closing'].is_set()
