"""GUI 生命周期测试（计划 4.2）：单实例、自动管线顺序、关闭 ACK、session 清理差异、watchdog。

GUI 是单文件 Tk 应用，无法在无显示环境实例化，故：
  - 单实例锁与 session 清理差异用真实原语（fcntl.flock / ProcessManager）验证；
  - 线程/关闭/watchdog 的结构性要求用源码断言回归保护。
"""
import fcntl
import os
import pathlib
import sys
import time

import pytest

WS = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WS))
from runtime_process_manager import ProcessManager  # noqa: E402

GUI_SRC = (WS / "sorting_gui_client.py").read_text(encoding="utf-8")


# ---------------- 单实例锁（真实 flock 原语） ----------------
def test_single_instance_lock_exclusive(tmp_path):
    lock_path = tmp_path / "gui.lock"
    fd1 = open(lock_path, "w")
    fcntl.flock(fd1, fcntl.LOCK_EX | fcntl.LOCK_NB)   # 第一个实例
    fd2 = open(lock_path, "w")
    with pytest.raises(OSError):
        fcntl.flock(fd2, fcntl.LOCK_EX | fcntl.LOCK_NB)  # 第二个实例被拒
    fd1.close()
    fcntl.flock(fd2, fcntl.LOCK_EX | fcntl.LOCK_NB)   # 释放后可获取
    fd2.close()


def test_gui_acquires_lock_before_spawn():
    i_lock = GUI_SRC.index("fcntl.flock(_lock_fd")
    i_spawn = GUI_SRC.index("ros_proc.start()")
    assert i_lock < i_spawn, "单实例锁必须在 spawn worker 之前取得"


# ---------------- session 清理差异（真实 ProcessManager） ----------------
@pytest.fixture()
def mgr(tmp_path):
    return ProcessManager(tmp_path, state_dir_name=".runtime")


def test_cleanup_session_only_stops_owned(mgr):
    """GUI 退出只清理本 session 创建的组件；复用的/手工组件保留。"""
    a = mgr.spawn("camera:d455", ["/bin/sleep", "60"], initiator="gui",
                  owner_session="gui:AAA")
    b = mgr.spawn("detector:d455", ["/bin/sleep", "60"], initiator="gui",
                  owner_session="gui:BBB")
    c = mgr.spawn("camera_tf:d455", ["/bin/sleep", "60"], initiator="manual",
                  owner_session="manual:1234")
    try:
        results = mgr.cleanup_session("gui:AAA")
        assert results and results[0]["result"] in ("stopped", "already_exited")
        assert mgr.get_record("camera:d455") is None       # 本 session 已清理
        assert mgr.get_record("detector:d455") is not None  # 他人 session 保留
        assert mgr.get_record("camera_tf:d455") is not None  # 手工组件保留
    finally:
        for k in ("camera:d455", "detector:d455", "camera_tf:d455"):
            mgr.stop(k)


def test_cleanup_session_does_not_touch_reused_manual(mgr):
    """reused(非本 session 创建) 组件不被清理。"""
    mgr.spawn("camera:d455", ["/bin/sleep", "60"], initiator="manual",
              owner_session="manual:999")
    try:
        res = mgr.cleanup_session("gui:XYZ")
        assert res == []
        assert mgr.get_record("camera:d455") is not None
    finally:
        mgr.stop("camera:d455")


# ---------------- 结构性回归断言 ----------------
def test_auto_pipeline_starts_after_rclpy_init():
    i_init = GUI_SRC.index("rclpy.init(); node = N()")
    i_thread = GUI_SRC.index("threading.Thread(target=_start_pipeline")
    assert i_init < i_thread, "自动管线线程必须在 rclpy.init + Node 创建之后启动"
    # 等待就绪期间 spinner 保持运转（spin 在主线，管线在旁路线程）


def test_auto_pipeline_covers_camera_tf_and_detector_only():
    i0 = GUI_SRC.index("def auto_start_pipeline(")
    i1 = GUI_SRC.index("def main():")
    body = GUI_SRC[i0:i1]
    assert 'ensure_component("camera"' in body
    assert 'ensure_component("camera_tf"' in body
    assert 'ensure_component("detector"' in body
    # 机械臂/动作/GraspNet 不在自动启动范围
    for forbidden in ('ensure_component("arm"', 'ensure_component("action"',
                      'ensure_component("graspnet"'):
        assert forbidden not in body


def _nested_function(name, namespace):
    import ast
    tree = ast.parse(GUI_SRC)
    node = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    mod = ast.Module(body=[node], type_ignores=[])
    exec(compile(ast.fix_missing_locations(mod), str(WS / "sorting_gui_client.py"), "exec"), namespace)
    return namespace[name]


def test_watchdog_runs_cleanup():
    from types import SimpleNamespace
    calls = []
    namespace = {"os": SimpleNamespace(getppid=lambda: 1), "_ppid0": 999,
                 "_log": lambda *a: None,
                 "_shutdown_worker": lambda **kw: calls.append(kw), "time": time}
    _nested_function("_parent_watchdog", namespace)()
    assert calls == [{"parent_lost": True}]


def test_shutdown_protocol_ack_then_terminate_fallback():
    import queue
    from types import SimpleNamespace
    callbacks, finished = [], []
    app = SimpleNamespace(_closing=False, auto_sort_running=True, dynamic_mode=True,
                          queue_running=True, withdraw=lambda: None,
                          after=lambda delay, callback: callbacks.append(callback))
    commands, ack = queue.Queue(), queue.Queue()
    namespace = {"app": app, "cmd_q": commands, "shutdown_q": ack,
                 "queue": queue, "time": time, "_log": lambda *a: None,
                 "ros_proc": SimpleNamespace(is_alive=lambda: True),
                 "_finalize_close": lambda ok: finished.append(ok)}
    _nested_function("request_shutdown", namespace)()
    assert app._closing and not app.auto_sort_running and not app.dynamic_mode
    assert not app.queue_running and commands.get_nowait() == {"cmd": "shutdown"}
    assert finished == []
    ack.put("__SHUTDOWN_ACK__")
    callbacks.pop()()
    assert finished == [True]


def test_no_pkill_by_filename():
    assert "pkill" not in GUI_SRC, "不得按文件名 pkill（改用统一入口按身份停止）"


def test_gui_uses_shared_launcher():
    assert "_RUN_COMPONENT" in GUI_SRC
    assert "scripts/run_component.sh" in GUI_SRC
    # 不再硬编码 orange_dataset 在线脚本路径
    assert "/home/lxf/orange_dataset/realsense_yolo_node.py" not in GUI_SRC
    assert "/home/lxf/orange_dataset/blue_block_detector.py" not in GUI_SRC


def test_mode_committed_only_on_success():
    i = GUI_SRC.index("def _on_mode_result(")
    body = GUI_SRC[i:i + 700]
    assert "if ok:" in body and "self.detection_mode = mode" in body
    # 失败分支不提交新模式
    assert "self._mode_switch_ok = bool(ok)" in body


def test_mode_switch_blocked_when_busy():
    i = GUI_SRC.index("def _visual_switch_blocked(")
    body = GUI_SRC[i:i + 700]
    # 传送带不再作为相机切换门禁（用户明确要求去掉），保留其余占用判定
    for g in ("self.auto_sort_running", "self.dynamic_mode", "self._dynamic_busy",
              "self.current_status == 'busy'"):
        assert g in body, g
