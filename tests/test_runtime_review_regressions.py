"""Review regressions at real CLI/process boundaries; never start robot hardware."""
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

import runtime_config as rc
import runtime_launcher as rl
from runtime_process_manager import ProcessManager, _alive


def test_ros_timeout_keeps_tf_output(monkeypatch):
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], 1,
                                        output=b"- Translation: [1, 2, 3]\n",
                                        stderr=b"waiting\n")
    monkeypatch.setattr(rl.subprocess, "run", timeout)
    code, out, err = rl._ros2(["run", "tf2_ros", "tf2_echo", "base", "cam"])
    assert code == 124
    assert "Translation:" in out
    assert "waiting" in err


@pytest.mark.parametrize("text", [
    '{"detections": []}\n---\n',
    '\'{"detections": []}\'\n---\n',
    '{"detections": [{"name": "---"}]}\n---\n',
])
def test_parse_real_echo_document_separator(text):
    payload = json.loads(rl._parse_echo_string_field(text))
    assert isinstance(payload["detections"], list)


def camera_queries(monkeypatch, *, width=640, height=480, depth=True, stamp=0):
    def query(args, camera="d455", timeout=15):
        joined = " ".join(args)
        if args[:2] == ["topic", "info"]:
            return 0, "Publisher count: 1\n", ""
        if "topic echo /camera/camera/color/camera_info" in joined:
            return 0, f"width: {width}\nheight: {height}\n---\n", ""
        if "topic echo /camera/camera/aligned_depth_to_color/image_raw" in joined:
            if not depth:
                return 124, "", "timeout"
            return 0, f"stamp:\n  sec: {stamp}\n  nanosec: 0\n---\n", ""
        if "topic echo /camera/camera/color/image_raw" in joined:
            return 0, f"stamp:\n  sec: {stamp}\n  nanosec: 0\n---\n", ""
        if args[:2] == ["param", "get"]:
            return 0, "Boolean value is: False\n", ""
        return 1, "", "unexpected query"
    monkeypatch.setattr(rl, "_ros2", query)


def test_camera_requires_configured_resolution(monkeypatch):
    camera_queries(monkeypatch, width=320, height=240)
    assert rl.check_ready("camera")["ready"] is False


def test_camera_requires_new_depth_not_just_publisher(monkeypatch):
    camera_queries(monkeypatch, depth=False)
    assert rl.check_ready("camera")["ready"] is False


def test_camera_new_receipt_not_compared_with_other_clock(monkeypatch):
    camera_queries(monkeypatch, stamp=12)
    assert rl.check_ready("camera")["ready"] is True


def test_graspnet_request_subscriber_is_interface(monkeypatch, tmp_path):
    def query(args, **kwargs):
        if args[-1] == "/graspnet/req":
            return 0, "Publisher count: 0\nSubscription count: 1\n", ""
        return 0, "Publisher count: 1\nSubscription count: 0\n", ""
    monkeypatch.setattr(rl, "_ros2", query)
    log = tmp_path / "model.log"
    log.write_text("GraspNet 模型加载成功\n就绪, 等待请求...\n")
    assert rl.check_ready("graspnet", record={"log_file": str(log)})["ready"] is True


def test_background_wait_failure_returns_nonzero(monkeypatch, capsys):
    monkeypatch.setattr(rl, "ensure_background", lambda *a, **kw:
                        {"result": "spawned", "ready": False})
    assert rl.main(["camera", "--background", "--wait-ready"]) != 0


def test_legacy_ros_arguments_reach_canonical_node(monkeypatch):
    seen = []
    monkeypatch.setattr(rl, "dry_run", lambda *a, **kw:
                        seen.append(kw["ros_args"]) or 0)
    assert rl.main(["detector", "--dry-run", "--ros-args", "-p",
                    "d455.z_offset_m:=0.04"]) == 0
    assert seen == [["--ros-args", "-p", "d455.z_offset_m:=0.04"]]


def test_explicit_camera_info_bypasses_missing_intrinsics(monkeypatch, tmp_path):
    monkeypatch.setattr(rc, "workspace_root", lambda: tmp_path)
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    monkeypatch.setattr(rl, "_runtime_log_dir", lambda: tmp_path)
    monkeypatch.setattr(rl, "_log_runtime", lambda *a: None)
    monkeypatch.setattr(rl, "build_command", lambda *a, **kw: ["unused"])
    monkeypatch.setattr(rl, "_manager", lambda: ProcessManager(tmp_path))
    def boundary(*a, **kw):
        raise RuntimeError("spawn boundary reached")
    monkeypatch.setattr(rl.subprocess, "Popen", boundary)
    with pytest.raises(RuntimeError, match="spawn boundary reached"):
        rl.run_foreground("calibrate-handeye",
                          ros_args=["--ros-args", "-p", "use_camera_info:=true"])


def test_foreground_refuses_registered_duplicate(monkeypatch, tmp_path):
    mgr = ProcessManager(tmp_path)
    rec = mgr.spawn("camera:d455", ["/bin/sleep", "30"], initiator="manual")
    monkeypatch.setattr(rl, "_manager", lambda: mgr)
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    monkeypatch.setattr(rl, "_runtime_log_dir", lambda: tmp_path)
    monkeypatch.setattr(rl, "_log_runtime", lambda *a: None)
    def unexpected_spawn(*a, **kw):
        raise AssertionError("duplicate subprocess was launched")
    monkeypatch.setattr(rl.subprocess, "Popen", unexpected_spawn)
    try:
        with pytest.raises(rl.LauncherError, match="已|already"):
            rl.run_foreground("camera")
        assert mgr.get_record("camera:d455")["record_id"] == rec["record_id"]
    finally:
        mgr.stop("camera:d455")


def test_stop_cli_respects_exact_gui_session(monkeypatch, tmp_path, capsys):
    mgr = ProcessManager(tmp_path)
    rec = mgr.spawn("detector:d455", ["/bin/sleep", "30"],
                    initiator="gui", owner_session="gui:B")
    monkeypatch.setattr(rl, "_manager", lambda: mgr)
    monkeypatch.setattr(rl, "_log_runtime", lambda *a: None)
    try:
        code = rl.main(["stop", "detector", "--initiator", "gui",
                        "--owner-session", "gui:A"])
        assert code != 0
        assert _alive(rec["pid"])
    finally:
        mgr.stop("detector:d455")


def test_stop_reports_survivors_after_sigint(tmp_path):
    mgr = ProcessManager(tmp_path)
    ready = tmp_path / "child-ready"
    child_code = ("import signal,time,pathlib;"
                  "signal.signal(signal.SIGINT,signal.SIG_IGN);"
                  f"pathlib.Path({str(ready)!r}).touch();time.sleep(30)")
    parent_code = ("import subprocess,time;"
                   f"subprocess.Popen([{sys.executable!r},'-c',{child_code!r}]);"
                   "time.sleep(30)")
    rec = mgr.spawn("test", [sys.executable, "-c", parent_code], initiator="manual")
    try:
        deadline = time.monotonic() + 3
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert ready.exists()
        result = mgr.stop("test", timeout_sigint=.5, timeout_sigterm=.5)
        assert result["result"] == "incomplete_exit"
        assert mgr.get_record("test") is not None
    finally:
        try:
            os.killpg(rec["pgid"], signal.SIGTERM)
        except ProcessLookupError:
            pass
        mgr.unregister("test", rec["record_id"])


def test_foreground_streams_prompt_without_newline(monkeypatch, tmp_path):
    marker = tmp_path / "prompt-displayed"
    script = ("import pathlib,time,sys;"
              "print('ENTER>',end='',flush=True);"
              f"p=pathlib.Path({str(marker)!r});"
              "deadline=time.monotonic()+2;"
              "\nwhile not p.exists() and time.monotonic()<deadline: time.sleep(.01)"
              "\nprint();sys.exit(0 if p.exists() else 3)")
    mgr = ProcessManager(tmp_path)
    monkeypatch.setattr(rl, "_manager", lambda: mgr)
    monkeypatch.setattr(rc, "workspace_root", lambda: tmp_path)
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": os.environ.copy())
    monkeypatch.setattr(rl, "_runtime_log_dir", lambda: tmp_path)
    monkeypatch.setattr(rl, "_log_runtime", lambda *a: None)
    monkeypatch.setattr(rl, "_node_exists", lambda *a, **kw: False)
    monkeypatch.setattr(rl, "build_command", lambda *a, **kw:
                        [sys.executable, "-u", "-c", script])
    class BytesSink:
        data = b""
        def write(self, data):
            self.data += data
            if b"ENTER>" in self.data:
                marker.touch()
        def flush(self):
            pass
    out = io.StringIO()
    out.buffer = BytesSink()
    monkeypatch.setattr(rl.sys, "stdout", out)
    assert rl.run_foreground("action") == 0


def test_handeye_override_rejects_other_camera_candidate(tmp_path):
    candidate = tmp_path / "dabai.json"
    candidate.write_text(json.dumps({
        "position": [.1, .2, .3], "orientation": [0, 0, 0, 1],
        "metadata": {"camera_id": "dabai", "mode": "eye_in_hand"},
    }))
    with pytest.raises(rc.ConfigError):
        rc.load_handeye("d455", str(candidate))


def test_stop_dry_run_never_stops_live_component(monkeypatch, tmp_path, capsys):
    mgr = ProcessManager(tmp_path)
    rec = mgr.spawn("camera:d455", ["/bin/sleep", "30"], initiator="manual")
    monkeypatch.setattr(rl, "_manager", lambda: mgr)
    monkeypatch.setattr(rl, "_log_runtime", lambda *a: None)
    try:
        assert rl.main(["stop", "camera", "--dry-run"]) == 0
        assert _alive(rec["pid"])
    finally:
        mgr.stop("camera:d455")


def test_foreground_refuses_external_camera(monkeypatch, tmp_path):
    monkeypatch.setattr(rl, "_manager", lambda: ProcessManager(tmp_path))
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    monkeypatch.setattr(rl, "_runtime_log_dir", lambda: tmp_path)
    monkeypatch.setattr(rl, "_log_runtime", lambda *a: None)
    monkeypatch.setattr(rl, "_topic_publisher_count", lambda *a, **kw: 1)
    def unexpected(*a, **kw):
        raise AssertionError("duplicate external camera was launched")
    monkeypatch.setattr(rl.subprocess, "Popen", unexpected)
    with pytest.raises(rl.LauncherError, match="外部|已"):
        rl.run_foreground("camera")


def test_reused_loading_detector_waits_for_ready(monkeypatch, tmp_path):
    mgr = ProcessManager(tmp_path)
    mgr.spawn("detector:d455", ["/bin/sleep", "30"], initiator="manual", mode="fruit")
    monkeypatch.setattr(rl, "_manager", lambda: mgr)
    monkeypatch.setattr(rl, "_log_runtime", lambda *a: None)
    replies = iter([{"ready": False, "detail": "loading"},
                    {"ready": True, "detail": "ready"}])
    monkeypatch.setattr(rl, "check_ready", lambda *a, **kw: next(replies))
    try:
        result = rl.ensure_background("detector", wait_ready=True)
        assert result["ready"] is True
        assert result["result"] == "reused"
    finally:
        mgr.stop("detector:d455")


def test_dabai_candidate_uses_recorded_tcp_frame(tmp_path):
    candidate = tmp_path / "dabai.json"
    candidate.write_text(json.dumps({
        "position": [.1, .2, .3], "orientation": [0., 0., 0., 1.],
        "metadata": {"camera_id": "dabai", "mode": "eye_in_hand",
                     "tcp_offset": [0.] * 6, "robot_pose_frame": "link6",
                     "transform_parent": "link6",
                     "transform_child": "dabai_color_optical_frame"},
    }))
    he = rc.load_handeye("dabai", str(candidate))
    assert he["parent"] == "link6"


def test_tf_ready_compares_normalized_rotation(monkeypatch):
    monkeypatch.setattr(rl.rc, 'load_handeye', lambda *a: {
        'position': [0, 0, 0], 'orientation': [0, 0, 0, 2],
        'quaternion_normalized': [0, 0, 0, 1], 'parent':'base_link','child':'camera_color_optical_frame'})
    monkeypatch.setattr(rl, '_ros2', lambda *a, **kw: (124,
        '- Translation: [0, 0, 0]\n- Rotation: in Quaternion [0, 0, 0, 1]\n', ''))
    assert rl.check_ready('camera_tf')['ready'] is True
