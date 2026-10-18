"""runtime_launcher 测试：命令构造、dry-run 零副作用、cwd 独立、就绪逻辑。"""
import json
import pathlib
import sys
import time

import pytest

WS = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WS))
import runtime_config as rc  # noqa: E402
import runtime_launcher as rl  # noqa: E402
from runtime_process_manager import ProcessManager  # noqa: E402


# ---------------- 命令构造 ----------------
def test_camera_d455_frozen_params():
    argv = rl.build_command("camera", camera="d455")
    joined = " ".join(argv)
    assert argv[:3] == ["ros2", "launch", "realsense2_camera"]
    for needle in ("rgb_camera.color_profile:=640,480,30",
                   "depth_module.profile:=640,480,30",
                   "align_depth.enable:=true",
                   "publish_tf:=false",
                   "spatial_filter.enable:=true",
                   "temporal_filter.enable:=true",
                   "hole_filling_filter.enable:=true"):
        assert needle in joined, needle


def test_camera_dabai_params():
    argv = rl.build_command("camera", camera="dabai")
    joined = " ".join(argv)
    assert "orbbec_camera" in joined and "dabai.launch.py" in joined
    assert "camera_name:=dabai" in joined
    assert "color_format:=MJPG" in joined
    assert "color_width:=640" in joined and "color_height:=480" in joined
    assert "color_fps:=15" in joined
    # 不指定深度 profile、不用 dabai_dcl.py 的 frame_sync
    assert "depth_width" not in joined and "depth_fps" not in joined
    assert "frame_sync" not in joined


def test_arm_command():
    argv = rl.build_command("arm")
    joined = " ".join(argv)
    assert argv[:3] == ["ros2", "launch", "agx_arm_ctrl"]
    for needle in ("can_port:=can0", "arm_type:=piper", "effector_type:=agx_gripper"):
        assert needle in joined


def test_camera_tf_uses_shared_entry():
    argv = rl.build_command("camera_tf")
    assert argv[0] == "ros2" and argv[1] == "launch"
    assert argv[2] == str(WS / "launch" / "eye_to_hand_camera_tf.launch.py")
    argv2 = rl.build_command("camera_tf", ros_args=["calibration_file:=/tmp/c.json"])
    assert argv2[-1] == "calibration_file:=/tmp/c.json"


def test_detector_fruit_uses_canonical_script_and_params():
    argv = rl.build_command("detector", mode="fruit")
    assert argv[0] == str(rc.resolve_resource("vision_python"))
    assert argv[1] == str(WS / "vision" / "realsense_yolo_node.py")
    assert "--params-file" in argv
    assert argv[argv.index("--params-file") + 1] == str(WS / "config" / "vision" / "fruit.yaml")
    # 包装不递归：真脚本在主空间 vision/，不指向 orange_dataset 旧脚本路径
    assert not argv[1].startswith("/home/lxf/orange_dataset")
    assert "realsense_yolo_node" in argv[1]


def test_detector_blue_block():
    argv = rl.build_command("detector", mode="blue_block")
    assert argv[1] == str(WS / "vision" / "blue_block_detector.py")
    assert argv[argv.index("--params-file") + 1] == str(WS / "config" / "vision" / "blue_block.yaml")


def test_graspnet_and_action_and_gui():
    g = rl.build_command("graspnet")
    assert g[0] == str(rc.resolve_resource("vision_python"))
    assert g[1] == str(WS / "graspnet_service_node.py")
    a = rl.build_command("action")
    assert a[0] == str(rc.resolve_resource("system_python"))
    u = rl.build_command("gui")
    assert u[1] == str(WS / "sorting_gui_client.py")


def test_calibrate_intrinsics_command():
    argv = rl.build_command("calibrate-intrinsics", camera="d455")
    joined = " ".join(argv)
    assert argv[:4] == ["ros2", "run", "handeye_calibration_ros",
                        "camera_intrinsic_calibration"]
    assert "image_topic:=/camera/camera/color/image_raw" in joined
    assert "camera_info_topic:=/camera/camera/color/camera_info" in joined
    assert "out_yaml:=" in joined and "candidate.yaml" in joined
    assert "squares_x:=7" in joined and "aruco_dict:=DICT_5X5_1000" in joined


def test_calibrate_handeye_command_d455():
    argv = rl.build_command("calibrate-handeye", camera="d455")
    joined = " ".join(argv)
    assert "mode:=eye_to_hand" in joined
    assert "piper_topic:=/feedback/tcp_pose" in joined
    assert f"intrinsics_yaml:={WS}/config/calibration/d455/intrinsics.yaml" in joined
    assert f"result_save_path:={WS}/data/calibration/d455/handeye" in joined


def test_calibrate_handeye_command_dabai():
    argv = rl.build_command("calibrate-handeye", camera="dabai")
    joined = " ".join(argv)
    assert "mode:=eye_in_hand" in joined
    assert "image_topic:=/dabai/color/image_raw" in joined
    assert f"result_save_path:={WS}/data/calibration/dabai/handeye" in joined


def test_component_key_mapping():
    assert rl.component_key("camera", camera="d455") == "camera:d455"
    assert rl.component_key("camera", camera="dabai") == "camera:dabai"
    assert rl.component_key("camera_tf") == "camera_tf:d455"
    assert rl.component_key("detector", camera="d455") == "detector:d455"


def test_build_command_cwd_independent(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    assert rl.build_command("detector", mode="fruit")[1] == str(WS / "vision" / "realsense_yolo_node.py")


# ---------------- dry-run 零副作用 ----------------
def _write_min_ws(tmp, handeye_file=("d455_eye_to_hand.json",
                                     {"position": [1, 2, 3], "orientation": [0, 0, 0, 1]})):
    (tmp / "config" / "calibration").mkdir(parents=True)
    # 复用真实配置的 cameras/calibration/readiness 段；paths 改为临时空间相对路径
    cfg = rc.load_runtime_config()
    cfg["paths"] = {k: "dummy/rel" for k in rc.RESOURCE_KEYS}
    (tmp / "config" / "runtime.json").write_text(json.dumps(cfg), encoding="utf-8")
    (tmp / "config" / "calibration" / "tcp.json").write_text(json.dumps({"quality_status":"passed","approved":True,"tcp_offset":[0.,0.,.148,0.,0.,0.]}))
    (tmp / "config" / "calibration" / "manifest.json").write_text(json.dumps({
        "cameras": {"d455": {"mode": "eye_to_hand", "parent": "base_link",
                             "child": "camera_color_optical_frame",
                             "active_file": handeye_file[0]}}}), encoding="utf-8")
    (tmp / "config" / "calibration" / handeye_file[0]).write_text(
        json.dumps(handeye_file[1]), encoding="utf-8")


def test_dry_run_has_no_side_effects(monkeypatch, tmp_path, capsys):
    _write_min_ws(tmp_path)
    monkeypatch.setenv("AGX_ARM_WS", str(tmp_path))
    monkeypatch.chdir("/")
    for comp, kw in (("detector", {"mode": "fruit"}),
                     ("camera", {"camera": "d455"}),
                     ("calibrate-intrinsics", {"camera": "d455"}),
                     ("calibrate-handeye", {"camera": "d455"}),
                     ("graspnet", {}), ("arm", {}), ("gui", {})):
        assert rl.dry_run(comp, **kw) == 0
    out = capsys.readouterr().out
    assert "dry-run 不加载模型" in out
    # 零副作用：不创建 .runtime / log / data 目录
    assert not (tmp_path / ".runtime").exists()
    assert not (tmp_path / "log").exists()
    assert not (tmp_path / "data").exists()


# ---------------- 就绪逻辑（monkeypatch ros2，不依赖硬件） ----------------
def _fake_ros2_factory(responses):
    """responses: 有序 (子串, (rc, out, err))；首个匹配命中。"""
    def _fake(args, camera="d455", timeout=15):
        joined = " ".join(str(a) for a in args)
        for needle, resp in responses:
            if needle in joined:
                return resp
        return (1, "", "no-match")
    return _fake


def test_ready_detector_fruit(monkeypatch):
    now = time.time()
    responses = [
        ("node list", (0, "/object_detector\n/other\n", "")),
        ("topic info /detection_info", (0, "Publisher count: 1\n", "")),
        ("topic echo /detection_info", (0, "'{\"detected\": false, \"objects\": []}'\n", "")),
    ]
    monkeypatch.setattr(rl, "_ros2", _fake_ros2_factory(responses))
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    r = rl.check_ready("detector", mode="fruit")
    assert r["ready"] is True, r["detail"]


def test_ready_detector_blue_block_empty_list_legal(monkeypatch):
    responses = [
        ("node list", (0, "/blue_block_detector\n", "")),
        ("topic info /detection_info", (0, "Publisher count: 1\n", "")),
        ("topic echo /detection_info", (0, "'{\"detections\": []}'\n", "")),
    ]
    monkeypatch.setattr(rl, "_ros2", _fake_ros2_factory(responses))
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    r = rl.check_ready("detector", mode="blue_block")
    assert r["ready"] is True, r["detail"]


def test_ready_detector_wrong_format(monkeypatch):
    responses = [
        ("node list", (0, "/object_detector\n", "")),
        ("topic info /detection_info", (0, "Publisher count: 1\n", "")),
        ("topic echo /detection_info", (0, "'{\"detections\": []}'\n", "")),  # 蓝方块格式
    ]
    monkeypatch.setattr(rl, "_ros2", _fake_ros2_factory(responses))
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    r = rl.check_ready("detector", mode="fruit")
    assert r["ready"] is False


def test_ready_camera_d455(monkeypatch):
    now = time.time()
    sec = int(now)
    nsec = int((now - sec) * 1e9)
    responses = [
        ("topic info /camera/camera/color/image_raw", (0, "Publisher count: 1\n", "")),
        ("topic info /camera/camera/aligned_depth_to_color/image_raw", (0, "Publisher count: 1\n", "")),
        ("topic info /camera/camera/color/camera_info", (0, "Publisher count: 1\n", "")),
        ("topic echo /camera/camera/color/camera_info", (0, "width: 640\nheight: 480\n", "")),
        ("topic echo /camera/camera/color/image_raw",
         (0, f"header:\n  stamp:\n    sec: {sec}\n    nanosec: {nsec}\n", "")),
        ("topic echo /camera/camera/aligned_depth_to_color/image_raw",
         (0, f"stamp:\n  sec: {sec}\n  nanosec: {nsec}\n", "")),
        ("param get /camera/camera publish_tf", (0, "Boolean value: False\n", "")),
    ]
    monkeypatch.setattr(rl, "_ros2", _fake_ros2_factory(responses))
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    r = rl.check_ready("camera", camera="d455")
    assert r["ready"] is True, r["detail"]


def test_ready_camera_d455_tf_enabled_fails(monkeypatch):
    now = time.time()
    responses = [
        ("topic info", (0, "Publisher count: 1\n", "")),
        ("topic echo /camera/camera/color/camera_info", (0, "width: 640\nheight: 480\n", "")),
        ("topic echo /camera/camera/color/image_raw",
         (0, f"header:\n  stamp:\n    sec: {int(now)}\n    nanosec: 0\n", "")),
        ("param get /camera/camera publish_tf", (0, "Boolean value: True\n", "")),
    ]
    monkeypatch.setattr(rl, "_ros2", _fake_ros2_factory(responses))
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    r = rl.check_ready("camera", camera="d455")
    assert r["ready"] is False
    assert any("publish_tf" in c["check"] and not c["ok"] for c in r["checks"])


def test_ready_camera_tf_matrix_matches(monkeypatch):
    he = rc.load_handeye("d455")
    x, y, z = he["position"]
    qx, qy, qz, qw = he["orientation"]
    out = (f"- Translation: [{x}, {y}, {z}]\n"
           f"- Rotation: in Quaternion (xyzw) [{qx}, {qy}, {qz}, {qw}]\n")
    responses = [("tf2_echo", (124, out, "timeout"))]
    monkeypatch.setattr(rl, "_ros2", _fake_ros2_factory(responses))
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    r = rl.check_ready("camera_tf")
    assert r["ready"] is True, r["detail"]


def test_ready_camera_tf_matrix_mismatch(monkeypatch):
    out = ("- Translation: [0.1, 0.2, 0.3]\n"
           "- Rotation: in Quaternion (xyzw) [0, 0, 0, 1]\n")
    monkeypatch.setattr(rl, "_ros2", _fake_ros2_factory([("tf2_echo", (124, out, ""))]))
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    r = rl.check_ready("camera_tf")
    assert r["ready"] is False


def test_ready_graspnet_interface_vs_model(monkeypatch, tmp_path):
    responses = [
        ("topic info /graspnet/req", (0, "Publisher count: 0\nSubscription count: 1\n", "")),
        ("topic info /graspnet/result", (0, "Publisher count: 1\n", "")),
    ]
    monkeypatch.setattr(rl, "_ros2", _fake_ros2_factory(responses))
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})

    # 1) 接口存在但外部（无登记）→ 不能判定模型就绪
    r = rl.check_ready("graspnet", record=None)
    assert r["ready"] is False
    assert "接口存在" in r["detail"] and "人工核查" in r["detail"]

    # 2) 托管进程日志已就绪 → ready
    log = tmp_path / "graspnet.log"
    log.write_text("✅ GraspNet 模型加载成功\n✅ graspnet_service_node 就绪, 等待请求...\n",
                   encoding="utf-8")
    r2 = rl.check_ready("graspnet", record={"log_file": str(log)})
    assert r2["ready"] is True, r2["detail"]

    # 3) 托管进程日志仍在加载 → 未就绪
    log2 = tmp_path / "g.log"
    log2.write_text("我正在加载模型...\n", encoding="utf-8")
    r3 = rl.check_ready("graspnet", record={"log_file": str(log2)})
    assert r3["ready"] is False and "仍在加载" in r3["detail"]


def test_ready_detector_no_publisher(monkeypatch):
    responses = [
        ("node list", (0, "/object_detector\n", "")),
        ("topic info /detection_info", (0, "Publisher count: 0\n", "")),
    ]
    monkeypatch.setattr(rl, "_ros2", _fake_ros2_factory(responses))
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    r = rl.check_ready("detector", mode="fruit")
    assert r["ready"] is False


# ---------------- 后台 ensure：复用 / 冲突拒绝 ----------------
_NOT_READY = {"ready": False, "detail": "未就绪", "checks": []}


def test_ensure_reuses_existing_record(monkeypatch, tmp_path):
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    mgr = ProcessManager(tmp_path, ".runtime")
    rec = mgr.spawn("camera:d455", ["/bin/sleep", "60"], initiator="manual",
                    camera="d455")
    monkeypatch.setattr(rl, "_manager", lambda: mgr)
    try:
        res = rl.ensure_background("camera", camera="d455", owner_session="gui:x")
        assert res["result"] == "reused"
        assert res["record"]["pid"] == rec["pid"]
    finally:
        mgr.stop("camera:d455")


def test_ensure_detector_mode_conflict(monkeypatch, tmp_path):
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    mgr = ProcessManager(tmp_path, ".runtime")
    mgr.spawn("detector:d455", ["/bin/sleep", "60"], initiator="manual", mode="fruit")
    monkeypatch.setattr(rl, "_manager", lambda: mgr)
    try:
        res = rl.ensure_background("detector", mode="blue_block", owner_session="gui:x")
        assert res["result"] == "mode_conflict"
        assert res["registered_mode"] == "fruit"
    finally:
        mgr.stop("detector:d455")


def test_ensure_camera_external_conflict_refuses(monkeypatch, tmp_path):
    """外部相机在发布但不就绪 → 拒绝叠加第二个相机。"""
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    monkeypatch.setattr(rl, "check_ready", lambda *a, **k: dict(_NOT_READY))
    monkeypatch.setattr(rl, "_manager", lambda: ProcessManager(tmp_path, ".runtime"))
    responses = [("topic info /camera/camera/color/image_raw",
                  (0, "Publisher count: 1\n", ""))]
    monkeypatch.setattr(rl, "_ros2", _fake_ros2_factory(responses))
    res = rl.ensure_background("camera", camera="d455", owner_session="gui:x")
    assert res["result"] == "external_conflict"


def test_ensure_camera_tf_external_conflict_refuses(monkeypatch, tmp_path):
    """已存在但与本项目不一致的外部 TF → 拒绝双 TF。"""
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    monkeypatch.setattr(rl, "check_ready", lambda *a, **k: dict(_NOT_READY))
    monkeypatch.setattr(rl, "_manager", lambda: ProcessManager(tmp_path, ".runtime"))
    out = ("- Translation: [0.1, 0.2, 0.3]\n"
           "- Rotation: in Quaternion (xyzw) [0, 0, 0, 1]\n")
    monkeypatch.setattr(rl, "_ros2",
                        _fake_ros2_factory([("tf2_echo", (124, out, ""))]))
    res = rl.ensure_background("camera_tf", owner_session="gui:x")
    assert res["result"] == "external_conflict"


def test_ensure_detector_external_conflict_refuses(monkeypatch, tmp_path):
    """外部未登记旧检测节点在发布 → 拒绝叠加第二个检测节点。"""
    monkeypatch.setattr(rl, "build_env", lambda camera="d455": {})
    monkeypatch.setattr(rl, "check_ready", lambda *a, **k: dict(_NOT_READY))
    monkeypatch.setattr(rl, "_manager", lambda: ProcessManager(tmp_path, ".runtime"))
    responses = [
        ("node list", (0, "/object_detector\n", "")),
        ("topic info /detection_info", (0, "Publisher count: 1\n", "")),
    ]
    monkeypatch.setattr(rl, "_ros2", _fake_ros2_factory(responses))
    res = rl.ensure_background("detector", mode="blue_block", owner_session="gui:x")
    assert res["result"] == "external_conflict"
