"""标定节点逻辑与批准门禁测试（计划 5.2/5.3/5.4/8.1）。

用 ROS 子进程加载真实模块（未绑定方法 + 伪造 self），无相机/无硬件：
  - 手眼 sync_status(receive/header) 与 setup_intrinsics 失败模式
  - 内参覆盖回退重算 (唯一覆盖帧撤回 → 该格重新判为未覆盖)
  - 合成内参标定 → 候选 YAML 质量门 + 元数据
  - approve 工具: 接受/拒绝(质量失败、相机不符、缺元数据、未 --scene-verified、模式不符) + 审计与备份
"""
import json
import pathlib
import subprocess
import sys

import pytest

WS = pathlib.Path(__file__).resolve().parent.parent

DRIVER = r'''
import importlib.util, json, os, shutil, subprocess, sys, types

WS = "''' + str(WS) + r'''"
PKG = os.path.join(WS, "src", "handeye_calibration_ros", "handeye_calibration_ros")
TMP = os.environ["CAL_TEST_TMP"]
# 包内绝对导入回退 (from charuco_detect import ... / from axxb import ...)
# 需要包目录在 sys.path 上
if PKG not in sys.path:
    sys.path.insert(0, PKG)
res = {}


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def fake_self(**kw):
    s = types.SimpleNamespace()
    s.get_logger = lambda: types.SimpleNamespace(
        warning=lambda *a, **k: None, info=lambda *a, **k: None,
        debug=lambda *a, **k: None)
    for k, v in kw.items():
        setattr(s, k, v)
    return s


def stamp(sec, nsec=0):
    return types.SimpleNamespace(sec=sec, nanosec=nsec)


def msg_with_stamp(sec, nsec=0):
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(sec, nsec)))


# ================= 手眼: sync_status =================
he = load(os.path.join(PKG, "handeye_calibration.py"), "he")
HE = he.HandEyeCalibrationNode

s = fake_self(sync_time_source="receive", max_time_diff=0.05)
ok, desc, src = HE.sync_status(s, msg_with_stamp(0), 100.0, msg_with_stamp(0), 100.02)
res["recv_ok"] = (ok is True and src == "receive")
ok2, _, _ = HE.sync_status(s, msg_with_stamp(0), 100.0, msg_with_stamp(0), 100.30)
res["recv_over_limit_rejected"] = (ok2 is False)

s2 = fake_self(sync_time_source="header", max_time_diff=0.05)
ok3, _, src3 = HE.sync_status(s2, msg_with_stamp(10.00), 100.0,
                              msg_with_stamp(10.01), 100.0)
res["header_ok"] = (ok3 is True and src3 == "header")
ok4, _, _ = HE.sync_status(s2, msg_with_stamp(10.00), 100.0,
                           msg_with_stamp(10.20), 100.0)
res["header_over_limit_rejected"] = (ok4 is False)
ok5, d5, _ = HE.sync_status(s2, msg_with_stamp(0.0), 100.0,
                            msg_with_stamp(0.0), 100.0)
res["header_invalid_rejected"] = (ok5 is False)
# header 超限但接收很近(100.0/100.001) 仍必须拒绝, 不能被接收近掩盖
ok6, _, _ = HE.sync_status(s2, msg_with_stamp(10.00), 100.0,
                           msg_with_stamp(10.30), 100.001)
res["header_no_receive_masking"] = (ok6 is False)

# ================= 手眼: setup_intrinsics 失败模式 =================
IY = os.path.join(TMP, "no_such_intrinsics.yaml")
s3 = fake_self(intrinsics_yaml=IY, use_camera_info=False,
               camera_info_topic="/camera/camera/color/camera_info",
               camera_id="d455", camera_info=None,
               camera_matrix=None, dist_coeffs=None, intrinsics_size=None,
               intrinsics_source=None)
try:
    HE.setup_intrinsics(s3)
    res["intr_missing_no_silent_fallback"] = False
except RuntimeError:
    res["intr_missing_no_silent_fallback"] = (s3.camera_matrix is None)

bad_yaml = os.path.join(TMP, "bad_intrinsics.yaml")
open(bad_yaml, "w").write("not: [a valid intrinsics file]\n")
s4 = fake_self(intrinsics_yaml=bad_yaml, use_camera_info=False,
               camera_info_topic="/x", camera_id="d455", camera_info=None,
               camera_matrix=None, dist_coeffs=None, intrinsics_size=None,
               intrinsics_source=None)
s4.use_camera_info = True
try:
    HE.setup_intrinsics(s4)
    res["intr_invalid_file_no_fallback"] = False
except RuntimeError:
    res["intr_invalid_file_no_fallback"] = (s4.camera_matrix is None)

# Read/freeze TCP endpoint through the real method with only RPC mocked.
from concurrent.futures import Future
for tag, offset, expected in [("zero", [0.] * 6, "link6"),
                              ("offset", [0., 0., .1, 0., 0., 0.], "tcp_link")]:
    future = Future()
    future.set_result(types.SimpleNamespace(values=[types.SimpleNamespace(
        type=8, double_array_value=offset)]))
    requests = []
    client = types.SimpleNamespace(wait_for_service=lambda **kw: True,
                                  call_async=lambda request: requests.append(request.names) or future)
    sample = fake_self(create_client=lambda *a: client, destroy_client=lambda *a: None)
    old_spin = he.rclpy.spin_until_future_complete
    he.rclpy.spin_until_future_complete = lambda *a, **kw: None
    HE._read_tcp_metadata(sample)
    he.rclpy.spin_until_future_complete = old_spin
    res["tcp_" + tag] = (sample.tcp_offset == offset
                          and sample.robot_pose_frame == expected
                          and requests == [["tcp_offset"]])

# ================= 内参: 覆盖回退重算 =================
ic = load(os.path.join(PKG, "intrinsic_calibration.py"), "ic")
IC = ic.CameraIntrinsicCalibrationNode

s5 = fake_self(frames=[
    {"cells": [0, 8], "resolution": [640, 480]},
    {"cells": [4], "resolution": [640, 480]},   # 4 = 中心 (1,1), 唯一覆盖
])
res["coverage_before"] = (IC._recompute_coverage(s5) == {(0, 0), (2, 2), (1, 1)})
s5.frames.pop()   # 撤回唯一覆盖中心的帧
cov = IC._recompute_coverage(s5)
res["coverage_after_rollback"] = ((1, 1) not in cov)
res["coverage_missing_required"] = bool(ic.REQUIRED_CELLS - cov)

# Waiting for user input must not pause the unique receiver.
import threading, time
if not hasattr(IC, "_spin_loop"):
    res["intr_spin_while_idle"] = False
else:
    fake = fake_self(_shutdown=False, count=0)
    old_ok, old_spin = ic.rclpy.ok, ic.rclpy.spin_once
    def spin_once(node, **kw):
        node.count += 1
        if node.count >= 5:
            node._shutdown = True
    ic.rclpy.ok = lambda: True
    ic.rclpy.spin_once = spin_once
    thread = threading.Thread(target=IC._spin_loop, args=(fake,))
    thread.start()
    time.sleep(.03)
    thread.join(1)
    res["intr_spin_while_idle"] = fake.count >= 5 and not thread.is_alive()
    ic.rclpy.ok, ic.rclpy.spin_once = old_ok, old_spin

# ================= 内参: 合成标定 → 候选 + 质量门 =================
import cv2
import numpy as np

K_true = np.array([[380.0, 1.2, 320.0], [0.0, 379.0, 240.0], [0.0, 0.0, 1.0]])
D_true = np.array([-0.05, 0.10, -0.0007, 0.0005, -0.09])
size = (640, 480)
board = ic.make_board(ic.DEFAULT_SQUARE_LENGTH, ic.DEFAULT_MARKER_LENGTH,
                      ic.DEFAULT_DICT, ic.DEFAULT_SQUARES_X, ic.DEFAULT_SQUARES_Y)
objp = np.asarray(board.getChessboardCorners(), dtype=np.float32).reshape(-1, 1, 3)

rng = np.random.default_rng(4242)
obj_list, img_list, frames = [], [], []
for i in range(18):
    rvec = rng.normal(size=3) * 0.4
    tvec = np.array([rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.05),
                     rng.uniform(0.35, 0.55)])
    imgp, _ = cv2.projectPoints(objp, rvec, tvec, K_true, D_true)
    imgp = imgp.astype(np.float32).reshape(-1, 1, 2)
    obj_list.append(objp.copy())
    img_list.append(imgp)
    frames.append({"cells": [0, 2, 4, 6, 8], "resolution": [640, 480]})  # 覆盖中心+四角

out_yaml = os.path.join(TMP, "session", "candidate.yaml")
os.makedirs(os.path.dirname(out_yaml), exist_ok=True)
s6 = fake_self(camera_info=types.SimpleNamespace(width=640, height=480),
               object_points=obj_list, image_points=img_list, frames=frames,
               covered_cells={(0, 0), (0, 2), (1, 1), (2, 0), (2, 2)},
               out_yaml=out_yaml, min_frames=15, max_reproj_rms=0.5,
               squares_x=7, squares_y=5, square_length=0.032, marker_length=0.024,
               aruco_dict="DICT_5X5_1000", camera_id="d455")
IC.compute_and_save(s6)
import yaml as _yaml
cand_doc = _yaml.safe_load(open(out_yaml))
res["intr_candidate_passed"] = (cand_doc["candidate"]["quality_status"] == "passed")
res["intr_candidate_meta"] = (cand_doc["candidate"]["camera_id"] == "d455"
                              and cand_doc["candidate"]["resolution"] == [640, 480]
                              and cand_doc["candidate"]["approved"] is False
                              and isinstance(cand_doc["candidate"]["board"], dict))
res["intr_candidate_rms"] = bool(cand_doc["reprojection_error"] < 0.5)
res["intr_K_close"] = bool(np.allclose(np.asarray(cand_doc["camera_matrix"]["data"]).reshape(3, 3),
                                       K_true, atol=5.0))

# 覆盖不足 → failed
s7 = fake_self(**{k: getattr(s6, k) for k in
                  ("camera_info", "object_points", "image_points", "min_frames",
                   "max_reproj_rms", "squares_x", "squares_y", "square_length",
                   "marker_length", "aruco_dict", "camera_id")})
s7.frames = [{"cells": [0], "resolution": [640, 480]}] * 18
s7.covered_cells = {(0, 0)}
s7.out_yaml = os.path.join(TMP, "session", "candidate_fail.yaml")
IC.compute_and_save(s7)
fail_doc = _yaml.safe_load(open(s7.out_yaml))
res["intr_coverage_gate_failed"] = (fail_doc["candidate"]["quality_status"] == "failed")

# A candidate YAML cannot masquerade as an approved intrinsic file.
s_unapproved = fake_self(intrinsics_yaml=out_yaml, use_camera_info=True,
                         camera_id="d455", camera_matrix=None)
try:
    HE.setup_intrinsics(s_unapproved)
    res["intr_unapproved_rejected"] = False
except RuntimeError:
    res["intr_unapproved_rejected"] = s_unapproved.camera_matrix is None

# All three resolutions must agree before attempting image conversion.
pose = he.PoseStamped()
converted = []
now = time.monotonic()
img = types.SimpleNamespace(width=640, height=480)
s_sizes = fake_self(spin_for=lambda *a: None, settle_delay=0., camera_matrix=np.eye(3),
                    _lock=threading.Lock(), latest_image=(img, now, 1),
                    latest_pose=(pose, now, 1),
                    camera_info=types.SimpleNamespace(width=320, height=240),
                    camera_info_recv=now, last_sampled_img_seq=None,
                    max_message_age_s=1., intrinsics_size=(640, 480),
                    bridge=types.SimpleNamespace(imgmsg_to_cv2=lambda *a, **kw:
                                                  converted.append(True)))
HE.capture_frame(s_sizes)
res["intr_three_sizes_rejected"] = not converted

# ================= approve 工具 =================
tmp_ws = os.path.join(TMP, "ws")
os.makedirs(os.path.join(tmp_ws, "config", "calibration"), exist_ok=True)
shutil.copy2(os.path.join(WS, "config", "calibration", "manifest.json"),
             os.path.join(tmp_ws, "config", "calibration", "manifest.json"))
TOOL = os.path.join(WS, "tools", "calibration", "approve_calibration.py")


def run_approve(args):
    env = dict(os.environ, AGX_ARM_WS=tmp_ws)
    p = subprocess.run([sys.executable, TOOL] + args, capture_output=True,
                       text=True, env=env, cwd=tmp_ws)
    return p.returncode, p.stdout, p.stderr


# 内参: 合法候选 → 接受 + 审计 + 备份
rc, out, err = run_approve(["intrinsics", "--camera", "d455", "--candidate", out_yaml])
res["approve_intr_ok"] = (rc == 0)
tgt = os.path.join(tmp_ws, "config", "calibration", "d455", "intrinsics.yaml")
res["approve_intr_target_exists"] = os.path.isfile(tgt)
if os.path.isfile(tgt):
    ap = _yaml.safe_load(open(tgt))
    res["approve_intr_marked"] = (ap["candidate"]["approved"] is True)
man = json.load(open(os.path.join(tmp_ws, "config", "calibration", "manifest.json")))
log = man["cameras"]["d455"].get("promotion_log", [])
res["approve_intr_audit"] = bool(log and log[-1]["kind"] == "intrinsics"
                                 and log[-1]["target_sha256"])
res["approve_intr_backup"] = os.path.isdir(
    os.path.join(tmp_ws, "config", "calibration", "approved_backup"))

# 内参: 质量失败的候选 → 拒绝
fail_cand = os.path.join(TMP, "session", "candidate_fail.yaml")
rc2, _, _ = run_approve(["intrinsics", "--camera", "d455", "--candidate", fail_cand])
res["approve_intr_reject_failed_quality"] = (rc2 == 2)

# 内参: 相机不符 → 拒绝
rc3, _, _ = run_approve(["intrinsics", "--camera", "dabai", "--candidate", out_yaml])
res["approve_intr_reject_camera_mismatch"] = (rc3 == 2)

# 内参: 缺元数据 → 拒绝
plain = os.path.join(TMP, "session", "plain.yaml")
_yaml.safe_dump({"image_width": 640, "image_height": 480,
                 "camera_matrix": {"data": [1, 0, 0, 0, 1, 0, 0, 0, 1]},
                 "distortion_coefficients": {"data": [0, 0, 0, 0, 0]},
                 "reprojection_error": 0.1}, open(plain, "w"))
rc4, _, _ = run_approve(["intrinsics", "--camera", "d455", "--candidate", plain])
res["approve_intr_reject_missing_meta"] = (rc4 == 2)

# 手眼: 构造合法候选
he_cand = {
    "position": [0.649, -0.054, 0.596],
    "orientation": [-0.6924182483257096, -0.7044102901442802,
                    0.08488321314295422, 0.1309883683803036],
    "rpy": [[-2.835, -0.067, 1.578]],
    "criteria": {"mode": "eye_to_hand", "frames_used": 20,
                 "consistency_deg": 0.001, "median_rel_angle_deg": 30.,
                 "n_pairs_ge25deg": 40,
                         "capture_frames": [dict(markers=17, corners=24, reproj_rms_px=.2, square_px=50.) for _ in range(20)],
                 "holdout": {"n": 4, "median_mm": 0.4, "p95_mm": 0.7, "max_mm": 0.9}},
    "metadata": {
        "kind": "handeye_candidate", "camera_id": "d455", "mode": "eye_to_hand",
        "resolution": [640, 480],
        "board": {"squares": [7, 5], "square_m": .032, "marker_m": .024,
                  "dict": "DICT_5X5_1000"}, "frames_used": 20,
        "session_dir": "/recorded/session",
        "robot_base_frame": "base_link", "robot_pose_frame": "link6",
        "camera_optical_frame": "camera_color_optical_frame",
        "transform_parent": "base_link", "transform_child": "camera_color_optical_frame",
        "tcp_offset": [0.] * 6,
        "tcp_offset_source": "/agx_arm_ctrl_single_node/tcp_offset",
        "units": "m", "quaternion_order": "xyzw",
        "intrinsics_source": "approved_intrinsics_file:x",
        "quality_status": "passed",
        "quality_checks": [{"check": "有效姿态数 >= min_num", "ok": True, "detail": "20/15"}],
    },
}
he_path = os.path.join(TMP, "session", "he_candidate.json")
json.dump(he_cand, open(he_path, "w"))

# 未 --scene-verified → 拒绝
rc5, _, err5 = run_approve(["handeye", "--camera", "d455", "--candidate", he_path])
res["approve_he_requires_scene_verified"] = (rc5 == 2)
# 带 --scene-verified → 接受
rc6, out6, _ = run_approve(["handeye", "--camera", "d455", "--candidate", he_path,
                            "--scene-verified"])
res["approve_he_ok"] = (rc6 == 0)
he_tgt = os.path.join(tmp_ws, "config", "calibration", "d455_eye_to_hand.json")
res["approve_he_target_schema"] = False
if os.path.isfile(he_tgt):
    d = json.load(open(he_tgt))
    res["approve_he_target_schema"] = (set(d.keys()) == {"position", "orientation", "rpy"})
man2 = json.load(open(os.path.join(tmp_ws, "config", "calibration", "manifest.json")))
res["approve_he_audit"] = any(e.get("kind") == "handeye" and e.get("scene_verified") is True
                              for e in man2["cameras"]["d455"]["promotion_log"])

# 手眼: 模式不符 → 拒绝
he_bad = json.loads(json.dumps(he_cand))
he_bad["metadata"]["mode"] = "eye_in_hand"
p_bad = os.path.join(TMP, "session", "he_bad.json")
json.dump(he_bad, open(p_bad, "w"))
rc7, _, _ = run_approve(["handeye", "--camera", "d455", "--candidate", p_bad,
                         "--scene-verified"])
res["approve_he_reject_mode_mismatch"] = (rc7 == 2)

# 手眼: dabai 成功只保存外参
he_dabai = json.loads(json.dumps(he_cand))
he_dabai["metadata"]["camera_id"] = "dabai"
he_dabai["metadata"]["mode"] = "eye_in_hand"
he_dabai["criteria"]["mode"] = "eye_in_hand"
he_dabai["metadata"]["transform_parent"] = "link6"
he_dabai["metadata"]["transform_child"] = "dabai_color_optical_frame"
he_dabai["metadata"]["camera_optical_frame"] = "dabai_color_optical_frame"
he_dabai["position"] = [0.01, 0.02, 0.03]
p_dabai = os.path.join(TMP, "session", "he_dabai.json")
json.dump(he_dabai, open(p_dabai, "w"))
rc8, _, _ = run_approve(["handeye", "--camera", "dabai", "--candidate", p_dabai,
                         "--scene-verified"])
res["approve_he_dabai_ok"] = (rc8 == 0)
res["approve_he_dabai_file"] = os.path.isfile(
    os.path.join(tmp_ws, "config", "calibration", "dabai_eye_in_hand.json"))
# 未生成假 dabai 内参
res["no_fake_dabai_intrinsics"] = not os.path.isfile(
    os.path.join(tmp_ws, "config", "calibration", "dabai", "intrinsics.yaml"))

print("__RESULT__" + json.dumps(res))
'''


@pytest.fixture(scope="module")
def cal_results(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("cal")
    driver = tmp / "driver.py"
    driver.write_text(DRIVER, encoding="utf-8")
    script = ("source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true; "
              f"CAL_TEST_TMP={tmp} /usr/bin/python3 {driver}")
    proc = subprocess.run(["bash", "-c", script], capture_output=True,
                          text=True, timeout=300)
    if proc.returncode != 0 or "__RESULT__" not in proc.stdout:
        pytest.fail(f"标定逻辑驱动失败: rc={proc.returncode} "
                    f"err={proc.stderr.strip()[-400:]}")
    line = [l for l in proc.stdout.splitlines() if l.startswith("__RESULT__")][-1]
    return json.loads(line[len("__RESULT__"):])


KEYS = [
    "recv_ok", "recv_over_limit_rejected", "header_ok",
    "header_over_limit_rejected", "header_invalid_rejected",
    "header_no_receive_masking",
    "intr_missing_no_silent_fallback", "intr_invalid_file_no_fallback", "intr_spin_while_idle", "tcp_zero", "tcp_offset",
    "intr_unapproved_rejected", "intr_three_sizes_rejected",
    "coverage_before", "coverage_after_rollback", "coverage_missing_required",
    "intr_candidate_passed", "intr_candidate_meta", "intr_candidate_rms",
    "intr_K_close", "intr_coverage_gate_failed",
    "approve_intr_ok", "approve_intr_target_exists", "approve_intr_marked",
    "approve_intr_audit", "approve_intr_backup",
    "approve_intr_reject_failed_quality", "approve_intr_reject_camera_mismatch",
    "approve_intr_reject_missing_meta",
    "approve_he_requires_scene_verified", "approve_he_ok",
    "approve_he_target_schema", "approve_he_audit",
    "approve_he_reject_mode_mismatch", "approve_he_dabai_ok",
    "approve_he_dabai_file", "no_fake_dabai_intrinsics",
]


@pytest.mark.parametrize("key", KEYS)
def test_calibration_logic(cal_results, key):
    assert cal_results.get(key) is True, f"{key} 失败: {cal_results}"
