"""Approval must verify numbers/frames, not only a candidate's passed label."""
import copy
import importlib.util
import json
from pathlib import Path

import pytest
import yaml

WS = Path(__file__).resolve().parents[1]
BOARD = {"squares": [7, 5], "square_m": .032, "marker_m": .024,
         "dict": "DICT_5X5_1000"}


def intrinsic(camera="d455"):
    return {
        "image_width": 640, "image_height": 480, "frames": 20,
        "reprojection_error": .2, "coverage": 5,
        "camera_matrix": {"rows": 3, "cols": 3,
                          "data": [600., 0., 320., 0., 600., 240., 0., 0., 1.]},
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": [0.] * 5},
        "candidate": {"kind": "intrinsics_candidate", "camera_id": camera,
                      "camera_optical_frame": ("camera_color_optical_frame"
                                               if camera == "d455" else "dabai_color_optical_frame"),
                      "quality_status": "passed", "valid_frames": 20,
                      "resolution": [640, 480], "board": copy.deepcopy(BOARD),
                      "source": "charuco_calibration",
                      "coverage_cells": [0, 2, 4, 6, 8],
                      "quality_checks": [{"check": "all", "ok": True}]}}


def handeye(camera="d455"):
    tool = "link6"
    optical = "camera_color_optical_frame" if camera == "d455" else "dabai_color_optical_frame"
    mode = "eye_to_hand" if camera == "d455" else "eye_in_hand"
    return {"position": [.6, -.05, .59], "orientation": [0., 0., 0., 1.],
            "rpy": [[0., 0., 0.]],
            "criteria": {"mode": mode, "frames_used": 20,
                         "consistency_deg": .001, "median_rel_angle_deg": 30.,
                         "n_pairs_ge25deg": 40,
                         "capture_frames": [dict(markers=17, corners=24, reproj_rms_px=.2, square_px=50.) for _ in range(20)],
                         "holdout": {"n": 4, "median_mm": .4, "p95_mm": .7, "max_mm": .9}},
            "metadata": {"kind": "handeye_candidate", "camera_id": camera,
                         "mode": mode, "resolution": [640, 480],
                         "board": copy.deepcopy(BOARD), "frames_used": 20,
                         "session_dir": "/recorded/session",
                         "intrinsics_source": "approved_intrinsics_file:/recorded/intrinsics.yaml",
                         "robot_base_frame": "base_link", "robot_pose_frame": tool,
                         "camera_optical_frame": optical,
                         "transform_parent": "base_link" if camera == "d455" else tool,
                         "transform_child": optical,
                         "tcp_offset": [0.] * 6,
                         "tcp_offset_source": "/agx_arm_ctrl_single_node/tcp_offset",
                         "units": "m", "quaternion_order": "xyzw",
                         "quality_status": "passed",
                         "quality_checks": [{"check": "all", "ok": True}]}}


@pytest.fixture
def approve(monkeypatch, tmp_path):
    cal = tmp_path / "config/calibration"
    cal.mkdir(parents=True)
    (cal / "manifest.json").write_bytes((WS / "config/calibration/manifest.json").read_bytes())
    (cal / "d455_eye_to_hand.json").write_bytes((WS / "config/calibration/d455_eye_to_hand.json").read_bytes())
    monkeypatch.setenv("AGX_ARM_WS", str(tmp_path))
    spec = importlib.util.spec_from_file_location(
        "review_approve", WS / "tools/calibration/approve_calibration.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def mutate_intr(data, case):
    if case == "rms":
        data["reprojection_error"] = .8
    elif case == "coverage":
        data["candidate"]["coverage_cells"] = [0, 1, 2, 3, 4]
    elif case == "nonfinite":
        data["camera_matrix"]["data"][0] = float("nan")
    elif case == "empty_checks":
        data["candidate"]["quality_checks"] = []
    elif case == "zero_focal":
        data["camera_matrix"]["data"][0] = 0.
    elif case == "frames_disagree":
        data["frames"] = 1


@pytest.mark.parametrize("case", ["rms", "coverage", "nonfinite", "empty_checks",
                                   "zero_focal", "frames_disagree"])
def test_intrinsics_label_cannot_bypass_gate(approve, tmp_path, case):
    data = intrinsic()
    mutate_intr(data, case)
    candidate = tmp_path / "bad.yaml"
    candidate.write_text(yaml.safe_dump(data))
    before = approve.MANIFEST.read_bytes()
    with pytest.raises(approve.ApproveError):
        approve.approve_intrinsics("d455", candidate)
    assert approve.MANIFEST.read_bytes() == before
    assert not (approve.CAL_DIR / "d455/intrinsics.yaml").exists()
    assert not approve.BACKUP_DIR.exists()


@pytest.mark.parametrize("case", ["nonfinite", "zero_quaternion", "bad_holdout",
                                   "missing_criteria", "missing_tcp",
                                   "wrong_frame", "empty_checks"])
def test_handeye_label_cannot_bypass_gate(approve, tmp_path, case):
    data = handeye()
    if case == "nonfinite":
        data["position"][0] = float("nan")
    elif case == "zero_quaternion":
        data["orientation"] = [0.] * 4
    elif case == "bad_holdout":
        data["criteria"]["holdout"]["median_mm"] = 16.
    elif case == "missing_criteria":
        data.pop("criteria")
    elif case == "missing_tcp":
        data["metadata"].pop("tcp_offset")
    elif case == "wrong_frame":
        data["metadata"]["transform_parent"] = "wrong_base"
    elif case == "empty_checks":
        data["metadata"]["quality_checks"] = []
    candidate = tmp_path / "bad.json"
    candidate.write_text(json.dumps(data))
    before_manifest = approve.MANIFEST.read_bytes()
    before_active = (approve.CAL_DIR / "d455_eye_to_hand.json").read_bytes()
    with pytest.raises(approve.ApproveError):
        approve.approve_handeye("d455", candidate, scene_verified=True)
    assert approve.MANIFEST.read_bytes() == before_manifest
    assert (approve.CAL_DIR / "d455_eye_to_hand.json").read_bytes() == before_active
    assert not approve.BACKUP_DIR.exists()


def test_approval_backups_unique_even_in_same_second(approve, tmp_path, monkeypatch):
    monkeypatch.setattr(approve, "_stamp", lambda: "same-second")
    results = []
    for camera in ("d455", "dabai"):
        target = approve.CAL_DIR / camera / "intrinsics.yaml"
        target.parent.mkdir()
        target.write_text("old-" + camera)
        candidate = tmp_path / (camera + ".yaml")
        candidate.write_text(yaml.safe_dump(intrinsic(camera)))
        results.append(approve.approve_intrinsics(camera, candidate)["entry"])
    assert results[0]["old_target_backup"] != results[1]["old_target_backup"]
    assert results[0]["manifest_backup"] != results[1]["manifest_backup"]
    assert (approve.WS / results[0]["old_target_backup"]).read_text() == "old-d455"
    assert (approve.WS / results[1]["old_target_backup"]).read_text() == "old-dabai"


def test_valid_handeye_promotes_and_records_tcp(approve, tmp_path):
    data = handeye("dabai")
    candidate = tmp_path / "valid.json"
    candidate.write_text(json.dumps(data))
    result = approve.approve_handeye("dabai", candidate, scene_verified=True)
    manifest = json.loads(approve.MANIFEST.read_text())
    assert manifest["cameras"]["dabai"]["parent"] == "link6"
    assert result["entry"]["tcp_offset"] == [0.] * 6


@pytest.mark.parametrize('key,value', [('markers', 13), ('corners', 19),
                                      ('reproj_rms_px', .6), ('square_px', 24.)])
def test_passed_label_cannot_bypass_per_frame_promotion_gate(approve, tmp_path, key, value):
    data = handeye()
    frames = [dict(markers=17, corners=24, reproj_rms_px=.2, square_px=50.) for _ in range(20)]
    frames[3][key] = value
    data['criteria']['capture_frames'] = frames
    source = tmp_path / 'relaxed.json'; source.write_text(json.dumps(data))
    before = approve.MANIFEST.read_bytes()
    with pytest.raises(approve.ApproveError):
        approve.approve_handeye('d455', source, scene_verified=True)
    assert approve.MANIFEST.read_bytes() == before
