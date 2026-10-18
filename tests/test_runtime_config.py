"""runtime_config 测试：cwd 独立、AGX_ARM_WS、资源解析、外参校验与基线哈希。"""
import hashlib
import json
import os
import pathlib
import sys

import pytest

WS = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WS))
import runtime_config as rc  # noqa: E402

BASELINE_D455_SHA256 = "2d0907d8333e5c4c91fe176754a4e45345b0c31ee41d4698cc720a610b7f5c4f"


def _write_min_workspace(tmp: pathlib.Path, handeye=None, manifest=None):
    (tmp / "config" / "calibration").mkdir(parents=True, exist_ok=True)
    (tmp / "config" / "runtime.json").write_text(json.dumps({
        "paths": {k: ("some/rel" if k in ("dds_xml", "saved_poses") else "/abs/x")
                  for k in rc.RESOURCE_KEYS}
    }), encoding="utf-8")
    man = manifest if manifest is not None else {
        "cameras": {
            "d455": {"mode": "eye_to_hand", "parent": "base_link",
                     "child": "camera_color_optical_frame",
                     "active_file": "d455_eye_to_hand.json"},
            "dabai": {"mode": "eye_in_hand", "parent": "tcp_link",
                      "child": "dabai_color_optical_frame", "active_file": None},
        }
    }
    (tmp / "config" / "calibration" / "manifest.json").write_text(
        json.dumps(man), encoding="utf-8")
    if handeye is not None:
        (tmp / "config" / "calibration" / "d455_eye_to_hand.json").write_text(
            json.dumps(handeye), encoding="utf-8")


# ---------------- cwd 独立 / root 定位 ----------------
def test_workspace_root_is_cwd_independent(monkeypatch, tmp_path):
    monkeypatch.delenv("AGX_ARM_WS", raising=False)
    monkeypatch.chdir(tmp_path)
    assert rc.workspace_root() == WS


def test_workspace_root_env_override(monkeypatch, tmp_path):
    monkeypatch.setenv("AGX_ARM_WS", str(tmp_path))
    monkeypatch.chdir("/")
    assert rc.workspace_root() == tmp_path.resolve()


# ---------------- 资源解析 ----------------
def test_resolve_all_keys_exist_in_config():
    cfg = rc.load_runtime_config()
    assert set(cfg["paths"]) >= set(rc.RESOURCE_KEYS)


def test_resolve_project_relative_and_absolute():
    py = rc.resolve_resource("vision_python")
    assert py.is_absolute()
    dds = rc.resolve_resource("dds_xml")
    assert dds.is_absolute() and str(dds).startswith(str(WS))
    urdf = rc.resolve_resource("ik_urdf")
    assert urdf.is_absolute() and str(urdf).startswith(str(WS))


def test_resolve_unknown_key_raises():
    with pytest.raises(rc.ConfigError):
        rc.resolve_resource("nonexistent_key")


def test_missing_config_raises(monkeypatch, tmp_path):
    monkeypatch.setenv("AGX_ARM_WS", str(tmp_path))
    with pytest.raises(rc.ConfigError):
        rc.load_runtime_config()


def test_missing_named_key_raises(monkeypatch, tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "runtime.json").write_text(
        json.dumps({"paths": {"dds_xml": "config/dds/cyclonedds.xml"}}),
        encoding="utf-8")
    monkeypatch.setenv("AGX_ARM_WS", str(tmp_path))
    with pytest.raises(rc.ConfigError) as e:
        rc.load_runtime_config()
    assert "缺少命名资源键" in str(e.value)


# ---------------- 活动外参：合法 ----------------
def test_load_handeye_d455_values():
    he = rc.load_handeye("d455")
    assert he["mode"] == "eye_to_hand"
    assert he["parent"] == "base_link"
    assert he["child"] == "camera_color_optical_frame"
    assert he["position"] == [0.610466572986308, -0.08928292449370928,
                              0.6187731911036556]
    assert len(he["orientation"]) == 4
    norm = sum(q * q for q in he["orientation"]) ** 0.5
    assert abs(norm - 1.0) < 1e-6


def test_d455_handeye_file_hash_unchanged():
    """原 JSON 不得被加载器重写。

    迁移基线哈希必须保持不变；唯一允许的变化是经 approve-handeye 的
    **合法批准推广**（此时 manifest promotion_log 必须有对应 target_sha256）。
    """
    h = hashlib.sha256((WS / "config" / "calibration" /
                        "d455_eye_to_hand.json").read_bytes()).hexdigest()
    if h == BASELINE_D455_SHA256:
        return
    man = json.loads((WS / "config" / "calibration" / "manifest.json")
                     .read_text(encoding="utf-8"))
    log = man["cameras"]["d455"].get("promotion_log", [])
    assert any(e.get("kind") == "handeye" and e.get("target_sha256") == h
               for e in log), \
        "活动外参既不是迁移基线, 又无对应的批准审计记录（不得被其它途径改写）"


def test_load_handeye_dabai_active_onsite_file():
    he = rc.load_handeye("dabai")
    assert he["mode"] == "eye_in_hand" and he["parent"] == "link6"
    assert he["child"] == "dabai_color_optical_frame"
    assert len(he["position"]) == 3


def test_load_handeye_calibration_file_override(tmp_path):
    alt = tmp_path / "candidate.json"
    alt.write_text(json.dumps({"position": [1, 2, 3],
                               "orientation": [0, 0, 0, 1]}), encoding="utf-8")
    he = rc.load_handeye("d455", str(alt))
    assert he["position"] == [1.0, 2.0, 3.0]
    assert he["source"] == str(alt)


# ---------------- 活动外参：非法（明确失败，不静默回退） ----------------
@pytest.mark.parametrize("bad,needle", [
    ({"position": [1, 2], "orientation": [0, 0, 0, 1]}, "长度 3"),
    ({"position": [1, 2, 3], "orientation": [0, 0, 1]}, "长度 4"),
    ({"position": [1, 2, float("nan")], "orientation": [0, 0, 0, 1]}, "非有限"),
    ({"position": [1, 2, 3], "orientation": [0, 0, 0, 0]}, "四元数为零"),
    ({"orientation": [0, 0, 0, 1]}, "缺少"),
    ({"position": [1, 2, 3]}, "缺少"),
])
def test_load_handeye_invalid_values(monkeypatch, tmp_path, bad, needle):
    _write_min_workspace(tmp_path, handeye=bad)
    monkeypatch.setenv("AGX_ARM_WS", str(tmp_path))
    with pytest.raises(rc.ConfigError) as e:
        rc.load_handeye("d455")
    assert needle in str(e.value)


def test_load_handeye_invalid_mode(monkeypatch, tmp_path):
    _write_min_workspace(tmp_path, handeye={"position": [1, 2, 3],
                                            "orientation": [0, 0, 0, 1]},
                         manifest={"cameras": {"d455": {
                             "mode": "wrong_mode", "parent": "base_link",
                             "child": "camera_color_optical_frame",
                             "active_file": "d455_eye_to_hand.json"}}})
    monkeypatch.setenv("AGX_ARM_WS", str(tmp_path))
    with pytest.raises(rc.ConfigError) as e:
        rc.load_handeye("d455")
    assert "mode 非法" in str(e.value)


def test_load_handeye_missing_file(monkeypatch, tmp_path):
    _write_min_workspace(tmp_path, handeye=None)
    monkeypatch.setenv("AGX_ARM_WS", str(tmp_path))
    with pytest.raises(rc.ConfigError) as e:
        rc.load_handeye("d455")
    assert "不存在" in str(e.value)


def test_load_handeye_unknown_camera():
    with pytest.raises(rc.ConfigError):
        rc.load_handeye("nosuchcam")
