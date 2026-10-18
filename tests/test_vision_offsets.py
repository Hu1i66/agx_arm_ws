"""标定模式零偏置热更新测试：原子拒绝 NaN/Inf/非法类型、下一帧完整补偿、EMA 隔离、depth 符号。

因视觉节点模块导入需要 rclpy（ROS 环境），测试在 source ROS 的子进程中
以真实模块的未绑定方法验证逻辑（不启动相机、不加载 YOLO、不访问硬件）。
"""
import json
import pathlib
import subprocess
import sys

import pytest

WS = pathlib.Path(__file__).resolve().parent.parent

DRIVER = r'''
import importlib.util, json, os, types

WS = "''' + str(WS) + r'''"

def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

def make_self(cams, hot_fields):
    s = types.SimpleNamespace()
    s.cameras = cams
    s._pending_offsets = {}
    s._HOT_UPDATE_FIELDS = hot_fields
    s.get_logger = lambda: types.SimpleNamespace(info=lambda *a, **k: None)
    return s

class P:
    def __init__(self, name, value):
        self.name = name
        self.value = value

res = {}

# ================= 水果节点 =================
ryn = load(os.path.join(WS, "vision", "realsense_yolo_node.py"), "ryn")
OD = ryn.ObjectDetector
import numpy as np
expected_R = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
assert np.allclose(OD.quaternion_to_rotation_matrix(None, [0, 0, 2, 2]), expected_R)

class Cam:
    def __init__(self, name):
        self.name = name
        self.x_offset_m = -0.035
        self.y_offset_m = -0.020
        self.z_offset_m = 0.030
        self.depth_offset_m = 0.0
        self.tracks = {"t1": object()}
        self.next_track_id = 5
        self.depth_fallback_count = 3

# 1) NaN 拒绝且不改状态
s = make_self({"d455": Cam("d455")}, OD._HOT_UPDATE_FIELDS)
r = OD._on_set_parameters(s, [P("d455.x_offset_m", float("nan"))])
res["nan_rejected"] = (r.successful is False)
res["nan_untouched"] = (s.cameras["d455"].x_offset_m == -0.035 and s._pending_offsets == {})

# 2) Inf 拒绝
r = OD._on_set_parameters(s, [P("d455.depth_offset_m", float("inf"))])
res["inf_rejected"] = (r.successful is False)

# 3) 非法类型拒绝
r = OD._on_set_parameters(s, [P("d455.y_offset_m", "abc")])
res["badtype_rejected"] = (r.successful is False)

# 4) 原子性：一个合法 + 一个 NaN → 整体拒绝
r = OD._on_set_parameters(s, [P("d455.y_offset_m", -0.05), P("d455.z_offset_m", float("nan"))])
res["atomic_reject"] = (r.successful is False and s._pending_offsets == {})
res["atomic_no_state_change"] = (s.cameras["d455"].y_offset_m == -0.020)

# 5) 合法提交：pending 暂存 → 帧边界提交 → 状态更新 + 受影响 track 清空、其他相机保留
d455 = Cam("d455"); dabai = Cam("dabai"); dabai.tracks = {"d": object()}
s = make_self({"d455": d455, "dabai": dabai}, OD._HOT_UPDATE_FIELDS)
r = OD._on_set_parameters(s, [P("d455.x_offset_m", 0.0), P("d455.depth_offset_m", 0.0)])
res["valid_accepted"] = (r.successful is True)
res["pending_staged"] = (s._pending_offsets.get("d455") ==
                         {"x_offset_m": 0.0, "depth_offset_m": 0.0})
res["state_before_frame"] = (d455.x_offset_m == -0.035 and d455.depth_offset_m == 0.0)
OD._apply_pending_offsets(s, d455)
res["state_after_frame"] = (d455.x_offset_m == 0.0 and d455.depth_offset_m == 0.0)
res["tracks_cleared_affected"] = (d455.tracks == {} and d455.next_track_id == 0)
res["tracks_preserved_other"] = (len(dabai.tracks) == 1)
res["pending_consumed"] = ("d455" not in s._pending_offsets)

src = open(os.path.join(WS, "vision", "realsense_yolo_node.py")).read()
res["raw_depth_semantics"] = ("depth_m = depth_m - cam.depth_offset_m" not in src)
res["no_yaml_write"] = ("yaml.dump" not in src and "safe_dump" not in src)
res["dabai_calibrated_supported"] = ("dabai.enabled=true 已拒绝" not in src and "self.camera_geometry.camera(cam.name, rgb_stamp)" in src)
r = OD._on_set_parameters(s, [P("d455.z_offset_m", .001)])
res["manual_bias_rejected"] = r.successful is False

# ================= 蓝方块节点 =================
bb = load(os.path.join(WS, "vision", "blue_block_detector.py"), "bb")
BBD = bb.BlueBlockDetector
assert np.allclose(BBD.quaternion_to_rotation_matrix(None, [0, 0, 2, 2]), expected_R)

class S2:
    def __init__(self):
        self.x_offset_m = -0.045; self.y_offset_m = -0.010; self.z_offset_m = 0.052
        self._tracks = {"t": object()}; self._next_track_id = 2
        self._pending_offsets = {}
        self._HOT_UPDATE_FIELDS = BBD._HOT_UPDATE_FIELDS
        self.get_logger = lambda: types.SimpleNamespace(info=lambda *a, **k: None)

s2 = S2()
r = BBD._on_set_parameters(s2, [P("x_offset_m", float("nan"))])
res["bb_nan_rejected"] = (r.successful is False and s2._pending_offsets == {}
                          and s2.x_offset_m == -0.045)
r = BBD._on_set_parameters(s2, [P("x_offset_m", 0.0), P("z_offset_m", 0.0)])
res["bb_valid_accepted"] = (r.successful is True)
res["bb_state_before_frame"] = (s2.x_offset_m == -0.045)
BBD._apply_pending_offsets(s2)
res["bb_state_after_frame"] = (s2.x_offset_m == 0.0 and s2.z_offset_m == 0.0)
res["bb_tracks_cleared"] = (s2._tracks == {})
r = BBD._on_set_parameters(s2, [P("h_low", 100)])
res["bb_ignores_other_params"] = (r.successful is True and s2._pending_offsets == {})

print("__RESULT__" + json.dumps(res))
'''


@pytest.fixture(scope="module")
def offset_results(tmp_path_factory):
    import runtime_config as rc
    driver = tmp_path_factory.mktemp("vis") / "driver.py"
    driver.write_text(DRIVER, encoding="utf-8")
    # 用生产视觉解释器（共享 venv 提供 open3d/cv2 等）+ ROS 环境
    vision_py = rc.resolve_resource("vision_python")
    script = ("source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true; "
              f"{vision_py} {driver}")
    proc = subprocess.run(["bash", "-c", script], capture_output=True,
                          text=True, timeout=180)
    if proc.returncode != 0 or "__RESULT__" not in proc.stdout:
        pytest.fail(f"视觉节点热更新驱动失败: "
                    f"rc={proc.returncode} err={proc.stderr.strip()[:300]}")
    payload = [l for l in proc.stdout.splitlines() if l.startswith("__RESULT__")][-1]
    return json.loads(payload[len("__RESULT__"):])


ALL_KEYS = [
    "nan_rejected", "nan_untouched", "inf_rejected", "badtype_rejected",
    "atomic_reject", "atomic_no_state_change", "valid_accepted", "pending_staged",
    "state_before_frame", "state_after_frame", "tracks_cleared_affected",
    "tracks_preserved_other", "pending_consumed", "raw_depth_semantics",
    "no_yaml_write", "dabai_calibrated_supported", "manual_bias_rejected",
    "bb_nan_rejected", "bb_valid_accepted", "bb_state_before_frame",
    "bb_state_after_frame", "bb_tracks_cleared", "bb_ignores_other_params",
]


@pytest.mark.parametrize("key", ALL_KEYS)
def test_vision_offset_behaviour(offset_results, key):
    assert offset_results.get(key) is True, f"{key} 失败: {offset_results}"
