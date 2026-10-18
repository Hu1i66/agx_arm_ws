"""合成板识别测试（计划 8.1）：不开相机，用生成的板图验证检测与位姿求解。

纯离线：生成 ChArUco 板 → 合成投影 → 检测 → 位姿/方格测量。
"""
import importlib.util
import math
import pathlib
import sys

import numpy as np
import pytest

WS = pathlib.Path(__file__).resolve().parent.parent
PKG = WS / "src" / "handeye_calibration_ros" / "handeye_calibration_ros"
sys.path.insert(0, str(PKG))
import cv2  # noqa: E402
import charuco_detect as cd  # noqa: E402


def test_board_constants_match_plan():
    assert (cd.DEFAULT_SQUARES_X, cd.DEFAULT_SQUARES_Y) == (7, 5)
    assert abs(cd.DEFAULT_SQUARE_LENGTH - 0.032) < 1e-12
    assert abs(cd.DEFAULT_MARKER_LENGTH - 0.024) < 1e-12
    assert cd.DEFAULT_DICT == "DICT_5X5_1000"
    assert cd.FULL_MARKERS == 17
    assert cd.FULL_CORNERS == (7 - 1) * (5 - 1) == 24


def test_synthetic_board_detect_and_pose():
    """合成一张正视板图 → detect_board 检出 → board_pose 求解成功且 RMS 极小。"""
    board = cd.make_board()
    detector = cd.make_detector(board)

    px = 3000
    img = board.generateImage((px, int(px * 5 / 7)), marginSize=60, borderBits=1)
    gray = np.asarray(img, dtype=np.uint8)

    obs = cd.detect_board(gray, board, detector)
    assert obs is not None, "合成板图应能检出"
    assert obs["n_markers"] == cd.FULL_MARKERS
    assert obs["n_corners"] == cd.FULL_CORNERS

    # 用"已知正确"内参求解位姿（正向投影的逆问题）
    K = np.array([[2000.0, 0.0, px / 2], [0.0, 2000.0, px * 5 / 14], [0.0, 0.0, 1.0]])
    D = np.zeros(5)
    pose = cd.board_pose(obs, board, K, D)
    assert pose is not None
    assert pose["reproj_rms_px"] < 1.0, f"合成图 RMS 应极小: {pose['reproj_rms_px']}"
    assert np.all(np.isfinite(pose["T_t2c"]))
    assert np.all(np.isfinite(pose["tvec"]))


def test_measure_square_px_scales_with_resolution():
    board = cd.make_board()
    detector = cd.make_detector(board)
    small = np.asarray(board.generateImage((1400, 1000), marginSize=40), dtype=np.uint8)
    large = np.asarray(board.generateImage((2800, 2000), marginSize=80), dtype=np.uint8)
    o1 = cd.detect_board(small, board, detector)
    o2 = cd.detect_board(large, board, detector)
    assert o1 is not None and o2 is not None
    s1 = cd.measure_square_px(o1)
    s2 = cd.measure_square_px(o2)
    assert s1 > 0 and s2 > 0
    # 分辨率翻倍 → 方格像素边长近似翻倍
    assert abs(s2 / s1 - 2.0) < 0.15


def test_intrinsics_roundtrip_yaml(tmp_path):
    """save_intrinsics/load_intrinsics 往返一致 + candidate 元数据段不破坏标准字段。"""
    K = np.array([[380.0, 1.2, 320.0], [0, 379.0, 240.0], [0, 0, 1]])
    D = np.array([[-0.05, 0.10, -0.0007, 0.0005, -0.09]])
    out = tmp_path / "camera_intrinsics.yaml"
    cd.save_intrinsics(str(out), K, D, 640, 480, 0.21, 20, 9,
                       {"squares_x": 7, "squares_y": 5},
                       metadata={"kind": "intrinsics_candidate", "camera_id": "d455",
                                 "quality_status": "passed"})
    loaded = cd.load_intrinsics(str(out))
    assert loaded is not None
    assert np.allclose(loaded["camera_matrix"], K, atol=1e-9)
    assert np.allclose(loaded["dist_coeffs"].ravel(), D.ravel(), atol=1e-9)
    assert (loaded["image_width"], loaded["image_height"]) == (640, 480)
    assert abs(loaded["reprojection_error"] - 0.21) < 1e-9

    import yaml
    doc = yaml.safe_load(out.read_text())
    assert doc["candidate"]["quality_status"] == "passed"


def test_print_board_preserves_physical_dpi(tmp_path):
    import subprocess
    from PIL import Image
    tool = WS / 'tools/calibration/generate_charuco_board.py'
    result = subprocess.run([sys.executable, str(tool), '--out-dir', str(tmp_path)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    with Image.open(tmp_path / 'charuco_7x5_32mm.png') as image:
        assert abs(image.info.get('dpi', (0, 0))[0] - 300.) < .1
        width_mm, height_mm = (side / 300. * 25.4 for side in image.size)
        assert abs(width_mm - 264.) < .15
        assert abs(height_mm - 200.) < .15
