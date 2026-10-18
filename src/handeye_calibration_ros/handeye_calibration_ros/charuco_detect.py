#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ChArUco 标定板检测与单帧位姿求解。

本模块不依赖 ROS, 在线标定节点与离线复算脚本共用同一套检测/求解逻辑,
保证「事后换阈值、换算法、剔除坏帧重解」时与在线结果可比。

标定板参数 (唯一真相源):
    CharucoBoard((7, 5), squareLength=0.032, markerLength=0.024, DICT_5X5_1000)
    17 个标记 (id 0-16) / 24 个内角点 (6x4); 板面原点在左上角,
    x 向右、y 向下、z 垂直板面向内; 板面宽度 0.224 m。
"""

import os

import cv2
import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

try:  # 包内导入 / 直接按文件导入 (离线复算) 两种方式都支持
    from .axxb import rt_to_T
except ImportError:
    from axxb import rt_to_T


# ── 新标定板默认参数 ──
DEFAULT_SQUARES_X = 7
DEFAULT_SQUARES_Y = 5
DEFAULT_SQUARE_LENGTH = 0.032   # m
DEFAULT_MARKER_LENGTH = 0.024   # m
DEFAULT_DICT = "DICT_5X5_1000"

# 质量门 (文档 §5.4)
FULL_MARKERS = 17
FULL_CORNERS = 24


def make_board(square_length=DEFAULT_SQUARE_LENGTH,
               marker_length=DEFAULT_MARKER_LENGTH,
               dict_name=DEFAULT_DICT,
               squares_x=DEFAULT_SQUARES_X,
               squares_y=DEFAULT_SQUARES_Y):
    """构造 ChArUco 标定板对象 (物点单位: m)。

    注意 OpenCV 构造函数参数顺序是 (squaresX, squaresY)。
    """
    if not hasattr(cv2.aruco, dict_name):
        raise ValueError(f"未找到 ArUco 字典: {dict_name}")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    return cv2.aruco.CharucoBoard(
        (int(squares_x), int(squares_y)),
        float(square_length), float(marker_length), dictionary)


def make_detector(board):
    """构造 ChArUco 检测器 (OpenCV >= 4.7 新 API)。"""
    return cv2.aruco.CharucoDetector(board)


def detect_board(gray, board, detector):
    """单帧检测。

    返回 None 表示未检出; 否则返回 dict:
        corners       (N,2) float64 角点像素坐标
        corner_ids    (N,)  int     角点 id (行优先: id = 行*(squares_x-1) + 列)
        marker_ids    标记 id 数组
        n_markers     检出标记数 (满分 17)
        n_corners     检出内角点数 (满分 24)
    """
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    n_corners = 0 if charuco_ids is None else int(len(charuco_ids))
    if n_corners == 0:
        return None
    n_markers = 0 if marker_ids is None else int(len(marker_ids))
    return {
        "corners": np.asarray(charuco_corners, dtype=np.float64).reshape(-1, 2),
        "corner_ids": np.asarray(charuco_ids, dtype=np.int32).reshape(-1),
        # matchImagePoints 直接吃 detectBoard 的原始输出, 故保留
        "corners_raw": charuco_corners,
        "ids_raw": charuco_ids,
        "marker_ids": marker_ids,
        "n_markers": n_markers,
        "n_corners": n_corners,
    }


def measure_square_px(obs, squares_x=DEFAULT_SQUARES_X):
    """实测方格像素边长 (文档 §2.3 的准入判据)。

    detectBoard 只返回**可见**角点, 因此不能照搬文档里 pts[:6] 的写法;
    这里按 corner_id 分行分组, 只在同一行的相邻角点之间取距离。
    """
    pts = obs["corners"]
    ids = obs["corner_ids"]
    if len(ids) < 2:
        return 0.0
    cols = int(squares_x) - 1  # 每行角点数
    order = np.argsort(ids)
    ids = ids[order]
    pts = pts[order]
    dists = []
    for i in range(len(ids) - 1):
        if ids[i + 1] - ids[i] == 1 and (ids[i] // cols) == (ids[i + 1] // cols):
            dists.append(float(np.linalg.norm(pts[i + 1] - pts[i])))
    return float(np.median(dists)) if dists else 0.0


def board_pose(obs, board, camera_matrix, dist_coeffs, squares_x=DEFAULT_SQUARES_X):
    """由单帧检测结果求解标定板在相机系下的位姿 T_t2c。

    流程与文档 §6.1 一致: matchImagePoints -> solvePnP -> solvePnPRefineLM。
    返回 None 表示角点不足或求解失败。
    """
    if obs is None:
        return None
    obj_points, img_points = board.matchImagePoints(obs["corners_raw"], obs["ids_raw"])
    if obj_points is None or img_points is None:
        return None
    obj_points = np.asarray(obj_points, dtype=np.float64).reshape(-1, 3)
    img_points = np.asarray(img_points, dtype=np.float64).reshape(-1, 2)
    if len(obj_points) < 4:
        return None

    ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    # 精修, 明显降低数值地板
    rvec, tvec = cv2.solvePnPRefineLM(obj_points, img_points, camera_matrix, dist_coeffs,
                                      rvec, tvec)

    projected, _ = cv2.projectPoints(obj_points, rvec, tvec, camera_matrix, dist_coeffs)
    diff = projected.reshape(-1, 2) - img_points
    reproj_rms = float(np.sqrt(np.mean(np.sum(diff * diff, axis=1))))

    rotation = cv2.Rodrigues(rvec)[0]
    translation = tvec.reshape(3)
    return {
        "T_t2c": rt_to_T(rotation, translation),
        "rvec": rvec.reshape(3),
        "tvec": translation,
        "reproj_rms_px": reproj_rms,
        "square_px": measure_square_px(obs, squares_x),
        "n_markers": obs["n_markers"],
        "n_corners": obs["n_corners"],
    }


# ────────────────────────── 内参 YAML 读写 ──────────────────────────

def save_intrinsics(path, camera_matrix, dist_coeffs, image_width, image_height,
                    reproj_rms_px, frames, coverage, board_info=None,
                    metadata=None):
    """把单目内参写入 YAML (schema 兼容 ROS camera_info 习惯)。

    metadata: 候选元数据 (camera_id/来源/质量状态等), 写入 candidate 段;
              load_intrinsics 只读取标准字段, 额外段不破坏兼容。
    """
    if yaml is None:
        raise RuntimeError("未安装 PyYAML, 无法写入内参文件")
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64).reshape(1, -1)
    data = {
        "image_width": int(image_width),
        "image_height": int(image_height),
        "camera_matrix": {
            "rows": 3, "cols": 3,
            "data": [float(v) for v in camera_matrix.reshape(-1)],
        },
        "distortion_coefficients": {
            "rows": 1, "cols": int(dist_coeffs.shape[1]),
            "data": [float(v) for v in dist_coeffs.reshape(-1)],
        },
        "reprojection_error": float(reproj_rms_px),
        "frames": int(frames),
        "coverage": int(coverage),
    }
    if board_info:
        data["board"] = dict(board_info)
    if metadata:
        data["candidate"] = dict(metadata)
    with open(path, "w") as fp:
        yaml.safe_dump(data, fp, sort_keys=False, default_flow_style=False)
    return path


def load_intrinsics(path):
    """读取内参 YAML; 文件不存在或格式非法返回 None。"""
    if yaml is None or not path:
        return None
    if not os.path.exists(path):
        return None
    with open(path) as fp:
        data = yaml.safe_load(fp)
    if not isinstance(data, dict):
        return None
    try:
        cm = data["camera_matrix"]
        cm = cm["data"] if isinstance(cm, dict) else cm
        dc = data["distortion_coefficients"]
        dc = dc["data"] if isinstance(dc, dict) else dc
        camera_matrix = np.asarray(cm, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.asarray(dc, dtype=np.float64).reshape(1, -1)
    except (KeyError, TypeError, ValueError):
        return None
    return {
        "camera_matrix": camera_matrix,
        "dist_coeffs": dist_coeffs,
        "image_width": data.get("image_width"),
        "image_height": data.get("image_height"),
        "reprojection_error": data.get("reprojection_error"),
        "raw": data,
    }
