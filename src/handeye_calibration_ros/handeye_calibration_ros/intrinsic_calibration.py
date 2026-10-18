#!/usr/bin/env python3
# -*-coding:utf8-*-
"""ChArUco 单目相机内参标定 (替代厂商出厂内参)。

用法 (必须用 ros2 run, 交互式 stdin 在 ros2 launch 下会 crash):
    ros2 run handeye_calibration_ros camera_intrinsic_calibration

采集 15-20 帧, 务必覆盖画面中心与四角 (畸变系数 k1/k2/k3 需要边缘样本才能被约束),
然后按 q 求解并写入 YAML; 手眼标定节点会优先加载该文件。

按键: Enter-采集, d-回退, q-计算并存盘, c-退出
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import termios
import tty
import time
import threading
import os
import json

try:
    from .charuco_detect import (
        make_board, make_detector, detect_board, save_intrinsics,
        DEFAULT_SQUARES_X, DEFAULT_SQUARES_Y, DEFAULT_SQUARE_LENGTH,
        DEFAULT_MARKER_LENGTH, DEFAULT_DICT)
    from .validation import camera_optical_frame
except ImportError:
    from charuco_detect import (
        make_board, make_detector, detect_board, save_intrinsics,
        DEFAULT_SQUARES_X, DEFAULT_SQUARES_Y, DEFAULT_SQUARE_LENGTH,
        DEFAULT_MARKER_LENGTH, DEFAULT_DICT)
    from validation import camera_optical_frame


def clear_input_buffer():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# 建议覆盖的区域: 中心 + 四角 (3x3 网格上的位置)
REQUIRED_CELLS = {(0, 0), (0, 2), (1, 1), (2, 0), (2, 2)}
GRID = 3


class CameraIntrinsicCalibrationNode(Node):
    def __init__(self):
        super().__init__("camera_intrinsic_calibration")

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('out_yaml', './result/camera_intrinsics.yaml')
        self.declare_parameter('min_frames', 15)
        self.declare_parameter('min_corners', 6)
        self.declare_parameter('settle_delay', 0.3)
        self.declare_parameter('squares_x', DEFAULT_SQUARES_X)
        self.declare_parameter('squares_y', DEFAULT_SQUARES_Y)
        self.declare_parameter('square_length', DEFAULT_SQUARE_LENGTH)
        self.declare_parameter('marker_length', DEFAULT_MARKER_LENGTH)
        self.declare_parameter('aruco_dict', DEFAULT_DICT)
        # ── 候选/质量/新鲜度 (计划 5.2/5.3/5.4) ──
        self.declare_parameter('camera_id', 'd455')
        self.declare_parameter('max_reproj_rms', 0.5)
        self.declare_parameter('max_message_age_s', 1.0)

        p = lambda n: self.get_parameter(n).get_parameter_value()
        self.image_topic = p('image_topic').string_value
        self.camera_info_topic = p('camera_info_topic').string_value
        self.out_yaml = p('out_yaml').string_value
        self.min_frames = p('min_frames').integer_value
        self.min_corners = p('min_corners').integer_value
        self.settle_delay = p('settle_delay').double_value
        self.squares_x = p('squares_x').integer_value
        self.squares_y = p('squares_y').integer_value
        self.square_length = p('square_length').double_value
        self.marker_length = p('marker_length').double_value
        self.aruco_dict = p('aruco_dict').string_value
        self.camera_id = p('camera_id').string_value
        self.max_reproj_rms = p('max_reproj_rms').double_value
        self.max_message_age_s = p('max_message_age_s').double_value

        print(f"image_topic: {self.image_topic}")
        print(f"camera_info_topic: {self.camera_info_topic}")
        print(f"out_yaml: {self.out_yaml}")
        print(f"camera_id: {self.camera_id}  max_reproj_rms: {self.max_reproj_rms}px  "
              f"max_message_age_s: {self.max_message_age_s}")
        print(f"标定板: {self.squares_x}x{self.squares_y} 方格={self.square_length}m "
              f"标记={self.marker_length}m 字典={self.aruco_dict}")

        self.bridge = CvBridge()
        self.board = make_board(self.square_length, self.marker_length, self.aruco_dict,
                                self.squares_x, self.squares_y)
        self.detector = make_detector(self.board)

        self.camera_info = None
        self.camera_info_recv = None
        self._lock = threading.Lock()
        self._img_seq = 0
        self.latest_image = None     # (Image msg, 接收 monotonic, 接收序号)
        self.last_sampled_img_seq = None
        self.frames = []             # 每帧: 角点数/接收序号/年龄/覆盖cells/分辨率
        self.object_points = []
        self.image_points = []
        self.corners_per_frame = []
        self.covered_cells = set()

        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)

        deadline = time.time() + 10.0
        while self.camera_info is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.camera_info is None:
            print(f"!! 未收到 {self.camera_info_topic}: 无法确定采集分辨率 !!")
        else:
            print(f"采集分辨率: {self.camera_info.width}x{self.camera_info.height} "
                  f"(内参必须与采集分辨率一致, 文档第 1 条)")
        self._shutdown = False
        self._spin_thread = threading.Thread(target=self._spin_loop,
                                             name="intrinsics_spin", daemon=True)
        self._spin_thread.start()

    def _spin_loop(self):
        while rclpy.ok() and not self._shutdown:
            try:
                rclpy.spin_once(self, timeout_sec=.05)
            except Exception:
                if self._shutdown or not rclpy.ok():
                    break
                time.sleep(.05)

    def stop_spin(self):
        self._shutdown = True
        self._spin_thread.join(timeout=1.)

    def camera_info_cb(self, msg):
        with self._lock:
            self.camera_info = msg
            self.camera_info_recv = time.monotonic()

    def image_cb(self, msg):
        with self._lock:
            self._img_seq += 1
            self.latest_image = (msg, time.monotonic(), self._img_seq)

    def spin_for(self, seconds):
        time.sleep(seconds)  # Only the background receiver spins this node.

    def update_coverage(self, corners, width, height):
        for x, y in corners:
            col = min(GRID - 1, max(0, int(x / max(1, width) * GRID)))
            row = min(GRID - 1, max(0, int(y / max(1, height) * GRID)))
            self.covered_cells.add((row, col))

    def capture_frame(self):
        """采集一帧 (计划 5.4: 锁保护快照 / 接收年龄 / 接收序号拒绝重复帧 / 尺寸一致)。"""
        self.spin_for(self.settle_delay)
        with self._lock:
            snap = self.latest_image
            snap_info = self.camera_info
            snap_info_recv = self.camera_info_recv
            last_seq = self.last_sampled_img_seq

        if snap is None:
            print("!! 未收到图像, 请检查 image_topic !!")
            return False
        if snap_info is None:
            print("!! 未收到 camera_info (尺寸/新鲜度核对需要) !!")
            return False

        img_msg, recv, seq = snap
        if last_seq is not None and seq == last_seq:
            print("!! 与上次采样是同一帧图像 (未收到新图), 请调整后重采 !!")
            return False
        age = time.monotonic() - recv
        if age > self.max_message_age_s:
            print(f"!! 图像过期 (接收年龄 {age:.2f}s > {self.max_message_age_s}s), 该帧作废")
            return False
        if snap_info_recv is not None and \
                (time.monotonic() - snap_info_recv) > self.max_message_age_s:
            print("!! CameraInfo 过期, 该帧作废")
            return False
        if (int(img_msg.width), int(img_msg.height)) != \
                (int(snap_info.width), int(snap_info.height)):
            print(f"!! 图像 {img_msg.width}x{img_msg.height} 与 CameraInfo "
                  f"{snap_info.width}x{snap_info.height} 尺寸不一致, 该帧作废")
            return False

        try:
            bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as exc:
            print(f"!! 图像转换失败: {exc} !!")
            return False
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        obs = detect_board(gray, self.board, self.detector)
        if obs is None:
            print("!! 未检出标定板, 该帧作废 !!")
            return False
        if obs["n_corners"] < self.min_corners:
            print(f"!! 检出角点 {obs['n_corners']} < {self.min_corners}, 该帧作废 !!")
            return False

        obj_points, img_points = self.board.matchImagePoints(obs["corners_raw"], obs["ids_raw"])
        if obj_points is None or img_points is None or len(obj_points) < 4:
            print("!! 物点/像点匹配失败, 该帧作废 !!")
            return False

        # 该帧覆盖 cells (回退后按保留帧重算, 唯一覆盖帧撤回则该格重新判为未覆盖)
        cells = set()
        for x, y in obs["corners"]:
            col = min(GRID - 1, max(0, int(x / max(1, img_msg.width) * GRID)))
            row = min(GRID - 1, max(0, int(y / max(1, img_msg.height) * GRID)))
            cells.add((row, col))

        self.frames.append({
            "img": f"f{len(self.frames):03d}.png",
            "corners": int(obs["n_corners"]),
            "markers": int(obs["n_markers"]),
            "img_seq": int(seq),
            "age_s": round(float(age), 4),
            "cells": sorted([int(r) * GRID + int(c) for r, c in cells]),
            "resolution": [int(img_msg.width), int(img_msg.height)],
            "camera_id": self.camera_id,
        })
        folder = os.path.dirname(os.path.abspath(self.out_yaml))
        os.makedirs(folder, exist_ok=True)
        image_path = os.path.join(folder, self.frames[-1]["img"])
        if not cv2.imwrite(image_path, bgr):
            self.frames.pop()
            print("!! 原始图像保存失败，不计入采样")
            return False
        self.object_points.append(np.asarray(obj_points, dtype=np.float32).reshape(-1, 1, 3))
        self.image_points.append(np.asarray(img_points, dtype=np.float32).reshape(-1, 1, 2))
        self.corners_per_frame.append(obs["n_corners"])
        self.covered_cells = self._recompute_coverage()
        self.write_frames_json()
        self.last_sampled_img_seq = seq

        index = len(self.object_points) - 1
        print(f"--- 采集成功 #{index}: 角点 {obs['n_corners']} "
              f"标记 {obs['n_markers']} 累计 {len(self.object_points)} 帧 ---")
        self.print_coverage()
        return True

    def _recompute_coverage(self):
        """按保留帧重算覆盖网格。

        回退/撤回一帧后必须重算: 若某格仅由此帧覆盖, 撤回后该格重新判为未覆盖
        (计划 5.3 "d 回退后按保留帧重新计算")。
        """
        cells = set()
        for f in self.frames:
            for packed in f["cells"]:
                cells.add((int(packed) // GRID, int(packed) % GRID))
        return cells

    def write_frames_json(self):
        folder = os.path.dirname(os.path.abspath(self.out_yaml))
        with open(os.path.join(folder, "frames.json"), "w") as stream:
            json.dump({"camera_id": self.camera_id, "image_topic": self.image_topic,
                       "camera_info_topic": self.camera_info_topic,
                       "frames": self.frames}, stream, indent=2)

    def print_coverage(self):
        missing = REQUIRED_CELLS - self.covered_cells
        print(f"  画面覆盖: {len(self.covered_cells)}/{GRID * GRID} 格")
        if missing:
            print(f"  !! 尚未覆盖 (行,列): {sorted(missing)} —— 请把标定板推到画面"
                  f"{'四角与中心' if len(missing) > 2 else '缺失区域'} (畸变约束全靠这些帧)")

    def compute_and_save(self):
        """求解内参并写入**候选** YAML（计划 5.2/5.3）: 质量门 + 元数据，不直接批准。"""
        if len(self.object_points) < self.min_frames:
            print(f"!! 当前仅 {len(self.object_points)} 帧 < {self.min_frames} 帧 !!")
            return
        if self.camera_info is None:
            print("!! 缺 camera_info, 无法确定标定用的图像尺寸 !!")
            return

        size = (int(self.camera_info.width), int(self.camera_info.height))
        print(f"\n开始求解内参 ({len(self.object_points)} 帧, 分辨率 {size[0]}x{size[1]}) ...")
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.object_points, self.image_points, size, None, None)

        print("内参矩阵 K:")
        print(np.array2string(camera_matrix, precision=6, suppress_small=False))
        print(f"畸变系数: {np.round(dist_coeffs.ravel(), 6)}")
        print(f"重投影误差 RMS: {rms:.4f} px")

        # ── 内参批准门 (计划 5.3) ──
        checks = []

        def _chk(name, ok, detail=""):
            checks.append({"check": name, "ok": bool(ok), "detail": detail})

        _chk("有效帧数 >= min_frames", len(self.object_points) >= self.min_frames,
             f"{len(self.object_points)}/{self.min_frames}")
        _chk("整体重投影 RMS < max_reproj_rms",
             np.isfinite(rms) and rms < self.max_reproj_rms,
             f"{rms:.4f}px < {self.max_reproj_rms}px")
        missing = REQUIRED_CELLS - self.covered_cells
        _chk("3x3 覆盖网格(中心+四角)", not missing,
             f"覆盖 {len(self.covered_cells)}/9, 缺 {sorted(missing)}")
        bad_sizes = {tuple(f["resolution"]) for f in self.frames} - {size}
        _chk("图像/CameraInfo/输出尺寸一致", not bad_sizes,
             f"不一致尺寸: {sorted(bad_sizes)}" if bad_sizes else "一致")
        finite = (np.isfinite(camera_matrix).all() and np.isfinite(dist_coeffs).all())
        _chk("内参/畸变系数为有限数值", bool(finite), "")
        status = "passed" if all(c["ok"] for c in checks) else "failed"

        metadata = {
            "kind": "intrinsics_candidate",
            "camera_id": self.camera_id,
            "camera_optical_frame": camera_optical_frame(self.camera_id),
            "source": "charuco_calibration",
            "resolution": [int(size[0]), int(size[1])],
            "board": {"squares": [self.squares_x, self.squares_y],
                      "square_m": self.square_length,
                      "marker_m": self.marker_length,
                      "dict": self.aruco_dict,
                      "required_cells": sorted([r * GRID + c for r, c in REQUIRED_CELLS])},
            "valid_frames": len(self.object_points),
            "coverage_cells": sorted([r * GRID + c for r, c in self.covered_cells]),
            "quality_status": status,
            "quality_checks": checks,
            "approved": False,
            "note": ("候选内参；文件生成不等于达标；须 approve-intrinsics 批准后才供手眼标定引用"),
        }
        save_intrinsics(self.out_yaml, camera_matrix, dist_coeffs, size[0], size[1],
                        rms, len(self.object_points), len(self.covered_cells),
                        {"squares_x": self.squares_x, "squares_y": self.squares_y,
                         "square_m": self.square_length, "marker_m": self.marker_length,
                         "dict": self.aruco_dict},
                        metadata=metadata)
        print(f"已写入候选: {self.out_yaml}")
        print(f"质量状态: {status}")
        for c in checks:
            mark = "OK " if c["ok"] else "NG "
            print(f"  [{mark}] {c['check']} ({c['detail']})")
        print("提示: 这是候选内参, 不自动成为生产内参; 需 approve-intrinsics 批准。")
        print("      批准后手眼标定节点在 intrinsics_yaml 指向该文件时加载使用。")

    def run(self):
        count = 1
        while True:
            print(f"\n------- 第{count}次采集 ------")
            if count <= self.min_frames:
                menu_str = "请输入(Enter-采集, d-回退):"
            else:
                menu_str = "请输入(Enter-采集, d-回退, q-计算并存盘, c-退出):"
            try:
                clear_input_buffer()
                user_input = input(menu_str + " ")
                if user_input == '':
                    if self.capture_frame():
                        count += 1
                elif count > 1 and user_input == 'd':
                    count -= 1
                    self.object_points.pop()
                    self.image_points.pop()
                    self.corners_per_frame.pop()
                    if self.frames:
                        frame = self.frames.pop()
                        image_path = os.path.join(os.path.dirname(os.path.abspath(self.out_yaml)),
                                                  frame["img"])
                        if os.path.isfile(image_path):
                            os.remove(image_path)
                    self.covered_cells = self._recompute_coverage()
                    self.write_frames_json()
                    print("已回退上一帧 (覆盖统计已按保留帧重算)")
                elif user_input == 'q' and len(self.object_points) >= self.min_frames:
                    self.compute_and_save()
                    break
                elif user_input == 'c':
                    print("exit")
                    exit(0)
                else:
                    print("无效输入")
            except Exception as e:
                print(f"\n程序异常或中断退出: {e}")
                exit(-1)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraIntrinsicCalibrationNode()
        node.run()
    finally:
        if node is not None:
            node.stop_spin()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
