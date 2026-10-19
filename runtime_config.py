#!/usr/bin/env python3
"""共享运行时配置模块（仅标准库）。

职责：
  - 根目录定位（按脚本位置，兼容 AGX_ARM_WS 环境变量，不按当前 cwd 解析）
  - config/runtime.json 读取
  - 命名资源键解析（项目内相对 workspace_root；外部绝对路径支持 expanduser）
  - 活动外参合法性校验（3 位位置、4 位 xyzw、有限数值、非零四元数、相机/方向）

约束：本模块只做 stdlib 工作；导入不启动节点、不加载模型、不访问硬件。
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import sys

# config/runtime.json -> paths 段的命名键（计划 3.1 固定）
RESOURCE_KEYS = (
    "vision_python",
    "system_python",
    "tablet_python",
    "ros_setup",
    "workspace_setup",
    "orbbec_setup",
    "dds_xml",
    "yolo_weights",
    "graspnet_checkpoint",
    "graspnet_baseline",
    "d455_handeye",
    "ik_site_packages",
    "cmeel_site_packages",
    "ik_urdf",
    "saved_poses",
    "conveyor_calib",
)

_HANDEYE_MODES = ("eye_to_hand", "eye_in_hand")


class ConfigError(Exception):
    """配置缺失或非法。明确失败，不静默回退。"""


def workspace_root() -> pathlib.Path:
    """主工作空间根目录。

    优先 AGX_ARM_WS 环境变量；否则按本文件所在位置确定。
    不依赖当前工作目录。
    """
    env = os.environ.get("AGX_ARM_WS")
    if env:
        return pathlib.Path(env).expanduser().resolve()
    return pathlib.Path(__file__).resolve().parent


def load_runtime_config() -> dict:
    """读取并做基本结构校验，返回 config/runtime.json 的内容。"""
    path = workspace_root() / "config" / "runtime.json"
    if not path.is_file():
        raise ConfigError(f"运行时配置不存在: {path}")
    try:
        with open(path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    except json.JSONDecodeError as e:
        raise ConfigError(f"运行时配置不是合法 JSON: {path}: {e}") from e
    if not isinstance(cfg, dict):
        raise ConfigError(f"运行时配置顶层必须是对象: {path}")
    paths = cfg.get("paths")
    if not isinstance(paths, dict):
        raise ConfigError(f"运行时配置缺少 paths 对象: {path}")
    missing = [k for k in RESOURCE_KEYS if k not in paths]
    if missing:
        raise ConfigError(f"运行时配置缺少命名资源键: {', '.join(missing)}")
    return cfg


def resolve_resource(key: str) -> pathlib.Path:
    """解析命名资源键为绝对路径。

    项目内路径相对 workspace_root；外部路径用绝对路径并支持 expanduser。
    """
    if key not in RESOURCE_KEYS:
        raise ConfigError(f"未知资源键: {key!r}（可用键见 runtime_config.RESOURCE_KEYS）")
    cfg = load_runtime_config()
    raw = cfg["paths"][key]
    if not isinstance(raw, str) or not raw.strip():
        raise ConfigError(f"资源键 {key} 的路径为空")
    p = pathlib.Path(os.path.expanduser(raw))
    if not p.is_absolute():
        p = workspace_root() / p
    return p


def _finite_vec(values, n, name):
    if not isinstance(values, (list, tuple)) or len(values) != n:
        raise ConfigError(f"{name} 必须是长度 {n} 的数组, 实际: {values!r}")
    out = []
    for v in values:
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ConfigError(f"{name} 含非数值元素: {values!r}")
        v = float(v)
        if not math.isfinite(v):
            raise ConfigError(f"{name} 含非有限数值 (NaN/Inf): {values!r}")
        out.append(v)
    return out


def _calibration_manifest() -> dict:
    path = workspace_root() / "config" / "calibration" / "manifest.json"
    if not path.is_file():
        raise ConfigError(f"标定 manifest 不存在: {path}")
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except json.JSONDecodeError as e:
        raise ConfigError(f"标定 manifest 不是合法 JSON: {path}: {e}") from e


def load_handeye(camera: str, calibration_file: str | None = None) -> dict:
    """读取并校验指定相机的活动外参。

    返回 dict: position(3), orientation(4, xyzw), mode, parent, child,
               quaternion_normalized(4), source, raw。
    语义 (mode/parent/child/active_file) 来自 config/calibration/manifest.json；
    calibration_file 显式覆盖用于离线/候选检查。
    缺失/非法明确失败, 不静默回退单位矩阵; 不重写原 JSON。
    """
    if not isinstance(camera, str) or not camera:
        raise ConfigError("camera 必须是非空字符串")
    manifest = _calibration_manifest()
    cameras = manifest.get("cameras")
    if not isinstance(cameras, dict) or camera not in cameras:
        raise ConfigError(f"标定 manifest 中没有相机 {camera!r} 的登记")
    entry = cameras[camera]
    if not isinstance(entry, dict):
        raise ConfigError(f"标定 manifest 中相机 {camera!r} 登记不是对象")

    if calibration_file:
        src = pathlib.Path(os.path.expanduser(calibration_file))
        if not src.is_absolute():
            src = workspace_root() / src
    else:
        active = entry.get("active_file")
        if not active:
            raise ConfigError(
                f"相机 {camera} 没有活动外参文件（尚未批准任何候选，不生成假结果）")
        src = workspace_root() / "config" / "calibration" / str(active)
    if not src.is_file():
        raise ConfigError(f"外参文件不存在: {src}")

    try:
        with open(src, "r", encoding="utf-8") as f:
            data = json.load(f)
    except json.JSONDecodeError as e:
        raise ConfigError(f"外参文件不是合法 JSON: {src}: {e}") from e
    if not isinstance(data, dict):
        raise ConfigError(f"外参文件顶层必须是对象: {src}")

    if "position" not in data or "orientation" not in data:
        raise ConfigError(f"外参文件缺少 position/orientation: {src}")
    position = _finite_vec(data["position"], 3, f"{src} position")
    orientation = _finite_vec(data["orientation"], 4, f"{src} orientation")
    norm = math.sqrt(sum(q * q for q in orientation))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ConfigError(f"外参四元数为零: {src}")

    mode = entry.get("mode")
    if mode not in _HANDEYE_MODES:
        raise ConfigError(
            f"相机 {camera} 的 mode 非法: {mode!r}（允许 {_HANDEYE_MODES}）")
    parent = entry.get("parent")
    child = entry.get("child")
    if not isinstance(parent, str) or not parent:
        raise ConfigError(f"相机 {camera} 的 parent frame 缺失")
    if not isinstance(child, str) or not child:
        raise ConfigError(f"相机 {camera} 的 child frame 缺失")
    expected_mode = {"d455": "eye_to_hand", "dabai": "eye_in_hand"}.get(camera)
    if expected_mode and mode != expected_mode:
        raise ConfigError(f"相机 {camera} 的标定模式必须为 {expected_mode}")
    metadata = data.get("metadata")
    if metadata is not None:
        if not isinstance(metadata, dict):
            raise ConfigError(f"外参 metadata 非法: {src}")
        if metadata.get("camera_id") != camera or metadata.get("mode") != mode:
            raise ConfigError(f"外参候选的相机/模式与请求 {camera}/{mode} 不符: {src}")
        if camera == "dabai" and "transform_parent" in metadata:
            offset = _finite_vec(metadata.get("tcp_offset"), 6, "tcp_offset")
            tool = "tcp_link" if any(offset) else "link6"
            if (metadata["transform_parent"] != tool
                    or metadata.get("robot_pose_frame") != tool):
                raise ConfigError(f"候选TCP端点与offset不符: {src}")
            parent = tool
        elif "transform_parent" in metadata and metadata["transform_parent"] != parent:
            raise ConfigError(f"候选transform_parent与坐标语义不符: {src}")
        if "transform_child" in metadata and metadata["transform_child"] != child:
            raise ConfigError(f"候选transform_child与坐标语义不符: {src}")
    if camera == "d455" and (parent != "base_link" or child != "camera_color_optical_frame"):
        raise ConfigError("D455必须为base_link→camera_color_optical_frame")
    if camera == "dabai" and (parent not in ("link6", "tcp_link")
                             or child != "dabai_color_optical_frame"):
        raise ConfigError("Dabai必须相对实际TCP→dabai_color_optical_frame")

    return {
        "camera": camera,
        "position": position,
        "orientation": orientation,
        "quaternion_normalized": [q / norm for q in orientation],
        "mode": mode,
        "parent": parent,
        "child": child,
        "source": str(src),
        "raw": data,
    }


def _main(argv):
    import argparse

    parser = argparse.ArgumentParser(description="共享运行时配置只读工具")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("root", help="打印主工作空间根目录")
    p_show = sub.add_parser("show", help="打印运行时配置（可限定资源键）")
    p_show.add_argument("--key", default=None, help="只打印该命名资源键解析结果")
    p_hand = sub.add_parser("handeye", help="打印并校验活动外参")
    p_hand.add_argument("camera")
    p_hand.add_argument("--calibration-file", default=None)
    args = parser.parse_args(argv)

    if args.cmd == "root":
        print(workspace_root())
        return 0
    if args.cmd == "show":
        if args.key:
            print(resolve_resource(args.key))
        else:
            json.dump(load_runtime_config(), sys.stdout, ensure_ascii=False, indent=2)
            sys.stdout.write("\n")
        return 0
    if args.cmd == "handeye":
        h = load_handeye(args.camera, args.calibration_file)
        json.dump({k: v for k, v in h.items() if k != "raw"},
                  sys.stdout, ensure_ascii=False, indent=2)
        sys.stdout.write("\n")
        return 0
    return 1


if __name__ == "__main__":
    try:
        sys.exit(_main(sys.argv[1:]))
    except ConfigError as e:
        print(f"[runtime_config][ERROR] {e}", file=sys.stderr)
        sys.exit(2)
