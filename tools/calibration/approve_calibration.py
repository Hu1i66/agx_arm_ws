#!/usr/bin/env python3
"""候选标定批准工具（计划 5.2）：把**通过质量门**的候选推广为活动标定。

用法（经统一入口）:
  ./scripts/run_component.sh approve-intrinsics --camera d455 --candidate /abs/候选.yaml
  ./scripts/run_component.sh approve-handeye    --camera d455 --candidate /abs/候选.json --scene-verified

规则:
  - 推广要求相机、模式、帧端点、分辨率、质量和来源完整；任缺一项拒绝。
  - 先保存旧批准文件及 manifest 到 approved_backup/，再**原子替换**并追加审计记录。
  - --scene-verified 仅由已完成现场坐标核查的操作者显式使用；工具不自动追加。
  - Dabai 推广只保存外参，不启用抓取、不发布基座 TF。
  - 已有运行节点不会自动重载：批准后需先结束任务再重启受影响的检测/GraspNet/TF。
"""
from __future__ import annotations

import argparse
import datetime
import fcntl
import uuid
import hashlib
import json
import os
import pathlib
import shutil
import sys

import yaml

_SOURCE_WS = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SOURCE_WS / "src" / "handeye_calibration_ros"))
from handeye_calibration_ros.validation import (
    CalibrationValidationError, validate_intrinsics, validate_handeye,
)

# 支持 AGX_ARM_WS 覆盖（统一入口导出；便于离线/测试在临时工作空间校验）
WS = pathlib.Path(os.environ.get("AGX_ARM_WS")
                  or pathlib.Path(__file__).resolve().parents[2]).expanduser().resolve()
CAL_DIR = WS / "config" / "calibration"
MANIFEST = CAL_DIR / "manifest.json"
BACKUP_DIR = CAL_DIR / "approved_backup"

# 活动外参文件与期望模式（与 config/calibration/manifest.json 一致）
HANDEYE_TARGET = {
    "d455": ("d455_eye_to_hand.json", "eye_to_hand"),
    "dabai": ("dabai_eye_in_hand.json", "eye_in_hand"),
}
INTRINSICS_TARGET = {
    "d455": "d455/intrinsics.yaml",
    "dabai": "dabai/intrinsics.yaml",
}


class ApproveError(Exception):
    pass


def _sha256(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def _stamp() -> str:
    return datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def _load_manifest() -> dict:
    with open(MANIFEST, "r", encoding="utf-8") as f:
        return json.load(f)


def _save_old_and_write(payload_writer, target: pathlib.Path):
    """每次推广唯一目录，保证同秒/两相机操作也能独立恢复。"""
    backup = BACKUP_DIR / (_stamp() + "_" + uuid.uuid4().hex)
    backup.mkdir(parents=True)
    old_target_backup = None
    if target.is_file():
        old_target_backup = backup / target.name
        shutil.copy2(target, old_target_backup)
    manifest_backup = backup / "manifest.json"
    shutil.copy2(MANIFEST, manifest_backup)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(target.suffix + ".tmp")
    payload_writer(tmp)
    os.replace(tmp, target)
    return old_target_backup, manifest_backup


def _write_manifest(manifest: dict):
    tmp = MANIFEST.with_suffix(".json.tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    os.replace(tmp, MANIFEST)


def _require(cond, msg):
    if not cond:
        raise ApproveError(msg)


def _promote(camera, target, writer, audit, updates):
    """锁覆盖读取/备份/替换/审计，manifest失败时恢复原活动文件。"""
    CAL_DIR.mkdir(parents=True, exist_ok=True)
    with open(CAL_DIR / ".promotion.lock", "a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        manifest = _load_manifest()
        _require(camera in manifest.get("cameras", {}), "manifest缺少相机登记")
        entry = manifest["cameras"][camera]
        old, old_manifest = _save_old_and_write(writer, target)
        audit.update(at=_stamp(), camera=camera, target=str(target.relative_to(WS)),
                     target_sha256=_sha256(target),
                     old_target_backup=str(old.relative_to(WS)) if old else None,
                     manifest_backup=str(old_manifest.relative_to(WS)))
        entry.update(updates)
        entry.setdefault("promotion_log", []).append(audit)
        try:
            _write_manifest(manifest)
        except Exception:
            if old:
                restore = target.with_suffix(target.suffix + ".restore")
                shutil.copy2(old, restore)
                os.replace(restore, target)
            else:
                target.unlink()
            raise
    return {"target": str(target), "sha256": audit["target_sha256"], "entry": audit}


def approve_intrinsics(camera: str, candidate: pathlib.Path) -> dict:
    _require(candidate.is_file(), f"候选文件不存在: {candidate}")
    with open(candidate, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    try:
        meta = validate_intrinsics(data, camera)
    except CalibrationValidationError as exc:
        raise ApproveError(str(exc)) from exc
    target = CAL_DIR / INTRINSICS_TARGET[camera]
    data["candidate"].update(approved=True, approved_at=_stamp())
    def writer(path):
        path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    audit = {"kind": "intrinsics", "candidate": str(candidate),
             "valid_frames": meta["valid_frames"], "resolution": meta["resolution"],
             "reprojection_error": data["reprojection_error"],
             "coverage_cells": meta["coverage_cells"], "scene_verified": False}
    return _promote(camera, target, writer, audit,
                    {"approved_intrinsics": str(target.relative_to(WS))})


def approve_handeye(camera: str, candidate: pathlib.Path, scene_verified: bool) -> dict:
    _require(scene_verified, "手眼推广必须显式 --scene-verified")
    _require(candidate.is_file(), f"候选文件不存在: {candidate}")
    with open(candidate, encoding="utf-8") as stream:
        data = json.load(stream)
    try:
        meta = validate_handeye(data, camera)
    except CalibrationValidationError as exc:
        raise ApproveError(str(exc)) from exc
    filename, _ = HANDEYE_TARGET[camera]
    target = CAL_DIR / filename
    active = {"position": data["position"], "orientation": data["orientation"],
              "rpy": data.get("rpy", [[0., 0., 0.]])}
    def writer(path):
        path.write_text(json.dumps(active, ensure_ascii=False, indent=4,
                                   allow_nan=False), encoding="utf-8")
    audit = {"kind": "handeye", "candidate": str(candidate), "scene_verified": True,
             "mode": meta["mode"], "frames_used": meta["frames_used"],
             "resolution": meta["resolution"], "intrinsics_source": meta["intrinsics_source"],
             "holdout": data["criteria"]["holdout"], "tcp_offset": meta["tcp_offset"],
             "robot_pose_frame": meta["robot_pose_frame"],
             "transform_parent": meta["transform_parent"],
             "transform_child": meta["transform_child"]}
    return _promote(camera, target, writer, audit,
                    {"active_file": filename, "status": "approved",
                     "status_note": "候选通过独立质量门禁，并由操作者完成现场坐标核查",
                     "parent": meta["transform_parent"], "child": meta["transform_child"],
                     "tcp_offset": meta["tcp_offset"], "source": str(candidate)})


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="候选标定批准工具")
    parser.add_argument("kind", choices=["intrinsics", "handeye"])
    parser.add_argument("--camera", required=True, choices=["d455", "dabai"])
    parser.add_argument("--candidate", required=True, help="候选文件绝对路径")
    parser.add_argument("--scene-verified", action="store_true",
                        help="手眼: 已由操作者完成现场坐标核查（必须显式）")
    args = parser.parse_args(argv)
    cand = pathlib.Path(os.path.expanduser(args.candidate))
    if not cand.is_absolute():
        cand = WS / cand
    try:
        if args.kind == "intrinsics":
            res = approve_intrinsics(args.camera, cand)
        else:
            res = approve_handeye(args.camera, cand, args.scene_verified)
    except ApproveError as e:
        print(f"[approve][ERROR] {e}", file=sys.stderr)
        return 2
    print(json.dumps(res, ensure_ascii=False, indent=2))
    print("\n提醒: 已有运行节点不会自动重载；请先结束任务, 再重启受影响的"
          "检测/GraspNet/TF 并重新核对完整矩阵。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
