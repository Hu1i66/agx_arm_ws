#!/usr/bin/env python3
"""Convert the active Piper URDF visual meshes into link-local GLB assets."""
from __future__ import annotations

import json
import pathlib
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]
URDF = ROOT / "src/agx_arm_ros/src/agx_arm_moveit/config/test2.urdf"
PACKAGE_ROOT = ROOT / "src/agx_arm_ros/src/agx_arm_description"
OUTPUT = ROOT / "tablet_hmi/public/models"


def mesh_path(filename):
    prefix = "package://agx_arm_description/"
    if filename.startswith(prefix):
        return PACKAGE_ROOT / filename[len(prefix):]
    raise ValueError(f"不支持的 mesh URI: {filename}")


def export_link(assimp, source, target):
    result = subprocess.run(
        [str(assimp), "export", str(source), str(target), "-fglb2"],
        capture_output=True, text=True,
    )
    if result.returncode or not target.is_file():
        raise RuntimeError((result.stderr or result.stdout).strip()[-800:])


def main():
    assimp = pathlib.Path.home() / ".local/bin/assimp"
    if not assimp.is_file():
        assimp = pathlib.Path(shutil.which("assimp") or "")
    if not assimp.is_file():
        raise SystemExit("未找到 assimp；请先安装 assimp-utils 或把它加入 PATH")
    root = ET.parse(URDF).getroot()
    links = {}
    joints = []
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        if parent is None or child is None:
            continue
        xyz = [float(value) for value in (origin.get("xyz", "0 0 0").split() if origin is not None else "0 0 0".split())]
        rpy = [float(value) for value in (origin.get("rpy", "0 0 0").split() if origin is not None else "0 0 0".split())]
        joints.append({
            "name": joint.get("name"),
            "type": joint.get("type"),
            "parent": parent.get("link"),
            "child": child.get("link"),
            "xyz": xyz,
            "rpy": rpy,
            "axis": [float(value) for value in (axis.get("xyz", "0 0 1").split() if axis is not None else "0 0 1".split())],
            "limit": None if limit is None else {
                "lower": float(limit.get("lower", 0)),
                "upper": float(limit.get("upper", 0)),
            },
        })
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for link in root.findall("link"):
        visual = link.find("visual")
        if visual is None:
            continue
        geometry = visual.find("geometry")
        mesh = None if geometry is None else geometry.find("mesh")
        if mesh is None:
            continue
        source = mesh_path(mesh.get("filename", ""))
        name = link.get("name")
        target = OUTPUT / f"{name}.glb"
        export_link(assimp, source, target)
        links[name] = {
            "url": f"/models/{target.name}",
            "source": str(source.relative_to(ROOT)),
            "bytes": target.stat().st_size,
        }
    manifest = {
        "schema_version": 1,
        "base_frame": "base_link",
        "source_urdf": str(URDF.relative_to(ROOT)),
        "links": links,
        "joints": joints,
    }
    (OUTPUT / "model_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    total = sum(row["bytes"] for row in links.values())
    print(f"[tablet-model] {len(links)} links, {total / 1024 / 1024:.1f} MiB -> {OUTPUT}")


if __name__ == "__main__":
    main()
