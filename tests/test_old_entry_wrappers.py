"""旧入口薄包装测试（计划 4.3）：调用 canonical 入口、保留 argv/退出码/信号、不递归。"""
import pathlib
import subprocess
import sys

WS = pathlib.Path(__file__).resolve().parent.parent
ORANGE = pathlib.Path("/home/lxf/orange_dataset")

WRAPPERS = {
    "realsense_yolo_node.py": str(WS / "vision" / "realsense_yolo_node.py"),
    "blue_block_detector.py": str(WS / "vision" / "blue_block_detector.py"),
}


def test_wrappers_and_canonical_exist():
    for name, canonical in WRAPPERS.items():
        assert (ORANGE / name).is_file(), name
        assert pathlib.Path(canonical).is_file(), canonical


def test_wrapper_delegates_to_canonical_with_execv():
    for name, canonical in WRAPPERS.items():
        src = (ORANGE / name).read_text(encoding="utf-8")
        assert canonical in src
        # execv 行必须使用 _CANONICAL（真实现），不指向旧路径 → 不递归
        exec_lines = [l for l in src.splitlines() if "os.execv(" in l]
        assert exec_lines and "_CANONICAL" in exec_lines[0]
        assert f'_CANONICAL = "{ORANGE}/{name}"' not in src


def test_wrapper_preserves_argv_exitcode_and_pid(tmp_path):
    """execv：argv 透传、退出码透传、PID 不变（信号直达真实现）。"""
    target = tmp_path / "canonical.py"
    target.write_text(
        "import os, sys\n"
        "print('ARGV=' + '|'.join(sys.argv[1:]))\n"
        "print('PID=%d' % os.getpid())\n"
        "sys.exit(7)\n", encoding="utf-8")
    src = (ORANGE / "realsense_yolo_node.py").read_text(encoding="utf-8")
    src = src.replace(
        '_CANONICAL = "/home/lxf/agx_arm_ws/vision/realsense_yolo_node.py"',
        f'_CANONICAL = "{target}"')
    wrapper = tmp_path / "wrapper.py"
    wrapper.write_text(src, encoding="utf-8")

    p = subprocess.Popen([sys.executable, str(wrapper), "--ros-args", "-p", "a:=1"],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out, err = p.communicate(timeout=30)
    assert p.returncode == 7, err          # 退出码透传
    assert "ARGV=--ros-args|-p|a:=1" in out  # argv 透传
    assert f"PID={p.pid}" in out             # PID 不变 → execv（无子进程）


def test_wrapper_missing_canonical_errors_clearly(tmp_path):
    src = (ORANGE / "blue_block_detector.py").read_text(encoding="utf-8")
    src = src.replace(
        '_CANONICAL = "/home/lxf/agx_arm_ws/vision/blue_block_detector.py"',
        f'_CANONICAL = "{tmp_path}/does_not_exist.py"')
    wrapper = tmp_path / "w.py"
    wrapper.write_text(src, encoding="utf-8")
    r = subprocess.run([sys.executable, str(wrapper)], capture_output=True, text=True)
    assert r.returncode == 127
    assert "canonical 脚本不存在" in r.stderr


def test_originals_preserved_as_orig():
    for name, _ in WRAPPERS.items():
        orig = ORANGE / f"{name}.orig"
        assert orig.is_file(), f"原始实现应保留为 {orig.name} 以便回退"
        assert len(orig.read_text(encoding="utf-8").splitlines()) > 500
