"""runtime_process_manager 测试：锁、登记、身份校验、停止、回收。

使用模拟进程（sleep/短命 python），不启动 arm 或串口设备。
"""
import os
import pathlib
import signal
import sys
import time

import pytest

WS = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WS))
from runtime_process_manager import (  # noqa: E402
    FileLock, ProcessManager, _alive, _parse_stat,
)


@pytest.fixture()
def mgr(tmp_path):
    return ProcessManager(tmp_path, state_dir_name=".runtime")


def test_filelock_exclusive(tmp_path):
    lock = FileLock(tmp_path / "l.lock")
    assert lock.acquire(timeout=1.0)
    other = FileLock(tmp_path / "l.lock")
    assert other.acquire(timeout=0.3) is False
    lock.release()
    assert other.acquire(timeout=1.0)
    other.release()


def test_spawn_register_and_identity(mgr):
    rec = mgr.spawn("testcomp", ["/bin/sleep", "30"], initiator="manual",
                    log_file=mgr.workspace / "log" / "t.log")
    try:
        assert rec["pid"] > 1
        assert rec["pgid"] == rec["pid"]           # start_new_session → 组长=子PID
        assert rec["uid"] == os.getuid()
        assert rec["boot_id"]
        assert rec["proc_starttime"] is not None
        assert rec["record_id"]
        ok, reason = mgr.verify_identity(rec)
        assert ok, reason
        assert mgr.get_record("testcomp")["record_id"] == rec["record_id"]
    finally:
        mgr.stop("testcomp")
    assert mgr.get_record("testcomp") is None


def test_stop_not_registered(mgr):
    res = mgr.stop("nothing")
    assert res["result"] == "not_registered"


def test_stop_requires_owner(mgr):
    mgr.spawn("testcomp", ["/bin/sleep", "30"], initiator="manual")
    try:
        res = mgr.stop("testcomp", require_owner="gui")
        assert res["result"] == "not_owner"
        # 允许跨发起方（GUI 显式模式切换）
        res2 = mgr.stop("testcomp", require_owner="gui", allow_cross_initiator=True)
        assert res2["result"] in ("stopped", "incomplete_exit")
    finally:
        mgr.stop("testcomp")


def test_pid_reuse_protection_removes_stale_record(mgr):
    """starttime 不匹配 → 身份校验失败，清理登记但不杀未知进程。"""
    rec = mgr.spawn("testcomp", ["/bin/sleep", "30"], initiator="manual")
    rec["proc_starttime"] = int(rec["proc_starttime"]) + 12345  # 伪造
    mgr.register(rec)
    res = mgr.stop("testcomp")
    assert res["result"] == "stale_record_removed"
    assert mgr.get_record("testcomp") is None
    # 真实进程仍在（未被误杀），手动清理
    assert _alive(rec["pid"])
    os.killpg(rec["pgid"], signal.SIGTERM)


def test_pgid_le_1_rejected(mgr):
    rec = mgr.spawn("testcomp", ["/bin/sleep", "30"], initiator="manual")
    rec["pgid"] = 1
    mgr.register(rec)
    res = mgr.stop("testcomp")
    assert res["result"] == "stale_record_removed"
    assert "PGID" in res["detail"]
    os.killpg(rec["pid"], signal.SIGTERM)


def test_unregister_record_id_protection(mgr):
    rec = mgr.spawn("testcomp", ["/bin/sleep", "30"], initiator="manual")
    assert mgr.unregister("testcomp", "wrong-id") is False
    assert mgr.get_record("testcomp") is not None
    assert mgr.unregister("testcomp", rec["record_id"]) is True
    os.killpg(rec["pgid"], signal.SIGTERM)


def test_boot_id_mismatch(mgr):
    rec = mgr.spawn("testcomp", ["/bin/sleep", "30"], initiator="manual")
    rec["boot_id"] = "not-the-real-boot-id"
    mgr.register(rec)
    ok, reason = mgr.verify_identity(rec)
    assert not ok and "boot_id" in reason
    mgr.unregister("testcomp", rec["record_id"])
    os.killpg(rec["pgid"], signal.SIGTERM)


def test_incomplete_exit_when_child_survives(mgr):
    """leader 退出但组内子孙仍存活 → 报告未完全退出并保留登记。"""
    # 子进程忽略 SIGINT，leader(bash) 立即退出
    rec = mgr.spawn("testcomp",
                    ["/bin/bash", "-c", "trap '' INT; sleep 30 & exit 0"],
                    initiator="manual")
    time.sleep(0.8)
    assert not _alive(rec["pid"])                    # leader 已退出
    res = mgr.stop("testcomp")
    # leader 已退出但子孙存活 → 报告未完全退出，保留登记
    assert res["result"] == "incomplete_exit", res
    assert res["alive_members"]
    assert mgr.get_record("testcomp") is not None
    assert mgr.group_alive("testcomp") is True
    # 清理子孙
    os.killpg(rec["pgid"], signal.SIGTERM)
    time.sleep(0.5)
    mgr.unregister("testcomp", rec["record_id"])


def test_group_alive_after_stop_is_false(mgr):
    mgr.spawn("testcomp", ["/bin/sleep", "30"], initiator="manual")
    res = mgr.stop("testcomp")
    assert res["result"] == "stopped"
    assert mgr.group_alive("testcomp") is False
    assert mgr.get_record("testcomp") is None


def test_registry_isolated_per_workspace(tmp_path):
    a = ProcessManager(tmp_path / "wa", state_dir_name=".runtime")
    b = ProcessManager(tmp_path / "wb", state_dir_name=".runtime")
    a.spawn("c1", ["/bin/sleep", "20"], initiator="manual")
    try:
        assert a.get_record("c1") is not None
        assert b.get_record("c1") is None
    finally:
        a.stop("c1")
