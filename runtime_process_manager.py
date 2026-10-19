#!/usr/bin/env python3
"""受控进程管理：组件登记、原子锁、身份校验、安全停止。

计划 3.6 要求：
  - .runtime/ 存本机登记；组件键如 camera:d455、camera:dabai、camera_tf:d455、detector:d455
  - 登记 managed_workspace、initiator(manual/gui)、owner_session、PID、PGID、UID、
    /proc starttime、boot_id、record_id、入口和模式
  - 短原子登记锁；每组件 transition 锁覆盖 stop/start/ready，不持全局锁跨长等待
  - Popen 使用 argv、start_new_session=True
  - 停止前逐项校验身份；SIGINT 等 5s，必要时 SIGTERM 等 5s，仍存活报告失败；不按文件名 pkill
  - 每次信号升级前重新校验身份；PGID 必须来自本工具创建的 session 且等于子 PID，
    拒绝 PGID≤1 或当前调用者进程组
  - 非父进程停止只轮询 /proc；原父进程负责 wait/reap
  - 旧监督进程退出只删除仍匹配 record_id 的登记
  - leader 退出但 launch 子孙仍占相机/话题时报告"未完全退出"

仅标准库；导入不启动节点、不访问硬件。
"""
from __future__ import annotations

import errno
import fcntl
import json
import os
import pathlib
import signal
import subprocess
import time
import uuid

SIGINT_WAIT_S = 5.0
SIGTERM_WAIT_S = 5.0


class ProcessError(Exception):
    """进程管理失败（身份不符、停止失败等）。明确报告，不静默忽略。"""


def _boot_id() -> str:
    try:
        with open("/proc/sys/kernel/random/boot_id", "r", encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return ""


def _parse_stat(pid: int):
    """返回 /proc/<pid>/stat 的关键字段 dict；进程不存在返回 None。"""
    try:
        with open(f"/proc/{pid}/stat", "r", encoding="utf-8") as f:
            data = f.read()
    except OSError:
        return None
    rparen = data.rfind(")")
    if rparen < 0:
        return None
    fields = data[rparen + 2:].split()
    # fields[0]=state, [1]=ppid, [2]=pgrp, [3]=session, ... [19]=starttime
    if len(fields) < 20:
        return None
    return {
        "state": fields[0],
        "ppid": int(fields[1]),
        "pgrp": int(fields[2]),
        "session": int(fields[3]),
        "starttime": int(fields[19]),
    }


def _proc_uid(pid: int):
    try:
        with open(f"/proc/{pid}/status", "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("Uid:"):
                    return int(line.split()[1])
    except OSError:
        return None
    return None


def _alive(pid: int) -> bool:
    st = _parse_stat(pid)
    if st is None:
        return False
    return st["state"] not in ("Z", "X", "x")


def _pgid_members(pgid: int):
    """仍在指定进程组内的 PID 列表（扫描 /proc）。"""
    members = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        st = _parse_stat(pid)
        if st is None:
            continue
        if st["pgrp"] == pgid and st["state"] not in ("Z", "X", "x"):
            members.append(pid)
    return members


class FileLock:
    """基于 flock 的短原子锁。"""

    def __init__(self, path: pathlib.Path):
        self.path = path
        self._fd = None

    def acquire(self, timeout: float = 10.0) -> bool:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fd = os.open(str(self.path), os.O_RDWR | os.O_CREAT, 0o644)
        deadline = time.monotonic() + timeout
        while True:
            try:
                fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return True
            except OSError as e:
                if e.errno not in (errno.EACCES, errno.EAGAIN):
                    raise
                if time.monotonic() >= deadline:
                    os.close(self._fd)
                    self._fd = None
                    return False
                time.sleep(0.05)

    def release(self):
        if self._fd is not None:
            try:
                fcntl.flock(self._fd, fcntl.LOCK_UN)
            finally:
                os.close(self._fd)
                self._fd = None

    def __enter__(self):
        if not self.acquire():
            raise ProcessError(f"获取锁超时: {self.path}")
        return self

    def __exit__(self, *exc):
        self.release()


class ProcessManager:
    """组件进程登记与受控停止。"""

    def __init__(self, workspace, state_dir_name: str = ".runtime"):
        self.workspace = pathlib.Path(workspace).expanduser().resolve()
        self.state_dir = self.workspace / state_dir_name
        self.locks_dir = self.state_dir / "locks"
        self.registry_path = self.state_dir / "registry.json"

    # ---------- 登记读写（短原子锁） ----------
    def read_registry(self) -> dict:
        if not self.registry_path.is_file():
            return {}
        try:
            with open(self.registry_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            return {}
        return data if isinstance(data, dict) else {}

    def _write_registry(self, reg: dict):
        self.state_dir.mkdir(parents=True, exist_ok=True)
        tmp = self.registry_path.with_suffix(".json.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(reg, f, ensure_ascii=False, indent=2)
        os.replace(tmp, self.registry_path)

    def transition_lock(self, component_key: str) -> FileLock:
        safe = component_key.replace(":", "_").replace("/", "_")
        return FileLock(self.locks_dir / f"{safe}.transition.lock")

    # ---------- 登记操作 ----------
    def register(self, record: dict) -> dict:
        component_key = record["component_key"]
        with FileLock(self.locks_dir / "registry.lock"):
            reg = self.read_registry()
            reg[component_key] = record
            self._write_registry(reg)
        return record

    def unregister(self, component_key: str, record_id: str) -> bool:
        """仅删除仍匹配 record_id 的登记；返回是否删除。"""
        with FileLock(self.locks_dir / "registry.lock"):
            reg = self.read_registry()
            cur = reg.get(component_key)
            if cur is None:
                return False
            if str(cur.get("record_id")) != str(record_id):
                return False
            del reg[component_key]
            self._write_registry(reg)
            return True

    def get_record(self, component_key: str):
        return self.read_registry().get(component_key)

    # ---------- 身份校验 ----------
    def verify_identity(self, record: dict):
        """逐项校验登记与真实进程身份。

        返回 (ok, reason)。校验：PID 存活、starttime、UID、boot_id、
        PGID 来自本工具创建的 session（== 子 PID、>1、非当前调用者进程组）。
        """
        if record.get("managed_workspace") != str(self.workspace):
            return False, "managed_workspace 不匹配"
        if any(record.get(k) is None for k in ("pid", "pgid", "uid", "proc_starttime")):
            return False, "进程身份字段缺失"
        if not record.get("boot_id"):
            return False, "boot_id 缺失"
        pid = int(record.get("pid", 0))
        if pid <= 1:
            return False, f"非法 PID: {pid}"
        st = _parse_stat(pid)
        if st is None:
            return False, f"进程不存在: PID {pid}"
        if not _alive(pid):
            return False, f"进程已非活动状态: PID {pid} state={st['state']}"
        rec_start = record.get("proc_starttime")
        if rec_start is not None and int(rec_start) != st["starttime"]:
            return False, (
                f"starttime 不匹配（PID 可能已复用）: 登记 {rec_start} 实际 {st['starttime']}")
        rec_uid = record.get("uid")
        actual_uid = _proc_uid(pid)
        if rec_uid is not None and actual_uid is not None and int(rec_uid) != actual_uid:
            return False, f"UID 不匹配: 登记 {rec_uid} 实际 {actual_uid}"
        rec_boot = record.get("boot_id")
        if rec_boot and rec_boot != _boot_id():
            return False, "boot_id 不匹配（重启后旧登记失效）"
        pgid = int(record.get("pgid", 0))
        if pgid <= 1:
            return False, f"拒绝 PGID≤1: {pgid}"
        try:
            caller_pgid = os.getpgrp()
        except OSError:
            caller_pgid = -1
        if pgid == caller_pgid:
            return False, f"拒绝停止当前调用者进程组: {pgid}"
        if pgid != pid:
            return False, f"PGID 必须等于子 PID（本工具创建的 session）: pgid={pgid} pid={pid}"
        if st["pgrp"] != pgid:
            return False, f"实际 PGID 与登记不符: 实际 {st['pgrp']} 登记 {pgid}"
        if st["session"] != pid:
            return False, "进程不是登记 session 的 leader"
        return True, "ok"

    # ---------- 启动 ----------
    def spawn(self, component_key: str, argv, *, initiator: str,
              mode=None, camera=None, log_file=None, env=None, cwd=None,
              stdin=None, owner_session=None):
        """以独立 session 启动组件并登记。

        argv 必须为列表（不字符串拼接）。返回登记记录。
        """
        argv = [str(a) for a in argv]
        if not argv:
            raise ProcessError("argv 不能为空")
        self.state_dir.mkdir(parents=True, exist_ok=True)
        log_path = None
        out = subprocess.DEVNULL
        if log_file:
            log_path = pathlib.Path(log_file)
            if not log_path.is_absolute():
                log_path = self.workspace / log_path
            log_path.parent.mkdir(parents=True, exist_ok=True)
            out = open(log_path, "ab", buffering=0)
        try:
            proc = subprocess.Popen(
                argv,
                stdout=out,
                stderr=subprocess.STDOUT,
                stdin=stdin if stdin is not None else subprocess.DEVNULL,
                cwd=str(cwd) if cwd else str(self.workspace),
                env=env if env is not None else os.environ.copy(),
                start_new_session=True,
            )
        finally:
            if log_path is not None:
                out.close()
        record = {
            "component_key": component_key,
            "managed_workspace": str(self.workspace),
            "initiator": initiator,
            "owner_session": owner_session or f"{initiator}:{os.getpid()}",
            "pid": proc.pid,
            "pgid": proc.pid,  # start_new_session=True → 子进程为 session/组 leader
            "uid": os.getuid(),
            "proc_starttime": (_parse_stat(proc.pid) or {}).get("starttime"),
            "boot_id": _boot_id(),
            "record_id": uuid.uuid4().hex,
            "entry": argv[0],
            "argv": argv,
            "mode": mode,
            "camera": camera,
            "started_at": time.time(),
            "log_file": str(log_path) if log_path else None,
        }
        self.register(record)
        return record

    # ---------- 停止 ----------
    def stop(self, component_key: str, *, require_owner: str | None = None,
             owner_session: str | None = None,
             allow_cross_initiator: bool = False,
             timeout_sigint: float = SIGINT_WAIT_S,
             timeout_sigterm: float = SIGTERM_WAIT_S) -> dict:
        """受控停止：身份校验 → SIGINT → SIGTERM → 报告失败。

        require_owner: 仅允许停止该 initiator 登记的组件（None 不限制）。
        allow_cross_initiator: GUI 显式模式切换时允许停止同 workspace 登记的
        manual 组件（旧 T3 监督终端随之结束）。
        非父进程只轮询 /proc，不 waitpid。leader 退出但组内仍有存活成员时
        报告"未完全退出"。
        """
        record = self.get_record(component_key)
        if record is None:
            return {"result": "not_registered", "component_key": component_key}
        if (owner_session is not None and not allow_cross_initiator
                and record.get("owner_session") != owner_session):
            return {"result": "not_owner", "component_key": component_key,
                    "detail": "组件不属于请求清理的 GUI 会话"}
        if (require_owner is not None and not allow_cross_initiator
                and record.get("initiator") != require_owner):
            return {
                "result": "not_owner",
                "component_key": component_key,
                "initiator": record.get("initiator"),
                "detail": "登记 initiator 与请求方不符，拒绝停止"
                          "（GUI 显式模式切换请用 --allow-cross-initiator）",
            }
        pid = int(record.get("pid", 0))
        record_id = record.get("record_id")
        pgid = int(record.get("pgid", 0))
        is_parent = (_parse_stat(pid) or {}).get("ppid") == os.getpid()

        leader_stat = _parse_stat(pid)
        leader_alive = leader_stat is not None and leader_stat["state"] not in ("Z", "X", "x")
        if not leader_alive:
            # leader 已退出：若组内仍有存活成员（launch 子孙可能仍占相机/话题），
            # 报告"未完全退出"并保留登记，避免删状态后重启第二实例
            try:
                caller_pgid = os.getpgrp()
            except OSError:
                caller_pgid = -1
            members = _pgid_members(pgid) if (pgid > 1 and pgid != caller_pgid) else []
            if members:
                return {
                    "result": "incomplete_exit",
                    "component_key": component_key,
                    "alive_members": members,
                    "detail": "leader 已退出但进程组仍有存活成员（可能仍占相机/话题），保留登记",
                }
            self.unregister(component_key, record_id)
            return {"result": "already_exited", "component_key": component_key,
                    "detail": "leader 已退出且组内无存活成员，已清理登记"}

        ok, reason = self.verify_identity(record)
        if not ok:
            # 身份不符（如 PID 复用/重启残留）→ 清理无效登记，不动未知进程
            self.unregister(component_key, record_id)
            return {"result": "stale_record_removed", "component_key": component_key,
                    "detail": reason}

        def _wait_gone(deadline_s: float) -> bool:
            deadline = time.monotonic() + deadline_s
            while time.monotonic() < deadline:
                if not _alive(pid):
                    return True
                time.sleep(0.1)
            return not _alive(pid)

        # SIGINT（升级前重新校验身份）
        ok, reason = self.verify_identity(record)
        if not ok:
            self.unregister(component_key, record_id)
            return {"result": "stale_record_removed", "component_key": component_key,
                    "detail": f"SIGINT 前身份复核失败: {reason}"}
        try:
            os.killpg(int(record["pgid"]), signal.SIGINT)
        except OSError as e:
            if e.errno != errno.ESRCH:
                raise
        if _wait_gone(timeout_sigint):
            result = self._finalize_stop(component_key, record, is_parent)
            if result["result"] == "incomplete_exit":
                return result
            return {"result": "stopped", "component_key": component_key,
                    "signal": "SIGINT"}

        # SIGTERM（升级前重新校验身份）
        ok, reason = self.verify_identity(record)
        if not ok:
            self.unregister(component_key, record_id)
            return {"result": "stale_record_removed", "component_key": component_key,
                    "detail": f"SIGTERM 前身份复核失败: {reason}"}
        try:
            os.killpg(int(record["pgid"]), signal.SIGTERM)
        except OSError as e:
            if e.errno != errno.ESRCH:
                raise
        if _wait_gone(timeout_sigterm):
            result = self._finalize_stop(component_key, record, is_parent)
            if result["result"] == "incomplete_exit":
                return result
            return {"result": "stopped", "component_key": component_key,
                    "signal": "SIGTERM"}

        return {
            "result": "stop_failed",
            "component_key": component_key,
            "pid": pid,
            "pgid": record.get("pgid"),
            "detail": "SIGINT/SIGTERM 后仍存活，未删除登记；请人工处理",
        }

    def stop_owned_group(self, component_key, *, timeout_sigint=SIGINT_WAIT_S, timeout_sigterm=SIGTERM_WAIT_S):
        """Finish a registered launch group even when its leader already exited.

        Every surviving PID must still belong to this boot/session/user/workspace.
        Signals target those checked PIDs, never a name match or an unowned group.
        """
        record=self.get_record(component_key)
        if record is None:return {'result':'not_registered','component_key':component_key}
        result=self.stop(component_key,timeout_sigint=timeout_sigint,timeout_sigterm=timeout_sigterm)
        if result['result'] not in ('incomplete_exit','stop_failed'):return result
        pgid=int(record.get('pgid',0));uid=int(record.get('uid',-1))
        if (record.get('managed_workspace')!=str(self.workspace) or record.get('boot_id')!=_boot_id()
                or pgid<=1 or pgid==os.getpgrp() or pgid!=record.get('pid') or uid!=os.getuid()):
            return dict(result,detail='残留进程组身份不匹配，未发送信号')
        last_signal=None
        for signum,timeout in [(signal.SIGINT,timeout_sigint),(signal.SIGTERM,timeout_sigterm),(signal.SIGKILL,2.)]:
            current=self.get_record(component_key)
            if not current or current.get('record_id')!=record.get('record_id'):return dict(result,detail='进程登记已改变，停止清理')
            members=_pgid_members(pgid);checked=[]
            for pid in members:
                st=_parse_stat(pid)
                try:
                    env=pathlib.Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
                    owner=('AGX_ARM_WS='+str(self.workspace)).encode() in env
                except OSError:owner=False
                if (not st or st['session']!=pgid or st['starttime']<int(record['proc_starttime']) or _proc_uid(pid)!=uid or not owner):
                    return dict(result,detail=f'残留PID {pid} 无法确认属于当前工作树，未强制终止')
                checked.append((pid,st['starttime']))
            for pid,start in checked:
                st=_parse_stat(pid)
                if not st or st['starttime']!=start:continue
                try:os.kill(pid,signum)
                except ProcessLookupError:pass
            last_signal=signal.Signals(signum).name
            deadline=time.monotonic()+timeout
            while _pgid_members(pgid) and time.monotonic()<deadline:time.sleep(.05)
            if not _pgid_members(pgid):
                self.unregister(component_key,record['record_id'])
                return dict(result='stopped',component_key=component_key,signal=last_signal,detail='已清理本工作树残留子进程')
        return dict(result='stop_failed',component_key=component_key,alive_members=_pgid_members(pgid),detail='已核验残留进程但无法退出')

    def _finalize_stop(self, component_key: str, record: dict, is_parent: bool):
        pid = int(record.get("pid", 0))
        pgid = int(record.get("pgid", 0))
        if is_parent:
            # 原父进程负责 wait/reap
            try:
                os.waitpid(pid, os.WNOHANG)
            except OSError:
                pass
        members = _pgid_members(pgid)
        if members:
            # leader 退出但 launch 子孙仍占资源：报告未完全退出，保留登记
            return {
                "result": "incomplete_exit",
                "component_key": component_key,
                "alive_members": members,
                "detail": "leader 已退出但进程组仍有存活成员（可能仍占相机/话题），保留登记",
            }
        self.unregister(component_key, record.get("record_id"))
        return {"result": "unregistered", "component_key": component_key}

    def reap_exit(self, component_key: str) -> dict | None:
        """监督进程退出路径：仅删除仍匹配 record_id 的登记。"""
        record = self.get_record(component_key)
        if record is None:
            return None
        pid = int(record.get("pid", 0))
        if not _alive(pid):
            return self._finalize_stop(component_key, record, is_parent=True)
        return {"result": "still_running", "component_key": component_key, "pid": pid}

    def cleanup_session(self, owner_session: str) -> list:
        """清理本 session 创建的组件（GUI 退出用）。

        仅清理 initiator/owner_session 归属本会话的登记；复用的 manual 组件
        不会被停止（无退出清理权）。
        """
        results = []
        for key, record in list(self.read_registry().items()):
            if record.get("owner_session") != owner_session:
                continue
            with self.transition_lock(key):
                results.append(self.stop(key, require_owner=record.get("initiator"),
                                         owner_session=owner_session))
        return results

    def group_alive(self, component_key: str):
        """登记进程组是否仍有存活成员（用于"未完全退出"判定）。"""
        record = self.get_record(component_key)
        if record is None:
            return False
        return bool(_pgid_members(int(record.get("pgid", 0))))
