"""Durable pairing, device and lease state for the tablet gateway."""
from __future__ import annotations

import datetime
import json
import pathlib
import secrets
import threading
import time

from ..settings import atomic_json
from ..tablet_shared import token_digest


class GatewayState:
    def __init__(self, path, pair_ttl=600, lease_ttl=15):
        self.path = pathlib.Path(path)
        self.pair_ttl = float(pair_ttl)
        self.lease_ttl = float(lease_ttl)
        self.lock = threading.RLock()
        self.value = self._load()

    def _default(self):
        return {
            "schema_version": 1,
            "devices": {},
            "pair": None,
            "lease": None,
            "events": [],
        }

    def _load(self):
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ValueError("state must be an object")
        except (OSError, ValueError):
            data = self._default()
        for key, value in self._default().items():
            data.setdefault(key, value)
        return data

    def _save(self):
        atomic_json(self.path, self.value)
        try:
            self.path.chmod(0o600)
        except OSError:
            pass

    def event(self, kind, detail=None):
        with self.lock:
            row = {
                "time": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "type": kind,
                "detail": detail or {},
            }
            self.value["events"] = (self.value["events"] + [row])[-200:]
            self._save()
            return row

    def create_pair(self, role="operator"):
        role = role if role in ("operator", "observer") else "operator"
        with self.lock:
            code = f"{secrets.randbelow(1_000_000):06d}"
            now = time.time()
            self.value["pair"] = {
                "code_hash": token_digest(code),
                "role": role,
                "created_at": now,
                "expires_at": now + self.pair_ttl,
            }
            self._save()
            self.event("pairing.created", {"role": role, "expires_at": now + self.pair_ttl})
            return {
                "code": code,
                "role": role,
                "created_at": now,
                "expires_at": now + self.pair_ttl,
            }

    def current_pair(self):
        with self.lock:
            pair = dict(self.value.get("pair") or {})
        if not pair or float(pair.get("expires_at", 0)) <= time.time():
            return None
        pair.pop("code_hash", None)
        return pair

    def claim_pair(self, code, name):
        with self.lock:
            pair = self.value.get("pair") or {}
            now = time.time()
            if not pair or float(pair.get("expires_at", 0)) <= now:
                raise ValueError("配对码已过期，请在电脑管理页重新生成")
            if not secrets.compare_digest(str(pair.get("code_hash", "")), token_digest(code)):
                raise ValueError("配对码不正确")
            device_id = "piper-" + secrets.token_hex(6)
            token = secrets.token_urlsafe(36)
            role = pair.get("role", "operator")
            clean_name = str(name or "Piper 平板").strip()[:64] or "Piper 平板"
            self.value["devices"][device_id] = {
                "id": device_id,
                "name": clean_name,
                "role": role,
                "token_hash": token_digest(token),
                "created_at": now,
                "last_seen": now,
                "seen_monotonic": time.monotonic(),
                "user_agent": "",
                "page": "",
                "reconnect_reason": "",
                "revoked": False,
            }
            self.value["pair"] = None
            self._save()
            self.event("device.paired", {"device_id": device_id, "name": clean_name, "role": role})
            return {
                "device_id": device_id,
                "token": token,
                "role": role,
                "name": clean_name,
            }

    def authenticate(self, cookie_value):
        raw = str(cookie_value or "")
        try:
            device_id, token = raw.split(".", 1)
        except ValueError:
            return None
        with self.lock:
            device = self.value["devices"].get(device_id)
            if not device or device.get("revoked"):
                return None
            if not secrets.compare_digest(str(device.get("token_hash", "")), token_digest(token)):
                return None
            return dict(device)

    def touch(self, device_id, user_agent=None, page=None, reconnect_reason=None,
              ip=None, ws_rtt_ms=None, app_version=None):
        with self.lock:
            device = self.value["devices"].get(device_id)
            if not device:
                return None
            device["last_seen"] = time.time()
            device["seen_monotonic"] = time.monotonic()
            if user_agent is not None:
                device["user_agent"] = str(user_agent)[:500]
            if page is not None:
                device["page"] = str(page)[:40]
            if reconnect_reason is not None:
                device["reconnect_reason"] = str(reconnect_reason)[:300]
            if ip:
                device["ip"] = str(ip)[:80]
            if ws_rtt_ms is not None:
                device["ws_rtt_ms"] = int(ws_rtt_ms)
            if app_version is not None:
                device["app_version"] = str(app_version)[:40]
            self._save()
            return dict(device)

    def public_devices(self, online_after=20):
        now = time.monotonic()
        with self.lock:
            rows = []
            for device in self.value["devices"].values():
                if device.get("revoked"):
                    continue
                online = now - float(device.get("seen_monotonic", 0)) <= online_after
                rows.append({
                    "id": device.get("id"),
                    "name": device.get("name"),
                    "role": device.get("role"),
                    "online": online,
                    "created_at": device.get("created_at"),
                    "last_seen": device.get("last_seen"),
                    "user_agent": device.get("user_agent", ""),
                    "page": device.get("page", ""),
                    "reconnect_reason": device.get("reconnect_reason", ""),
                    "ip": device.get("ip", ""),
                    "ws_rtt_ms": device.get("ws_rtt_ms"),
                    "app_version": device.get("app_version", ""),
                })
        return sorted(rows, key=lambda row: (not row["online"], row["name"]))

    def device(self, device_id):
        with self.lock:
            device = self.value["devices"].get(device_id)
            return dict(device) if device else None

    def revoke(self, device_id):
        with self.lock:
            device = self.value["devices"].get(device_id)
            if not device:
                raise ValueError("设备不存在")
            device["revoked"] = True
            lease = self.value.get("lease") or {}
            if lease.get("device_id") == device_id:
                self.value["lease"] = None
            self._save()
            self.event("device.revoked", {"device_id": device_id, "name": device.get("name")})
            return True

    def revoke_all(self):
        with self.lock:
            count = 0
            for device in self.value["devices"].values():
                if not device.get("revoked"):
                    device["revoked"] = True
                    count += 1
            self.value["pair"] = None
            self.value["lease"] = None
            self._save()
            self.event("pairing.revoked_all", {"devices": count})
            return count

    def lease(self):
        with self.lock:
            lease = self.value.get("lease")
            if not lease:
                return None
            if float(lease.get("expires_at", 0)) <= time.time():
                self.value["lease"] = None
                self._save()
                return None
            result = dict(lease)
            device = self.value["devices"].get(result.get("device_id"), {})
            result["device_name"] = device.get("name", result.get("device_id"))
            result["remaining_s"] = max(0, round(float(result["expires_at"]) - time.time(), 1))
            return result

    def acquire_lease(self, device_id):
        with self.lock:
            device = self.value["devices"].get(device_id)
            if not device or device.get("revoked"):
                raise ValueError("设备已失效")
            if device.get("role") != "operator":
                raise PermissionError("观察者设备不能取得控制权")
            current = self.lease()
            if current and current["device_id"] != device_id:
                raise RuntimeError("控制权已被 " + current.get("device_name", "其他设备") + " 占用")
            now = time.time()
            changed = not current or current["device_id"] != device_id
            self.value["lease"] = {
                "device_id": device_id,
                "acquired_at": current.get("acquired_at", now) if current else now,
                "expires_at": now + self.lease_ttl,
            }
            self._save()
            if changed:
                self.event("lease.acquired", {"device_id": device_id, "name": device.get("name")})
            return self.lease()

    def release_lease(self, device_id=None):
        with self.lock:
            current = self.value.get("lease")
            if current and (device_id is None or current.get("device_id") == device_id):
                self.value["lease"] = None
                self._save()
                self.event("lease.released", {"device_id": current.get("device_id")})
                return True
            return False

    def verify_lease(self, device_id):
        current = self.lease()
        if not current:
            raise PermissionError("尚未取得控制权")
        if current.get("device_id") != device_id:
            raise PermissionError("控制权由 " + current.get("device_name", "其他设备") + " 持有")
        return True

