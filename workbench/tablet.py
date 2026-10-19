"""Local management proxy for the tablet gateway, media service and certificates."""
from __future__ import annotations

import json
import pathlib
import ssl
import urllib.error
import urllib.parse
import urllib.request

import runtime_config
import runtime_launcher

from .tablet_shared import TabletPaths, certificate_info, load_config


class TabletManager:
    def __init__(self, root, launcher=runtime_launcher):
        self.root = pathlib.Path(root)
        self.launcher = launcher
        self.config = load_config(self.root / "config/tablet_hmi.json")
        self.paths = TabletPaths(self.root, self.config)
        self.base_url = "https://127.0.0.1:" + str(self.config.get("gateway", {}).get("https_port", 8443))
        self._ssl = ssl.create_default_context()
        self._ssl.check_hostname = False
        self._ssl.verify_mode = ssl.CERT_NONE

    def _request(self, path, method="GET", data=None, timeout=3):
        headers = {"X-Piper-Admin": self.paths.admin_token()}
        body = None
        if data is not None:
            body = json.dumps(data, ensure_ascii=False).encode()
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(self.base_url + path, data=body, headers=headers, method=method)
        with urllib.request.urlopen(request, timeout=timeout, context=self._ssl) as response:
            payload = response.read()
            if "application/json" in response.headers.get("Content-Type", ""):
                return json.loads(payload)
            return payload

    def _process(self, component):
        manager = self.launcher._manager()
        record = manager.get_record(component)
        valid = bool(record and manager.verify_identity(record)[0])
        return {
            "state": "running" if valid else "stopped",
            "record": record if valid else None,
            "log_file": str(self.launcher._runtime_log_dir() / f"{component}.log"),
        }

    def status(self):
        import re
        package_version = "—"
        cache_version = "—"
        built_at = None
        try:
            package_version = json.loads((self.root / "tablet_hmi/package.json").read_text())["version"]
        except (OSError, ValueError, KeyError):
            pass
        try:
            match = re.search(r"piper-tablet-v([0-9.]+)", (self.root / "tablet_hmi/public/sw.js").read_text())
            if match:
                cache_version = match.group(1)
        except OSError:
            pass
        dist_index = self.root / "tablet_hmi/dist/index.html"
        if dist_index.is_file():
            built_at = dist_index.stat().st_mtime
        result = {
            "gateway_process": self._process("tablet-gateway"),
            "media_process": self._process("tablet-media"),
            "gateway": None,
            "error": None,
            "version": "0.3.4",
            "pwa": {"version": package_version, "cache_version": cache_version, "built_at": built_at},
        }
        try:
            result["gateway"] = self._request("/api/v1/admin/status")
        except Exception as exc:
            result["error"] = str(exc)
            result["gateway"] = {
                "state": "offline",
                "certificate": certificate_info(self.paths),
                "devices": [],
                "events": [],
                "media": {"state": "offline", "cameras": {}},
            }
        return result

    def start(self, component):
        name = "tablet-gateway" if component == "gateway" else "tablet-media" if component == "media" else None
        if not name:
            raise ValueError("未知平板服务")
        result = self.launcher.ensure_background(name, initiator="workbench", wait_ready=False)
        if result.get("result") not in ("spawned", "reused", "reused_external"):
            raise RuntimeError(result.get("detail", str(result)))
        return result

    def stop(self, component):
        name = "tablet-gateway" if component == "gateway" else "tablet-media" if component == "media" else None
        if not name:
            raise ValueError("未知平板服务")
        return self.launcher.stop_component(
            name, initiator="workbench", allow_cross_initiator=True, cleanup_owned=True
        )

    def restart(self, component):
        stopped = self.stop(component)
        started = self.start(component)
        return {"stopped": stopped, "started": started}

    def start_all(self):
        gateway = self.start("gateway")
        media = self.start("media")
        return {"gateway": gateway, "media": media}

    def stop_all(self):
        media = self.stop("media")
        gateway = self.stop("gateway")
        return {"media": media, "gateway": gateway}

    def pair_create(self, role="operator"):
        return self._request("/api/v1/admin/pair", "POST", {"role": role})

    def pair_revoke_all(self):
        return self._request("/api/v1/admin/pair/revoke_all", "POST", {})

    def device_revoke(self, device_id):
        return self._request(
            "/api/v1/admin/devices/" + urllib.parse.quote(device_id, safe="") + "/revoke",
            "POST", {},
        )

    def lease_revoke(self):
        return self._request("/api/v1/admin/lease/revoke", "POST", {})

    def certificate_info(self):
        try:
            return self._request("/api/v1/admin/certificate")["certificate"]
        except Exception:
            return certificate_info(self.paths)

    def certificate_regenerate(self):
        return self._request("/api/v1/admin/certificate/regenerate", "POST", {})

    def certificate_path(self):
        cert = self.certificate_info()
        return cert["ca_path"]

    def stream_keyframe(self, camera):
        return self._request(
            "/api/v1/admin/stream/keyframe?camera=" + urllib.parse.quote(camera, safe=""), "POST", {}
        )
