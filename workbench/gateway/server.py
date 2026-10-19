"""HTTPS gateway and PWA host for the tablet HMI."""
from __future__ import annotations

import argparse
import asyncio
import base64
import io
import json
import pathlib
import secrets
import time
from urllib.parse import quote

import aiohttp
from aiohttp import web

from ..tablet_shared import (
    TabletPaths, certificate_info, create_certificates, issue_media_token,
    load_config, local_addresses, server_ssl_context, token_digest,
)
from .service_client import ServiceClient
from .state import GatewayState

VERSION = "0.3.0"
COOKIE = "piper_device"
ALLOWED_ACTIONS = {
    "status.get", "motion.start", "motion.pause_after_cycle", "motion.send",
    "system.start_missing", "system.cancel", "system.stop_all", "arm.recover",
    "component.start", "component.stop",
}
ALLOWED_MOTION_COMMANDS = {"standby", "observe", "reset", "emergency_stop"}


def _now():
    return time.time()


def _json_error(status, message, **extra):
    return web.json_response({"ok": False, "error": {"message": str(message)}, **extra}, status=status)


class TabletGateway:
    def __init__(self, root=None, config=None):
        root_path = pathlib.Path(root) if root else None
        self.config = config or load_config((root_path or pathlib.Path(__file__).resolve().parents[2]) / "config/tablet_hmi.json")
        self.paths = TabletPaths(root, self.config)
        pairing = self.config.get("pairing", {})
        lease = self.config.get("lease", {})
        gateway = self.config.get("gateway", {})
        self.state = GatewayState(
            self.paths.state,
            pair_ttl=pairing.get("ttl_s", 600),
            lease_ttl=lease.get("ttl_s", 15),
        )
        self.socket_path = self.paths.root / gateway.get("service_socket", ".runtime/workbench.sock")
        self.media_url = "http://127.0.0.1:{}".format(self.config.get("media", {}).get("port", 8090))
        self.service = ServiceClient(self.socket_path, self._service_event)
        self.sequence = 0
        self.ws_clients = set()
        self.status = {}
        self.telemetry = {}
        self.detections = {}
        self.media_health = {"state": "stopped", "cameras": {}}
        self.last_service_event = None
        self._tasks = []
        self._action_lock = asyncio.Lock()
        self._requests = {}
        self._pair_failures = {}
        self.http = None
        self.started_at = _now()
        self.loop = None
        self._current_pair_code = None

    async def start(self):
        self.loop = asyncio.get_running_loop()
        create_certificates(self.paths)
        self.paths.admin_token()
        self.paths.media_token_secret()
        pair = self.state.create_pair("operator")
        self._current_pair_code = pair["code"]
        self.http = aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=8))
        self._tasks = [
            asyncio.create_task(self._status_pump()),
            asyncio.create_task(self._telemetry_pump()),
            asyncio.create_task(self._media_pump()),
            asyncio.create_task(self._lease_pump()),
        ]

    async def stop(self):
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        if self.http:
            await self.http.close()
        await self.service.close()

    def next_sequence(self):
        self.sequence += 1
        return self.sequence

    def _service_event(self, row):
        data = row.get("data")
        event_type = row.get("event", "event")
        self.last_service_event = _now()
        if event_type == "vision.targets" and isinstance(data, dict):
            camera = data.get("camera_id")
            if camera in ("d455", "dabai"):
                self.detections[camera] = data.get("detection") or {}
        self.loop.create_task(self._broadcast({
            "sequence": self.next_sequence(),
            "stamp": _now(),
            "type": event_type,
            "data": data,
        }))

    async def _status_pump(self):
        while True:
            try:
                self.status = await self.service.call("status.get", timeout=3.5) or {}
            except Exception as exc:
                self.status = dict(self.status)
                self.status["gateway_error"] = str(exc)
            await asyncio.sleep(1.0)

    async def _telemetry_pump(self):
        while True:
            if self.ws_clients:
                try:
                    row = await self.service.call("tablet.telemetry", timeout=1.0) or {}
                    self.telemetry = row
                    await self._broadcast({
                        "sequence": self.next_sequence(),
                        "stamp": _now(),
                        "type": "robot.link_poses",
                        "data": row,
                    })
                except Exception:
                    pass
            await asyncio.sleep(1 / 15)

    async def _media_pump(self):
        previous = None
        while True:
            try:
                async with self.http.get(self.media_url + "/health", timeout=aiohttp.ClientTimeout(total=1.5)) as response:
                    self.media_health = await response.json()
            except Exception as exc:
                self.media_health = {"state": "offline", "error": str(exc), "cameras": {}}
            state = self.media_health.get("state")
            if previous is not None and state != previous:
                self.state.event("media." + ("online" if state == "online" else "offline"), {"state": state})
            previous = state
            await asyncio.sleep(1.0)

    async def _lease_pump(self):
        last = None
        while True:
            current = self.state.lease()
            token = (current or {}).get("device_id")
            if last != token:
                last = token
                await self._broadcast({
                    "sequence": self.next_sequence(),
                    "stamp": _now(),
                    "type": "lease.changed",
                    "data": current,
                })
            await asyncio.sleep(0.5)

    async def _broadcast(self, row):
        for ws in list(self.ws_clients):
            if ws.closed:
                self.ws_clients.discard(ws)
                continue
            try:
                await ws.send_json(row)
            except Exception:
                self.ws_clients.discard(ws)

    def _device(self, request):
        device = self.state.authenticate(request.cookies.get(COOKIE))
        if device:
            self.state.touch(
                device["id"],
                user_agent=request.headers.get("User-Agent", ""),
                ip=request.remote,
            )
        return device

    def _require_device(self, request):
        device = self._device(request)
        if not device:
            raise web.HTTPUnauthorized(
                text=json.dumps({"ok": False, "error": {"message": "设备尚未配对或授权已撤销"}}, ensure_ascii=False),
                content_type="application/json",
            )
        return device

    def _require_admin(self, request):
        token = request.headers.get("X-Piper-Admin", "")
        expected = self.paths.admin_token()
        if not secrets.compare_digest(token, expected):
            raise web.HTTPUnauthorized(
                text=json.dumps({"ok": False, "error": {"message": "仅电脑本地管理接口可用"}}, ensure_ascii=False),
                content_type="application/json",
            )

    def snapshot(self, device):
        workbench = self.status or {}
        lease = self.state.lease()
        online = sum(1 for row in self.state.public_devices() if row["online"])
        return {
            "schema_version": 1,
            "sequence": self.sequence,
            "server_time": _now(),
            "gateway": {
                "state": "online",
                "service_connected": self.service.connected,
                "version": VERSION,
                "online_devices": online,
                "lan_addresses": local_addresses(self.config.get("gateway", {}).get("hostname", "piper.local")),
            },
            "device": {
                "id": device["id"],
                "name": device["name"],
                "role": device["role"],
            },
            "lease": lease,
            "workbench": workbench,
            "robot": self.telemetry,
            "detections": self.detections,
            "media": self.media_health,
        }

    async def health(self, request):
        return web.json_response({
            "ok": True,
            "state": "online",
            "version": VERSION,
            "service_connected": self.service.connected,
            "clients": len(self.ws_clients),
            "started_at": self.started_at,
        })

    async def pair(self, request):
        address = request.remote or "unknown"
        failure = self._pair_failures.get(address)
        limit = self.config.get("limits", {}).get("pair_failures", 5)
        backoff = self.config.get("limits", {}).get("pair_backoff_s", 30)
        if failure and failure.get("until", 0) > _now():
            return _json_error(429, f"配对失败次数过多，请 {int(failure['until'] - _now())} 秒后重试")
        try:
            body = await request.json()
        except Exception:
            body = {}
        try:
            result = self.state.claim_pair(body.get("code", ""), body.get("name", ""))
        except ValueError as exc:
            count = (failure or {}).get("count", 0) + 1
            self._pair_failures[address] = {
                "count": count,
                "until": _now() + backoff if count >= limit else 0,
            }
            return _json_error(403, str(exc))
        self._pair_failures.pop(address, None)
        cookie = result["device_id"] + "." + result["token"]
        max_age = int(self.config.get("pairing", {}).get("cookie_days", 365) * 86400)
        response = web.json_response({"ok": True, "device": {
            "id": result["device_id"], "name": result["name"], "role": result["role"],
        }})
        response.set_cookie(
            COOKIE, cookie, max_age=max_age, httponly=True, samesite="Lax",
            secure=request.secure, path="/",
        )
        await self._broadcast({
            "sequence": self.next_sequence(), "stamp": _now(),
            "type": "device.paired", "data": result,
        })
        return response

    async def session(self, request):
        device = self._require_device(request)
        return web.json_response({"ok": True, "snapshot": self.snapshot(device)})

    async def lease(self, request):
        device = self._require_device(request)
        if request.headers.get("X-Piper-Action") != "1":
            return _json_error(403, "缺少本地操作标识")
        try:
            body = await request.json()
        except Exception:
            body = {}
        action = body.get("action", "renew")
        try:
            if action == "release":
                released = self.state.release_lease(device["id"])
                value = None if released else self.state.lease()
            else:
                value = self.state.acquire_lease(device["id"])
        except PermissionError as exc:
            return _json_error(403, str(exc))
        except RuntimeError as exc:
            return _json_error(409, str(exc), lease=self.state.lease())
        await self._broadcast({
            "sequence": self.next_sequence(), "stamp": _now(),
            "type": "lease.changed", "data": value,
        })
        return web.json_response({"ok": True, "lease": value})

    async def action(self, request):
        device = self._require_device(request)
        if request.headers.get("X-Piper-Action") != "1":
            return _json_error(403, "缺少本地操作标识")
        try:
            body = await request.json()
        except Exception:
            return _json_error(400, "请求体不是有效 JSON")
        op = body.get("op", "")
        args = body.get("args") or {}
        request_id = str(body.get("request_id", "")).strip()
        if op not in ALLOWED_ACTIONS:
            return _json_error(403, "平板不允许执行该命令")
        if not request_id or len(request_id) > 100:
            return _json_error(400, "缺少 request_id")
        if not isinstance(args, dict):
            return _json_error(400, "args 必须是对象")
        emergency_stop = False
        if op == "motion.send":
            cmd = (args.get("cmd") or "").strip()
            if cmd not in ALLOWED_MOTION_COMMANDS:
                return _json_error(403, "不允许的机械臂快捷命令")
            emergency_stop = cmd == "emergency_stop"
        if op != "status.get":
            if device.get("role") != "operator":
                return _json_error(403, "观察者设备为只读模式")
            if not emergency_stop:
                try:
                    self.state.verify_lease(device["id"])
                except PermissionError as exc:
                    return _json_error(409, str(exc), needs_lease=True)
        cache_key = device["id"] + "|" + request_id
        cache_ttl = self.config.get("limits", {}).get("request_cache_s", 60)
        async with self._action_lock:
            cached = self._requests.get(cache_key)
            if cached and _now() - cached[0] <= cache_ttl:
                return web.json_response(cached[1])
            try:
                result = await self.service.call(op, args, timeout=15)
                response = {"ok": True, "result": result, "request_id": request_id}
            except Exception as exc:
                return _json_error(502, str(exc), request_id=request_id)
            self._requests[cache_key] = (_now(), response)
            expired = [key for key, row in self._requests.items() if _now() - row[0] > cache_ttl]
            for key in expired:
                self._requests.pop(key, None)
        return web.json_response(response)

    async def websocket(self, request):
        device = self._require_device(request)
        ws = web.WebSocketResponse(heartbeat=20, max_msg_size=256 * 1024)
        await ws.prepare(request)
        ws._piper_device_id = device["id"]
        self.ws_clients.add(ws)
        self.state.event("device.connected", {"device_id": device["id"], "name": device["name"], "ip": request.remote})
        self.state.touch(device["id"], user_agent=request.headers.get("User-Agent", ""), ip=request.remote)
        await ws.send_json({
            "sequence": self.next_sequence(),
            "stamp": _now(),
            "type": "snapshot",
            "data": self.snapshot(device),
        })
        try:
            async for message in ws:
                if message.type == aiohttp.WSMsgType.TEXT:
                    try:
                        row = json.loads(message.data)
                    except ValueError:
                        continue
                    if row.get("type") == "heartbeat":
                        self.state.touch(
                            device["id"],
                            page=row.get("page"),
                            reconnect_reason=row.get("reconnect_reason"),
                            ip=request.remote,
                            ws_rtt_ms=row.get("ws_rtt_ms"),
                            app_version=row.get("app_version"),
                        )
                        await ws.send_json({"sequence": self.next_sequence(), "stamp": _now(), "type": "heartbeat", "data": {}})
                elif message.type == aiohttp.WSMsgType.ERROR:
                    break
        finally:
            self.ws_clients.discard(ws)
            self.state.event("device.disconnected", {"device_id": device["id"], "name": device["name"]})
        return ws

    async def media_token(self, request):
        device = self._require_device(request)
        camera = request.query.get("camera", "")
        if camera not in ("d455", "dabai"):
            return _json_error(400, "未知相机")
        token = issue_media_token(self.paths, device["id"], camera, ttl=60)
        query = "&".join(f"{key}={quote(str(value))}" for key, value in token.items())
        return web.json_response({
            "ok": True,
            "expires": token["expires"],
            "mjpeg_url": f"/media/{camera}/mjpeg?{query}",
            "whep_url": f"/api/v1/media/whep/{camera}?{query}",
            "snapshot_url": f"/media/{camera}/snapshot?{query}",
        })

    async def media_mjpeg(self, request):
        device = self._require_device(request)
        camera = request.match_info["camera"]
        if camera not in ("d455", "dabai"):
            return _json_error(404, "未知相机")
        token = issue_media_token(self.paths, device["id"], camera, ttl=60)
        url = f"{self.media_url}/mjpeg/{camera}?" + "&".join(f"{key}={quote(str(value))}" for key, value in token.items())
        try:
            upstream = await self.http.get(url)
        except Exception as exc:
            return _json_error(503, "媒体服务不可用: " + str(exc))
        if upstream.status != 200:
            text = await upstream.text()
            upstream.close()
            return _json_error(upstream.status, text)
        self.state.event("media.jpeg_stream", {"device_id": device["id"], "camera": camera})
        response = web.StreamResponse(
            status=200,
            headers={
                "Content-Type": upstream.headers.get("Content-Type", "multipart/x-mixed-replace; boundary=frame"),
                "Cache-Control": "no-store, no-cache, must-revalidate",
                "Pragma": "no-cache",
            },
        )
        await response.prepare(request)
        try:
            async for chunk in upstream.content.iter_any():
                await response.write(chunk)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        finally:
            upstream.close()
            try:
                await response.write_eof()
            except Exception:
                pass
        return response

    async def media_jpeg_ws(self, request):
        device = self._require_device(request)
        camera = request.match_info["camera"]
        if camera not in ("d455", "dabai"):
            return _json_error(404, "未知相机")
        token = issue_media_token(self.paths, device["id"], camera, ttl=60)
        url = f"{self.media_url}/ws/jpeg/{camera}?" + "&".join(f"{key}={quote(str(value))}" for key, value in token.items())
        try:
            upstream = await self.http.ws_connect(url, heartbeat=20)
        except Exception as exc:
            return _json_error(503, str(exc))
        client = web.WebSocketResponse(heartbeat=20, max_msg_size=2 * 1024 * 1024)
        await client.prepare(request)
        self.state.event("media.jpeg_stream", {"device_id": device["id"], "camera": camera})

        async def upstream_to_client():
            async for message in upstream:
                if message.type == aiohttp.WSMsgType.BINARY:
                    await client.send_bytes(message.data)
                elif message.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.ERROR):
                    break

        async def client_to_upstream():
            async for message in client:
                if message.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.ERROR):
                    break

        done, pending = await asyncio.wait(
            [asyncio.create_task(upstream_to_client()), asyncio.create_task(client_to_upstream())],
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in pending:
            task.cancel()
        await asyncio.gather(*pending, return_exceptions=True)
        await upstream.close()
        return client

    async def media_snapshot(self, request):
        device = self._require_device(request)
        camera = request.match_info["camera"]
        token = issue_media_token(self.paths, device["id"], camera, ttl=60)
        url = f"{self.media_url}/snapshot/{camera}?" + "&".join(f"{key}={quote(str(value))}" for key, value in token.items())
        try:
            async with self.http.get(url) as upstream:
                data = await upstream.read()
                return web.Response(status=upstream.status, body=data, content_type="image/jpeg", headers={"Cache-Control": "no-store"})
        except Exception as exc:
            return _json_error(503, "媒体服务不可用: " + str(exc))

    async def media_whep(self, request):
        device = self._require_device(request)
        camera = request.match_info["camera"]
        if camera not in ("d455", "dabai"):
            return _json_error(404, "未知相机")
        token = issue_media_token(self.paths, device["id"], camera, ttl=60)
        url = f"{self.media_url}/whep/{camera}?" + "&".join(f"{key}={quote(str(value))}" for key, value in token.items())
        offer = await request.text()
        try:
            async with self.http.post(url, data=offer, headers={"Content-Type": "application/sdp"}) as upstream:
                body = await upstream.read()
                if upstream.status == 200:
                    self.state.event("webrtc.started", {"device_id": device["id"], "camera": camera})
                else:
                    self.state.event("webrtc.failed", {"device_id": device["id"], "camera": camera, "status": upstream.status})
                return web.Response(status=upstream.status, body=body, content_type=upstream.content_type)
        except Exception as exc:
            return _json_error(503, "媒体服务不可用: " + str(exc))

    async def media_whep_delete(self, request):
        self._require_device(request)
        session_id = request.match_info["session"]
        try:
            async with self.http.delete(self.media_url + "/whep/" + quote(session_id)) as upstream:
                return web.json_response(await upstream.json(), status=upstream.status)
        except Exception as exc:
            return _json_error(503, "媒体服务不可用: " + str(exc))

    def _pair_payload(self, pair):
        hostname = self.config.get("gateway", {}).get("hostname", "piper.local")
        port = self.config.get("gateway", {}).get("https_port", 8443)
        addresses = local_addresses(hostname)
        primary = next((x for x in addresses if x not in ("localhost", "127.0.0.1", "::1")), hostname)
        url = f"https://{primary}:{port}/pair?code={pair['code']}"
        import qrcode
        image = qrcode.make(url)
        buffer = io.BytesIO()
        image.save(buffer, format="PNG")
        return {
            "code": pair["code"],
            "role": pair["role"],
            "expires_at": pair["expires_at"],
            "remaining_s": max(0, round(pair["expires_at"] - _now(), 1)),
            "url": url,
            "hostname_url": f"https://{hostname}:{port}/",
            "addresses": [f"https://{x}:{port}/" for x in addresses],
            "qr_png_base64": base64.b64encode(buffer.getvalue()).decode("ascii"),
        }

    async def admin_status(self, request):
        self._require_admin(request)
        pair = self.state.current_pair()
        pair_payload = None
        if pair:
            with self.state.lock:
                stored = self.state.value.get("pair") or {}
            # The plaintext code is not persisted across gateway restarts.
            code = self._current_pair_code
            if code and secrets.compare_digest(stored.get("code_hash", ""), token_digest(code)):
                pair_payload = self._pair_payload(dict(pair, code=code))
            else:
                pair_payload = {
                    "role": pair["role"],
                    "expires_at": pair["expires_at"],
                    "remaining_s": max(0, round(pair["expires_at"] - _now(), 1)),
                    "code": None,
                    "url": None,
                }
        return web.json_response({
            "ok": True,
            "state": "online",
            "version": VERSION,
            "service_connected": self.service.connected,
            "started_at": self.started_at,
            "pair": pair_payload,
            "devices": self.state.public_devices(),
            "lease": self.state.lease(),
            "media": self.media_health,
            "certificate": certificate_info(self.paths),
            "events": list(reversed(self.state.value.get("events", [])[-100:])),
            "addresses": local_addresses(self.config.get("gateway", {}).get("hostname", "piper.local")),
        })

    async def admin_pair_create(self, request):
        self._require_admin(request)
        try:
            body = await request.json()
        except Exception:
            body = {}
        pair = self.state.create_pair(body.get("role", "operator"))
        self._current_pair_code = pair["code"]
        return web.json_response({"ok": True, "pair": self._pair_payload(pair)})

    async def admin_pair_revoke_all(self, request):
        self._require_admin(request)
        count = self.state.revoke_all()
        for ws in list(self.ws_clients):
            await ws.close(code=aiohttp.WSCloseCode.POLICY_VIOLATION, message=b"pairing revoked")
        self.ws_clients.clear()
        return web.json_response({"ok": True, "revoked_devices": count})

    async def admin_device_revoke(self, request):
        self._require_admin(request)
        device_id = request.match_info["device"]
        self.state.revoke(device_id)
        for ws in list(self.ws_clients):
            room = getattr(ws, "_piper_device_id", None)
            if room == device_id:
                await ws.close(code=aiohttp.WSCloseCode.POLICY_VIOLATION, message=b"device revoked")
                self.ws_clients.discard(ws)
        return web.json_response({"ok": True})

    async def admin_lease_revoke(self, request):
        self._require_admin(request)
        released = self.state.release_lease()
        await self._broadcast({
            "sequence": self.next_sequence(), "stamp": _now(),
            "type": "lease.changed", "data": None,
        })
        return web.json_response({"ok": True, "released": released})

    async def admin_certificate(self, request):
        self._require_admin(request)
        return web.json_response({"ok": True, "certificate": certificate_info(self.paths)})

    async def admin_certificate_regenerate(self, request):
        self._require_admin(request)
        result = create_certificates(self.paths, force=True)
        return web.json_response({"ok": True, "certificate": certificate_info(self.paths), "paths": result})

    async def admin_certificate_download(self, request):
        self._require_admin(request)
        path = self.paths.certs / "ca.crt"
        return web.FileResponse(path, headers={"Content-Disposition": 'attachment; filename="piper-workbench-ca.crt"'})

    async def admin_keyframe(self, request):
        self._require_admin(request)
        camera = request.query.get("camera", "")
        try:
            async with self.http.post(self.media_url + "/keyframe/" + quote(camera)) as upstream:
                return web.json_response(await upstream.json(), status=upstream.status)
        except Exception as exc:
            return _json_error(503, "媒体服务不可用: " + str(exc))

    async def static_file(self, request):
        dist = self.paths.root / self.config.get("gateway", {}).get("pwa_dist", "tablet_hmi/dist")
        relative = request.match_info.get("tail", "")
        if not dist.is_dir():
            return web.Response(
                status=503,
                text="平板 PWA 尚未构建。请运行 npm --prefix tablet_hmi run build。",
                content_type="text/plain",
            )
        if relative:
            target = (dist / relative).resolve()
            try:
                target.relative_to(dist.resolve())
            except ValueError:
                raise web.HTTPNotFound()
            if target.is_file():
                response = web.FileResponse(target)
                if target.name in ("sw.js", "manifest.webmanifest"):
                    response.headers["Cache-Control"] = "no-cache"
                return response
        index = dist / "index.html"
        if index.is_file():
            return web.FileResponse(index, headers={"Cache-Control": "no-cache"})
        raise web.HTTPNotFound()


APP_GATEWAY = web.AppKey("gateway", TabletGateway)


def create_app(root=None, config=None):
    gateway = TabletGateway(root, config)

    async def startup(app):
        await gateway.start()

    async def cleanup(app):
        await gateway.stop()

    app = web.Application(client_max_size=2 * 1024 * 1024)
    app[APP_GATEWAY] = gateway
    app.on_startup.append(startup)
    app.on_cleanup.append(cleanup)
    app.router.add_get("/api/v1/health", gateway.health)
    app.router.add_post("/api/v1/pair", gateway.pair)
    app.router.add_get("/api/v1/session", gateway.session)
    app.router.add_post("/api/v1/lease", gateway.lease)
    app.router.add_post("/api/v1/actions", gateway.action)
    app.router.add_get("/api/v1/media/token", gateway.media_token)
    app.router.add_route("POST", "/api/v1/media/whep/{camera}", gateway.media_whep)
    app.router.add_delete("/api/v1/media/whep/{camera}/{session}", gateway.media_whep_delete)
    app.router.add_get("/media/{camera}/mjpeg", gateway.media_mjpeg)
    app.router.add_get("/ws/v1/jpeg/{camera}", gateway.media_jpeg_ws)
    app.router.add_get("/media/{camera}/snapshot", gateway.media_snapshot)
    app.router.add_get("/ws/v1/events", gateway.websocket)
    app.router.add_get("/api/v1/admin/status", gateway.admin_status)
    app.router.add_post("/api/v1/admin/pair", gateway.admin_pair_create)
    app.router.add_post("/api/v1/admin/pair/revoke_all", gateway.admin_pair_revoke_all)
    app.router.add_post("/api/v1/admin/devices/{device}/revoke", gateway.admin_device_revoke)
    app.router.add_post("/api/v1/admin/lease/revoke", gateway.admin_lease_revoke)
    app.router.add_get("/api/v1/admin/certificate", gateway.admin_certificate)
    app.router.add_post("/api/v1/admin/certificate/regenerate", gateway.admin_certificate_regenerate)
    app.router.add_get("/api/v1/admin/certificate/download", gateway.admin_certificate_download)
    app.router.add_post("/api/v1/admin/stream/keyframe", gateway.admin_keyframe)
    app.router.add_get("/{tail:.*}", gateway.static_file)
    return app


async def serve(config=None, http_only=False, port_override=None):
    config = config or load_config()
    gateway_config = config.get("gateway", {})
    bind = gateway_config.get("bind", "0.0.0.0")
    https_port = int(gateway_config.get("https_port", 8443))
    http_port = int(gateway_config.get("http_port", 8080))
    app = create_app(config=config)
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    ssl_context = None if http_only else server_ssl_context(app[APP_GATEWAY].paths)
    main_port = https_port if not http_only else int(port_override or http_port)
    site = web.TCPSite(runner, bind, main_port, ssl_context=ssl_context)
    await site.start()

    redirect_runner = None
    if not http_only:
        async def redirect(request):
            host = (request.host or gateway_config.get("hostname", "piper.local")).split(":", 1)[0]
            target = f"https://{host}:{https_port}{request.rel_url}"
            raise web.HTTPPermanentRedirect(target)

        redirect_app = web.Application()
        redirect_app.router.add_get("/{tail:.*}", redirect)
        redirect_runner = web.AppRunner(redirect_app, access_log=None)
        await redirect_runner.setup()
        redirect_site = web.TCPSite(redirect_runner, bind, http_port)
        await redirect_site.start()
    print(f"[tablet-gateway] {'http' if http_only else 'https'}://{bind}:{main_port}", flush=True)
    stop = asyncio.Event()
    try:
        await stop.wait()
    finally:
        if redirect_runner:
            await redirect_runner.cleanup()
        await runner.cleanup()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Piper tablet HMI gateway")
    parser.add_argument("--http-only", action="store_true", help="local development without TLS")
    parser.add_argument("--port", type=int, default=None, help="override the HTTP development port")
    args = parser.parse_args(argv)
    try:
        asyncio.run(serve(http_only=args.http_only, port_override=args.port))
    except KeyboardInterrupt:
        return 0
    return 0

