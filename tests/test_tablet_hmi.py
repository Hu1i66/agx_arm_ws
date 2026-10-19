import pathlib
import time
import asyncio
import json

import pytest

from workbench.gateway.state import GatewayState
from workbench.tablet_shared import (
    TabletPaths, certificate_info, issue_media_token, load_config, verify_media_token,
)


def test_tablet_certificate_and_short_lived_media_token(tmp_path):
    paths = TabletPaths(tmp_path, load_config())
    info = certificate_info(paths)
    assert info["state"] == "valid"
    assert pathlib.Path(info["ca_path"]).is_file()
    token = issue_media_token(paths, "device-1", "d455", ttl=60)
    assert verify_media_token(
        paths, "device-1", "d455", token["expires"], token["signature"]
    )
    assert not verify_media_token(
        paths, "device-1", "dabai", token["expires"], token["signature"]
    )
    assert not verify_media_token(
        paths, "device-1", "d455", int(time.time()) - 1, token["signature"]
    )


def test_gateway_pair_device_and_exclusive_lease(tmp_path):
    state = GatewayState(tmp_path / "state.json", pair_ttl=60, lease_ttl=15)
    pair = state.create_pair("operator")
    first = state.claim_pair(pair["code"], "iPad A")
    cookie = first["device_id"] + "." + first["token"]
    device = state.authenticate(cookie)
    assert device["id"] == first["device_id"] and device["role"] == "operator"
    with pytest.raises(ValueError):
        state.claim_pair(pair["code"], "iPad replay")

    lease = state.acquire_lease(first["device_id"])
    assert lease["device_id"] == first["device_id"] and lease["remaining_s"] > 0

    second_pair = state.create_pair("operator")
    second = state.claim_pair(second_pair["code"], "iPad B")
    with pytest.raises(RuntimeError):
        state.acquire_lease(second["device_id"])
    state.release_lease(first["device_id"])
    assert state.acquire_lease(second["device_id"])["device_id"] == second["device_id"]

    state.revoke(second["device_id"])
    assert state.authenticate(second["device_id"] + "." + second["token"]) is None
    assert state.lease() is None


def test_observer_cannot_acquire_control_lease(tmp_path):
    state = GatewayState(tmp_path / "state.json")
    pair = state.create_pair("observer")
    device = state.claim_pair(pair["code"], "readonly iPad")
    with pytest.raises(PermissionError):
        state.acquire_lease(device["device_id"])


def test_media_hub_reports_real_frame_state():
    from workbench.media.server import FrameHub
    import numpy as np

    hub = FrameHub(stale_after=1.0)
    assert hub.health()["cameras"]["d455"]["state"] == "waiting"
    hub.update("d455", np.zeros((480, 640, 3), dtype=np.uint8), 12.0)
    health = hub.health()["cameras"]["d455"]
    assert health["state"] == "streaming"
    assert health["last_frame_stamp"] == 12.0
    assert health["resolution"] == [640, 480]


def test_launcher_registers_isolated_tablet_services():
    import runtime_launcher

    assert "tablet-gateway" in runtime_launcher.LAUNCHABLE
    assert "tablet-media" in runtime_launcher.LAUNCHABLE
    assert runtime_launcher.component_key("tablet-gateway") == "tablet-gateway"
    assert runtime_launcher.component_key("tablet-media") == "tablet-media"


def test_gateway_pair_lease_and_idempotent_action_with_fake_service(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    from workbench.gateway.server import APP_GATEWAY, create_app

    async def scenario():
        socket_path = tmp_path / "service.sock"
        calls = []

        async def fake_service(reader, writer):
            while line := await reader.readline():
                request = json.loads(line)
                calls.append(request)
                response = {
                    "id": request.get("id"),
                    "ok": True,
                    "result": {"fake": True, "op": request.get("op")},
                }
                writer.write((json.dumps(response) + "\n").encode())
                await writer.drain()
            writer.close()

        server = await asyncio.start_unix_server(fake_service, socket_path)
        config = load_config()
        config["gateway"]["service_socket"] = str(socket_path)
        config["gateway"]["pwa_dist"] = str(tmp_path / "dist")
        config["paths"]["runtime_dir"] = ".runtime/tablet_hmi"
        config["paths"]["certificates"] = ".runtime/certs"
        (tmp_path / "config").mkdir()
        (tmp_path / "config/tablet_hmi.json").write_text(json.dumps(config))
        app = create_app(tmp_path)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            admin = {"X-Piper-Admin": app[APP_GATEWAY].paths.admin_token()}
            pair = (await (await client.post("/api/v1/admin/pair", json={"role": "operator"}, headers=admin)).json())["pair"]
            assert (await client.post("/api/v1/pair", json={"code": pair["code"], "name": "iPad test"})).status == 200
            assert (await client.get("/api/v1/session")).status == 200
            headers = {"X-Piper-Action": "1"}
            assert (await client.post("/api/v1/lease", json={"action": "acquire"}, headers=headers)).status == 200
            payload = {
                "op": "component.start",
                "args": {"component": "camera", "camera_id": "d455"},
                "request_id": "same-request",
            }
            assert (await client.post("/api/v1/actions", json=payload, headers=headers)).status == 200
            assert (await client.post("/api/v1/actions", json=payload, headers=headers)).status == 200
            counted = [row for row in calls if row.get("op") == "component.start"]
            assert len(counted) == 1
            forbidden = {"op": "motion.send", "args": {"cmd": "home"}, "request_id": "bad-motion"}
            assert (await client.post("/api/v1/actions", json=forbidden, headers=headers)).status == 403
        finally:
            await client.close()
            server.close()
            await server.wait_closed()

    asyncio.run(scenario())
