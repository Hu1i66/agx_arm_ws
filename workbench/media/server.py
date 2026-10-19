"""Independent ROS camera service with WebRTC H.264 and JPEG fallback."""
from __future__ import annotations

import argparse
import asyncio
import collections
from fractions import Fraction
import fractions
import threading
import time
import uuid

import cv2
import numpy as np
from aiohttp import web

from ..tablet_shared import TabletPaths, load_config, verify_media_token

try:
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    ROS_AVAILABLE = True
except Exception:
    ROS_AVAILABLE = False

try:
    import av
    from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
    from aiortc import codecs as _aiortc_codecs
    from aiortc.codecs.h264 import H264Encoder as _Libx264Encoder
    from aiortc.rtcrtpsender import RTCRtpSender
    WEBRTC_AVAILABLE = True
except Exception:
    WEBRTC_AVAILABLE = False
    MediaStreamTrack = object


if WEBRTC_AVAILABLE:
    class NvencH264Encoder(_Libx264Encoder):
        """Keep aiortc's H.264 packetizer while encoding on NVENC."""

        def _encode_frame(self, frame, force_keyframe):
            if self.codec and (
                frame.width != self.codec.width
                or frame.height != self.codec.height
                or abs(self.target_bitrate - self.codec.bit_rate) / self.codec.bit_rate > 0.1
            ):
                self.buffer_data = b""
                self.buffer_pts = None
                self.codec = None
            frame.pict_type = (
                av.video.frame.PictureType.I if force_keyframe else av.video.frame.PictureType.NONE
            )
            if self.codec is None:
                try:
                    self.codec = av.CodecContext.create("h264_nvenc", "w")
                    self.codec.options = {"preset": "llhp", "tune": "ll", "zerolatency": "1"}
                except Exception:
                    self.codec = av.CodecContext.create("libx264", "w")
                    self.codec.options = {"level": "31", "tune": "zerolatency"}
                    self.codec.profile = "Baseline"
                self.codec.width = frame.width
                self.codec.height = frame.height
                self.codec.bit_rate = self.target_bitrate
                self.codec.pix_fmt = "yuv420p"
                self.codec.framerate = fractions.Fraction(30, 1)
                self.codec.time_base = fractions.Fraction(1, 30)
            data = b""
            for package in self.codec.encode(frame):
                data += bytes(package)
            if data:
                yield from self._split_bitstream(data)


class FrameHub:
    def __init__(self, stale_after=2.0):
        self.stale_after = float(stale_after)
        self.lock = threading.RLock()
        self.frames = {}
        self.stamps = {}
        self.rates = {camera: collections.deque(maxlen=180) for camera in ("d455", "dabai")}
        self.received = {camera: 0 for camera in ("d455", "dabai")}
        self.jpeg = {}
        self.jpeg_stamps = {}
        self.subscribers = {camera: 0 for camera in ("d455", "dabai")}
        self.peers = {camera: 0 for camera in ("d455", "dabai")}
        self.bitrate_kbps = {camera: 0.0 for camera in ("d455", "dabai")}

    def update(self, camera, pixels, stamp):
        now = time.monotonic()
        with self.lock:
            self.frames[camera] = pixels.copy()
            self.stamps[camera] = (stamp, now)
            self.received[camera] += 1
            samples = self.rates[camera]
            samples.append(now)
            while len(samples) > 2 and samples[0] < now - 2:
                samples.popleft()

    def latest(self, camera):
        with self.lock:
            return None if camera not in self.frames else self.frames[camera].copy()

    def latest_jpeg(self, camera):
        with self.lock:
            return self.jpeg.get(camera), self.jpeg_stamps.get(camera, 0.0)

    def set_jpeg(self, camera, payload, stamp):
        with self.lock:
            self.jpeg[camera] = payload
            self.jpeg_stamps[camera] = stamp

    def input_fps(self, camera):
        with self.lock:
            samples = self.rates[camera]
            if len(samples) < 2:
                return 0.0
            return max(0.0, (len(samples) - 1) / max(samples[-1] - samples[0], 1e-6))

    def frame_age(self, camera):
        with self.lock:
            stamp = self.stamps.get(camera)
        return None if not stamp else max(0.0, time.monotonic() - stamp[1])

    def health(self):
        now = time.monotonic()
        cameras = {}
        for camera in ("d455", "dabai"):
            age = self.frame_age(camera)
            with self.lock:
                stamp_pair = self.stamps.get(camera)
                jpeg_pair = self.jpeg_stamps.get(camera, 0.0)
                cameras[camera] = {
                    "state": "streaming" if age is not None and age <= self.stale_after else "waiting",
                    "input_fps": round(self.input_fps(camera), 1),
                    "frame_age_s": None if age is None else round(age, 2),
                    "last_frame_stamp": None if not stamp_pair else stamp_pair[0],
                    "jpeg_age_s": None if not jpeg_pair else round(max(0.0, now - jpeg_pair), 2),
                    "subscribers": self.subscribers[camera],
                    "webrtc_peers": self.peers[camera],
                    "bitrate_kbps": round(self.bitrate_kbps[camera], 1),
                    "codec": "H.264 / JPEG" if WEBRTC_AVAILABLE else "JPEG",
                    "resolution": [640, 480],
                    "received_frames": self.received[camera],
                }
        return {
            "state": "online",
            "webrtc_available": WEBRTC_AVAILABLE,
            "ros_available": ROS_AVAILABLE,
            "cameras": cameras,
        }


if ROS_AVAILABLE:
    class CameraNode(Node):
        def __init__(self, hub):
            super().__init__("piper_workbench_media")
            self.hub = hub
            self.bridge = CvBridge()
            self.image_subscriptions = []
            for camera, topic in (
                ("d455", "/camera/camera/color/image_raw"),
                ("dabai", "/dabai/color/image_raw"),
            ):
                self.image_subscriptions.append(self.create_subscription(
                    Image, topic,
                    lambda message, camera_id=camera: self.on_image(camera_id, message),
                    qos_profile_sensor_data,
                ))

        def on_image(self, camera, message):
            try:
                pixels = self.bridge.imgmsg_to_cv2(message, "bgr8")
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                self.hub.update(camera, pixels, stamp)
            except Exception as exc:
                self.get_logger().warning(f"{camera} image decode failed: {exc}")


if WEBRTC_AVAILABLE:
    class RosVideoTrack(MediaStreamTrack):
        kind = "video"

        def __init__(self, hub, camera, fps=15):
            super().__init__()
            self.hub = hub
            self.camera = camera
            self.fps = max(1, int(fps))
            self.started = time.monotonic()
            self.last_sent = 0.0

        async def recv(self):
            loop = asyncio.get_running_loop()
            wait = max(0.0, (1 / self.fps) - (time.monotonic() - self.last_sent))
            if wait:
                await asyncio.sleep(wait)
            pixels = await loop.run_in_executor(None, self.hub.latest, self.camera)
            if pixels is None:
                pixels = np.zeros((480, 640, 3), np.uint8)
            self.last_sent = time.monotonic()
            frame = av.VideoFrame.from_ndarray(pixels, format="bgr24")
            frame.pts = int((time.monotonic() - self.started) * 90000)
            frame.time_base = Fraction(1, 90000)
            return frame


class MediaService:
    def __init__(self, root=None, config=None):
        root_path = pathlib.Path(root) if root else None
        self.config = config or load_config((root_path or pathlib.Path(__file__).resolve().parents[2]) / "config/tablet_hmi.json")
        self.paths = TabletPaths(root, self.config)
        media = self.config.get("media", {})
        self.hub = FrameHub(media.get("inactive_after_s", 2.0))
        self.jpeg_fps = int(media.get("jpeg_fps", 6))
        self.jpeg_quality = int(media.get("jpeg_quality", 72))
        self.webrtc_fps = int(media.get("webrtc_fps", 15))
        self.h264_encoder = str(media.get("h264_encoder", "nvenc"))
        self.node = None
        self.executor = None
        self.ros_thread = None
        self.peers = {}
        self.tasks = []

    async def start(self):
        if WEBRTC_AVAILABLE and self.h264_encoder == "nvenc":
            _aiortc_codecs.H264Encoder = NvencH264Encoder
        if ROS_AVAILABLE:
            rclpy.init()
            self.node = CameraNode(self.hub)
            self.executor = MultiThreadedExecutor(num_threads=2)
            self.executor.add_node(self.node)
            self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.ros_thread.start()
        self.tasks = [
            asyncio.create_task(self._encode_loop()),
            asyncio.create_task(self._stats_loop()),
        ]

    async def stop(self):
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        for peer in list(self.peers.values()):
            await peer["pc"].close()
        self.peers.clear()
        if self.executor:
            self.executor.shutdown()
        if self.node:
            self.node.destroy_node()
        if ROS_AVAILABLE and rclpy.ok():
            rclpy.shutdown()

    async def _encode_loop(self):
        interval = 1 / max(self.jpeg_fps, 1)
        while True:
            started = time.monotonic()
            for camera in ("d455", "dabai"):
                age = self.hub.frame_age(camera)
                if age is None or age > 2.5:
                    continue
                pixels = await asyncio.to_thread(self.hub.latest, camera)
                if pixels is None:
                    continue
                ok, encoded = await asyncio.to_thread(
                    cv2.imencode, ".jpg", pixels, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                )
                if ok:
                    self.hub.set_jpeg(camera, encoded.tobytes(), time.monotonic())
            await asyncio.sleep(max(0.0, interval - (time.monotonic() - started)))

    async def _stats_loop(self):
        previous = {}
        while True:
            await asyncio.sleep(2.0)
            totals = {"d455": 0, "dabai": 0}
            for peer in list(self.peers.values()):
                try:
                    stats = await peer["pc"].getStats()
                    for report in stats.values():
                        if report.type == "outbound-rtp" and report.kind == "video":
                            totals[peer["camera"]] += int(getattr(report, "bytesSent", 0))
                            break
                except Exception:
                    continue
            for camera in totals:
                old = previous.get(camera)
                self.hub.bitrate_kbps[camera] = max(0.0, (totals[camera] - old) * 8 / 1000 / 2.0) if old is not None else 0.0
            previous = totals
            self.hub.peers["d455"] = sum(1 for peer in self.peers.values() if peer["camera"] == "d455")
            self.hub.peers["dabai"] = sum(1 for peer in self.peers.values() if peer["camera"] == "dabai")

    def authorized(self, request, camera):
        return verify_media_token(
            self.paths,
            request.query.get("device_id", ""),
            camera,
            request.query.get("expires", ""),
            request.query.get("signature", ""),
        )

    async def health(self, request):
        payload = self.hub.health()
        payload["h264_encoder"] = self.h264_encoder
        return web.json_response(payload)

    async def mjpeg(self, request):
        camera = request.match_info["camera"]
        if camera not in ("d455", "dabai") or not self.authorized(request, camera):
            raise web.HTTPForbidden(text="invalid media token")
        response = web.StreamResponse(headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-store, no-cache, must-revalidate",
            "Pragma": "no-cache",
        })
        await response.prepare(request)
        self.hub.subscribers[camera] += 1
        last_stamp = 0.0
        try:
            while True:
                payload, stamp = self.hub.latest_jpeg(camera)
                if payload is None:
                    if self.hub.frame_age(camera) is not None:
                        await asyncio.sleep(.05)
                        continue
                    await asyncio.sleep(.05)
                    continue
                if stamp != last_stamp:
                    await response.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                        str(len(payload)).encode() + b"\r\n\r\n" + payload + b"\r\n"
                    )
                    last_stamp = stamp
                await asyncio.sleep(1 / max(self.jpeg_fps, 1))
        except (ConnectionResetError, RuntimeError, asyncio.CancelledError):
            pass
        finally:
            self.hub.subscribers[camera] = max(0, self.hub.subscribers[camera] - 1)
        return response

    async def jpeg_ws(self, request):
        camera = request.match_info["camera"]
        if camera not in ("d455", "dabai") or not self.authorized(request, camera):
            raise web.HTTPForbidden(text="invalid media token")
        ws = web.WebSocketResponse(heartbeat=20, max_msg_size=2 * 1024 * 1024)
        await ws.prepare(request)
        self.hub.subscribers[camera] += 1
        last_stamp = 0.0
        try:
            while not ws.closed:
                payload, stamp = self.hub.latest_jpeg(camera)
                if payload is not None and stamp != last_stamp:
                    await ws.send_bytes(payload)
                    last_stamp = stamp
                await asyncio.sleep(1 / max(self.jpeg_fps, 1))
        except (ConnectionResetError, RuntimeError, asyncio.CancelledError):
            pass
        finally:
            self.hub.subscribers[camera] = max(0, self.hub.subscribers[camera] - 1)
        return ws

    async def snapshot(self, request):
        camera = request.match_info["camera"]
        if camera not in ("d455", "dabai") or not self.authorized(request, camera):
            raise web.HTTPForbidden(text="invalid media token")
        payload, _ = self.hub.latest_jpeg(camera)
        if payload is None:
            raise web.HTTPNotFound(text="camera has no frame")
        return web.Response(body=payload, content_type="image/jpeg", headers={"Cache-Control": "no-store"})

    async def whep(self, request):
        camera = request.match_info["camera"]
        if not WEBRTC_AVAILABLE:
            raise web.HTTPServiceUnavailable(text="WebRTC is unavailable; use JPEG fallback")
        if camera not in ("d455", "dabai") or not self.authorized(request, camera):
            raise web.HTTPForbidden(text="invalid media token")
        offer = await request.text()
        if not offer.strip():
            raise web.HTTPBadRequest(text="SDP offer is empty")
        pc = RTCPeerConnection()
        track = RosVideoTrack(self.hub, camera, self.webrtc_fps)
        sender = pc.addTrack(track)
        if pc.getTransceivers():
            capabilities = RTCRtpSender.getCapabilities("video")
            codecs = list(capabilities.codecs)
            h264 = [item for item in codecs if item.mimeType.lower() == "video/h264"]
            rest = [item for item in codecs if item.mimeType.lower() != "video/h264"]
            if h264:
                pc.getTransceivers()[0].setCodecPreferences(h264 + rest)
        session_id = uuid.uuid4().hex
        self.peers[session_id] = {"pc": pc, "sender": sender, "camera": camera, "created_at": time.monotonic()}
        self.hub.peers[camera] += 1
        removed = False

        def remove_peer():
            nonlocal removed
            if removed:
                return
            removed = True
            if self.peers.pop(session_id, None):
                self.hub.peers[camera] = max(0, self.hub.peers[camera] - 1)

        @pc.on("connectionstatechange")
        async def state_changed():
            if pc.connectionState in ("failed", "closed"):
                remove_peer()
                await pc.close()

        try:
            await pc.setRemoteDescription(RTCSessionDescription(sdp=offer, type="offer"))
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
        except Exception as exc:
            remove_peer()
            await pc.close()
            raise web.HTTPBadRequest(text=f"WebRTC offer rejected: {exc}")
        deadline = time.monotonic() + 4
        while pc.iceGatheringState != "complete" and time.monotonic() < deadline:
            await asyncio.sleep(.05)
        return web.json_response({
            "type": "answer",
            "sdp": pc.localDescription.sdp,
            "session_id": session_id,
            "camera": camera,
            "codec": "H264" if "H264" in pc.localDescription.sdp else "fallback",
        })

    async def whep_delete(self, request):
        session_id = request.match_info["session"]
        peer = self.peers.pop(session_id, None)
        if peer:
            self.hub.peers[peer["camera"]] = max(0, self.hub.peers[peer["camera"]] - 1)
            await peer["pc"].close()
        return web.json_response({"ok": True, "closed": bool(peer)})

    async def keyframe(self, request):
        camera = request.match_info["camera"]
        requested = 0
        for peer in self.peers.values():
            if peer["camera"] == camera:
                request_keyframe = getattr(peer.get("sender"), "_send_keyframe", None)
                if request_keyframe:
                    request_keyframe()
                    requested += 1
        return web.json_response({"ok": True, "camera": camera, "peers_requested": requested})


APP_MEDIA = web.AppKey("media", MediaService)


def create_app(root=None, config=None):
    service = MediaService(root, config)

    async def startup(app):
        await service.start()

    async def cleanup(app):
        await service.stop()

    app = web.Application(client_max_size=2 * 1024 * 1024)
    app[APP_MEDIA] = service
    app.on_startup.append(startup)
    app.on_cleanup.append(cleanup)
    app.router.add_get("/health", service.health)
    app.router.add_get("/mjpeg/{camera}", service.mjpeg)
    app.router.add_get("/ws/jpeg/{camera}", service.jpeg_ws)
    app.router.add_get("/snapshot/{camera}", service.snapshot)
    app.router.add_post("/whep/{camera}", service.whep)
    app.router.add_delete("/whep/{session}", service.whep_delete)
    app.router.add_post("/keyframe/{camera}", service.keyframe)
    return app


async def serve(config=None):
    config = config or load_config()
    bind = config.get("media", {}).get("bind", "127.0.0.1")
    port = int(config.get("media", {}).get("port", 8090))
    app = create_app(config=config)
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    site = web.TCPSite(runner, bind, port)
    await site.start()
    print(f"[tablet-media] http://{bind}:{port} webrtc={WEBRTC_AVAILABLE} ros={ROS_AVAILABLE}", flush=True)
    try:
        await asyncio.Event().wait()
    finally:
        await runner.cleanup()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Piper tablet media service")
    parser.parse_args(argv)
    try:
        asyncio.run(serve())
    except KeyboardInterrupt:
        return 0
    return 0

