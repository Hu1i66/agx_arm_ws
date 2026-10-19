<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from "vue";
import { Camera, PauseCircle, Radio } from "lucide-vue-next";
import type { DetectionObject } from "../types";
import { useWorkbench } from "../stores/workbench";

const props = defineProps<{ camera: "d455" | "dabai" }>();
const store = useWorkbench();
const video = ref<HTMLVideoElement | null>(null);
const canvas = ref<HTMLCanvasElement | null>(null);
const wrapper = ref<HTMLElement | null>(null);
const mode = ref<"connecting" | "webrtc" | "jpeg" | "paused">("connecting");
const fallbackUrl = ref("");
const jpegFrame = ref("");
const jpegReloadKey = ref(0);
const streamError = ref("");
let pc: RTCPeerConnection | null = null;
let sessionId = "";
let whepBase = "";
let disposed = false;
let fallbackTimer = 0;
let watchdogTimer = 0;
let lastVideoFrames = -1;
let lastVideoTime = 0;
let lastVideoAdvance = 0;
let jpegSocket: WebSocket | null = null;
let jpegFrameUrl = "";
let jpegRetryTimer = 0;
let jpegWatchdog = 0;
let lastJpegAt = 0;

const label = computed(() => props.camera === "d455" ? "D455" : "Dabai");
const subtitle = computed(() => props.camera === "d455" ? "固定相机" : "腕载相机");
const detection = computed(() => store.detections[props.camera] || {});
const cameraHealth = computed(() => store.media?.cameras?.[props.camera] || {});
const objects = computed<DetectionObject[]>(() => (detection.value.objects || []) as DetectionObject[]);
const framePaused = computed(() => mode.value === "paused" && Number(cameraHealth.value.frame_age_s || 0) > 1);
const selected = computed(() => store.selectedCamera === props.camera ? store.selectedTrack : null);

function drawOverlay() {
  const element = canvas.value;
  if (!element) return;
  const context = element.getContext("2d");
  if (!context) return;
  context.clearRect(0, 0, 640, 480);
  for (const [index, object] of objects.value.entries()) {
    const bbox = object.bbox_pixel as { x1: number; y1: number; x2: number; y2: number } | undefined;
    if (!bbox) continue;
    const active = Number(object.track_id) === Number(selected.value);
    const color = active ? "#1769ff" : props.camera === "d455" ? "#20b486" : "#5a8dee";
    const padding = active ? 8 : 2;
    context.strokeStyle = color;
    context.lineWidth = active ? 3 : 2;
    context.shadowColor = active ? "rgba(23,105,255,.45)" : "transparent";
    context.shadowBlur = active ? 10 : 0;
    context.strokeRect(
      bbox.x1 - padding,
      bbox.y1 - padding,
      Math.max(1, bbox.x2 - bbox.x1 + padding * 2),
      Math.max(1, bbox.y2 - bbox.y1 + padding * 2),
    );
    context.shadowBlur = 0;
    const name = String(object.object_name || object.raw_object_name || "目标");
    const confidence = Number(object.confidence || 0);
    const text = `#${object.track_id || index + 1} ${name} ${Math.round(confidence * 100)}%`;
    context.font = "600 14px ui-monospace, SFMono-Regular, Menlo, monospace";
    const width = context.measureText(text).width + 14;
    const top = Math.max(4, bbox.y1 - 28);
    context.fillStyle = active ? "rgba(23,105,255,.92)" : "rgba(17,34,56,.82)";
    context.beginPath();
    context.roundRect(Math.max(4, bbox.x1 - padding), top, width, 23, 5);
    context.fill();
    context.fillStyle = "#fff";
    context.fillText(text, Math.max(10, bbox.x1 - padding + 7), top + 16);
  }
}

function pause() {
  mode.value = "paused";
  streamError.value = props.camera === "d455" ? "D455 视频流已暂停" : "Dabai 视频流已暂停";
}

function fallbackToJpeg(message: string) {
  if (disposed) return;
  streamError.value = message;
  mode.value = "jpeg";
  window.clearInterval(watchdogTimer);
  if (pc && sessionId) fetch(`${whepBase}/${sessionId}`, { method: "DELETE", keepalive: true }).catch(() => undefined);
  pc?.close();
  pc = null;
  sessionId = "";
  if (!jpegSocket || jpegSocket.readyState >= WebSocket.CLOSING) startJpegSocket();
}

function startJpegSocket() {
  if (disposed || jpegSocket?.readyState === WebSocket.OPEN || jpegSocket?.readyState === WebSocket.CONNECTING) return;
  const socket = new WebSocket(`${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/ws/v1/jpeg/${props.camera}`);
  jpegSocket = socket;
  socket.binaryType = "blob";
  socket.onopen = () => {
    window.clearTimeout(jpegRetryTimer);
    jpegRetryTimer = 0;
  };
  socket.onmessage = (event) => {
    if (!(event.data instanceof Blob)) return;
    if (jpegFrameUrl) URL.revokeObjectURL(jpegFrameUrl);
    jpegFrameUrl = URL.createObjectURL(event.data);
    jpegFrame.value = jpegFrameUrl;
    lastJpegAt = performance.now();
  };
  socket.onerror = () => {
    if (mode.value === "jpeg") streamError.value = "WebSocket JPEG 不可用，正在使用 MJPEG";
  };
  socket.onclose = () => {
    if (jpegSocket !== socket) return;
    jpegSocket = null;
    if (disposed || mode.value !== "jpeg" || jpegRetryTimer) return;
    jpegRetryTimer = window.setTimeout(() => {
      jpegRetryTimer = 0;
      startJpegSocket();
    }, 1500);
  };
}

function stopJpegSocket() {
  window.clearTimeout(jpegRetryTimer);
  jpegRetryTimer = 0;
  window.clearInterval(jpegWatchdog);
  jpegWatchdog = 0;
  jpegSocket?.close();
  jpegSocket = null;
}

async function startWebRTC(url: string) {
  if (typeof RTCPeerConnection === "undefined") throw new Error("浏览器不支持 WebRTC");
  const peer = new RTCPeerConnection({ iceServers: [] });
  pc = peer;
  const transceiver = peer.addTransceiver("video", { direction: "recvonly" });
  const capabilities = RTCRtpSender.getCapabilities?.("video");
  if (capabilities) {
    const h264 = capabilities.codecs.filter((codec) => codec.mimeType.toLowerCase() === "video/h264");
    if (h264.length) transceiver.setCodecPreferences(h264);
  }
  peer.ontrack = (event) => {
    if (!video.value) return;
    const element = video.value;
    element.srcObject = event.streams[0] || new MediaStream([event.track]);
    const ready = () => {
      if (disposed || mode.value === "jpeg") return;
      mode.value = "webrtc";
      streamError.value = "";
      stopJpegSocket();
      lastVideoAdvance = performance.now();
      lastVideoTime = 0;
      window.clearTimeout(fallbackTimer);
      element.play().catch(() => fallbackToJpeg("浏览器暂停了视频播放，已切换 JPEG"));
    };
    element.addEventListener("loadeddata", ready, { once: true });
    element.addEventListener("playing", ready, { once: true });
    element.play().catch(() => undefined);
    window.clearInterval(watchdogTimer);
    watchdogTimer = window.setInterval(() => {
      if (!video.value || mode.value !== "webrtc") return;
      const quality = video.value.getVideoPlaybackQuality?.();
      const frames = quality ? quality.totalVideoFrames : Math.round(video.value.currentTime * 30);
      if (frames !== lastVideoFrames || video.value.currentTime > lastVideoTime + 0.001) lastVideoAdvance = performance.now();
      lastVideoFrames = frames;
      lastVideoTime = video.value.currentTime;
      if (performance.now() - lastVideoAdvance > 2500 || video.value.readyState < 2) {
        fallbackToJpeg("H.264 视频暂时无新帧，已切换 JPEG");
      }
    }, 1000);
  };
  peer.onconnectionstatechange = () => {
    if (["failed", "disconnected"].includes(peer.connectionState)) fallbackToJpeg("WebRTC 已断开，自动切换 JPEG");
  };
  const offer = await peer.createOffer();
  await peer.setLocalDescription(offer);
  await new Promise<void>((resolve) => {
    if (peer.iceGatheringState === "complete") return resolve();
    const timeout = window.setTimeout(resolve, 800);
    peer.addEventListener("icegatheringstatechange", () => {
      if (peer.iceGatheringState === "complete") {
        window.clearTimeout(timeout);
        resolve();
      }
    });
  });
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/sdp" },
    body: peer.localDescription?.sdp || offer.sdp,
  });
  if (!response.ok) {
    const text = await response.text();
    try {
      const parsed = JSON.parse(text);
      throw new Error(parsed?.error?.message || text);
    } catch (error) {
      if (error instanceof SyntaxError) throw new Error(text || "WebRTC 建连失败");
      throw error;
    }
  }
  const body = await response.json();
  if (disposed || mode.value === "jpeg" || pc !== peer) {
    peer.close();
    return;
  }
  await peer.setRemoteDescription({ type: "answer", sdp: body.sdp });
  sessionId = body.session_id;
  whepBase = url.split("?")[0];
}

async function start() {
  try {
    const response = await fetch(`/api/v1/media/token?camera=${props.camera}`, { cache: "no-store" });
    const token = await response.json();
    if (!response.ok) throw new Error(token?.error?.message || "获取视频令牌失败");
    fallbackUrl.value = token.mjpeg_url;
    if (new URLSearchParams(location.search).get("transport") === "jpeg") {
      fallbackToJpeg("当前使用 JPEG 低带宽模式");
      return;
    }
    fallbackTimer = window.setTimeout(() => {
      if (!disposed && mode.value === "connecting") fallbackToJpeg("WebRTC 建连超时，已切换 JPEG");
    }, 3000);
    await startWebRTC(token.whep_url);
  } catch (err) {
    if (disposed) return;
    window.clearTimeout(fallbackTimer);
    fallbackToJpeg(err instanceof Error ? err.message : String(err));
  }
}

function choose(event: MouseEvent) {
  const rect = canvas.value?.getBoundingClientRect();
  if (!rect) return;
  const x = (event.clientX - rect.left) * 640 / rect.width;
  const y = (event.clientY - rect.top) * 480 / rect.height;
  let best: DetectionObject | null = null;
  let distance = Infinity;
  for (const object of objects.value) {
    const box = object.bbox_pixel as { x1: number; y1: number; x2: number; y2: number } | undefined;
    if (!box) continue;
    const expanded = 8;
    if (x >= box.x1 - expanded && x <= box.x2 + expanded && y >= box.y1 - expanded && y <= box.y2 + expanded) {
      const centerX = (box.x1 + box.x2) / 2;
      const centerY = (box.y1 + box.y2) / 2;
      const dist = (centerX - x) ** 2 + (centerY - y) ** 2;
      if (dist < distance && object.track_id !== undefined) {
        best = object;
        distance = dist;
      }
    }
  }
  if (best?.track_id !== undefined) store.selectTarget(props.camera, Number(best.track_id));
}

function visibilityChanged() {
  if (document.visibilityState === "hidden" && (mode.value === "webrtc" || mode.value === "connecting")) {
    window.clearTimeout(fallbackTimer);
    window.clearInterval(watchdogTimer);
    stopJpegSocket();
    if (pc && sessionId) fetch(`${whepBase}/${sessionId}`, { method: "DELETE", keepalive: true }).catch(() => undefined);
    pc?.close();
    pc = null;
    sessionId = "";
    mode.value = "paused";
  } else if (document.visibilityState === "visible" && mode.value === "paused") {
    mode.value = "connecting";
    streamError.value = "";
    start();
  }
}

watch(objects, drawOverlay, { deep: true });
watch(() => store.selectedTrack, drawOverlay);
watch(mode, (value) => {
  window.clearInterval(jpegWatchdog);
  jpegWatchdog = 0;
  if (value !== "jpeg") return;
  lastJpegAt = performance.now();
  jpegWatchdog = window.setInterval(() => {
    if (performance.now() - lastJpegAt > 3000) {
      if (!jpegSocket || jpegSocket.readyState >= WebSocket.CLOSING) startJpegSocket();
      if (!jpegFrame.value) jpegReloadKey.value++;
    }
  }, 2000);
});
onMounted(() => {
  document.addEventListener("visibilitychange", visibilityChanged);
  start();
});
onBeforeUnmount(() => {
  disposed = true;
  document.removeEventListener("visibilitychange", visibilityChanged);
  window.clearTimeout(fallbackTimer);
  window.clearInterval(watchdogTimer);
  stopJpegSocket();
  if (jpegFrameUrl) URL.revokeObjectURL(jpegFrameUrl);
  if (pc && sessionId) fetch(`${whepBase}/${sessionId}`, { method: "DELETE", keepalive: true }).catch(() => undefined);
  pc?.close();
  pc = null;
});
</script>

<template>
  <article class="camera-card panel" :class="{ selected: selected !== null }">
    <header class="camera-head">
      <div class="camera-title">
        <span class="icon-tile"><Camera :size="19" /></span>
        <div><h3>{{ label }}</h3><p>{{ subtitle }}</p></div>
      </div>
      <div class="stream-pill" :class="mode">
        <Radio :size="13" />
        {{ mode === "webrtc" ? "H.264" : mode === "jpeg" ? "JPEG 降级" : mode === "paused" ? "已暂停" : "建连中" }}
      </div>
    </header>
    <div class="camera-stats">
      <span>640 × 480</span>
      <span>{{ Number(cameraHealth.input_fps || 0).toFixed(1) }} 帧/秒</span>
      <span>检测 {{ Number(detection.header_stamp || 0) ? `${Math.max(0, Number(store.clock) - Number(detection.header_stamp)).toFixed(2)} 秒前` : "等待" }}</span>
      <span>{{ objects.length }} 个目标</span>
    </div>
    <div ref="wrapper" class="camera-viewport" @click="choose">
      <!-- Keep the video element visible during ICE setup; Safari can refuse to decode a display:none stream. -->
      <video ref="video" autoplay muted playsinline></video>
      <img v-if="mode === 'jpeg'" :key="jpegReloadKey" :src="jpegFrame || fallbackUrl" alt="" />
      <div v-if="mode !== 'webrtc' && mode !== 'jpeg'" class="camera-waiting">
        <span class="pulse-ring"></span>
        <p>正在建立低延迟视频</p>
      </div>
      <div v-if="framePaused" class="paused-ribbon">
        <PauseCircle :size="15" />
        画面已暂停 · 上次更新 {{ Math.round(Number(cameraHealth.frame_age_s || 0)) }} 秒前
      </div>
      <canvas ref="canvas" width="640" height="480"></canvas>
    </div>
    <p v-if="selected !== null" class="selected-hint">
      已选择目标 #{{ selected }} · {{ store.currentTarget?.object_name || "目标" }}
    </p>
    <p v-else-if="streamError" class="stream-note">{{ streamError }}</p>
    <p v-else class="stream-note">点击检测框选择本次抓取目标</p>
  </article>
</template>
