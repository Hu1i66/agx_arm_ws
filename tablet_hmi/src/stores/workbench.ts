import { computed, ref, shallowRef } from "vue";
import { defineStore } from "pinia";
import type { DetectionObject, DetectionPayload, Snapshot } from "../types";

type ActionOptions = { requiresLease?: boolean };

function requestId() {
  return crypto.randomUUID ? crypto.randomUUID() : `${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

export const useWorkbench = defineStore("workbench", () => {
  const snapshot = shallowRef<Snapshot>({});
  const socketState = ref<"connecting" | "online" | "offline">("connecting");
  const socketLatency = ref(0);
  const clock = ref(Date.now() / 1000);
  const error = ref("");
  const selectedCamera = ref<"d455" | "dabai">("d455");
  const selectedTrack = ref<number | null>(null);
  const selectedSession = ref("");
  const selectedTargetSnapshot = shallowRef<DetectionObject | null>(null);
  const pending = ref(0);
  const componentErrors = ref<Record<string, string>>({});
  const graspStage = ref("waiting");
  const graspLabel = ref("等待开始");
  let socket: WebSocket | null = null;
  let reconnectTimer = 0;
  let heartbeatTimer = 0;
  let leaseTimer = 0;
  let clockTimer = 0;
  let reconnectDelay = 800;
  let heartbeatSent = 0;
  let stopped = false;

  const device = computed(() => snapshot.value.device || null);
  const lease = computed(() => snapshot.value.lease || null);
  const isOperator = computed(() => device.value?.role === "operator");
  const hasLease = computed(() => !!lease.value && lease.value.device_id === device.value?.id);
  const detections = computed<Record<string, DetectionPayload>>(() => snapshot.value.detections || {});
  const media = computed<Record<string, any>>(() => snapshot.value.media || {});
  const workbench = computed<Record<string, any>>(() => snapshot.value.workbench || {});
  const components = computed<any[]>(() => workbench.value.components || []);
  const task = computed<Record<string, any>>(() => workbench.value.task || {});

  function updateLease(value: any) {
    snapshot.value = { ...snapshot.value, lease: value || null };
  }

  function handleEvent(row: any) {
    if (row.type === "snapshot") {
      snapshot.value = row.data;
      if (selectedTrack.value !== null && !currentTarget.value) {
        selectedTrack.value = null;
        selectedSession.value = "";
      }
      return;
    }
    if (row.type === "heartbeat") {
      socketLatency.value = Math.max(1, Math.round(performance.now() - heartbeatSent));
      return;
    }
    if (row.type === "robot.link_poses") {
      snapshot.value = { ...snapshot.value, robot: row.data };
      return;
    }
    if (row.type === "lease.changed") {
      updateLease(row.data);
      return;
    }
    if (row.type === "grasp.stage") {
      graspStage.value = String(row.data?.stage || "waiting");
      graspLabel.value = String(row.data?.label || "");
      return;
    }
    if (row.type === "task.status" && row.data?.result) {
      const okay = row.data.result.ok || Number(row.data.result.cycle_success || 0) > 0;
      graspStage.value = okay ? "complete" : "error";
      graspLabel.value = okay ? "本轮完成" : "本轮异常";
      window.setTimeout(() => {
        if (graspStage.value === "complete") {
          graspStage.value = "waiting";
          graspLabel.value = "等待下一轮";
        }
      }, 1600);
      refreshSoon();
      return;
    }
    if (row.type === "task.status" && row.data?.action_state) {
      const state = String(row.data.action_state);
      if (state === "busy" && ["waiting", "target"].includes(graspStage.value)) {
        graspStage.value = "approach";
        graspLabel.value = "动作程序执行中";
      } else if (state === "error") {
        graspStage.value = "error";
        graspLabel.value = "动作程序报错";
      }
      refreshSoon();
      return;
    }
    if (row.type === "vision.targets" && row.data?.camera_id) {
      const camera = String(row.data.camera_id);
      const next = { ...(snapshot.value.detections || {}), [row.data.camera_id]: row.data.detection || {} };
      snapshot.value = { ...snapshot.value, detections: next };
      if (camera === selectedCamera.value && selectedTrack.value !== null) {
        const session = String(row.data.detection?.motion_session || "");
        if (selectedSession.value && session && session !== selectedSession.value) {
          selectedTrack.value = null;
          selectedSession.value = "";
          selectedTargetSnapshot.value = null;
        } else {
          const object = (row.data.detection?.objects || []).find(
            (item: DetectionObject) => Number(item.track_id) === selectedTrack.value,
          );
          if (object) selectedTargetSnapshot.value = object;
        }
      }
      return;
    }
    if (["component.status", "task.status", "arm.recovery.progress", "settings.applied"].includes(row.type)) {
      if (row.type === "component.status" && row.data?.error) {
        const key = `${row.data.component}:${row.data.camera_id || ""}`;
        componentErrors.value = { ...componentErrors.value, [key]: String(row.data.error) };
      } else if (row.type === "component.status") {
        const key = `${row.data.component}:${row.data.camera_id || ""}`;
        if (componentErrors.value[key]) {
          const next = { ...componentErrors.value };
          delete next[key];
          componentErrors.value = next;
        }
      }
      refreshSoon();
    }
  }

  function connect() {
    stopped = false;
    socketState.value = "connecting";
    const protocol = location.protocol === "https:" ? "wss:" : "ws:";
    socket = new WebSocket(`${protocol}//${location.host}/ws/v1/events`);
    socket.onopen = () => {
      socketState.value = "online";
      reconnectDelay = 800;
      window.clearInterval(heartbeatTimer);
      heartbeatTimer = window.setInterval(() => {
        if (socket?.readyState !== WebSocket.OPEN) return;
        heartbeatSent = performance.now();
        socket.send(JSON.stringify({
          type: "heartbeat",
          page: location.hash || "#/grab",
          reconnect_reason: error.value,
          ws_rtt_ms: socketLatency.value,
          app_version: "0.3.0",
        }));
      }, 5000);
      window.clearInterval(leaseTimer);
      leaseTimer = window.setInterval(() => {
        if (document.visibilityState === "visible" && isOperator.value && hasLease.value) acquireLease(true, true);
      }, 5000);
      window.clearInterval(clockTimer);
      clockTimer = window.setInterval(() => { clock.value = Date.now() / 1000; }, 1000);
    };
    socket.onmessage = (event) => {
      try {
        handleEvent(JSON.parse(event.data));
      } catch {
        error.value = "收到无法解析的实时状态";
      }
    };
    socket.onerror = () => {
      socketState.value = "offline";
    };
    socket.onclose = (event) => {
      socketState.value = "offline";
      if (event.code === 1008) {
        error.value = event.reason || "设备授权已被撤销";
        return;
      }
      if (stopped) return;
      window.clearTimeout(reconnectTimer);
      reconnectTimer = window.setTimeout(connect, reconnectDelay);
      reconnectDelay = Math.min(8000, reconnectDelay * 1.7);
    };
  }

  function disconnect() {
    stopped = true;
    window.clearTimeout(reconnectTimer);
    window.clearInterval(heartbeatTimer);
    window.clearInterval(leaseTimer);
    window.clearInterval(clockTimer);
    socket?.close();
    socket = null;
  }

  async function refresh() {
    const response = await fetch("/api/v1/session", { cache: "no-store" });
    if (response.status === 401) {
      error.value = "设备尚未配对";
      return false;
    }
    const body = await response.json();
    if (!response.ok) throw new Error(body?.error?.message || "刷新状态失败");
    snapshot.value = body.snapshot;
    return true;
  }

  let refreshQueued = false;
  function refreshSoon() {
    if (refreshQueued) return;
    refreshQueued = true;
    window.setTimeout(async () => {
      refreshQueued = false;
      try { await refresh(); } catch { /* WebSocket reconnection remains authoritative. */ }
    }, 250);
  }

  async function acquireLease(silent = false, renew = false) {
    if (!isOperator.value) throw new Error("观察者设备为只读模式");
    if (hasLease.value && !renew) return lease.value;
    if (!silent) pending.value++;
    try {
      const response = await fetch("/api/v1/lease", {
        method: "POST",
        headers: { "Content-Type": "application/json", "X-Piper-Action": "1" },
        body: JSON.stringify({ action: "acquire" }),
      });
      const body = await response.json();
      if (!response.ok) {
        if (body?.lease) updateLease(body.lease);
        throw new Error(body?.error?.message || "取得控制权失败");
      }
      updateLease(body.lease);
      return body.lease;
    } finally {
      if (!silent) pending.value = Math.max(0, pending.value - 1);
    }
  }

  async function releaseLease() {
    if (!hasLease.value) return;
    try {
      const response = await fetch("/api/v1/lease", {
        method: "POST",
        headers: { "Content-Type": "application/json", "X-Piper-Action": "1" },
        body: JSON.stringify({ action: "release" }),
        keepalive: true,
      });
      const body = await response.json();
      if (response.ok) updateLease(body.lease);
    } catch {
      // Server expiry releases the lease even when background tabs suspend fetch.
    }
  }

  async function action(op: string, args: Record<string, any> = {}, options: ActionOptions = {}) {
    const requiresLease = options.requiresLease !== false;
    if (requiresLease) await acquireLease();
    pending.value++;
    try {
      const response = await fetch("/api/v1/actions", {
        method: "POST",
        headers: { "Content-Type": "application/json", "X-Piper-Action": "1" },
        body: JSON.stringify({ op, args, request_id: requestId() }),
      });
      const body = await response.json();
      if (!response.ok) {
        if (body?.needs_lease) await acquireLease();
        throw new Error(body?.error?.message || "操作未完成");
      }
      refreshSoon();
      return body.result;
    } finally {
      pending.value = Math.max(0, pending.value - 1);
    }
  }

  async function pair(code: string, name: string) {
    const response = await fetch("/api/v1/pair", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ code, name }),
    });
    const body = await response.json();
    if (!response.ok) throw new Error(body?.error?.message || "配对失败");
    history.replaceState(null, "", "/");
    await refresh();
    connect();
  }

  const currentTarget = computed(() => {
    const data = detections.value[selectedCamera.value];
    if (selectedTrack.value === null) return null;
    return (data?.objects || []).find((item) => Number(item.track_id) === selectedTrack.value)
      || selectedTargetSnapshot.value;
  });

  function selectTarget(camera: "d455" | "dabai", trackId: number) {
    const data = detections.value[camera];
    selectedCamera.value = camera;
    selectedTrack.value = trackId;
    selectedSession.value = String(data?.motion_session || "");
    selectedTargetSnapshot.value = (data?.objects || []).find((item) => Number(item.track_id) === trackId) || null;
  }

  function setError(message: string) {
    error.value = message;
  }

  function clearError() {
    error.value = "";
  }

  return {
    snapshot, socketState, socketLatency, clock, error, selectedCamera, selectedTrack,
    selectedSession, selectedTargetSnapshot, pending, componentErrors, graspStage, graspLabel, device, lease, isOperator, hasLease, detections,
    media, workbench, components, task, currentTarget,
    connect, disconnect, refresh, acquireLease, releaseLease, action, pair,
    selectTarget, setError, clearError, updateLease,
  };
});
