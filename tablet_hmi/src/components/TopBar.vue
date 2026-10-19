<script setup lang="ts">
import { computed, ref } from "vue";
import { Eye, Home, OctagonX, RotateCcw } from "lucide-vue-next";
import { useWorkbench } from "../stores/workbench";
import StatusBadge from "./StatusBadge.vue";

const store = useWorkbench();
const busy = ref("");
type Tone = "ok" | "warn" | "error" | "idle";
type Badge = { label: string; value: string; tone: Tone };
const counts = computed(() => {
  const workbench = store.workbench;
  const cameras = workbench.cameras || {};
  const nodes = new Set(workbench.nodes || []);
  const conveyor = workbench.conveyor || {};
  return {
    arm: !!workbench.feedback,
    d455: cameras.d455?.state === "receiving",
    dabai: cameras.dabai?.state === "receiving",
    yolo: nodes.has("object_detector"),
    moveit: nodes.has("move_group"),
    action: nodes.has("piper_moveit_action_client"),
    conveyor: !!conveyor.connected,
  };
});

const badges = computed<Badge[]>(() => {
  const cameraState = (v: boolean): Pick<Badge, "value" | "tone"> => ({ value: v ? "有画面" : "等待数据", tone: v ? "ok" : "warn" });
  const mediaState = store.media?.state === "online";
  return [
    { label: "机械臂", value: counts.value.arm ? "有反馈" : "等待反馈", tone: counts.value.arm ? "ok" : "warn" },
    { label: "D455", ...cameraState(counts.value.d455) },
    { label: "Dabai", ...cameraState(counts.value.dabai) },
    { label: "YOLO", value: counts.value.yolo ? "运行中" : "未运行", tone: counts.value.yolo ? "ok" : "idle" },
    { label: "MoveIt / T1", value: counts.value.moveit ? "运行中" : "未运行", tone: counts.value.moveit ? "ok" : "idle" },
    { label: "T5", value: counts.value.action ? "运行中" : "未运行", tone: counts.value.action ? "ok" : "idle" },
    { label: "传送带", value: counts.value.conveyor ? "已连接" : "未连接", tone: counts.value.conveyor ? "ok" : "idle" },
    { label: "网络", value: store.socketState === "online" ? `${store.socketLatency || "<"}ms` : "重连中", tone: store.socketState === "online" ? "ok" : "error" },
    { label: "网关", value: store.snapshot.gateway?.state === "online" ? "在线" : "离线", tone: store.snapshot.gateway?.state === "online" ? "ok" : "error" },
    { label: "媒体", value: mediaState ? "在线" : "降级等待", tone: mediaState ? "ok" : "warn" },
  ];
});

async function send(cmd: "standby" | "observe" | "reset" | "emergency_stop") {
  if (busy.value) return;
  busy.value = cmd;
  try {
    await store.action("motion.send", {
      cmd,
      camera_id: store.commandCamera,
      dynamic: store.selectedMode === "d455-dynamic",
      workbench: true,
    }, { requiresLease: cmd !== "emergency_stop" });
  } catch (err) {
    store.setError(err instanceof Error ? err.message : String(err));
  } finally {
    busy.value = "";
  }
}
</script>

<template>
  <header class="topbar">
    <div class="brand-block">
      <div class="brand-mark"><i></i><i></i><i></i><i></i></div>
      <div>
        <strong>Piper 工作台</strong>
        <small>{{ store.device?.role === "observer" ? "观察者终端" : "便携操作终端" }}</small>
      </div>
    </div>
    <div class="top-status-rail">
      <StatusBadge v-for="badge in badges" :key="badge.label" v-bind="badge" />
    </div>
    <div class="top-actions">
      <button class="button secondary" aria-label="待机位" title="待机位" :disabled="!!busy || !store.isOperator" @click="send('standby')">
        <Home :size="18" /><span>待机位</span>
      </button>
      <button class="button secondary" aria-label="观察位" title="观察位" :disabled="!!busy || !store.isOperator" @click="send('observe')">
        <Eye :size="18" /><span>观察位</span>
      </button>
      <button class="button secondary" aria-label="复位" title="复位" :disabled="!!busy || !store.isOperator" @click="send('reset')">
        <RotateCcw :size="18" /><span>复位</span>
      </button>
      <button class="button danger emergency" aria-label="急停" title="急停" :disabled="!!busy || !store.isOperator" @click="send('emergency_stop')">
        <OctagonX :size="20" /><span>急停</span>
      </button>
    </div>
  </header>
</template>
