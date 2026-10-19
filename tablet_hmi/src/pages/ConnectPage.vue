<script setup lang="ts">
import { computed, ref } from "vue";
import {
  Activity, Cable, CirclePower, Cpu, Flame, Power, RefreshCw, ShieldAlert, Truck, Video,
} from "lucide-vue-next";
import ConfirmModal from "../components/ConfirmModal.vue";
import { useWorkbench } from "../stores/workbench";

const store = useWorkbench();
const busy = ref("");
const dialog = ref<{ title: string; message: string; action: () => Promise<void>; danger?: boolean } | null>(null);

interface ConnectionDefinition {
  key: string;
  title: string;
  subtitle: string;
  icon: any;
  component: string;
  camera: string;
  risky?: boolean;
}

const definitions: ConnectionDefinition[] = [
  { key: "can", title: "CAN 激活", subtitle: "SocketCAN can0 · 1 Mbps", icon: Flame, component: "can", camera: "d455", risky: true },
  { key: "activate", title: "机械臂激活", subtitle: "使能机械臂控制板", icon: ShieldAlert, component: "activate", camera: "d455", risky: true },
  { key: "arm", title: "T1 · 机械臂 / MoveIt", subtitle: "实际反馈与运动规划", icon: Cpu, component: "arm", camera: "d455" },
  { key: "d455", title: "D455 相机", subtitle: "固定视角 · USB", icon: Video, component: "camera", camera: "d455" },
  { key: "dabai", title: "Dabai 相机", subtitle: "腕载视角 · RGB-D", icon: Video, component: "camera", camera: "dabai" },
  { key: "yolo", title: "双路 YOLO", subtitle: "共享检测节点", icon: Activity, component: "detector", camera: "dual" },
  { key: "action", title: "T5 · 动作程序", subtitle: "抓取与放置任务", icon: CirclePower, component: "action", camera: "d455" },
  { key: "conveyor", title: "传送带", subtitle: "STM32 串口控制", icon: Truck, component: "conveyor", camera: "d455" },
];

const componentMap = computed(() => {
  const map: Record<string, any> = {};
  for (const row of store.components) map[`${row.component}:${row.camera_id}`] = row;
  return map;
});

const cards = computed(() => definitions.map((item) => {
  const state = componentMap.value[`${item.component}:${item.camera}`];
  const commandState = item.component === "can" || item.component === "activate";
  const running = commandState
    ? state?.state === "command_completed"
    : state?.state === "running" || (item.component === "conveyor" && store.workbench.conveyor?.connected);
  return {
    ...item,
    state,
    running,
    pid: state?.record?.pid,
    error: store.componentErrors[`${item.component}:${item.camera}`] || "",
  };
}));

async function execute(label: string, action: () => Promise<any>) {
  if (busy.value) return;
  busy.value = label;
  try {
    await action();
  } catch (err) {
    store.setError(err instanceof Error ? err.message : String(err));
  } finally {
    busy.value = "";
  }
}

function manage(card: ConnectionDefinition, start: boolean) {
  const label = `${card.key}:${start ? "start" : "stop"}`;
  const run = () => execute(label, () => store.action(start ? "component.start" : "component.stop", {
    component: card.component,
    camera_id: card.camera,
  }));
  if (start && card.risky) {
    dialog.value = {
      title: card.title,
      message: card.component === "can"
        ? "将激活 can0 并发送一次诊断帧。请确认机械臂控制箱已上电、CAN 线连接牢固。"
        : "将重新使能机械臂。请确认动作区域无人、无障碍物，机械臂处于安全姿态。",
      action: run,
      danger: card.component === "activate",
    };
    return;
  }
  run();
}

function confirmSystem(kind: "start" | "stop" | "recover") {
  const content = kind === "start"
    ? { title: "一键启动全部组件", message: "将按现有顺序启动 CAN、机械臂、双相机、双路 YOLO、T5 和传送带；已经运行的组件会复用。", action: () => store.action("system.start_missing"), danger: false }
    : kind === "stop"
      ? { title: "关闭所有连接", message: "将暂停任务、关闭动作节点与相机、断开传送带并关闭 CAN。工作台和三电平终端保留。", action: () => store.action("system.stop_all"), danger: true }
      : { title: "碰撞后恢复", message: "将取消旧目标、停带、重新激活 CAN 和机械臂并重启 T1。抓取会保持暂停，由你重新开始。", action: () => store.action("arm.recover"), danger: true };
  dialog.value = { ...content, action: async () => { await content.action(); } };
}

async function acceptDialog() {
  const current = dialog.value;
  dialog.value = null;
  if (current) await current.action();
}
</script>

<template>
  <div class="connect-page">
    <header class="page-heading">
      <div><h1>设备连接</h1><p>组件彼此独立，可按当前调试需要单独启停</p></div>
      <div class="connect-toolbar">
        <button class="button secondary" :disabled="!!busy || !store.isOperator" @click="confirmSystem('start')">
          <Power :size="17" />一键启动全部
        </button>
        <button class="button secondary danger-text" :disabled="!!busy || !store.isOperator" @click="confirmSystem('stop')">
          <Cable :size="17" />关闭所有连接
        </button>
        <button class="button secondary" :disabled="!!busy || !store.isOperator" @click="confirmSystem('recover')">
          <ShieldAlert :size="17" />碰撞后恢复
        </button>
      </div>
    </header>
    <div class="connection-grid">
      <article v-for="card in cards" :key="card.key" class="connection-card panel" :class="{ live: card.running }">
        <header>
          <span class="icon-tile"><component :is="card.icon" :size="20" /></span>
          <span class="state-lamp" :class="{ live: card.running }"></span>
        </header>
        <h2>{{ card.title }}</h2>
        <p>{{ card.subtitle }}</p>
        <div class="connection-state">
          <strong>{{ card.running ? (card.component === "conveyor" ? "已连接" : card.component === "can" || card.component === "activate" ? "本次已执行" : "运行中") : "未启动" }}</strong>
          <small v-if="card.pid">PID {{ card.pid }}</small>
          <small v-else>{{ card.state?.state || "等待操作" }}</small>
          <small v-if="card.error" class="connection-error">{{ card.error }}</small>
        </div>
        <div class="connection-actions">
          <button
            class="button primary"
            :disabled="!!busy || !store.isOperator || card.running"
            @click="manage(card, true)"
          >
            <RefreshCw v-if="busy.startsWith(card.key)" :size="15" class="spin" />
            {{ busy.startsWith(card.key) ? "执行中" : card.running ? (card.component === "conveyor" ? "已连接" : "已启动") : card.component === "conveyor" ? "连接" : "启动" }}
          </button>
          <button
            v-if="card.component !== 'can' && card.component !== 'activate'"
            class="button secondary"
            :disabled="!!busy || !store.isOperator || !card.running"
            @click="manage(card, false)"
          >{{ card.component === "conveyor" ? "断开" : "停止" }}</button>
        </div>
      </article>
    </div>
    <section class="connection-footnote panel">
      <ShieldAlert :size="18" />
      <div>
        <strong>开发模式，不设置联动门禁</strong>
        <p>每个相机、YOLO、T1、T5 和传送带都可独立操作。高风险按钮使用二次确认，其余操作直接执行并由后台真实状态维护按钮。</p>
      </div>
    </section>
    <ConfirmModal
      :open="!!dialog"
      :title="dialog?.title || ''"
      :message="dialog?.message || ''"
      :danger="dialog?.danger"
      @close="dialog = null"
      @confirm="acceptDialog"
    />
  </div>
</template>
