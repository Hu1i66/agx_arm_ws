<script setup lang="ts">
import { computed, ref, watch } from "vue";
import {
  BoxSelect, CheckCircle2, MapPin, Pause, Play, RotateCw, Sparkles, Target,
} from "lucide-vue-next";
import CameraCard from "../components/CameraCard.vue";
import GraspFlow from "../components/GraspFlow.vue";
import RobotView from "../components/RobotView.vue";
import { useWorkbench } from "../stores/workbench";

const store = useWorkbench();
const mode = computed({
  get: () => store.selectedMode,
  set: (value: "d455-dynamic" | "d455-static" | "dabai-static") => store.setMode(value),
});
const placementId = ref("");
const busy = ref("");
const modes = [
  { id: "d455-dynamic", label: "D455 动态" },
  { id: "d455-static", label: "D455 静态" },
  { id: "dabai-static", label: "Dabai 静态" },
] as const;
const chineseClasses: Record<string, string> = {
  apple: "苹果", green_apple: "青苹果", orange: "橙子", banana: "香蕉",
  yellow_pear: "黄梨", green_pear: "青梨", strawberry: "草莓", peach: "桃子",
  lemon: "柠檬", mango: "芒果", star_fruit: "杨桃", pomegranate: "石榴",
};
const plans = computed(() => store.workbench.placements?.plans || []);
const targetData = computed(() => store.detections[store.selectedCamera] || {});
const targetRows = computed(() => [...(targetData.value.objects || [])].sort((a, b) => Number(a.track_id) - Number(b.track_id)));
const taskActive = computed(() => {
  const task = store.task || {};
  return !!task.inflight || !!task.continuous || !!task.queue_running;
});
const modeIndex = computed(() => modes.findIndex((item) => item.id === mode.value));

watch(plans, (value) => {
  if (!placementId.value && value.length) {
    placementId.value = store.workbench.placements?.selection?.default_plan_id || value[0].id;
  }
}, { immediate: true });

function classLabel(name?: string) {
  return chineseClasses[name || ""] || name || "目标";
}

async function start(continuous: boolean) {
  if (busy.value) return;
  if (!continuous && store.selectedTrack === null) {
    store.setError("请先在相机画面或目标列表中选择目标");
    return;
  }
  if (continuous && mode.value === "dabai-static") {
    store.setError("Dabai 静态模式使用所选目标单次抓取");
    return;
  }
  busy.value = continuous ? "start" : "single";
  try {
    await store.action("motion.start", {
      mode: mode.value,
      continuous,
      track_id: store.selectedTrack,
      session: store.selectedSession,
      target_snapshot: store.currentTarget || {},
      placement_id: placementId.value || undefined,
    });
  } catch (err) {
    store.setError(err instanceof Error ? err.message : String(err));
  } finally {
    busy.value = "";
  }
}

async function pauseAfterCycle() {
  if (busy.value) return;
  busy.value = "pause";
  try {
    await store.action("motion.pause_after_cycle");
  } catch (err) {
    store.setError(err instanceof Error ? err.message : String(err));
  } finally {
    busy.value = "";
  }
}
</script>

<template>
  <div class="grab-page">
    <header class="page-heading">
      <div>
        <h1>抓取工作台</h1>
        <p>双相机感知、实际机械臂反馈与任务操作保持在同一控制平面</p>
      </div>
      <div class="heading-status">
        <Sparkles :size="16" />
        {{ store.task.phase === "paused" ? "任务已暂停" : store.task.phase === "grasping" ? "机械臂抓放中" : "等待目标" }}
      </div>
    </header>
    <GraspFlow />
    <div class="grab-layout">
      <section class="camera-stack">
        <CameraCard camera="d455" />
        <CameraCard camera="dabai" />
      </section>
      <RobotView />
      <aside class="operation-stack">
        <section class="operation-panel panel">
          <header class="section-title"><BoxSelect :size="19" /><div><h2>抓取操作</h2><span>模式与放置策略</span></div></header>
          <div class="segmented mode-switch">
            <span class="segment-indicator" :style="{ transform: `translateX(${modeIndex * 100}%)` }"></span>
            <button
              v-for="item in modes"
              :key="item.id"
              :class="{ active: mode === item.id }"
              :disabled="taskActive"
              @click="mode = item.id"
            >{{ item.label }}</button>
          </div>
          <label class="field-label">
            <span><MapPin :size="15" />放置方案</span>
            <select v-model="placementId" :disabled="taskActive">
              <option v-for="plan in plans" :key="plan.id" :value="plan.id">{{ plan.name }}</option>
            </select>
          </label>
          <div class="task-buttons">
            <button class="button primary action-wide" :disabled="!!busy || taskActive || !store.isOperator" @click="start(true)">
              <RotateCw v-if="busy === 'start'" :size="18" class="spin" />
              <CheckCircle2 v-else-if="taskActive" :size="18" />
              <Play v-else :size="18" />
              {{ taskActive && store.task.continuous ? "抓取任务运行中" : "开始连续抓取" }}
            </button>
            <button class="button secondary action-wide" :disabled="!!busy || taskActive || !store.isOperator" @click="start(false)">
              <Target :size="18" />抓取所选目标
            </button>
            <button class="button secondary action-wide" :disabled="!!busy || !taskActive || !store.isOperator" @click="pauseAfterCycle">
              <Pause :size="17" />本轮后停止
            </button>
          </div>
        </section>

        <section class="target-panel panel">
          <header class="section-title">
            <Target :size="19" />
            <div><h2>检测目标</h2><span>{{ store.selectedCamera === "d455" ? "D455 · 固定相机" : "Dabai · 腕载相机" }}</span></div>
            <em>{{ targetRows.length }}</em>
          </header>
          <div class="target-tabs">
            <button :class="{ active: store.selectedCamera === 'd455' }" @click="store.selectedCamera = 'd455'">D455</button>
            <button :class="{ active: store.selectedCamera === 'dabai' }" @click="store.selectedCamera = 'dabai'">Dabai</button>
          </div>
          <div v-if="targetRows.length" class="target-list">
            <button
              v-for="object in targetRows"
              :key="String(object.track_id)"
              class="target-row"
              :class="{ selected: Number(object.track_id) === Number(store.selectedTrack) }"
              @click="store.selectTarget(store.selectedCamera, Number(object.track_id))"
            >
              <span class="target-id">#{{ object.track_id }}</span>
              <strong>{{ classLabel(String(object.object_name || object.raw_object_name || "")) }}</strong>
              <span>{{ Math.round(Number(object.confidence || 0) * 100) }}%</span>
              <small v-if="object.base_position_m">
                {{ Number(object.base_position_m.x).toFixed(2) }}, {{ Number(object.base_position_m.y).toFixed(2) }}, {{ Number(object.base_position_m.z).toFixed(2) }} m
              </small>
            </button>
          </div>
          <div v-else class="empty-target">
            <Target :size="24" />
            <p>当前相机还没有可用目标</p>
          </div>
          <div class="selected-target" :class="{ active: store.selectedTrack !== null }">
            {{ store.selectedTrack !== null ? `已选择 #${store.selectedTrack} · ${classLabel(String(store.currentTarget?.object_name || ""))}` : "点击画面检测框或列表选择目标" }}
          </div>
        </section>

        <section class="belt-panel panel">
          <div>
            <span class="mini-label">传送带</span>
            <strong>{{ store.workbench.conveyor?.motor_running ? "运行中" : "已停止" }}</strong>
            <small>{{ Number(store.workbench.conveyor?.speed || 0).toFixed(3) }} m/s · {{ store.workbench.conveyor?.connected ? "通信正常" : "未连接" }}</small>
          </div>
          <div class="belt-actions">
            <button class="button secondary" :disabled="!store.isOperator" @click="store.action('component.start', { component: 'conveyor' })">连接</button>
            <button class="button secondary" :disabled="!store.isOperator" @click="store.action('component.stop', { component: 'conveyor' })">断开</button>
          </div>
        </section>
      </aside>
    </div>
  </div>
</template>
