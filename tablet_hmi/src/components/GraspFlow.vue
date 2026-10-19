<script setup lang="ts">
import { computed } from "vue";
import { Check, Circle, LoaderCircle } from "lucide-vue-next";
import { useWorkbench } from "../stores/workbench";

const store = useWorkbench();
const stages = [
  { id: "target", label: "识别" },
  { id: "approach", label: "接近" },
  { id: "descend", label: "对齐" },
  { id: "grasp", label: "夹取" },
  { id: "lift", label: "搬运" },
  { id: "release", label: "释放" },
  { id: "return", label: "返回" },
];
const activeIndex = computed(() => stages.findIndex((stage) => stage.id === store.graspStage));
const progress = computed(() => {
  if (store.graspStage === "complete") return 100;
  if (store.graspStage === "error" || store.graspStage === "waiting") return 0;
  return Math.max(7, ((activeIndex.value + 1) / stages.length) * 100);
});
</script>

<template>
  <section class="grasp-flow panel">
    <header>
      <div><strong>抓取流程</strong><span>真实 T5 阶段事件</span></div>
      <em :class="store.graspStage">{{ store.graspLabel || "等待开始" }}</em>
    </header>
    <div class="flow-track">
      <div class="flow-progress" :style="{ width: `${progress}%` }"></div>
      <div
        v-for="(stage, index) in stages"
        :key="stage.id"
        class="flow-stage"
        :class="{
          done: activeIndex > index || store.graspStage === 'complete',
          active: activeIndex === index && !['complete', 'error', 'waiting'].includes(store.graspStage),
          error: store.graspStage === 'error' && activeIndex === index,
        }"
      >
        <span class="flow-node">
          <Check v-if="activeIndex > index || store.graspStage === 'complete'" :size="13" />
          <LoaderCircle v-else-if="activeIndex === index" :size="13" class="spin" />
          <Circle v-else :size="10" />
        </span>
        <small>{{ stage.label }}</small>
      </div>
    </div>
  </section>
</template>
