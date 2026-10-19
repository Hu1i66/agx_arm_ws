<script setup lang="ts">
import { Cable, Hand, ShieldCheck, ShieldAlert } from "lucide-vue-next";
import { useWorkbench } from "../stores/workbench";

defineProps<{ active: "grab" | "connect" }>();
const emit = defineEmits<{ change: [page: "grab" | "connect"] }>();
const store = useWorkbench();

async function leaseControl() {
  if (!store.isOperator || store.hasLease) return;
  try {
    await store.acquireLease();
  } catch (err) {
    store.setError(err instanceof Error ? err.message : String(err));
  }
}
</script>

<template>
  <aside class="side-nav">
    <nav>
      <button :class="{ active: active === 'grab' }" @click="emit('change', 'grab')">
        <Hand :size="22" /><span>抓取</span>
      </button>
      <button :class="{ active: active === 'connect' }" @click="emit('change', 'connect')">
        <Cable :size="22" /><span>连接</span>
      </button>
    </nav>
    <div class="nav-spacer"></div>
    <button v-if="store.isOperator" class="lease-chip" :class="{ held: store.hasLease }" @click="leaseControl">
      <ShieldCheck v-if="store.hasLease" :size="18" />
      <ShieldAlert v-else :size="18" />
      <span>{{ store.hasLease ? `控制 ${Math.ceil(store.lease?.remaining_s || 0)}s` : "取得控制权" }}</span>
    </button>
    <div class="device-signature">
      <strong>{{ store.device?.name || "未配对设备" }}</strong>
      <small>{{ store.device?.role === "observer" ? "只读" : "操作者" }}</small>
    </div>
  </aside>
</template>
