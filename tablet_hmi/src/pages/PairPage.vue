<script setup lang="ts">
import { computed, ref } from "vue";
import { KeyRound, ShieldCheck, Wifi } from "lucide-vue-next";
import { useWorkbench } from "../stores/workbench";

const emit = defineEmits<{ paired: [] }>();
const store = useWorkbench();
const initial = new URLSearchParams(location.search).get("code") || "";
const code = ref(initial);
const name = ref(/iPad/i.test(navigator.userAgent) ? "iPad 工作终端" : /Android/i.test(navigator.userAgent) ? "Android 工作终端" : "Piper 平板");
const busy = ref(false);
const error = ref("");
const hasCode = computed(() => location.search.includes("code="));

async function submit() {
  if (busy.value) return;
  busy.value = true;
  error.value = "";
  try {
    await store.pair(code.value.trim(), name.value.trim());
    emit("paired");
  } catch (err) {
    error.value = err instanceof Error ? err.message : String(err);
  } finally {
    busy.value = false;
  }
}
</script>

<template>
  <main class="pair-screen">
    <section class="pair-card">
      <div class="pair-brand">
        <div class="brand-mark large"><i></i><i></i><i></i><i></i></div>
        <div><strong>Piper 工作台</strong><small>便携操作终端配对</small></div>
      </div>
      <div class="pair-illustration">
        <span><Wifi :size="26" /></span>
        <i></i>
        <span><ShieldCheck :size="26" /></span>
      </div>
      <h1>连接实验室机械臂</h1>
      <p>请在电脑工作台的“调试 → 平板上位机”中生成一次性配对码。</p>
      <form @submit.prevent="submit">
        <label>
          <span><KeyRound :size="16" />一次性配对码</span>
          <input v-model="code" inputmode="numeric" autocomplete="one-time-code" maxlength="6" placeholder="000000" />
        </label>
        <label>
          <span>设备名称</span>
          <input v-model="name" autocomplete="off" maxlength="64" />
        </label>
        <p v-if="error" class="pair-error">{{ error }}</p>
        <button class="button primary pair-submit" :disabled="busy || code.length < 6">
          {{ busy ? "正在建立安全连接" : "配对并进入工作台" }}
        </button>
      </form>
      <small v-if="!hasCode">也可以直接使用电脑管理页二维码中的完整地址打开，配对码会自动填入。</small>
      <div class="pair-security">设备令牌保存在 HttpOnly Cookie；控制权仍需单独获取。</div>
    </section>
  </main>
</template>
