<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from "vue";
import { useWorkbench } from "./stores/workbench";
import TopBar from "./components/TopBar.vue";
import SideNav from "./components/SideNav.vue";
import GrabPage from "./pages/GrabPage.vue";
import ConnectPage from "./pages/ConnectPage.vue";
import PairPage from "./pages/PairPage.vue";
import { AlertTriangle, X } from "lucide-vue-next";

const store = useWorkbench();
const route = ref<"grab" | "connect">(location.hash.startsWith("#/connect") ? "connect" : "grab");
const pairing = ref(location.pathname === "/pair");
const loading = ref(true);

function syncRoute() {
  route.value = location.hash.startsWith("#/connect") ? "connect" : "grab";
  pairing.value = location.pathname === "/pair";
}

function setRoute(next: "grab" | "connect") {
  route.value = next;
  history.replaceState(null, "", `#/${next}`);
}

function visibilityChanged() {
  if (document.visibilityState === "hidden") store.releaseLease();
  else if (!pairing.value) store.refresh().catch(() => undefined);
}

function pageHidden() {
  store.releaseLease();
}

onMounted(async () => {
  window.addEventListener("hashchange", syncRoute);
  window.addEventListener("pagehide", pageHidden);
  document.addEventListener("visibilitychange", visibilityChanged);
  if (!pairing.value) {
    try {
      const okay = await store.refresh();
      if (!okay) pairing.value = true;
      else store.connect();
    } catch (err) {
      store.setError(String(err instanceof Error ? err.message : err));
      pairing.value = true;
    }
  }
  loading.value = false;
});

onBeforeUnmount(() => {
  window.removeEventListener("hashchange", syncRoute);
  window.removeEventListener("pagehide", pageHidden);
  document.removeEventListener("visibilitychange", visibilityChanged);
  store.disconnect();
});

watch(() => store.error, (value) => {
  if (value) window.setTimeout(() => store.clearError(), 6000);
});
</script>

<template>
  <div v-if="loading" class="splash">
    <div class="splash-mark"><span></span><span></span><span></span></div>
    <p>正在连接 Piper 工作台</p>
  </div>
  <PairPage v-else-if="pairing" @paired="pairing = false" />
  <div v-else class="app-shell">
    <TopBar @open-connect="setRoute('connect')" />
    <div class="app-body">
      <SideNav :active="route" @change="setRoute" />
      <main class="page-stage">
        <Transition name="page" mode="out-in">
          <GrabPage v-if="route === 'grab'" key="grab" />
          <ConnectPage v-else key="connect" />
        </Transition>
      </main>
    </div>
    <Transition name="toast">
      <button v-if="store.error" class="error-toast" @click="store.clearError()">
        <AlertTriangle :size="18" />
        <span>{{ store.error }}</span>
        <X :size="16" />
      </button>
    </Transition>
  </div>
</template>
