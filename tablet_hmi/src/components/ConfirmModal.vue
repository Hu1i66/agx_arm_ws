<script setup lang="ts">
import { AlertTriangle } from "lucide-vue-next";
defineProps<{
  open: boolean;
  title: string;
  message: string;
  confirmText?: string;
  danger?: boolean;
}>();
const emit = defineEmits<{ close: []; confirm: [] }>();
</script>

<template>
  <Teleport to="body">
    <Transition name="dialog">
      <div v-if="open" class="dialog-backdrop" @click.self="emit('close')">
        <section class="confirm-dialog" role="dialog" aria-modal="true">
          <div class="dialog-icon" :class="{ danger }"><AlertTriangle :size="22" /></div>
          <h3>{{ title }}</h3>
          <p>{{ message }}</p>
          <div class="dialog-actions">
            <button class="button secondary" @click="emit('close')">取消</button>
            <button class="button" :class="danger ? 'danger' : 'primary'" @click="emit('confirm')">
              {{ confirmText || "确认执行" }}
            </button>
          </div>
        </section>
      </div>
    </Transition>
  </Teleport>
</template>
