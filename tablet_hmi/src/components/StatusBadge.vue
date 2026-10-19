<script setup lang="ts">
defineProps<{
  label: string;
  value: string;
  tone?: "ok" | "warn" | "error" | "idle";
}>();
</script>

<template>
  <div class="status-badge" :class="`is-${tone || 'idle'}`">
    <span class="status-dot"></span>
    <span class="status-copy">
      <small>{{ label }}</small>
      <strong>{{ value }}</strong>
    </span>
  </div>
</template>
