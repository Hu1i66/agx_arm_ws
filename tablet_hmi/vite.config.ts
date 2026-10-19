import { defineConfig } from "vite";
import vue from "@vitejs/plugin-vue";

export default defineConfig({
  plugins: [vue()],
  server: {
    port: 5173,
    proxy: {
      "/api": { target: "https://127.0.0.1:8443", secure: false },
      "/media": { target: "https://127.0.0.1:8443", secure: false },
      "/ws": { target: "wss://127.0.0.1:8443", ws: true, secure: false },
    },
  },
  build: {
    target: "es2022",
    sourcemap: false,
    chunkSizeWarningLimit: 1000,
  },
});
