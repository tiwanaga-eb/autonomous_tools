import { defineConfig } from "vitest/config";
import react from "@vitejs/plugin-react";
import { fileURLToPath, URL } from "node:url";

// FE は API を /api /health 経由で呼ぶ。dev では backend(8077) にプロキシ。
export default defineConfig({
  plugins: [react()],
  resolve: {
    alias: { "@": fileURLToPath(new URL("./src", import.meta.url)) },
  },
  server: {
    port: 5173,
    proxy: {
      "/api": "http://127.0.0.1:8077",
      "/health": "http://127.0.0.1:8077",
    },
  },
  test: {
    environment: "node",
    globals: true,
  },
});
