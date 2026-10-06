import { defineConfig } from "vite";
import { runtimeDataPlugin } from "./build/runtime-data.mjs";

export default defineConfig({
  base: "./",
  plugins: [runtimeDataPlugin()],
  build: {
    target: "es2022",
    sourcemap: true,
    rollupOptions: {
      output: {
        // Separate the shared rendering engine from mission/UI code for browser caching.
        manualChunks(id) {
          if (id.includes("/node_modules/") && id.includes("/three/")) return "three";
        },
      },
      input: {
        paths: "index.html",
        results: "results.html",
        dynamic: "dynamic.html",
        predictive: "predictive.html",
      },
    },
  },
});
