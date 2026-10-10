import { defineConfig } from "vite";
import { runtimeDataPlugin } from "./build/runtime-data.mjs";
import { publicAssetsPlugin } from "./build/public-assets.mjs";
import { staticAnalysis, dynamicAnalysis, predictiveAnalysis } from "./src/results-data";
import { validateBundle } from "./src/static-validation";
import { validateDynamicBundle } from "./src/dynamic-validation";
import { validatePredictiveBundle } from "./src/predictive-validation";

export default defineConfig({
  base: "./",
  plugins: [publicAssetsPlugin(), runtimeDataPlugin((name: string, value: unknown) => {
    if (name === "demo-data") { const bundle = validateBundle(value); return { analysis: staticAnalysis(bundle), citySha256: bundle.scenarios[0]!.city!.sourceSha256 }; }
    if (name === "dynamic-data") { const bundle = validateDynamicBundle(value); return { analysis: dynamicAnalysis(bundle), citySha256: bundle.scenarios[0]!.city!.sourceSha256 }; }
    const bundle = validatePredictiveBundle(value); return { analysis: predictiveAnalysis(bundle), citySha256: bundle.scenarios[0]!.city!.sourceSha256 };
  })],
  build: {
    copyPublicDir: false,
    target: "es2022",
    sourcemap: true,
    rollupOptions: {
      output: {
        // Stable engine/map content need not be downloaded again after UI changes.
        manualChunks(id) {
          if (id.includes("/node_modules/") && id.includes("/three/")) return "three";
          if (id.endsWith("/src/map-background-data.ts")) return "map-context";
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
