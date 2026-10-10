import "./workspace.css";
import "./simulator-ui.css";
import "./results.css";
import { createBenchmarkPanel, populateBenchmark } from "./benchmark-panel";
import { loadDemoBundle } from "./data";
import { SceneViewer } from "./scene-viewer";
import { staticRoutes } from "./route-overview";
import { mountPageLifecycle } from "./page-lifecycle";
export { renderStudy } from "./benchmark-panel";
export { analysesFromBundles, observedMedian, predictiveObservation, summarizeObservedMissions, type MissionObservation } from "./results-data";

async function start(): Promise<void> {
  const host = document.querySelector<HTMLElement>("#benchmark-host")!;
  host.append(createBenchmarkPanel(() => { window.location.href = "./"; }));
  const summary = populateBenchmark(host.querySelector<HTMLElement>(".benchmark-panel")!).catch(console.error);
  let viewer: SceneViewer | undefined;
  try {
    const bundle = await loadDemoBundle(); viewer = new SceneViewer(document.querySelector<HTMLElement>("#benchmark-map")!);
    const planner = "lazy-theta-star" as const;
    viewer.setScenario(bundle.scenarios[0]!, new Set([planner]), "smoothed");
    viewer.setRoutes(staticRoutes(bundle.scenarios, planner, "smoothed"), bundle.scenarios[0]!.id);
  } catch (error) { console.warn("Benchmark background map unavailable", error); }
  mountPageLifecycle({ pause: () => {}, restore: () => viewer?.setTime(0), dispose: () => viewer?.dispose() });
  await summary;
}
if (typeof document !== "undefined") void start().catch(console.error);
