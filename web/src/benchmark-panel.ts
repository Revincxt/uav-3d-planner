import "./results.css";
import { loadDataset } from "./data-loader";
import { analysesFromBundles, type Analysis } from "./results-data";
import { fetchBenchmarkSummary } from "./benchmark-summary";
import { metricComparisons, renderMetricChart } from "./results-chart";
import { uiSymbol } from "./ui-symbol";

async function fallback(): Promise<Analysis[]> {
  const summary = await fetchBenchmarkSummary(new URL(import.meta.env.BASE_URL, document.baseURI).href);
  if (summary) return summary;
  const [{ loadDemoBundle }, { loadDynamicBundle }, { loadPredictiveBundle }] = await Promise.all([import("./data"), import("./dynamic-data"), import("./predictive-data")]);
  return analysesFromBundles(...await Promise.all([loadDemoBundle(), loadDynamicBundle(), loadPredictiveBundle()]));
}
export const loadBenchmark = (): Promise<Analysis[]> => loadDataset("results", fallback);
export function renderStudy(analysis: Analysis): HTMLElement {
  const section = document.createElement("section"); section.className = "study-comparison"; section.dataset.study = analysis.id;
  const heading = document.createElement("h2"); heading.className = "study-comparison-heading";
  heading.id = `results-${analysis.id}`; heading.append(uiSymbol(analysis.id), document.createTextNode(analysis.label)); section.setAttribute("aria-labelledby", heading.id);
  const charts = document.createElement("div"); charts.className = "comparison-charts";
  charts.append(...metricComparisons(analysis.id, analysis.planners).map(renderMetricChart));
  section.append(heading, charts); return section;
}
export async function populateBenchmark(panel: HTMLElement): Promise<void> {
  const state = panel.querySelector<HTMLElement>(".benchmark-state")!; panel.setAttribute("aria-busy", "true");
  try {
    const analyses = await loadBenchmark();
    panel.querySelector(".results-overview")!.replaceChildren(...analyses.map(renderStudy));
    panel.querySelector(".results-note")!.textContent = analyses.some(a => a.planners.some(p => p.summary.completed < p.summary.attempts))
      ? "Completed mission medians · Incomplete cohort" : "Completed mission medians";
    state.hidden = true;
  } catch (error) { state.textContent = error instanceof Error ? error.message : "Benchmark could not load."; state.classList.add("is-error"); throw error; }
  finally { panel.setAttribute("aria-busy", "false"); }
}
export function createBenchmarkPanel(close: () => void): HTMLElement {
  const panel = document.createElement("section"); panel.className = "benchmark-panel"; panel.setAttribute("aria-label", "Overall algorithm comparisons");
  const header = document.createElement("header"); header.className = "benchmark-heading";
  const title = document.createElement("h1"); title.id = "benchmark-title"; title.append(uiSymbol("benchmark"), document.createTextNode("Benchmark"));
  const button = document.createElement("button"); button.type = "button"; button.className = "benchmark-close";
  button.setAttribute("aria-label", "Close benchmark"); button.title = "Close · Esc";
  button.append(uiSymbol("close")); button.addEventListener("click", close); header.append(title, button);
  const content = document.createElement("div"); content.className = "results-panel";
  const charts = document.createElement("div"); charts.id = "comparison-charts"; charts.className = "results-overview";
  const note = document.createElement("p"); note.id = "results-note"; note.className = "results-note";
  const state = document.createElement("p"); state.className = "benchmark-state"; state.setAttribute("role", "status"); state.setAttribute("aria-live", "polite"); state.textContent = "Loading benchmark…";
  content.append(charts, note, state); panel.append(header, content); return panel;
}
interface PlaybackAccess { isPlaying(): boolean; pause(): void; resume(): void }
/** An in-place visit preserves the existing renderer, camera and flight clock. */
export function mountBenchmark(playback: PlaybackAccess): void {
  const link = document.querySelector<HTMLAnchorElement>('.study-nav a[href="./results.html"]'); if (!link) return;
  const dialog = document.createElement("dialog"); dialog.className = "benchmark-dialog"; dialog.setAttribute("aria-labelledby", "benchmark-title");
  const panel = createBenchmarkPanel(() => dialog.close()); dialog.append(panel); document.body.append(dialog);
  let resume = false, loaded = false, loading = false;
  link.setAttribute("aria-haspopup", "dialog"); link.setAttribute("aria-expanded", "false");
  link.addEventListener("click", event => {
    if (event.ctrlKey || event.metaKey || event.shiftKey || event.altKey || event.button !== 0) return;
    event.preventDefault(); if (dialog.open) return;
    resume = playback.isPlaying(); playback.pause(); dialog.showModal(); link.setAttribute("aria-expanded", "true");
    if (!loaded && !loading) { loading = true; void populateBenchmark(panel).then(() => { loaded = true; }).catch(() => {}).finally(() => { loading = false; }); }
  });
  dialog.addEventListener("click", event => { if (event.target === dialog) dialog.close(); });
  dialog.addEventListener("close", () => { link.setAttribute("aria-expanded", "false"); link.focus(); if (resume && !document.hidden) playback.resume(); resume = false; });
  dialog.addEventListener("keydown", event => { event.stopPropagation(); });
}
