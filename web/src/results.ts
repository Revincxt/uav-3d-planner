import "./results.css";
import "./workspace.css";
import "./simulator-ui.css";
import { loadDataset } from "./data-loader";
import { analysesFromBundles, type Analysis } from "./results-data";
import { metricComparisons, renderMetricChart } from "./results-chart";
export { analysesFromBundles, observedMedian, predictiveObservation, summarizeObservedMissions, type MissionObservation } from "./results-data";
const element = (selector: string): HTMLElement => {
  const node = document.querySelector<HTMLElement>(selector);
  if (!node) throw new Error(`Missing required element: ${selector}`);
  return node;
};
async function fallback(): Promise<Analysis[]> {
  const [{ loadDemoBundle }, { loadDynamicBundle }, { loadPredictiveBundle }] = await Promise.all([import("./data"), import("./dynamic-data"), import("./predictive-data")]);
  return analysesFromBundles(...await Promise.all([loadDemoBundle(), loadDynamicBundle(), loadPredictiveBundle()]));
}
async function start(): Promise<void> {
  const analyses = await loadDataset("results", fallback);
  element("#comparison-charts").replaceChildren(...analyses.map(renderStudy));
  element("#results-note").textContent = analyses.some(analysis => analysis.planners.some(planner => planner.summary.completed < planner.summary.attempts))
    ? "Completed mission medians · Incomplete cohort" : "Completed mission medians";
  element("#load-state").hidden = true;
}
export function renderStudy(analysis: Analysis): HTMLElement {
  const section = document.createElement("section"); section.className = "study-comparison"; section.dataset.study = analysis.id;
  const heading = document.createElement("h2"); heading.className = "study-comparison-heading";
  heading.id = `results-${analysis.id}`; heading.textContent = analysis.label; section.setAttribute("aria-labelledby", heading.id);
  const charts = document.createElement("div"); charts.className = "comparison-charts";
  charts.append(...metricComparisons(analysis.id, analysis.planners).map(renderMetricChart));
  section.append(heading, charts); return section;
}
if (typeof document !== "undefined") start().catch((error: unknown) => {
  const state = element("#load-state"); state.hidden = false; state.classList.add("is-error"); state.textContent = error instanceof Error ? error.message : "The Manhattan mission records could not load."; console.error(error);
});
