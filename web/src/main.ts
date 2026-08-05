import "./styles.css";

import { loadDemoBundle } from "./data";
import { SceneViewer } from "./scene-viewer";
import type { DemoBundle, DemoScenario, PlannerId } from "./schema";

const element = <T extends HTMLElement>(selector: string): T => {
  const value = document.querySelector<T>(selector);
  if (!value) throw new Error(`Missing required element: ${selector}`);
  return value;
};

const formatMetric = (value: number | null, digits = 1): string =>
  value === null ? "—" : value.toFixed(digits);

function renderTable(bundle: DemoBundle, scenario: DemoScenario): void {
  const body = element<HTMLTableSectionElement>("#results-body");
  body.replaceChildren();
  for (const result of scenario.results) {
    const row = document.createElement("tr");
    const planner = bundle.planners.find((item) => item.id === result.plannerId);
    const values = [
      planner?.label ?? result.plannerId,
      result.status,
      formatMetric(result.metrics.planningTimeMs),
      formatMetric(result.metrics.rawLengthM),
      formatMetric(result.metrics.smoothedLengthM),
      formatMetric(result.metrics.minClearanceM, 2),
      `${result.searchEffort.value.toLocaleString()} ${result.searchEffort.kind}`,
      result.plannerSeed === null ? "deterministic" : String(result.plannerSeed),
    ];
    values.forEach((value, index) => {
      const cell = document.createElement(index === 0 ? "th" : "td");
      if (index === 0) cell.setAttribute("scope", "row");
      cell.textContent = value;
      row.append(cell);
    });
    body.append(row);
  }
}

function renderLegend(bundle: DemoBundle): void {
  const legend = element("#legend");
  legend.replaceChildren();
  bundle.planners.forEach((planner) => {
    const item = document.createElement("span");
    item.className = `legend-item planner-${planner.id}`;
    const swatch = document.createElement("i");
    swatch.setAttribute("aria-hidden", "true");
    item.append(swatch, document.createTextNode(planner.label));
    legend.append(item);
  });
  const labels = [
    ["building", "Building"],
    ["no-fly", "No-fly zone"],
    ["endpoint", "Start / goal"],
  ];
  labels.forEach(([kind, label]) => {
    const item = document.createElement("span");
    item.className = `legend-item ${kind}`;
    const swatch = document.createElement("i");
    swatch.setAttribute("aria-hidden", "true");
    item.append(swatch, document.createTextNode(label ?? ""));
    legend.append(item);
  });
}

async function start(): Promise<void> {
  const bundle = await loadDemoBundle();
  const select = element<HTMLSelectElement>("#scene-select");
  const viewerElement = element("#scene-viewer");
  const visible = new Set<PlannerId>(bundle.planners.map((planner) => planner.id));
  let pathMode: "raw" | "smoothed" = "smoothed";
  let viewer: SceneViewer;
  try {
    viewer = new SceneViewer(viewerElement);
  } catch (error) {
    viewerElement.classList.add("viewer-error");
    viewerElement.textContent = "WebGL is unavailable. The recorded result table remains accessible.";
    console.error(error);
    return;
  }

  bundle.scenarios.forEach((scenario) => {
    const option = document.createElement("option");
    option.value = scenario.id;
    option.textContent = scenario.label;
    option.selected = scenario.id === bundle.defaultScenarioId;
    select.append(option);
  });

  const plannerControls = element<HTMLFieldSetElement>("#planner-controls");
  bundle.planners.forEach((planner) => {
    const label = document.createElement("label");
    const input = document.createElement("input");
    input.type = "checkbox";
    input.value = planner.id;
    input.checked = true;
    input.addEventListener("change", () => {
      if (input.checked) visible.add(planner.id);
      else visible.delete(planner.id);
      viewer.setPlannerVisibility(visible);
    });
    label.append(input, document.createTextNode(` ${planner.label}`));
    plannerControls.append(label);
  });

  const selectedScenario = (): DemoScenario => {
    const scenario = bundle.scenarios.find((item) => item.id === select.value);
    if (!scenario) throw new Error(`Unknown scenario: ${select.value}`);
    return scenario;
  };

  const update = (): void => {
    const scenario = selectedScenario();
    viewer.setScenario(scenario, visible, pathMode);
    renderTable(bundle, scenario);
    element("#scene-caption").textContent =
      `${scenario.description} ENU frame; distances in metres. ` +
      `Vehicle radius ${scenario.constraints.vehicleRadiusM} m plus ` +
      `${scenario.constraints.safetyMarginM} m safety margin.`;
    element("#provenance").textContent =
      `Dataset ${bundle.generatedAt}; ${bundle.verificationStatus}; ${scenario.fingerprint}. ` +
      "Paths were planned and collision-checked in Python, then recorded as polylines. " +
      "Single runs illustrate behavior and are not statistical evidence.";
    viewerElement.setAttribute(
      "aria-label",
      `${scenario.label}: ${scenario.buildings.length} buildings, ` +
        `${scenario.noFlyZones.length} no-fly zones, and ${scenario.results.length} planner paths.`,
    );
    element("#live-region").textContent = `${scenario.label} loaded`;
  };

  select.addEventListener("change", update);
  document.querySelectorAll<HTMLInputElement>('input[name="path-mode"]').forEach((input) => {
    input.addEventListener("change", () => {
      if (!input.checked) return;
      pathMode = input.value === "raw" ? "raw" : "smoothed";
      viewer.setPathMode(pathMode);
    });
  });
  document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((button) => {
    button.addEventListener("click", () => {
      const view = button.dataset.view;
      if (view === "isometric" || view === "top" || view === "reset") viewer.setView(view);
    });
  });
  window.addEventListener("pagehide", () => viewer.dispose(), { once: true });
  renderLegend(bundle);
  update();
}

start().catch((error: unknown) => {
  const viewer = element("#scene-viewer");
  viewer.classList.add("viewer-error");
  viewer.textContent = error instanceof Error ? error.message : "The recorded dataset could not load.";
  console.error(error);
});

