import "./dynamic.css";

import { buildDynamicComparisonRows, loadDynamicBundle } from "./dynamic-data";
import type {
  DynamicBundleV1,
  DynamicFrame,
  DynamicRun,
  DynamicScenario,
} from "./dynamic-schema";
import { DynamicViewer } from "./dynamic-viewer";

const element = <T extends HTMLElement>(selector: string): T => {
  const value = document.querySelector<T>(selector);
  if (!value) throw new Error(`Missing required element: ${selector}`);
  return value;
};

const METRIC_LABELS: Record<string, string> = {
  completionTimeS: "Completion time",
  executedPathLengthM: "Executed path length",
  directDistanceM: "Direct start–goal distance",
  pathExcessPct: "Path excess",
  replans: "Replanning episodes",
  failedReplans: "Failed replans",
  holds: "Hold steps",
  safetyGateActivations: "Safety-gate activations",
  collisionCount: "Audited collisions",
  totalPlanningWork: "Total planning work",
  workUnit: "Planning-work unit",
  totalChangedEdges: "Changed edges",
  deadlineMisses: "Deadline misses",
  minimumClearanceM: "Minimum clearance",
};

function readableMetric(key: string): string {
  return (
    METRIC_LABELS[key] ??
    key
      .replace(/([a-z0-9])([A-Z])/g, "$1 $2")
      .replace(/^./, (letter) => letter.toUpperCase())
  );
}

function formatScalar(key: string, value: string | number | boolean | null): string {
  if (value === null) return "—";
  if (typeof value === "boolean") return value ? "yes" : "no";
  if (typeof value === "string") return value;
  const suffix = key.endsWith("Pct")
    ? "%"
    : key.endsWith("Ms")
      ? " ms"
      : key.endsWith("M")
        ? " m"
        : key.endsWith("S")
          ? " s"
          : "";
  const digits = Number.isInteger(value) ? 0 : 2;
  return `${value.toLocaleString(undefined, { maximumFractionDigits: digits })}${suffix}`;
}

function appendCells(row: HTMLTableRowElement, values: string[]): void {
  for (const value of values) {
    const cell = document.createElement("td");
    cell.textContent = value;
    row.append(cell);
  }
}

function renderOutcome(run: DynamicRun): void {
  const body = element<HTMLTableSectionElement>("#outcome-metrics-body");
  body.replaceChildren();
  for (const [key, value] of Object.entries(run.metrics)) {
    const row = document.createElement("tr");
    const label = document.createElement("th");
    label.scope = "row";
    label.textContent = readableMetric(key);
    const metric = document.createElement("td");
    metric.textContent = formatScalar(key, value);
    row.append(label, metric);
    body.append(row);
  }
  element("#run-status").textContent =
    run.status === "success"
      ? "The recorded vehicle reached the declared goal."
      : `${run.status}: ${run.failureReason ?? "no failure reason recorded"}`;
}

function renderComparison(bundle: DynamicBundleV1, scenario: DynamicScenario): void {
  const body = element<HTMLTableSectionElement>("#comparison-body");
  body.replaceChildren();
  for (const result of buildDynamicComparisonRows(bundle, scenario)) {
    const row = document.createElement("tr");
    const planner = document.createElement("th");
    planner.scope = "row";
    planner.textContent = result.plannerLabel;
    row.append(planner);
    appendCells(row, [
      result.status,
      result.completionTimeS === null ? "—" : result.completionTimeS.toFixed(1),
      result.executedPathLengthM.toFixed(2),
      result.replans.toLocaleString(),
      result.holds.toLocaleString(),
      result.safetyGateActivations.toLocaleString(),
      `${result.totalPlanningWork.toLocaleString()} ${result.workUnit}`,
    ]);
    body.append(row);
  }
}

function renderFrameMetrics(frame: DynamicFrame, index: number, total: number): void {
  const body = element<HTMLTableSectionElement>("#frame-metrics-body");
  body.replaceChildren();
  const row = document.createElement("tr");
  appendCells(row, [
    frame.timeS.toFixed(1),
    frame.event?.label ?? "No discrete event",
    frame.replanned
      ? `${frame.replanReason ?? "replan"} · ${frame.plannerSuccess ? "path" : "no path"}`
      : "No",
    frame.planningTimeMs === null ? "Not timed" : `${frame.planningTimeMs.toFixed(2)} ms`,
    frame.workUsed.toLocaleString(),
    frame.changedEdges.toLocaleString(),
    frame.activeTemporaryZoneIds.length.toLocaleString(),
    frame.movingSpheres.length.toLocaleString(),
  ]);
  body.append(row);
  element("#frame-summary").textContent =
    `Frame ${index + 1} of ${total}; vehicle ENU ` +
    `[${frame.vehicle.map((coordinate) => coordinate.toFixed(1)).join(", ")}] m.`;
}

async function start(): Promise<void> {
  const bundle: DynamicBundleV1 = await loadDynamicBundle();
  const scenarioSelect = element<HTMLSelectElement>("#scenario-select");
  const plannerSelect = element<HTMLSelectElement>("#planner-select");
  const timeline = element<HTMLInputElement>("#timeline");
  const timelineValue = element<HTMLOutputElement>("#timeline-value");
  const playPause = element<HTMLButtonElement>("#play-pause");
  const previous = element<HTMLButtonElement>("#previous-frame");
  const next = element<HTMLButtonElement>("#next-frame");
  const viewerHost = element("#dynamic-viewer");
  let viewer: DynamicViewer | null = null;
  try {
    viewer = new DynamicViewer(viewerHost);
  } catch (error) {
    viewerHost.classList.add("viewer-error");
    viewerHost.textContent = "WebGL is unavailable. The recorded state and outcome tables remain accessible.";
    console.error(error);
  }

  let currentScenario: DynamicScenario;
  let currentRun: DynamicRun;
  let frameIndex = 0;
  let animationFrame: number | null = null;
  let playbackWallStart = 0;
  let playbackTraceStart = 0;

  for (const scenario of bundle.scenarios) {
    const option = document.createElement("option");
    option.value = scenario.id;
    option.textContent = scenario.label;
    scenarioSelect.append(option);
  }

  const announce = (message: string): void => {
    element("#live-region").textContent = message;
  };

  const renderFrame = (index: number, shouldAnnounce = false): void => {
    frameIndex = Math.max(0, Math.min(index, currentRun.frames.length - 1));
    const frame = currentRun.frames[frameIndex]!;
    timeline.value = String(frameIndex);
    timelineValue.value = `${frame.timeS.toFixed(1)} s`;
    previous.disabled = frameIndex === 0;
    next.disabled = frameIndex === currentRun.frames.length - 1;
    viewer?.setFrame(frame);
    renderFrameMetrics(frame, frameIndex, currentRun.frames.length);
    if (shouldAnnounce) announce(`Showing ${frame.timeS.toFixed(1)} seconds`);
  };

  const setPlaying = (playing: boolean): void => {
    if (!playing && animationFrame !== null) {
      cancelAnimationFrame(animationFrame);
      animationFrame = null;
    }
    playPause.textContent = playing ? "Pause" : "Play";
    playPause.setAttribute("aria-pressed", String(playing));
  };

  const playbackTick = (now: number): void => {
    const targetTraceTime = playbackTraceStart + (now - playbackWallStart) / 1000;
    let targetIndex = frameIndex;
    while (
      targetIndex + 1 < currentRun.frames.length &&
      currentRun.frames[targetIndex + 1]!.timeS <= targetTraceTime
    ) {
      targetIndex += 1;
    }
    if (targetIndex !== frameIndex) renderFrame(targetIndex);
    if (frameIndex >= currentRun.frames.length - 1) {
      setPlaying(false);
      announce("Playback complete");
      return;
    }
    animationFrame = requestAnimationFrame(playbackTick);
  };

  const beginPlayback = (): void => {
    if (frameIndex === currentRun.frames.length - 1) renderFrame(0);
    playbackWallStart = performance.now();
    playbackTraceStart = currentRun.frames[frameIndex]!.timeS;
    setPlaying(true);
    animationFrame = requestAnimationFrame(playbackTick);
  };

  const updateRun = (plannerId: string, shouldAnnounce = true): void => {
    setPlaying(false);
    const run = currentScenario.runs.find((candidate) => candidate.plannerId === plannerId);
    if (!run) throw new Error(`Missing ${plannerId} run in ${currentScenario.id}`);
    currentRun = run;
    timeline.max = String(run.frames.length - 1);
    renderOutcome(run);
    renderFrame(0);
    const planner = bundle.planners.find((candidate) => candidate.id === plannerId);
    if (shouldAnnounce) announce(`${planner?.label ?? plannerId} trace loaded`);
  };

  const updateScenario = (): void => {
    setPlaying(false);
    const scenario = bundle.scenarios.find((candidate) => candidate.id === scenarioSelect.value);
    if (!scenario) throw new Error(`Unknown scenario: ${scenarioSelect.value}`);
    currentScenario = scenario;
    viewer?.setScenario(scenario);
    const previousPlanner = plannerSelect.value;
    plannerSelect.replaceChildren();
    for (const planner of bundle.planners) {
      const option = document.createElement("option");
      option.value = planner.id;
      option.textContent = planner.label;
      option.selected = planner.id === previousPlanner;
      plannerSelect.append(option);
    }
    if (!plannerSelect.value) plannerSelect.value = bundle.planners[0]!.id;
    updateRun(plannerSelect.value, false);
    renderComparison(bundle, scenario);
    element("#scene-caption").textContent =
      `${scenario.description} ENU coordinates; distances in metres and trace time in seconds.`;
    viewerHost.setAttribute(
      "aria-label",
      `${scenario.label}: ${scenario.buildings.length} buildings, ` +
        `${scenario.staticNoFlyZones.length} static zones, ` +
        `${scenario.temporaryNoFlyZones.length} temporary zones, and ` +
        `${scenario.movingSpheres.length} moving obstacles.`,
    );
    announce(`${scenario.label} loaded`);
  };

  scenarioSelect.addEventListener("change", updateScenario);
  plannerSelect.addEventListener("change", () => updateRun(plannerSelect.value));
  timeline.addEventListener("input", () => {
    setPlaying(false);
    renderFrame(Number(timeline.value), true);
  });
  previous.addEventListener("click", () => {
    setPlaying(false);
    renderFrame(frameIndex - 1, true);
  });
  next.addEventListener("click", () => {
    setPlaying(false);
    renderFrame(frameIndex + 1, true);
  });
  playPause.addEventListener("click", () => {
    if (animationFrame === null) beginPlayback();
    else {
      setPlaying(false);
      announce("Playback paused");
    }
  });
  document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((button) => {
    button.addEventListener("click", () => {
      const view = button.dataset.view;
      if (view === "isometric" || view === "top" || view === "reset") viewer?.setView(view);
    });
  });

  const commitUrl = `https://github.com/Revincxt/uav-3d-planner-lab/commit/${bundle.sourceCommit}`;
  const provenance = element("#provenance");
  const sourceLink = document.createElement("a");
  sourceLink.href = commitUrl;
  sourceLink.textContent = bundle.sourceCommit.slice(0, 12);
  const generated = document.createElement("p");
  generated.append(
    document.createTextNode(`Generated ${bundle.generatedAt}; ${bundle.verificationStatus}; source commit `),
    sourceLink,
    document.createTextNode("."),
  );
  const method = document.createElement("p");
  method.textContent =
    "Frames are deterministic records produced by the Python simulator. Geometry and event streams are shared across planners within each scenario. Wall-clock planning time is not recorded in this deterministic demonstration. Work is an algorithm-specific diagnostic; expanded nodes and D* Lite queue pops are not equivalent units. The page interpolates no states, and these illustrative runs are not statistical evidence.";
  const protocol = document.createElement("p");
  protocol.textContent =
    `Protocol ${bundle.protocol.id}: ${bundle.protocol.timeStepS} s simulation step, ` +
    `${bundle.protocol.replanIntervalS} s replanning interval, ` +
    `${bundle.protocol.cruiseSpeedMps} m/s cruise speed, ${bundle.protocol.maxTimeS} s horizon, ` +
    `${bundle.protocol.resolutionM} m grid, and ${bundle.protocol.maxExpansions.toLocaleString()} maximum expansions.`;
  const downloads = document.createElement("p");
  downloads.append(document.createTextNode("Download: "));
  const recordsLink = document.createElement("a");
  recordsLink.href = `./${bundle.downloads.recordsCsv.path}`;
  recordsLink.textContent = "run records (CSV)";
  const manifestLink = document.createElement("a");
  manifestLink.href = `./${bundle.downloads.scenarioManifest.path}`;
  manifestLink.textContent = "scenario manifest (JSON)";
  downloads.append(
    recordsLink,
    document.createTextNode(" and "),
    manifestLink,
    document.createTextNode("."),
  );
  provenance.replaceChildren(generated, method, protocol, downloads);

  scenarioSelect.value = bundle.scenarios[0]!.id;
  currentScenario = bundle.scenarios[0]!;
  currentRun = currentScenario.runs[0]!;
  updateScenario();
  element("#load-state").remove();
  window.addEventListener(
    "pagehide",
    () => {
      setPlaying(false);
      viewer?.dispose();
    },
    { once: true },
  );
}

start().catch((error: unknown) => {
  const state = element("#load-state");
  state.classList.add("is-error");
  state.textContent = error instanceof Error ? error.message : "Dynamic traces could not load.";
  console.error(error);
});
