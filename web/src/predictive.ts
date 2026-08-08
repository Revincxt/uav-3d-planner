import "./predictive.css";

import { buildPredictiveComparisonRows, loadPredictiveBundle } from "./predictive-data";
import type {
  PredictiveBundleV3,
  PredictiveEvent,
  PredictiveFrame,
  PredictivePathMode,
  PredictiveRun,
  PredictiveScenario,
  TimedWaypoint,
  Vec3,
} from "./predictive-schema";
import {
  isMinimumSeparationEvidenceTime,
  minimumSeparationEvidence,
  PredictiveViewer,
  type ViewPreset,
  type ViewerLayer,
} from "./predictive-viewer";

const SVG_NS = "http://www.w3.org/2000/svg";
const EPSILON = 1e-8;

interface TurnSample {
  timeS: number;
  angleDeg: number;
}

function element<T extends HTMLElement>(selector: string): T {
  const match = document.querySelector<T>(selector);
  if (!match) throw new Error(`Missing page element: ${selector}`);
  return match;
}

function svgElement<K extends keyof SVGElementTagNameMap>(
  name: K,
  attributes: Record<string, string> = {},
): SVGElementTagNameMap[K] {
  const node = document.createElementNS(SVG_NS, name);
  for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, value);
  return node;
}

function samePoint(left: Vec3, right: Vec3): boolean {
  return left.every(
    (coordinate, index) =>
      Math.abs(coordinate - right[index]!) <=
      1e-7 * Math.max(1, Math.abs(coordinate), Math.abs(right[index]!)),
  );
}

function timedPosition(waypoints: TimedWaypoint[], timeS: number): Vec3 {
  const first = waypoints[0]!;
  const last = waypoints.at(-1)!;
  if (timeS <= first.timeS) return first.position;
  if (timeS >= last.timeS) return last.position;
  for (let index = 1; index < waypoints.length; index += 1) {
    const right = waypoints[index]!;
    const left = waypoints[index - 1]!;
    if (timeS <= right.timeS) {
      const fraction = (timeS - left.timeS) / (right.timeS - left.timeS);
      return left.position.map(
        (coordinate, axis) => coordinate + (right.position[axis]! - coordinate) * fraction,
      ) as Vec3;
    }
  }
  return last.position;
}

function selectedPath(run: PredictiveRun, mode: PredictivePathMode): TimedWaypoint[] {
  if (mode === "raw") return run.rawTimedPath;
  if (mode === "execution" && run.executionTimedPath !== null) return run.executionTimedPath;
  return run.geometryTimedPath;
}

function selectedMetrics(run: PredictiveRun, mode: PredictivePathMode) {
  if (mode === "raw") return run.plannerMetrics;
  if (mode === "execution" && run.executionMetrics !== null) return run.executionMetrics;
  return run.geometryMetrics;
}

function selectedWaitIntervals(run: PredictiveRun, mode: PredictivePathMode) {
  if (mode === "execution" && run.executionWaitIntervals !== null) {
    return run.executionWaitIntervals;
  }
  return run.geometryWaitIntervals;
}

function selectedFrames(run: PredictiveRun, mode: PredictivePathMode): PredictiveFrame[] {
  if (mode === "execution" && run.executionFrames !== null) return run.executionFrames;
  return run.geometryFrames;
}

function defaultPathMode(run: PredictiveRun): PredictivePathMode {
  return run.executionTimedPath === null ? "geometry" : "execution";
}

function formatNumber(value: number | null, digits = 1): string {
  return value === null ? "—" : value.toFixed(digits);
}

function formatWorkUnit(unit: PredictiveRun["plannerMetrics"]["workUnit"]): string {
  if (unit === "expanded-nodes") return "nodes";
  if (unit === "queue-pops") return "queue pops";
  return "space–time states";
}

function formatPosition(point: Vec3): string {
  return `${point[0].toFixed(1)}, ${point[1].toFixed(1)}, ${point[2].toFixed(1)} m`;
}

function formatStatus(status: PredictiveRun["status"]): string {
  if (status === "no-path") return "No path";
  return status.charAt(0).toUpperCase() + status.slice(1);
}

function setDefinitionRows(host: HTMLElement, rows: Array<[string, string]>): void {
  host.replaceChildren(
    ...rows.map(([term, description]) => {
      const row = document.createElement("div");
      const dt = document.createElement("dt");
      dt.textContent = term;
      const dd = document.createElement("dd");
      dd.textContent = description;
      row.append(dt, dd);
      return row;
    }),
  );
}

function visibleEvents(
  run: PredictiveRun,
  mode: PredictivePathMode,
): Array<{ frame: PredictiveFrame; index: number }> {
  return selectedFrames(run, mode)
    .map((frame, index) => ({ frame, index }))
    .filter(({ frame }) => frame.event !== null && frame.event.kind !== "none");
}

function mostRecentEvent(
  run: PredictiveRun,
  mode: PredictivePathMode,
  timeS: number,
): PredictiveFrame | null {
  return (
    [...visibleEvents(run, mode)]
      .reverse()
      .find(({ frame }) => frame.timeS <= timeS + EPSILON)?.frame ?? null
  );
}

function turnSamples(waypoints: TimedWaypoint[]): TurnSample[] {
  const samples: TurnSample[] = [{ timeS: waypoints[0]!.timeS, angleDeg: 0 }];
  for (let index = 1; index < waypoints.length - 1; index += 1) {
    const previous = waypoints[index - 1]!;
    const current = waypoints[index]!;
    const next = waypoints[index + 1]!;
    const incoming = current.position.map(
      (coordinate, axis) => coordinate - previous.position[axis]!,
    ) as Vec3;
    const outgoing = next.position.map(
      (coordinate, axis) => coordinate - current.position[axis]!,
    ) as Vec3;
    const incomingLength = Math.hypot(...incoming);
    const outgoingLength = Math.hypot(...outgoing);
    if (incomingLength <= EPSILON || outgoingLength <= EPSILON) continue;
    const cosine = Math.max(
      -1,
      Math.min(
        1,
        incoming.reduce((total, coordinate, axis) => total + coordinate * outgoing[axis]!, 0) /
          (incomingLength * outgoingLength),
      ),
    );
    samples.push({ timeS: current.timeS, angleDeg: (Math.acos(cosine) * 180) / Math.PI });
  }
  samples.push({ timeS: waypoints.at(-1)!.timeS, angleDeg: 0 });
  return samples;
}

function sampledValue(samples: TurnSample[], timeS: number): number {
  if (samples.length === 0) return 0;
  const nearest = samples.reduce((best, sample) =>
    Math.abs(sample.timeS - timeS) < Math.abs(best.timeS - timeS) ? sample : best,
  );
  return nearest.angleDeg;
}

function renderEventAxis(
  run: PredictiveRun,
  mode: PredictivePathMode,
  duration: number,
  timeS: number,
): void {
  const host = element<HTMLDivElement>("#event-axis");
  const width = Math.max(300, Math.round(host.clientWidth || 760));
  const height = 42;
  const margin = 10;
  const axisY = 14;
  const x = (value: number): number => margin + (value / Math.max(duration, 1)) * (width - margin * 2);
  const events = visibleEvents(run, mode).filter(({ frame }) => frame.timeS <= duration + EPSILON);
  const witness = selectedMetrics(run, mode).minimumSeparationWitness;
  const visibleWitness = witness !== null && witness.timeS <= duration + EPSILON ? witness : null;

  const svg = svgElement("svg", {
    viewBox: `0 0 ${width} ${height}`,
    role: "img",
    "aria-label":
      `${events.length} recorded events over ${duration.toFixed(1)} seconds` +
      (visibleWitness === null ? "; closest-approach witness unavailable" : "; closest-approach witness marked"),
  });
  const title = svgElement("title");
  title.textContent = "Recorded event positions";
  const desc = svgElement("desc");
  desc.textContent =
    "Marks locate planning, restriction, waiting, prediction, failure, and arrival events. " +
    "The brown witness rule locates minimum separation; dashed means approximate. The blue rule is continuous replay time.";
  svg.append(title, desc);
  svg.append(
    svgElement("line", {
      x1: String(margin),
      x2: String(width - margin),
      y1: String(axisY),
      y2: String(axisY),
      class: "axis-line",
    }),
  );

  for (const { frame } of events) {
    const tick = svgElement("line", {
      x1: String(x(frame.timeS)),
      x2: String(x(frame.timeS)),
      y1: "5",
      y2: "23",
      class: "event-tick",
      "data-event-time": String(frame.timeS),
    });
    const tickTitle = svgElement("title");
    tickTitle.textContent = `${frame.timeS.toFixed(1)} s — ${frame.event!.label}`;
    tick.append(tickTitle);
    svg.append(tick);
  }

  if (visibleWitness !== null) {
    const witnessTick = svgElement("line", {
      x1: String(x(visibleWitness.timeS)),
      x2: String(x(visibleWitness.timeS)),
      y1: "2",
      y2: "27",
      class: `witness-tick ${visibleWitness.exact ? "is-exact" : "is-approximate"}`,
      "data-witness-time": String(visibleWitness.timeS),
    });
    const witnessTitle = svgElement("title");
    witnessTitle.textContent =
      `${visibleWitness.timeS.toFixed(2)} s — ${visibleWitness.separationM.toFixed(2)} m ` +
      `${visibleWitness.exact ? "exact" : "approximate"} minimum-separation witness`;
    witnessTick.append(witnessTitle);
    svg.append(witnessTick);
  }

  svg.append(
    svgElement("line", {
      x1: String(x(timeS)),
      x2: String(x(timeS)),
      y1: "2",
      y2: "27",
      class: "event-tick is-current replay-time-rule",
    }),
  );
  const startLabel = svgElement("text", { x: String(margin), y: "39" });
  startLabel.textContent = "0 s";
  const endLabel = svgElement("text", {
    x: String(width - margin),
    y: "39",
    "text-anchor": "end",
  });
  endLabel.textContent = `${duration.toFixed(1)} s`;
  svg.append(startLabel, endLabel);
  host.replaceChildren(svg);
  host.dataset.duration = String(duration);
  host.dataset.width = String(width);
  host.dataset.margin = String(margin);
  updateEventAxis(timeS);
}

function updateEventAxis(timeS: number): void {
  const host = element<HTMLDivElement>("#event-axis");
  const duration = Number(host.dataset.duration ?? 1);
  const width = Number(host.dataset.width ?? 300);
  const margin = Number(host.dataset.margin ?? 10);
  const x = margin + (Math.max(0, Math.min(timeS, duration)) / Math.max(duration, 1)) * (width - margin * 2);
  host.querySelector<SVGLineElement>(".replay-time-rule")?.setAttribute("x1", String(x));
  host.querySelector<SVGLineElement>(".replay-time-rule")?.setAttribute("x2", String(x));
}

function renderLineChart(
  host: HTMLElement,
  active: Array<{ timeS: number; value: number }>,
  raw: Array<{ timeS: number; value: number }> | null,
  duration: number,
  yMin: number,
  yMax: number,
  yLabel: string,
  waits: PredictiveRun["geometryWaitIntervals"],
): void {
  const width = Math.max(320, Math.round(host.clientWidth || 620));
  const height = 220;
  const margin = { top: 15, right: 16, bottom: 36, left: 49 };
  const plotWidth = width - margin.left - margin.right;
  const plotHeight = height - margin.top - margin.bottom;
  const domainSpan = Math.max(EPSILON, yMax - yMin);
  const x = (timeS: number): number => margin.left + (timeS / Math.max(duration, 1)) * plotWidth;
  const y = (value: number): number => margin.top + ((yMax - value) / domainSpan) * plotHeight;
  const svg = svgElement("svg", {
    viewBox: `0 0 ${width} ${height}`,
    role: "img",
    "aria-label": `${yLabel} over ${duration.toFixed(1)} seconds`,
  });

  for (const interval of waits) {
    svg.append(
      svgElement("rect", {
        x: String(x(interval.startTimeS)),
        y: String(margin.top),
        width: String(Math.max(1, x(interval.endTimeS) - x(interval.startTimeS))),
        height: String(plotHeight),
        class: "wait-band",
      }),
    );
  }
  const xTicks = width < 470 ? 3 : 5;
  for (let index = 0; index <= xTicks; index += 1) {
    const fraction = index / xTicks;
    const tickX = margin.left + fraction * plotWidth;
    svg.append(
      svgElement("line", {
        x1: String(tickX),
        x2: String(tickX),
        y1: String(margin.top),
        y2: String(margin.top + plotHeight),
        class: "grid-line",
      }),
    );
    const label = svgElement("text", {
      x: String(tickX),
      y: String(height - 16),
      "text-anchor": "middle",
    });
    label.textContent = (duration * fraction).toFixed(0);
    svg.append(label);
  }
  for (let index = 0; index <= 4; index += 1) {
    const fraction = index / 4;
    const tickY = margin.top + fraction * plotHeight;
    svg.append(
      svgElement("line", {
        x1: String(margin.left),
        x2: String(margin.left + plotWidth),
        y1: String(tickY),
        y2: String(tickY),
        class: "grid-line",
      }),
    );
    const label = svgElement("text", {
      x: String(margin.left - 8),
      y: String(tickY + 4),
      "text-anchor": "end",
    });
    label.textContent = (yMax - fraction * domainSpan).toFixed(0);
    svg.append(label);
  }
  svg.append(
    svgElement("line", {
      x1: String(margin.left),
      x2: String(margin.left + plotWidth),
      y1: String(margin.top + plotHeight),
      y2: String(margin.top + plotHeight),
      class: "axis-line",
    }),
    svgElement("line", {
      x1: String(margin.left),
      x2: String(margin.left),
      y1: String(margin.top),
      y2: String(margin.top + plotHeight),
      class: "axis-line",
    }),
  );
  if (raw) {
    svg.append(
      svgElement("polyline", {
        points: raw.map((point) => `${x(point.timeS)},${y(point.value)}`).join(" "),
        class: "raw-line",
      }),
    );
  }
  svg.append(
    svgElement("polyline", {
      points: active.map((point) => `${x(point.timeS)},${y(point.value)}`).join(" "),
      class: "data-line",
    }),
  );
  for (const point of active) {
    if (active.length > 50 && point !== active[0] && point !== active.at(-1)) continue;
    svg.append(
      svgElement("circle", {
        cx: String(x(point.timeS)),
        cy: String(y(point.value)),
        r: "2.2",
        class: "waypoint",
      }),
    );
  }
  svg.append(
    svgElement("line", {
      x1: String(x(0)),
      x2: String(x(0)),
      y1: String(margin.top),
      y2: String(margin.top + plotHeight),
      class: "current-rule",
    }),
    svgElement("circle", {
      cx: String(x(0)),
      cy: String(y(active[0]!.value)),
      r: "3.7",
      class: "current-point",
    }),
  );
  const currentLabel = svgElement("text", {
    x: String(x(0) + 7),
    y: String(Math.max(margin.top + 12, y(active[0]!.value) - 7)),
    class: "current-label",
  });
  currentLabel.textContent = `0.0 s · ${active[0]!.value.toFixed(1)}`;
  svg.append(currentLabel);
  const xLabel = svgElement("text", {
    x: String(margin.left + plotWidth / 2),
    y: String(height - 2),
    "text-anchor": "middle",
  });
  xLabel.textContent = "Time (s)";
  const verticalLabel = svgElement("text", {
    x: "12",
    y: String(margin.top + plotHeight / 2),
    transform: `rotate(-90 12 ${margin.top + plotHeight / 2})`,
    "text-anchor": "middle",
  });
  verticalLabel.textContent = yLabel;
  svg.append(xLabel, verticalLabel);
  host.replaceChildren(svg);
  Object.assign(host.dataset, {
    duration: String(duration),
    yMin: String(yMin),
    yMax: String(yMax),
    width: String(width),
    marginLeft: String(margin.left),
    marginRight: String(margin.right),
    marginTop: String(margin.top),
    plotHeight: String(plotHeight),
  });
}

function updateChartCursor(host: HTMLElement, timeS: number, value: number, suffix: string): void {
  const duration = Number(host.dataset.duration ?? 1);
  const yMin = Number(host.dataset.yMin ?? 0);
  const yMax = Number(host.dataset.yMax ?? 1);
  const width = Number(host.dataset.width ?? 320);
  const marginLeft = Number(host.dataset.marginLeft ?? 49);
  const marginRight = Number(host.dataset.marginRight ?? 16);
  const marginTop = Number(host.dataset.marginTop ?? 15);
  const plotHeight = Number(host.dataset.plotHeight ?? 169);
  const plotWidth = width - marginLeft - marginRight;
  const x = marginLeft + (Math.max(0, Math.min(timeS, duration)) / Math.max(duration, 1)) * plotWidth;
  const y = marginTop + ((yMax - value) / Math.max(EPSILON, yMax - yMin)) * plotHeight;
  const rule = host.querySelector<SVGLineElement>(".current-rule");
  rule?.setAttribute("x1", String(x));
  rule?.setAttribute("x2", String(x));
  const point = host.querySelector<SVGCircleElement>(".current-point");
  point?.setAttribute("cx", String(x));
  point?.setAttribute("cy", String(y));
  const label = host.querySelector<SVGTextElement>(".current-label");
  if (label) {
    const end = x > width * 0.72;
    label.setAttribute("x", String(x + (end ? -7 : 7)));
    label.setAttribute("y", String(Math.max(marginTop + 12, y - 7)));
    label.setAttribute("text-anchor", end ? "end" : "start");
    label.textContent = `${timeS.toFixed(1)} s · ${value.toFixed(1)} ${suffix}`;
  }
}

function renderComparison(
  bundle: PredictiveBundleV3,
  scenario: PredictiveScenario,
  selectedPlannerId: string,
  onSelect: (plannerId: string) => void,
): void {
  const body = element<HTMLTableSectionElement>("#comparison-body");
  body.replaceChildren();
  for (const row of buildPredictiveComparisonRows(bundle, scenario)) {
    const tableRow = document.createElement("tr");
    tableRow.tabIndex = 0;
    tableRow.setAttribute("aria-current", String(row.plannerId === selectedPlannerId));
    tableRow.setAttribute("aria-label", `Inspect ${row.plannerLabel}`);
    tableRow.dataset.information = row.predictive ? "complete-schedule" : "snapshot";
    const planner = document.createElement("th");
    planner.scope = "row";
    planner.textContent = row.plannerLabel;
    const values = [
      row.predictive ? "Complete schedule" : "Current snapshot",
      formatStatus(row.status),
      formatNumber(row.arrivalTimeS),
      formatNumber(row.waitTimeS),
      formatNumber(row.executedPathLengthM),
      row.executionQualified
        ? "Qualified · collision audited"
        : row.executionStatus.replaceAll("-", " "),
      row.maxTurnAngleAfterDeg === null ? "—" : `${row.maxTurnAngleAfterDeg.toFixed(1)}°`,
      `${row.safetyViolations.toLocaleString()} violations`,
      `${row.expandedStates.toLocaleString()} ${formatWorkUnit(row.workUnit)}`,
    ];
    tableRow.append(planner);
    values.forEach((value, index) => {
      const cell = document.createElement("td");
      cell.textContent = value;
      if (index === 1) cell.className = row.status === "success" ? "status-success" : "status-failed";
      tableRow.append(cell);
    });
    const activate = (): void => onSelect(row.plannerId);
    tableRow.addEventListener("click", activate);
    tableRow.addEventListener("keydown", (event) => {
      if (event.key !== "Enter" && event.key !== " ") return;
      event.preventDefault();
      activate();
    });
    body.append(tableRow);
  }
}

async function start(): Promise<void> {
  const bundle = await loadPredictiveBundle();
  element("#study-design-count").textContent =
    `${bundle.scenarios.length} cases × ${bundle.planners.length} conditions`;
  const recordedRuns = bundle.scenarios.flatMap((scenario) => scenario.runs);
  const qualifiedRuns = recordedRuns.filter((run) => run.smoothing.execution.qualified).length;
  element("#study-evidence-count").textContent =
    `${qualifiedRuns}/${recordedRuns.length} execution-qualified`;
  const scenarioSelect = element<HTMLSelectElement>("#scenario-select");
  const plannerSelect = element<HTMLSelectElement>("#planner-select");
  const timeline = element<HTMLInputElement>("#timeline");
  const timelineValue = element<HTMLOutputElement>("#timeline-value");
  const previous = element<HTMLButtonElement>("#previous-frame");
  const next = element<HTMLButtonElement>("#next-frame");
  const playPause = element<HTMLButtonElement>("#play-pause");
  const jumpToWitness = element<HTMLButtonElement>("#jump-to-witness");
  const viewerHost = element<HTMLDivElement>("#predictive-viewer");
  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
  let viewer: PredictiveViewer | null = null;
  try {
    viewer = new PredictiveViewer(viewerHost);
  } catch (error) {
    viewerHost.classList.add("viewer-error");
    viewerHost.textContent = "WebGL is unavailable; recorded charts and tables remain accessible.";
    console.warn(error);
  }

  const query = new URLSearchParams(window.location.search);
  const preferredScenario = bundle.scenarios.find((scenario) => scenario.id === query.get("scenario"));
  let currentScenario =
    preferredScenario ??
    bundle.scenarios.find(
      (scenario) =>
        scenario.cohort === "demo" &&
        scenario.environment.buildingCount >= 14 &&
        scenario.runs.some((run) => run.predictive && run.smoothing.applied),
    ) ??
    bundle.scenarios.find((scenario) => scenario.cohort === "demo") ??
    bundle.scenarios[0]!;
  const preferredPlanner = bundle.planners.find((planner) => planner.id === query.get("planner"));
  let currentRun =
    currentScenario.runs.find((run) => run.plannerId === preferredPlanner?.id) ??
    currentScenario.runs.find((run) => bundle.planners.find((planner) => planner.id === run.plannerId)?.predictive) ??
    currentScenario.runs[0]!;
  const requestedPathMode = query.get("path");
  let pathMode: PredictivePathMode =
    requestedPathMode === "raw" ||
    requestedPathMode === "geometry" ||
    requestedPathMode === "execution"
      ? requestedPathMode
      : defaultPathMode(currentRun);
  if (pathMode === "execution" && currentRun.executionTimedPath === null) {
    pathMode = "geometry";
  }
  const requestedInitialTimeS = Math.max(0, Number(query.get("time")) || 0);
  let currentTimeS = requestedInitialTimeS;
  let playbackSpeed = 1;
  let animationFrame: number | null = null;
  let lastWallTime = 0;
  let lastUrlWrite = 0;
  let altitudeResizeObserver: ResizeObserver | null = null;

  for (const scenario of bundle.scenarios) {
    const option = document.createElement("option");
    option.value = scenario.id;
    option.textContent =
      `${scenario.label} · ${scenario.environment.buildingCount} buildings · ` +
      `${scenario.environment.hazardCount} hazards`;
    scenarioSelect.append(option);
  }

  const announce = (message: string): void => {
    element("#live-region").textContent = message;
  };

  const updateUrl = (force = false): void => {
    const now = performance.now();
    if (!force && now - lastUrlWrite < 250) return;
    lastUrlWrite = now;
    const parameters = new URLSearchParams();
    parameters.set("scenario", currentScenario.id);
    parameters.set("planner", currentRun.plannerId);
    parameters.set("path", pathMode);
    const evidence = minimumSeparationEvidence(currentScenario, currentRun, pathMode);
    parameters.set(
      "time",
      isMinimumSeparationEvidenceTime(evidence, currentTimeS)
        ? String(evidence!.witness.timeS)
        : currentTimeS.toFixed(2),
    );
    window.history.replaceState(null, "", `${window.location.pathname}?${parameters}${window.location.hash}`);
  };

  const currentPath = (): TimedWaypoint[] => selectedPath(currentRun, pathMode);
  const duration = (): number => currentPath().at(-1)!.timeS;

  const renderCharts = (): void => {
    const path = currentPath();
    const raw = currentRun.rawTimedPath;
    const total = duration();
    const altitudeHost = element<HTMLDivElement>("#altitude-chart");
    const turnHost = element<HTMLDivElement>("#turn-chart");
    const altitudeData = path.map((waypoint) => ({ timeS: waypoint.timeS, value: waypoint.position[2] }));
    const rawAltitude =
      pathMode !== "raw" && currentRun.smoothing.applied
        ? raw.map((waypoint) => ({ timeS: waypoint.timeS, value: waypoint.position[2] }))
        : null;
    renderLineChart(
      altitudeHost,
      altitudeData,
      rawAltitude,
      total,
      currentScenario.bounds.min[2],
      currentScenario.bounds.max[2],
      "Altitude (m)",
      selectedWaitIntervals(currentRun, pathMode),
    );
    const activeTurns = turnSamples(path);
    const rawTurns = pathMode !== "raw" && currentRun.smoothing.applied ? turnSamples(raw) : null;
    const maximumTurn = Math.max(
      30,
      ...activeTurns.map((sample) => sample.angleDeg),
      ...(rawTurns?.map((sample) => sample.angleDeg) ?? []),
    );
    const turnCeiling = Math.min(180, Math.ceil(maximumTurn / 15) * 15);
    renderLineChart(
      turnHost,
      activeTurns.map((sample) => ({ timeS: sample.timeS, value: sample.angleDeg })),
      rawTurns?.map((sample) => ({ timeS: sample.timeS, value: sample.angleDeg })) ?? null,
      total,
      0,
      turnCeiling,
      "Turn angle (°)",
      [],
    );
  };

  const renderTime = (timeS: number, shouldAnnounce = false): void => {
    const total = duration();
    currentTimeS = Math.max(0, Math.min(timeS, total));
    const path = currentPath();
    const position = timedPosition(path, currentTimeS);
    const events = visibleEvents(currentRun, pathMode).filter(
      ({ frame }) => frame.timeS <= total + EPSILON,
    );
    const event = mostRecentEvent(currentRun, pathMode, currentTimeS);
    const activeZones = currentScenario.temporaryNoFlyZones.filter(
      (zone) => zone.activeFromS <= currentTimeS && currentTimeS < zone.activeUntilS,
    );
    const evidence = minimumSeparationEvidence(currentScenario, currentRun, pathMode);
    const witnessIsVisible = isMinimumSeparationEvidenceTime(evidence, currentTimeS);
    const witnessReadout = element<HTMLDivElement>("#viewer-witness-readout");

    timeline.value = String(currentTimeS);
    timelineValue.value = `${currentTimeS.toFixed(2)} s`;
    element("#viewer-time-value").textContent = `${currentTimeS.toFixed(2)} s`;
    viewer?.setTime(currentTimeS);
    witnessReadout.hidden = !witnessIsVisible;
    if (witnessIsVisible && evidence !== null) {
      const { witness } = evidence;
      witnessReadout.dataset.quality = witness.exact ? "exact" : "approximate";
      element("#viewer-witness-value").textContent =
        `${witness.separationM.toFixed(2)} m surface separation`;
      element("#viewer-witness-detail").textContent =
        `${witness.exact ? "Exact · solid connector" : "Approximate · dashed connector"}; ` +
        `sphere = vehicle center, diamond = obstacle surface, wire shell = ` +
        `${evidence.safetyEnvelopeRadiusM.toFixed(2)} m safety envelope. Surface separation ` +
        "subtracts the vehicle radius from the connector span.";
    }
    updateEventAxis(currentTimeS);
    updateChartCursor(element("#altitude-chart"), currentTimeS, position[2], "m");
    updateChartCursor(
      element("#turn-chart"),
      currentTimeS,
      sampledValue(turnSamples(path), currentTimeS),
      "°",
    );
    const currentRows: Array<[string, string]> = [
      ["Mission time", `${currentTimeS.toFixed(2)} s`],
      ["ENU position", formatPosition(position)],
      ["Altitude", `${position[2].toFixed(1)} m`],
      ["Active restrictions", activeZones.length.toLocaleString()],
      ["Moving hazards", currentScenario.movingSpheres.length.toLocaleString()],
    ];
    if (witnessIsVisible && evidence !== null) {
      currentRows.push([
        "Evidence layer",
        `${evidence.witness.exact ? "Exact" : "Approximate"} closest approach`,
      ]);
    }
    setDefinitionRows(element("#current-meta"), currentRows);
    element("#current-event").textContent = event?.event
      ? `Most recent event · ${event.timeS.toFixed(1)} s — ${event.event.label}`
      : "No recorded event has occurred yet.";
    element("#frame-summary").textContent =
      `${currentTimeS.toFixed(1)} s of ${total.toFixed(1)} s · ` +
      `${position[2].toFixed(1)} m altitude · ` +
      (pathMode === "raw"
        ? "raw planner output"
        : pathMode === "geometry"
          ? "collision-certified geometry candidate"
          : "discrete-envelope-qualified execution candidate");
    element("#canvas-summary").textContent =
      `${currentScenario.label}. At ${currentTimeS.toFixed(1)} seconds the vehicle is at ` +
      `${formatPosition(position)}. ${activeZones.length} temporary restrictions are active and ` +
      `${currentScenario.movingSpheres.length} moving hazards are shown.` +
      (witnessIsVisible && evidence !== null
        ? ` The ${evidence.witness.exact ? "exact" : "approximate"} closest-approach evidence layer ` +
          `shows ${evidence.witness.separationM.toFixed(2)} metres of surface separation to ` +
          `${evidence.witness.obstacleId}.`
        : "");
    previous.disabled = !events.some(({ frame }) => frame.timeS < currentTimeS - EPSILON);
    next.disabled = !events.some(({ frame }) => frame.timeS > currentTimeS + EPSILON);
    if (shouldAnnounce) announce(`Showing ${currentTimeS.toFixed(1)} seconds`);
    updateUrl();
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
    if (lastWallTime === 0) lastWallTime = now;
    const elapsed = Math.min(0.12, Math.max(0, (now - lastWallTime) / 1000));
    lastWallTime = now;
    renderTime(currentTimeS + elapsed * playbackSpeed);
    if (currentTimeS >= duration() - EPSILON) {
      setPlaying(false);
      announce("Playback complete");
      return;
    }
    animationFrame = requestAnimationFrame(playbackTick);
  };

  const beginPlayback = (): void => {
    if (currentTimeS >= duration() - EPSILON) renderTime(0);
    lastWallTime = 0;
    setPlaying(true);
    animationFrame = requestAnimationFrame(playbackTick);
  };

  const syncPathControls = (): void => {
    document.querySelectorAll<HTMLButtonElement>("[data-path-mode]").forEach((button) => {
      button.setAttribute("aria-pressed", String(button.dataset.pathMode === pathMode));
      button.disabled =
        (button.dataset.pathMode === "geometry" && !currentRun.smoothing.certified) ||
        (button.dataset.pathMode === "execution" && currentRun.executionTimedPath === null);
    });
    const rawLayer = element<HTMLInputElement>("[data-layer='raw']");
    rawLayer.disabled = pathMode === "raw" || !currentRun.smoothing.applied;
    if (rawLayer.disabled) {
      rawLayer.checked = false;
      viewer?.setLayerVisibility("raw", false);
    }
  };

  const renderRunEvidence = (): void => {
    const smoothing = currentRun.smoothing;
    const metrics = currentRun.plannerMetrics;
    const layerMetrics = selectedMetrics(currentRun, pathMode);
    const diagnostics = smoothing.kinematicDiagnostics;
    const separationWitness = layerMetrics.minimumSeparationWitness;
    const evidence = minimumSeparationEvidence(currentScenario, currentRun, pathMode);
    const planner = bundle.planners.find((candidate) => candidate.id === currentRun.plannerId);
    const status = element("#run-status");
    status.textContent = formatStatus(currentRun.status);
    status.className = `run-status ${currentRun.status === "success" ? "is-success" : "is-failed"}`;
    element("#case-information").textContent = currentRun.predictive
      ? "Complete deterministic schedule"
      : "Current conservative snapshot";
    element("#case-mission").textContent =
      `${planner?.label ?? currentRun.plannerId} · ${formatStatus(currentRun.status)}`;

    const outcome = element("#metric-outcome");
    outcome.textContent = formatStatus(currentRun.status);
    outcome.className = metrics.success ? "metric-positive" : "metric-negative";
    element("#metric-arrival").textContent =
      metrics.arrivalTimeS === null ? "—" : `${metrics.arrivalTimeS.toFixed(1)} s`;
    element("#metric-wait").textContent = `${metrics.waitTimeS.toFixed(1)} s`;
    element("#metric-path").textContent = `${metrics.executedPathLengthM.toFixed(1)} m`;
    const safety = element("#metric-safety");
    safety.textContent =
      metrics.safetyViolations === 0
        ? metrics.minimumSeparationWitness === null
          ? "0 violations"
          : `0 violations · ${metrics.minimumSeparationWitness.separationM.toFixed(2)} m min`
        : `${metrics.safetyViolations.toLocaleString()} violations`;
    safety.className = metrics.safetyViolations === 0 ? "metric-positive" : "metric-negative";

    const witnessQuality = element<HTMLSpanElement>("#witness-quality");
    if (evidence === null) {
      witnessQuality.textContent = "Unavailable";
      witnessQuality.className = "witness-quality is-unavailable";
      element("#witness-summary").textContent =
        "No minimum-separation witness was recorded for this condition.";
      setDefinitionRows(element("#witness-meta"), [
        ["Status", "Unavailable"],
        ["Interpretation", "No spatial witness can be displayed"],
      ]);
      jumpToWitness.disabled = true;
      jumpToWitness.textContent = "Closest approach unavailable";
      jumpToWitness.removeAttribute("data-witness-time");
      jumpToWitness.setAttribute("aria-label", "Closest approach unavailable for this condition");
    } else {
      const { witness } = evidence;
      const evidenceKind = witness.exact ? "Exact" : "Approximate";
      witnessQuality.textContent = evidenceKind;
      witnessQuality.className =
        `witness-quality ${witness.exact ? "is-exact" : "is-approximate"}`;
      element("#witness-summary").textContent =
        `${witness.separationM.toFixed(2)} m physical surface separation at ` +
        `${witness.timeS.toFixed(2)} s. The recorded witness is ` +
        `${witness.exact ? "exact" : "approximate"}; it does not alter the independent collision verdict.`;
      setDefinitionRows(element("#witness-meta"), [
        ["Obstacle", `${witness.obstacleId} · ${witness.obstacleKind.replace("-", " ")}`],
        ["Vehicle center", formatPosition(witness.vehiclePosition)],
        ["Obstacle surface", formatPosition(witness.obstaclePosition)],
        ["Declared margin", `${witness.declaredSafetyMarginM.toFixed(2)} m`],
        [
          "Safety envelope",
          `${currentScenario.constraints.vehicleRadiusM.toFixed(2)} + ` +
            `${witness.declaredSafetyMarginM.toFixed(2)} = ` +
            `${evidence.safetyEnvelopeRadiusM.toFixed(2)} m radius`,
        ],
        [
          "Connector",
          `${witness.exact ? "Solid · exact" : "Dashed · approximate"} · ${witness.method}`,
        ],
      ]);
      jumpToWitness.disabled = false;
      jumpToWitness.textContent = "Jump to closest approach";
      jumpToWitness.dataset.witnessTime = String(witness.timeS);
      jumpToWitness.setAttribute(
        "aria-label",
        `Jump to closest approach at ${witness.timeS.toFixed(2)} seconds; ${evidenceKind.toLowerCase()} witness`,
      );
    }
    setDefinitionRows(element("#smoothing-meta"), [
      [
        "Evidence layer",
        pathMode === "raw"
          ? "Raw planner output"
          : pathMode === "geometry"
            ? "Collision-certified geometry candidate"
            : "Qualified execution candidate",
      ],
      ["Method", smoothing.method],
      [
        "Collision audit",
        smoothing.collisionCertified ? "Passed · dense space–time polyline" : "Unavailable",
      ],
      [
        "Minimum dynamic separation",
        separationWitness === null
          ? "Not recorded"
          : `${separationWitness.separationM.toFixed(2)} m at ${separationWitness.timeS.toFixed(2)} s · ` +
            `${separationWitness.exact ? "exact" : "approximate"} witness · ${separationWitness.obstacleId}`,
      ],
      ["Waypoints", `${smoothing.rawWaypointCount} → ${smoothing.outputWaypointCount}`],
      [
        "Reversals",
        `${diagnostics.raw.reversalCount} → ${diagnostics.output.reversalCount}`,
      ],
      [
        "Acceleration proxy",
        `${diagnostics.raw.maxDiscreteAccelerationProxyMps2.toFixed(1)} → ` +
          `${diagnostics.output.maxDiscreteAccelerationProxyMps2.toFixed(1)} m/s²`,
      ],
      [
        "Execution qualification",
        smoothing.execution.qualified
          ? `Qualified · +${formatNumber(smoothing.execution.addedDurationS, 2)} s`
          : `Not qualified · ${smoothing.execution.status.replaceAll("-", " ")}`,
      ],
      [
        "Declared envelope",
        `${smoothing.execution.envelope.maxSpeedMps.toFixed(0)} m/s speed · ` +
          `${smoothing.execution.envelope.maxAbsClimbRateMps.toFixed(0)} m/s climb · ` +
          `${smoothing.execution.envelope.maxDiscreteAccelerationProxyMps2.toFixed(0)} m/s² proxy`,
      ],
      [
        "Maximum |climb rate|",
        `${diagnostics.raw.maxAbsClimbRateMps.toFixed(1)} → ` +
          `${diagnostics.output.maxAbsClimbRateMps.toFixed(1)} m/s`,
      ],
      [
        "Turn radius",
        smoothing.appliedTurnRadiusM === null ? "Fallback" : `${smoothing.appliedTurnRadiusM.toFixed(1)} m`,
      ],
      [
        "Peak turn",
        `${formatNumber(smoothing.maxTurnAngleBeforeDeg)}° → ${formatNumber(smoothing.maxTurnAngleAfterDeg)}°`,
      ],
    ]);
    element("#smoothing-note").textContent =
      `${smoothing.applied ? "The rounded dense polyline passed the declared collision audit." : "The collision-certified raw fallback is the geometry candidate."} ` +
      `${smoothing.execution.qualified ? "A separately retimed candidate also passed the declared discrete envelope and a repeated space–time collision audit." : "No execution candidate is published for this run."} ` +
      "These finite-difference checks do not certify continuous dynamics, jerk, attitude, thrust, or control feasibility.";
  };

  const updateRun = (plannerId: string, shouldAnnounce = true): void => {
    setPlaying(false);
    const run = currentScenario.runs.find((candidate) => candidate.plannerId === plannerId);
    if (!run) throw new Error(`Missing ${plannerId} run in ${currentScenario.id}`);
    currentRun = run;
    plannerSelect.value = plannerId;
    if (pathMode === "execution" && currentRun.executionTimedPath === null) {
      pathMode = "geometry";
    }
    if (pathMode === "geometry" && !currentRun.smoothing.certified) pathMode = "raw";
    currentTimeS = Math.min(currentTimeS, duration());
    timeline.max = String(duration());
    viewer?.setRun(run);
    viewer?.setPathMode(pathMode);
    syncPathControls();
    renderRunEvidence();
    renderComparison(bundle, currentScenario, plannerId, (selectedPlannerId) => {
      updateRun(selectedPlannerId);
    });
    renderEventAxis(currentRun, pathMode, duration(), currentTimeS);
    renderCharts();
    renderTime(currentTimeS);
    updateUrl(true);
    const planner = bundle.planners.find((candidate) => candidate.id === plannerId);
    if (shouldAnnounce) announce(`${planner?.label ?? plannerId} trace loaded`);
  };

  const updateScenario = (scenarioId: string, shouldAnnounce = true): void => {
    setPlaying(false);
    const scenario = bundle.scenarios.find((candidate) => candidate.id === scenarioId);
    if (!scenario) throw new Error(`Unknown scenario: ${scenarioId}`);
    currentScenario = scenario;
    scenarioSelect.value = scenario.id;
    viewer?.setScenario(scenario);
    const previousPlanner = currentRun?.plannerId;
    plannerSelect.replaceChildren();
    for (const planner of bundle.planners) {
      const option = document.createElement("option");
      option.value = planner.id;
      option.textContent = `${planner.label} · ${planner.predictive ? "complete forecast" : "reactive"}`;
      plannerSelect.append(option);
    }
    const nextRun =
      scenario.runs.find((run) => run.plannerId === previousPlanner) ??
      scenario.runs.find(
        (run) => bundle.planners.find((planner) => planner.id === run.plannerId)?.predictive,
      ) ??
      scenario.runs[0]!;
    currentTimeS = 0;
    element("#scenario-description").textContent = scenario.description;
    element("#case-geometry").textContent =
      `${scenario.environment.buildingCount} buildings · ` +
      `${scenario.staticNoFlyZones.length + scenario.temporaryNoFlyZones.length} restricted volumes · ` +
      `${scenario.movingSpheres.length} moving hazards`;
    setDefinitionRows(element("#scenario-meta"), [
      ["District", scenario.environment.district],
      ["Street pattern", scenario.environment.streetPattern],
      ["Buildings", scenario.environment.buildingCount.toLocaleString()],
      ["Hazards", scenario.environment.hazardCount.toLocaleString()],
      [
        "Bounds",
        `${(scenario.bounds.max[0] - scenario.bounds.min[0]).toFixed(0)} × ` +
          `${(scenario.bounds.max[1] - scenario.bounds.min[1]).toFixed(0)} × ` +
          `${(scenario.bounds.max[2] - scenario.bounds.min[2]).toFixed(0)} m`,
      ],
    ]);
    element("#scene-caption").textContent =
      `${scenario.environment.district}; ${scenario.environment.streetPattern}. ` +
      `Distances use ENU metres and recorded mission time.`;
    viewerHost.setAttribute(
      "aria-label",
      `${scenario.label}: ${scenario.environment.buildingCount} buildings, ` +
        `${scenario.staticNoFlyZones.length} static restrictions, ` +
        `${scenario.temporaryNoFlyZones.length} temporary restrictions, and ` +
        `${scenario.movingSpheres.length} moving hazards.`,
    );
    updateRun(nextRun.plannerId, false);
    updateUrl(true);
    if (shouldAnnounce) announce(`${scenario.label} loaded`);
  };

  scenarioSelect.addEventListener("change", () => updateScenario(scenarioSelect.value));
  plannerSelect.addEventListener("change", () => updateRun(plannerSelect.value));
  jumpToWitness.addEventListener("click", () => {
    const witness = selectedMetrics(currentRun, pathMode).minimumSeparationWitness;
    if (witness === null) return;
    setPlaying(false);
    renderTime(witness.timeS);
    updateUrl(true);
    announce(
      `Closest approach at ${witness.timeS.toFixed(2)} seconds; ` +
        `${witness.separationM.toFixed(2)} metres; ${witness.exact ? "exact" : "approximate"} witness`,
    );
  });
  timeline.addEventListener("input", () => {
    setPlaying(false);
    renderTime(Number(timeline.value), true);
    updateUrl(true);
  });
  previous.addEventListener("click", () => {
    setPlaying(false);
    const target = [...visibleEvents(currentRun, pathMode)]
      .reverse()
      .find(({ frame }) => frame.timeS < currentTimeS - EPSILON);
    if (target) renderTime(target.frame.timeS, true);
  });
  next.addEventListener("click", () => {
    setPlaying(false);
    const target = visibleEvents(currentRun, pathMode).find(
      ({ frame }) => frame.timeS > currentTimeS + EPSILON,
    );
    if (target) renderTime(target.frame.timeS, true);
  });
  playPause.addEventListener("click", () => {
    if (animationFrame === null) beginPlayback();
    else {
      setPlaying(false);
      announce("Playback paused");
    }
  });

  document.querySelectorAll<HTMLButtonElement>("[data-path-mode]").forEach((button) => {
    button.addEventListener("click", () => {
      const mode = button.dataset.pathMode;
      if (mode !== "raw" && mode !== "geometry" && mode !== "execution") return;
      if (mode === "geometry" && !currentRun.smoothing.certified) return;
      if (mode === "execution" && currentRun.executionTimedPath === null) return;
      setPlaying(false);
      pathMode = mode;
      viewer?.setPathMode(pathMode);
      currentTimeS = Math.min(currentTimeS, duration());
      timeline.max = String(duration());
      syncPathControls();
      renderRunEvidence();
      renderEventAxis(currentRun, pathMode, duration(), currentTimeS);
      renderCharts();
      renderTime(currentTimeS, true);
      updateUrl(true);
    });
  });

  document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((button) => {
    button.addEventListener("click", () => {
      const view = button.dataset.view as ViewPreset | undefined;
      if (!view || !["isometric", "xy", "xz", "yz", "fit"].includes(view)) return;
      viewer?.setView(view);
      const activeView = view === "fit" ? "isometric" : view;
      document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((candidate) => {
        if (candidate.dataset.view === "fit") return;
        candidate.setAttribute("aria-pressed", String(candidate.dataset.view === activeView));
      });
    });
  });

  document.querySelectorAll<HTMLInputElement>("[data-layer]").forEach((input) => {
    input.addEventListener("change", () => {
      const layer = input.dataset.layer as ViewerLayer | undefined;
      if (!layer || !["buildings", "zones", "dynamic", "raw"].includes(layer)) return;
      viewer?.setLayerVisibility(layer, input.checked);
    });
  });

  document.querySelectorAll<HTMLButtonElement>("[data-speed]").forEach((button) => {
    button.addEventListener("click", () => {
      const speed = Number(button.dataset.speed);
      if (![0.5, 1, 2].includes(speed)) return;
      playbackSpeed = speed;
      document.querySelectorAll<HTMLButtonElement>("[data-speed]").forEach((candidate) => {
        candidate.setAttribute("aria-pressed", String(candidate === button));
      });
      announce(`Playback speed ${speed} times`);
    });
  });

  document.addEventListener("keydown", (event) => {
    const target = event.target;
    if (
      target instanceof HTMLInputElement ||
      target instanceof HTMLSelectElement ||
      target instanceof HTMLButtonElement ||
      target instanceof HTMLTextAreaElement
    ) {
      return;
    }
    if (event.key === " ") {
      event.preventDefault();
      if (animationFrame === null) beginPlayback();
      else setPlaying(false);
    } else if (event.key === "ArrowLeft" || event.key === "ArrowRight") {
      event.preventDefault();
      setPlaying(false);
      renderTime(currentTimeS + (event.key === "ArrowLeft" ? -0.25 : 0.25), true);
    } else if (event.key === "Home" || event.key === "End") {
      event.preventDefault();
      setPlaying(false);
      renderTime(event.key === "Home" ? 0 : duration(), true);
    }
  });

  const commitUrl = `https://github.com/Revincxt/uav-3d-planner/commit/${bundle.sourceCommit}`;
  const provenance = element("#provenance");
  const sourceLink = document.createElement("a");
  sourceLink.href = commitUrl;
  sourceLink.textContent = bundle.sourceCommit.slice(0, 12);
  const generated = document.createElement("p");
  generated.append(
    document.createTextNode(
      `Schema v${bundle.schemaVersion}; generated ${bundle.generatedAt}; ${bundle.verificationStatus}; source commit `,
    ),
    sourceLink,
    document.createTextNode("."),
  );
  const method = document.createElement("p");
  method.textContent =
    "Raw planner output, the collision-certified geometry candidate, and the optional discrete-envelope-qualified execution candidate are separate evidence layers. The browser draws only exported polyline samples. Frames store event anchors; vehicle and moving-hazard positions are interpolated from declared timed paths and keyframes.";
  const protocol = document.createElement("p");
  protocol.textContent =
    `Protocol ${bundle.protocol.id}: ${bundle.protocol.timeResolutionS} s search-time resolution, ` +
    `${bundle.protocol.resolutionM} m spatial grid, ${bundle.protocol.cruiseSpeedMps} m/s cruise speed, ` +
    `${bundle.protocol.planningHorizonS} s planning horizon, and trajectory post-processor ` +
    `${bundle.protocol.trajectoryPostprocessor}. The execution envelope declares ` +
    `${bundle.protocol.executionEnvelope.maxAbsClimbRateMps} m/s climb and ` +
    `${bundle.protocol.executionEnvelope.maxDiscreteAccelerationProxyMps2} m/s² finite-difference ` +
    "acceleration-proxy limits; it is not a continuous-dynamics certificate. Work budgets retain planner-specific units.";
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
    document.createTextNode(
      ` [${bundle.downloads.recordsCsv.sha256.slice(7, 19)}…], and `,
    ),
    manifestLink,
    document.createTextNode(
      ` [${bundle.downloads.scenarioManifest.sha256.slice(7, 19)}…].`,
    ),
  );
  provenance.replaceChildren(generated, method, protocol, downloads);

  scenarioSelect.value = currentScenario.id;
  updateScenario(currentScenario.id, false);
  if (preferredPlanner && currentScenario.runs.some((run) => run.plannerId === preferredPlanner.id)) {
    updateRun(preferredPlanner.id, false);
  }
  currentTimeS = Math.min(requestedInitialTimeS, duration());
  renderTime(currentTimeS);
  updateUrl(true);
  element("#load-state").remove();

  altitudeResizeObserver = new ResizeObserver(() => {
    renderEventAxis(currentRun, pathMode, duration(), currentTimeS);
    renderCharts();
    renderTime(currentTimeS);
  });
  altitudeResizeObserver.observe(element("#event-axis"));
  altitudeResizeObserver.observe(element("#altitude-chart"));
  altitudeResizeObserver.observe(element("#turn-chart"));

  document.addEventListener("visibilitychange", () => {
    if (document.hidden) setPlaying(false);
  });
  reducedMotion.addEventListener("change", () => {
    if (reducedMotion.matches) setPlaying(false);
  });
  window.addEventListener(
    "pagehide",
    () => {
      setPlaying(false);
      altitudeResizeObserver?.disconnect();
      viewer?.dispose();
    },
    { once: true },
  );
}

start().catch((error: unknown) => {
  const state = element("#load-state");
  state.classList.add("is-error");
  state.textContent = error instanceof Error ? error.message : "Predictive traces could not load.";
  console.error(error);
});
