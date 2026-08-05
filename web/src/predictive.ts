import "./predictive.css";

import {
  buildPredictiveComparisonRows,
  loadPredictiveBundle,
} from "./predictive-data";
import type {
  PredictiveBundleV1,
  PredictiveFrame,
  PredictiveRun,
  PredictiveScenario,
  TimedWaypoint,
  Vec3,
} from "./predictive-schema";
import { PredictiveViewer } from "./predictive-viewer";

const SVG_NS = "http://www.w3.org/2000/svg";

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

function sameNumber(left: number, right: number): boolean {
  return Math.abs(left - right) <= 1e-6 * Math.max(1, Math.abs(left), Math.abs(right));
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

function formatNumber(value: number | null, digits = 1): string {
  return value === null ? "—" : value.toFixed(digits);
}

function formatWorkUnit(unit: PredictiveRun["metrics"]["workUnit"]): string {
  if (unit === "expanded-nodes") return "nodes";
  if (unit === "queue-pops") return "queue pops";
  return "space–time states";
}

function renderEventAxis(run: PredictiveRun, frameIndex: number): void {
  const host = element<HTMLDivElement>("#event-axis");
  const width = Math.max(280, Math.round(host.clientWidth || 760));
  const height = 58;
  const margin = 12;
  const axisY = 20;
  const duration = Math.max(1, run.frames.at(-1)!.timeS);
  const x = (timeS: number): number => margin + (timeS / duration) * (width - margin * 2);
  const events = run.frames
    .map((frame, index) => ({ frame, index }))
    .filter(({ frame }) => frame.event !== null && frame.event.kind !== "none");
  const currentEvent = [...events].reverse().find((event) => event.index <= frameIndex) ?? null;

  const svg = svgElement("svg", {
    viewBox: `0 0 ${width} ${height}`,
    role: "img",
    "aria-label": `${events.length} recorded events from 0 to ${duration.toFixed(1)} seconds`,
  });
  const title = svgElement("title");
  title.textContent = "Event positions on the recorded trace";
  const desc = svgElement("desc");
  desc.textContent =
    "Vertical marks locate planner, restriction, waiting, prediction, and arrival events. The heavier mark is the most recent event at the selected frame.";
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

  for (const event of events) {
    const tick = svgElement("line", {
      x1: String(x(event.frame.timeS)),
      x2: String(x(event.frame.timeS)),
      y1: "8",
      y2: "30",
      class: `event-tick${event === currentEvent ? " is-current" : ""}`,
    });
    const tickTitle = svgElement("title");
    tickTitle.textContent = `${event.frame.timeS.toFixed(1)} s — ${event.frame.event!.label}`;
    tick.append(tickTitle);
    svg.append(tick);
  }

  const startLabel = svgElement("text", { x: String(margin), y: "50" });
  startLabel.textContent = "0 s";
  const endLabel = svgElement("text", {
    x: String(width - margin),
    y: "50",
    "text-anchor": "end",
  });
  endLabel.textContent = `${duration.toFixed(1)} s`;
  svg.append(startLabel, endLabel);

  if (currentEvent) {
    const eventX = x(currentEvent.frame.timeS);
    const anchor = eventX < width * 0.3 ? "start" : eventX > width * 0.7 ? "end" : "middle";
    const label = svgElement("text", {
      x: String(eventX),
      y: "50",
      "text-anchor": anchor,
      class: "current-event-label",
    });
    label.textContent = `${currentEvent.frame.timeS.toFixed(1)} s · ${currentEvent.frame.event!.label}`;
    if (eventX < margin + 48 || eventX > width - margin - 48) {
      if (eventX < width / 2) startLabel.remove();
      else endLabel.remove();
    }
    svg.append(label);
  }
  host.replaceChildren(svg);
}

function renderAltitudeChart(
  scenario: PredictiveScenario,
  run: PredictiveRun,
  frame: PredictiveFrame,
): void {
  const host = element<HTMLDivElement>("#altitude-chart");
  const width = Math.max(300, Math.round(host.clientWidth || 900));
  const height = 230;
  const margin = { top: 18, right: 18, bottom: 40, left: 56 };
  const plotWidth = width - margin.left - margin.right;
  const plotHeight = height - margin.top - margin.bottom;
  const duration = Math.max(1, run.timedPath.at(-1)!.timeS);
  const zMin = scenario.bounds.min[2];
  const zMax = scenario.bounds.max[2];
  const x = (timeS: number): number => margin.left + (timeS / duration) * plotWidth;
  const y = (altitudeM: number): number =>
    margin.top + ((zMax - altitudeM) / (zMax - zMin)) * plotHeight;

  const svg = svgElement("svg", {
    viewBox: `0 0 ${width} ${height}`,
    role: "img",
    "aria-label": `Altitude over time for ${run.plannerId}`,
  });
  const title = svgElement("title");
  title.textContent = "Recorded time–height trajectory";
  const desc = svgElement("desc");
  desc.textContent =
    `Altitude ranges from ${zMin.toFixed(1)} to ${zMax.toFixed(1)} metres over ` +
    `${duration.toFixed(1)} seconds. ${run.waitIntervals.length} stationary intervals are shaded.`;
  svg.append(title, desc);

  for (const interval of run.waitIntervals) {
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

  const tickCount = width < 520 ? 3 : 5;
  for (let index = 0; index <= tickCount; index += 1) {
    const fraction = index / tickCount;
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
      y: String(height - 20),
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
      x: String(margin.left - 9),
      y: String(tickY + 4),
      "text-anchor": "end",
    });
    label.textContent = (zMax - fraction * (zMax - zMin)).toFixed(0);
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

  const points = run.timedPath
    .map((waypoint) => `${x(waypoint.timeS)},${y(waypoint.position[2])}`)
    .join(" ");
  svg.append(svgElement("polyline", { points, class: "height-line" }));
  for (const waypoint of run.timedPath) {
    const marker = svgElement("circle", {
      cx: String(x(waypoint.timeS)),
      cy: String(y(waypoint.position[2])),
      r: "2.6",
      class: "waypoint",
    });
    const markerTitle = svgElement("title");
    markerTitle.textContent = `${waypoint.timeS.toFixed(1)} s, ${waypoint.position[2].toFixed(1)} m`;
    marker.append(markerTitle);
    svg.append(marker);
  }

  const currentTime = Math.min(frame.timeS, duration);
  const currentPosition = timedPosition(run.timedPath, currentTime);
  const currentX = x(currentTime);
  const currentY = y(currentPosition[2]);
  svg.append(
    svgElement("line", {
      x1: String(currentX),
      x2: String(currentX),
      y1: String(margin.top),
      y2: String(margin.top + plotHeight),
      class: "current-rule",
    }),
    svgElement("circle", {
      cx: String(currentX),
      cy: String(currentY),
      r: "4",
      class: "current-point",
    }),
  );
  const currentAnchor = currentX > width * 0.72 ? "end" : "start";
  const currentLabel = svgElement("text", {
    x: String(currentX + (currentAnchor === "start" ? 7 : -7)),
    y: String(Math.max(margin.top + 12, currentY - 8)),
    "text-anchor": currentAnchor,
    class: "current-label",
  });
  currentLabel.textContent = `${currentTime.toFixed(1)} s · ${currentPosition[2].toFixed(1)} m`;
  svg.append(currentLabel);

  const xLabel = svgElement("text", {
    x: String(margin.left + plotWidth / 2),
    y: String(height - 4),
    "text-anchor": "middle",
  });
  xLabel.textContent = "Time (s)";
  const yLabel = svgElement("text", {
    x: "14",
    y: String(margin.top + plotHeight / 2),
    transform: `rotate(-90 14 ${margin.top + plotHeight / 2})`,
    "text-anchor": "middle",
  });
  yLabel.textContent = "Altitude (m)";
  svg.append(xLabel, yLabel);
  host.replaceChildren(svg);
}

function renderComparison(
  bundle: PredictiveBundleV1,
  scenario: PredictiveScenario,
  selectedPlannerId: string,
): void {
  const body = element<HTMLTableSectionElement>("#comparison-body");
  body.replaceChildren();
  for (const row of buildPredictiveComparisonRows(bundle, scenario)) {
    const tableRow = document.createElement("tr");
    tableRow.setAttribute("aria-current", String(row.plannerId === selectedPlannerId));
    const planner = document.createElement("th");
    planner.scope = "row";
    planner.textContent = row.plannerLabel;
    const values = [
      row.predictive ? "Yes" : "No",
      row.status,
      formatNumber(row.arrivalTimeS),
      formatNumber(row.waitTimeS),
      formatNumber(row.executedPathLengthM),
      formatNumber(row.minimumSeparationM),
      row.safetyViolations.toLocaleString(),
      `${row.expandedStates.toLocaleString()} ${formatWorkUnit(row.workUnit)}`,
    ];
    tableRow.append(planner);
    values.forEach((value, index) => {
      const cell = document.createElement("td");
      cell.textContent = value;
      if (index === 1) {
        cell.className = row.status === "success" ? "status-success" : "status-failed";
      }
      tableRow.append(cell);
    });
    body.append(tableRow);
  }
}

function renderFrameSummary(run: PredictiveRun, frame: PredictiveFrame, index: number): void {
  const plannerKind = run.predictive ? "predictive" : "reactive";
  const event = frame.event && frame.event.kind !== "none" ? frame.event.label : "no event";
  const wait = run.waitIntervals.find(
    (interval) =>
      interval.startTimeS <= frame.timeS &&
      frame.timeS < interval.endTimeS,
  );
  element("#frame-summary").textContent =
    `Frame ${index + 1}/${run.frames.length} · ${frame.timeS.toFixed(1)} s · ` +
    `${frame.vehicle[2].toFixed(1)} m altitude · ${plannerKind} · ${event}` +
    (wait ? ` · stationary: ${wait.reason}` : "");
}

async function start(): Promise<void> {
  const bundle = await loadPredictiveBundle();
  const scenarioSelect = element<HTMLSelectElement>("#scenario-select");
  const plannerSelect = element<HTMLSelectElement>("#planner-select");
  const timeline = element<HTMLInputElement>("#timeline");
  const timelineValue = element<HTMLOutputElement>("#timeline-value");
  const previous = element<HTMLButtonElement>("#previous-frame");
  const next = element<HTMLButtonElement>("#next-frame");
  const playPause = element<HTMLButtonElement>("#play-pause");
  const viewerHost = element<HTMLDivElement>("#predictive-viewer");
  let viewer: PredictiveViewer | null = null;
  try {
    viewer = new PredictiveViewer(viewerHost);
  } catch (error) {
    viewerHost.classList.add("viewer-error");
    viewerHost.textContent = "WebGL is unavailable; the recorded charts and tables remain accessible.";
    console.warn(error);
  }

  let currentScenario = bundle.scenarios[0]!;
  let currentRun = currentScenario.runs[0]!;
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
    renderEventAxis(currentRun, frameIndex);
    renderAltitudeChart(currentScenario, currentRun, frame);
    renderFrameSummary(currentRun, frame, frameIndex);
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
    viewer?.setRun(run);
    renderComparison(bundle, currentScenario, plannerId);
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
      option.textContent = `${planner.label}${planner.predictive ? " · predictive" : " · reactive"}`;
      option.selected = planner.id === previousPlanner;
      plannerSelect.append(option);
    }
    if (!plannerSelect.value) plannerSelect.value = bundle.planners[0]!.id;
    updateRun(plannerSelect.value, false);
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
    document.createTextNode(
      `Generated ${bundle.generatedAt}; ${bundle.verificationStatus}; source commit `,
    ),
    sourceLink,
    document.createTextNode("."),
  );
  const method = document.createElement("p");
  method.textContent =
    "Traces are deterministic simulator records. Geometry, restriction intervals, and moving-obstacle keyframes are shared across planners within each scenario. Total stationary time includes both time-lattice endpoint alignment and forecast-aware waiting actions, distinguished by each interval reason; it is not solely a policy-wait measure. Arrival, separation, and safety values are descriptive outcomes, not statistical evidence. Planning-work units retain algorithm-specific semantics and must not be compared as a common operation count.";
  const protocol = document.createElement("p");
  protocol.textContent =
    `Protocol ${bundle.protocol.id}: ${bundle.protocol.timeStepS} s replay step, ` +
    `${bundle.protocol.timeResolutionS} s space–time discretization, ` +
    `${bundle.protocol.cruiseSpeedMps} m/s cruise speed, ${bundle.protocol.resolutionM} m grid, ` +
    `${bundle.protocol.planningHorizonS} s planning horizon, ` +
    `${bundle.protocol.predictionHorizonS} s prediction horizon and ${bundle.protocol.maxTimeS} s maximum time. ` +
    `Reactive conditions allow ${bundle.protocol.reactiveMaxWorkPerReplan.toLocaleString()} ` +
    `algorithm-specific work events per replan; Space–Time A* allows ` +
    `${bundle.protocol.predictiveMaxExpandedStatesPerMission.toLocaleString()} expanded states per mission.`;
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

  scenarioSelect.value = bundle.scenarios[0]!.id;
  currentScenario = bundle.scenarios[0]!;
  currentRun = currentScenario.runs[0]!;
  updateScenario();
  element("#load-state").remove();

  const chartResizeObserver = new ResizeObserver(() => {
    const frame = currentRun.frames[frameIndex];
    if (!frame) return;
    renderEventAxis(currentRun, frameIndex);
    renderAltitudeChart(currentScenario, currentRun, frame);
  });
  chartResizeObserver.observe(element("#event-axis"));
  chartResizeObserver.observe(element("#altitude-chart"));

  window.addEventListener(
    "pagehide",
    () => {
      setPlaying(false);
      chartResizeObserver.disconnect();
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
