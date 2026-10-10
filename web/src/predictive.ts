import "./predictive.css";
import "./workspace.css";
import "./simulator-ui.css";
import { mountBenchmark } from "./benchmark-panel";
import { mountPageLifecycle } from "./page-lifecycle";
import { RouteInteraction } from "./route-interaction";
import { FlightHud } from "./flight-hud";
import { predictiveAvoidanceEvents, FlightAnnouncements, type AvoidanceEvent } from "./flight-announcements";

import { missionCaption, mountWorkspace, mountInspector, mountTabs } from "./workspace";
import { mountFollowControls } from "./drone-follow";
import { TaskArrivalNotice } from "./task-arrival-notice";
import { PlaybackClock, mountPlaybackSpeedControls } from "./playback-clock";
import { overviewDuration, predictiveRoutes, timedPosition, waypointIndex } from "./route-overview";
import { finalFlight, type FinalFlight } from "./final-flight";
import { playbackAction, showPlaybackButton, showPlaybackState } from "./playback-state";
import { flightPhase, RouteStatusStrip } from "./trajectory-semantics";
import { encounterView, mountEncounterControl, showChallenge } from "./encounter-view";

import { loadPredictiveBundle } from "./predictive-data";
import type {
  PredictiveBundleV3,
  PredictiveFrame,
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

function formatNumber(value: number | null, digits = 1): string {
  return value === null ? "—" : value.toFixed(digits);
}

function formatPosition(point: Vec3): string {
  return `${point[0].toFixed(1)}, ${point[1].toFixed(1)}, ${point[2].toFixed(1)} m`;
}

function formatStatus(status: PredictiveRun["status"]): string {
  if (status === "no-path") return "No path";
  return status.charAt(0).toUpperCase() + status.slice(1);
}

function setDefinitionRows(host: HTMLElement, rows: Array<[string, string]>): void {
  if (host.children.length === rows.length) {
    rows.forEach(([term, description], index) => {
      const row = host.children[index]!, dt = row.firstElementChild!, dd = row.lastElementChild!;
      if (dt.textContent !== term) dt.textContent = term;
      if (dd.textContent !== description) dd.textContent = description;
    });
    return;
  }
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

const eventCache = new WeakMap<FinalFlight, Array<{ frame: PredictiveFrame; index: number }>>();
function visibleEvents(flight: FinalFlight): Array<{ frame: PredictiveFrame; index: number }> {
  const cached = eventCache.get(flight);
  if (cached) return cached;
  const events = flight.frames
    .map((frame, index) => ({ frame, index }))
    .filter(({ frame }) => frame.event !== null && frame.event.kind !== "none");
  eventCache.set(flight, events); return events;
}

function mostRecentEvent(
  flight: FinalFlight,
  timeS: number,
): PredictiveFrame | null {
  return (
    [...visibleEvents(flight)]
      .reverse()
      .find(({ frame }) => frame.timeS <= timeS + EPSILON)?.frame ?? null
  );
}

const turnSampleCache = new WeakMap<TimedWaypoint[], TurnSample[]>();
function turnSamples(waypoints: TimedWaypoint[]): TurnSample[] {
  const cached = turnSampleCache.get(waypoints);
  if (cached) return cached;
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
  turnSampleCache.set(waypoints, samples);
  return samples;
}

function sampledValue(samples: TurnSample[], timeS: number): number {
  if (samples.length === 0) return 0;
  const index = waypointIndex(samples, timeS), left = samples[index]!, right = samples[index + 1];
  const nearest = right && Math.abs(right.timeS - timeS) < Math.abs(left.timeS - timeS) ? right : left;
  return nearest.angleDeg;
}

function renderEventAxis(
  flight: FinalFlight,
  duration: number,
  timeS: number,
): void {
  const host = element<HTMLDivElement>("#event-axis");
  const width = Math.max(300, Math.round(host.clientWidth || 760));
  const height = 42;
  const margin = 10;
  const axisY = 14;
  const x = (value: number): number => margin + (value / Math.max(duration, 1)) * (width - margin * 2);
  const events = visibleEvents(flight).filter(({ frame }) => frame.timeS <= duration + EPSILON);
  const witness = flight.metrics.minimumSeparationWitness;
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
  duration: number,
  yMin: number,
  yMax: number,
  yLabel: string,
  waits: FinalFlight["waits"],
): void {
  // Match the displayed chart dimensions so 12px labels don't shrink with its SVG viewBox.
  const width = Math.max(240, Math.round(host.clientWidth || 620));
  const height = Math.max(80, Math.round(host.querySelector("svg")?.getBoundingClientRect().height || host.clientHeight || 142));
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
  const yTicks = height < 120 ? 2 : 4;
  for (let index = 0; index <= yTicks; index += 1) {
    const fraction = index / yTicks;
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
  const host = element("#planner-comparison");
  host.replaceChildren();
  for (const planner of bundle.planners) {
    const run = scenario.runs.find((candidate) => candidate.plannerId === planner.id);
    if (!run) throw new Error(`Missing ${planner.id} flight in ${scenario.id}`);
    const flight = finalFlight(run);
    const button = document.createElement("button");
    button.type = "button";
    button.className = "comparison-card";
    button.dataset.planner = planner.id;
    button.setAttribute("aria-pressed", String(planner.id === selectedPlannerId));
    const heading = document.createElement("span");
    heading.className = "comparison-title";
    const title = document.createElement("strong");
    title.textContent = planner.label;
    const badge = document.createElement("span");
    badge.textContent = planner.predictive ? "Forecast" : "Reactive";
    heading.append(title, badge);
    const metrics = document.createElement("span");
    metrics.className = "comparison-metrics";
    for (const [label, value] of [
      ["Arrival", `${formatNumber(flight.metrics.arrivalTimeS)} s`],
      ["Path", `${formatNumber(flight.metrics.executedPathLengthM)} m`],
      ["Waiting", `${formatNumber(flight.metrics.waitTimeS)} s`],
    ]) {
      const metric = document.createElement("span");
      const name = document.createElement("span");
      name.textContent = label!;
      const number = document.createElement("strong");
      number.textContent = value!;
      metric.append(name, number);
      metrics.append(metric);
    }
    const note = document.createElement("span");
    note.className = "comparison-note";
    note.textContent = formatStatus(run.status);
    button.append(heading, metrics, note);
    button.addEventListener("click", () => onSelect(planner.id));
    host.append(button);
  }
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
  const jumpToWitness = element<HTMLButtonElement>("#jump-to-witness");
  const viewerHost = element<HTMLDivElement>("#predictive-viewer");
  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
  let viewer: PredictiveViewer | null = null;
  try {
    viewer = new PredictiveViewer(viewerHost);
  } catch (error) {
    viewerHost.classList.add("viewer-error");
    viewerHost.textContent = "WebGL is unavailable; recorded charts and tables remain accessible.";
    document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((button) => { button.disabled = true; });
    console.warn(error);
  }

  const query = new URLSearchParams(window.location.search);
  const preferredScenario = bundle.scenarios.find((scenario) => scenario.id === query.get("scenario"));
  let currentScenario = preferredScenario ?? bundle.scenarios[0]!;
  const preferredPlanner = bundle.planners.find((planner) => planner.id === query.get("planner"));
  let currentRun =
    currentScenario.runs.find((run) => run.plannerId === preferredPlanner?.id) ??
    currentScenario.runs.find((run) => bundle.planners.find((planner) => planner.id === run.plannerId)?.predictive) ??
    currentScenario.runs[0]!;
  let currentFlight = finalFlight(currentRun);
  const requestedInitialTimeS = Math.max(0, Number(query.get("time")) || 0);
  mountInspector();
  mountTabs();
  let currentTimeS = requestedInitialTimeS;
  const playbackClock = new PlaybackClock();
  let animationFrame: number | null = null;
  let lastUrlWrite = 0;
  let altitudeResizeObserver: ResizeObserver | null = null;
  let cameraView: Exclude<ViewPreset, "fit"> = "isometric";

  for (const scenario of bundle.scenarios) {
    const option = document.createElement("option");
    option.value = scenario.id;
    option.textContent = scenario.label;
    scenarioSelect.append(option);
  }

  const announce = (message: string): void => {
    element("#live-region").textContent = message;
  };
  const updateSceneCaption = (): void => {
    element("#case-geometry").textContent = missionCaption(currentScenario, "city");
  };
  const setCameraView = (view: ViewPreset): void => {
    if (view !== "fit") cameraView = view;
    viewer?.setView(view);
    document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((button) => {
      if (button.dataset.view === "fit") return;
      button.setAttribute("aria-pressed", String(button.dataset.view === cameraView));
    });
  };

  const updateUrl = (force = false): void => {
    const now = performance.now();
    if (!force && now - lastUrlWrite < 250) return;
    lastUrlWrite = now;
    const parameters = new URLSearchParams();
    parameters.set("scenario", currentScenario.id);
    parameters.set("planner", currentRun.plannerId);
    const evidence = minimumSeparationEvidence(currentScenario, currentRun, "execution");
    parameters.set(
      "time",
      isMinimumSeparationEvidenceTime(evidence, currentTimeS)
        ? String(evidence!.witness.timeS)
        : currentTimeS.toFixed(2),
    );
    window.history.replaceState(null, "", `${window.location.pathname}?${parameters}${window.location.hash}`);
  };

  const currentPath = (): TimedWaypoint[] => currentFlight.path;
  let routes = predictiveRoutes(bundle.scenarios, currentRun.plannerId, "execution");
  const duration = (): number => overviewDuration(routes);

  const renderCharts = (): void => {
    const path = currentPath();
    const total = duration();
    const altitudeHost = element<HTMLDivElement>("#altitude-chart");
    const turnHost = element<HTMLDivElement>("#turn-chart");
    const altitudeData = path.map((waypoint) => ({ timeS: waypoint.timeS, value: waypoint.position[2] }));
    renderLineChart(
      altitudeHost,
      altitudeData,
      total,
      currentScenario.bounds.min[2],
      currentScenario.bounds.max[2],
      "Altitude (m)",
      currentFlight.waits,
    );
    const activeTurns = turnSamples(path);
    const maximumTurn = Math.max(
      30,
      ...activeTurns.map((sample) => sample.angleDeg),
    );
    const turnCeiling = Math.min(180, Math.ceil(maximumTurn / 15) * 15);
    renderLineChart(
      turnHost,
      activeTurns.map((sample) => ({ timeS: sample.timeS, value: sample.angleDeg })),
      total,
      0,
      turnCeiling,
      "Turn angle (°)",
      [],
    );
  };

  const taskNotice = new TaskArrivalNotice(viewerHost);
  const flightHud = new FlightHud(element(".stage-viewport"), element(".playback-bar"));
  const routeInteraction = new RouteInteraction(viewerHost, viewer, scenarioSelect, () => routes);
  const routeStates = new RouteStatusStrip();
  const flightAnnouncements = new FlightAnnouncements(element(".stage-viewport"));
  const avoidanceCache = new WeakMap<PredictiveRun, AvoidanceEvent[]>();
  let avoidanceEvents = predictiveAvoidanceEvents(currentRun);
  avoidanceCache.set(currentRun, avoidanceEvents);
  let lastReadoutMs = -Infinity;
  const renderTime = (timeS: number, shouldAnnounce = false, notify = false): void => {
    const previousTimeS = currentTimeS;
    const total = duration();
    currentTimeS = Math.max(0, Math.min(timeS, total));
    // Motion/arrivals remain frame-rate driven; text and hidden inspector readouts
    // need only 10 Hz. Explicit seeking always refreshes immediately.
    timeline.value = String(currentTimeS);
    viewer?.setTime(currentTimeS);
    if (notify) taskNotice.advance(routes.find(route => route.id === viewer?.followedRouteId), previousTimeS, currentTimeS);
    else taskNotice.reset();
    const focusedRoute = routes.find(route => route.id === currentScenario.id)!;
    if (notify) flightAnnouncements.advance(focusedRoute.id, routes.indexOf(focusedRoute), avoidanceEvents, previousTimeS, currentTimeS);
    else flightAnnouncements.reset();
    flightHud.update(focusedRoute, currentTimeS, routes.indexOf(focusedRoute), !notify);
    routeInteraction.update(currentTimeS, !notify);
    routeStates.update(routes, currentTimeS, !notify);
    const nowMs = performance.now();
    if (notify && nowMs - lastReadoutMs < 100 && currentTimeS < total) return;
    lastReadoutMs = nowMs;
    const path = currentPath();
    const position = timedPosition(path, currentTimeS);
    const events = visibleEvents(currentFlight).filter(
      ({ frame }) => frame.timeS <= total + EPSILON,
    );
    const event = mostRecentEvent(currentFlight, currentTimeS);
    const focusedTime = Math.min(currentTimeS, path.at(-1)!.timeS);
    const activeZones = currentScenario.temporaryNoFlyZones.filter(
      (zone) => zone.activeFromS <= focusedTime && focusedTime < zone.activeUntilS,
    );
    const evidence = minimumSeparationEvidence(currentScenario, currentRun, "execution");
    const witnessIsVisible = isMinimumSeparationEvidenceTime(evidence, currentTimeS);
    const witnessReadout = element<HTMLDivElement>("#viewer-witness-readout");

    timelineValue.value = `${currentTimeS.toFixed(2)} s`;
    element("#viewer-time-value").textContent = `${currentTimeS.toFixed(2)} s`;
    showPlaybackState(element("#playback-state"), currentRun.predictive ? "predictive" : "reactive",
      playbackAction(path, currentTimeS, currentScenario.mission), !currentRun.predictive, flightPhase(focusedRoute, currentTimeS));
    showChallenge(document.querySelector("#scene-challenge"), currentScenario.mission);
    witnessReadout.hidden = !witnessIsVisible;
    if (witnessIsVisible && evidence !== null) {
      const { witness } = evidence;
      witnessReadout.dataset.quality = witness.exact ? "exact" : "approximate";
      element("#viewer-witness-value").textContent =
        `${witness.separationM.toFixed(2)} m surface separation`;
      element("#viewer-witness-detail").textContent =
        `${witness.exact ? "Exact" : "Approximate"} witness · ${witness.timeS.toFixed(2)} s`;
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
      ["Position", formatPosition(position)],
      ["Altitude", `${position[2].toFixed(1)} m`],
      ["Active hazards", `${activeZones.length} zones · ${currentScenario.movingSpheres.length} moving`],
    ];
    setDefinitionRows(element("#current-meta"), currentRows);
    element("#current-event").textContent = event?.event
      ? `${event.event.label} · ${event.timeS.toFixed(1)} s`
      : "Ready to depart";
    element("#frame-summary").textContent =
      `${currentTimeS.toFixed(1)} s of ${total.toFixed(1)} s · ` +
      `${position[2].toFixed(1)} m altitude`;
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
    viewer?.setPlaying(playing);
    if (!playing) flightAnnouncements.suspend();
    if (!playing && animationFrame !== null) {
      cancelAnimationFrame(animationFrame);
      animationFrame = null;
    }
    showPlaybackButton(playPause, playing);
  };
  mountEncounterControl(() => encounterView(routes.find(route => route.id === currentScenario.id)!, currentScenario.movingSpheres, currentScenario.temporaryNoFlyZones), view => {
    setPlaying(false); renderTime(view.timeS); viewer?.observeEncounter(view.position);
  });

  const playbackTick = (now: number): void => {
    renderTime(playbackClock.sample(now), false, true);
    if (currentTimeS >= duration() - EPSILON) {
      setPlaying(false);
      announce("Playback complete");
      return;
    }
    animationFrame = requestAnimationFrame(playbackTick);
  };

  const beginPlayback = (): void => {
    if (currentTimeS >= duration() - EPSILON) renderTime(0);
    playbackClock.start(currentTimeS);
    setPlaying(true);
    animationFrame = requestAnimationFrame(playbackTick);
  };

  const renderFlightDetails = (): void => {
    const metrics = currentFlight.metrics;
    const evidence = minimumSeparationEvidence(currentScenario, currentRun, "execution");
    const status = element("#run-status");
    status.textContent = formatStatus(currentRun.status);
    status.className = `run-status ${currentRun.status === "success" ? "is-success" : "is-failed"}`;
    const outcome = element("#metric-outcome");
    outcome.textContent = formatStatus(currentRun.status);
    outcome.className = metrics.success ? "metric-positive" : "metric-negative";
    element("#metric-arrival").textContent =
      metrics.arrivalTimeS === null ? "—" : `${metrics.arrivalTimeS.toFixed(1)} s`;
    element("#metric-wait").textContent = `${metrics.waitTimeS.toFixed(1)} s`;
    element("#metric-path").textContent = `${metrics.executedPathLengthM.toFixed(1)} m`;
    const safety = element("#metric-safety");
    safety.textContent = metrics.safetyViolations === 0
      ? "0 recorded violations"
      : `${metrics.safetyViolations} recorded violations`;
    safety.className = `safety-result ${metrics.safetyViolations === 0 ? "metric-positive" : "metric-negative"}`;
    const quality = element("#witness-quality");
    const witnessSummary = element("#witness-summary");
    if (evidence === null) {
      quality.textContent = "Unavailable";
      witnessSummary.textContent = "No dynamic separation witness recorded.";
      witnessSummary.removeAttribute("title");
      jumpToWitness.disabled = true;
      jumpToWitness.textContent = "Closest approach unavailable";
      jumpToWitness.removeAttribute("data-witness-time");
    } else {
      const { witness } = evidence;
      quality.textContent = witness.exact ? "Exact" : "Approximate";
      const obstacleLabel = witness.obstacleKind === "moving-sphere" ? "traffic" : "temporary zone";
      witnessSummary.textContent =
        `${witness.separationM.toFixed(2)} m to ${obstacleLabel} at ${witness.timeS.toFixed(2)} s`;
      witnessSummary.title = witness.obstacleId;
      jumpToWitness.disabled = false;
      jumpToWitness.textContent = "View closest approach ↗";
      jumpToWitness.dataset.witnessTime = String(witness.timeS);
      jumpToWitness.setAttribute("aria-label",
        `View ${witness.exact ? "exact" : "approximate"} closest approach at ${witness.timeS.toFixed(2)} seconds`);
    }
  };

  const updateRun = (plannerId: string, shouldAnnounce = true): void => {
    const resume = animationFrame !== null;
    setPlaying(false);
    const run = currentScenario.runs.find((candidate) => candidate.plannerId === plannerId);
    if (!run) throw new Error(`Missing ${plannerId} run in ${currentScenario.id}`);
    const flight = finalFlight(run);
    currentRun = run;
    currentFlight = flight;
    avoidanceEvents = avoidanceCache.get(run) ?? predictiveAvoidanceEvents(run);
    avoidanceCache.set(run, avoidanceEvents);
    plannerSelect.value = plannerId;
    routes = predictiveRoutes(bundle.scenarios, plannerId, "execution");
    currentTimeS = Math.min(currentTimeS, duration());
    timeline.max = String(duration());
    viewer?.setRun(run);
    viewer?.setRoutes(routes, currentScenario.id);
    renderFlightDetails();
    renderComparison(bundle, currentScenario, plannerId, (selectedPlannerId) => {
      updateRun(selectedPlannerId);
    });
    renderEventAxis(currentFlight, duration(), currentTimeS);
    renderCharts();
    renderTime(currentTimeS);
    updateUrl(true);
    const planner = bundle.planners.find((candidate) => candidate.id === plannerId);
    if (shouldAnnounce) announce(`${planner?.label ?? plannerId} trace loaded`);
    if (resume) beginPlayback();
  };

  const updateScenario = (scenarioId: string, shouldAnnounce = true): void => {
    const resume = animationFrame !== null;
    setPlaying(false);
    const scenario = bundle.scenarios.find((candidate) => candidate.id === scenarioId);
    if (!scenario) throw new Error(`Unknown scenario: ${scenarioId}`);
    currentScenario = scenario;
    scenarioSelect.value = scenario.id;
    viewer?.setScenario(scenario);
    viewer?.focusRoute(scenario.id);
    const previousPlanner = currentRun?.plannerId;
    plannerSelect.replaceChildren();
    for (const planner of bundle.planners) {
      const option = document.createElement("option");
      option.value = planner.id;
      option.textContent = planner.label;
      plannerSelect.append(option);
    }
    const nextRun =
      scenario.runs.find((run) => run.plannerId === previousPlanner) ??
      scenario.runs.find(
        (run) => bundle.planners.find((planner) => planner.id === run.plannerId)?.predictive,
      ) ??
      scenario.runs[0]!;
    element("#scene-title").textContent = "Manhattan";
    updateSceneCaption();
    setDefinitionRows(element("#scenario-meta"), [

      ["Buildings", scenario.environment.buildingCount.toLocaleString()],
      ["Hazards", scenario.environment.hazardCount.toLocaleString()],
      [
        "Bounds",
        `${(scenario.bounds.max[0] - scenario.bounds.min[0]).toFixed(0)} × ` +
          `${(scenario.bounds.max[1] - scenario.bounds.min[1]).toFixed(0)} × ` +
          `${(scenario.bounds.max[2] - scenario.bounds.min[2]).toFixed(0)} m`,
      ],
    ]);
    viewerHost.setAttribute(
      "aria-label",
      `${bundle.scenarios.length} tasks share one obstacle world and clock. Focus: ${scenario.label}: ${scenario.environment.buildingCount} buildings, ` +
        `${scenario.staticNoFlyZones.length} static restrictions, ` +
        `${scenario.temporaryNoFlyZones.length} temporary restrictions, and ` +
        `${scenario.movingSpheres.length} moving hazards.`,
    );
    updateRun(nextRun.plannerId, false);
    updateUrl(true);
    if (shouldAnnounce) announce(`${scenario.label} loaded`);
    if (resume) beginPlayback();
  };

  scenarioSelect.addEventListener("change", () => updateScenario(scenarioSelect.value));
  plannerSelect.addEventListener("change", () => updateRun(plannerSelect.value));
  jumpToWitness.addEventListener("click", () => {
    const witness = currentFlight.metrics.minimumSeparationWitness;
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
    const target = [...visibleEvents(currentFlight)]
      .reverse()
      .find(({ frame }) => frame.timeS < currentTimeS - EPSILON);
    if (target) renderTime(target.frame.timeS, true);
  });
  next.addEventListener("click", () => {
    setPlaying(false);
    const target = visibleEvents(currentFlight).find(
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

  document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((button) => {
    button.addEventListener("click", () => {
      const view = button.dataset.view as ViewPreset | undefined;
      if (!view || !["isometric", "xy"].includes(view)) return;
      setCameraView(view);
    });
  });

  document.querySelectorAll<HTMLInputElement>("[data-layer]").forEach((input) => {
    input.addEventListener("change", () => {
      const layer = input.dataset.layer as ViewerLayer | undefined;
      if (!layer || !["buildings", "zones", "dynamic"].includes(layer)) return;
      viewer?.setLayerVisibility(layer, input.checked);
    });
  });

  mountPlaybackSpeedControls(playbackClock, () => currentTimeS, rate => announce(`Playback speed ${rate} times`));

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

  scenarioSelect.value = currentScenario.id;
  updateScenario(currentScenario.id, false);
  if (preferredPlanner && currentScenario.runs.some((run) => run.plannerId === preferredPlanner.id)) {
    updateRun(preferredPlanner.id, false);
  }
  currentTimeS = Math.min(requestedInitialTimeS, duration());
  renderTime(currentTimeS);
  updateUrl(true);
  mountWorkspace({
    select: scenarioSelect,
    scenarios: bundle.scenarios.map((scenario) => ({
      ...scenario,
      group: "Manhattan",
      summary: scenario.mission ? `${scenario.mission.origin} → ${scenario.mission.destination}` : scenario.description,
    })),
  });
  element("#load-state").remove();
  routeStates.update(routes, currentTimeS, true);
  mountFollowControls(viewer, scenarioSelect, enabled => { taskNotice.reset(); flightAnnouncements.reset(); routeInteraction.followChanged(enabled); });

  altitudeResizeObserver = new ResizeObserver(() => {
    renderEventAxis(currentFlight, duration(), currentTimeS);
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
  mountBenchmark({ isPlaying: () => animationFrame !== null, pause: () => setPlaying(false), resume: beginPlayback });
  mountPageLifecycle({ pause: () => setPlaying(false), restore: () => renderTime(currentTimeS), dispose: () => {
      routeInteraction.dispose();
      altitudeResizeObserver?.disconnect();
      taskNotice.dispose();
      flightAnnouncements.dispose();
      flightHud.dispose();
      viewer?.dispose();
  } });
}

start().catch((error: unknown) => {
  const state = element("#load-state");
  state.classList.add("is-error");
  state.textContent = error instanceof Error ? error.message : "Predictive traces could not load.";
  console.error(error);
});
