import "./dynamic.css";
import "./workspace.css";
import "./simulator-ui.css";
import { mountBenchmark } from "./benchmark-panel";
import { mountPageLifecycle } from "./page-lifecycle";
import { RouteInteraction } from "./route-interaction";
import { FlightHud } from "./flight-hud";
import { dynamicAvoidanceEvents, FlightAnnouncements, type AvoidanceEvent } from "./flight-announcements";

import { buildDynamicComparisonRows, loadDynamicBundle } from "./dynamic-data";
import type {
  DynamicBundleV1,
  DynamicFrame,
  DynamicRun,
  DynamicScenario,
} from "./dynamic-schema";
import { DynamicViewer } from "./dynamic-viewer";
import { mountFollowControls } from "./drone-follow";
import { TaskArrivalNotice } from "./task-arrival-notice";
import { PlaybackClock, mountPlaybackSpeedControls } from "./playback-clock";
import { mountInspector, mountTabs, mountWorkspace } from "./workspace";
import { dynamicRoutes, overviewDuration, waypointIndex, type OverviewRoute } from "./route-overview";
import { playbackAction, showPlaybackButton, showPlaybackState } from "./playback-state";
import { encounterView, mountEncounterControl } from "./encounter-view";
import { flightPhase, RouteStatusStrip } from "./trajectory-semantics";

const element = <T extends HTMLElement>(selector: string): T => {
  const value = document.querySelector<T>(selector);
  if (!value) throw new Error(`Missing required element: ${selector}`);
  return value;
};

const shortPlanner = (label: string): string =>
  label.replace("Repeated ", "").replace("3D D* Lite", "D* Lite");

function metric(label: string, value: string, title?: string): HTMLElement {
  const card = document.createElement("div");
  card.className = "metric-card";
  const name = document.createElement("span");
  name.textContent = label;
  const number = document.createElement("strong");
  number.textContent = value;
  card.append(name, number);
  if (title) card.title = title;
  return card;
}

function renderOutcome(run: DynamicRun): void {
  const values = run.metrics;
  element("#outcome-metrics-body").replaceChildren(
    metric("Completion", values.completionTimeS === null ? "—" : `${values.completionTimeS.toFixed(1)} s`),
    metric("Path length", `${values.executedPathLengthM.toFixed(1)} m`),
    metric("Replans", values.replans.toLocaleString()),
    metric("Safety holds", values.holds.toLocaleString(), "Recorded steps spent holding position"),
  );
  const status = element("#run-status");
  status.textContent = run.status === "success" ? "✓ Goal reached" : run.status === "no-path" ? "No path" : run.status === "timeout" ? "Time limit reached" : "Invalid run";
  status.classList.toggle("is-failure", run.status !== "success");
  status.title = run.failureReason ?? "Recorded run outcome";
}

function renderComparison(bundle: DynamicBundleV1, scenario: DynamicScenario): void {
  const body = element("#comparison-body");
  const select = element<HTMLSelectElement>("#planner-select");
  body.replaceChildren();
  for (const result of buildDynamicComparisonRows(bundle, scenario)) {
    const planner = bundle.planners.find((candidate) => candidate.label === result.plannerLabel);
    if (!planner) continue;
    const card = document.createElement("button");
    card.type = "button";
    card.className = "planner-result";
    card.dataset.planner = planner.id;
    card.setAttribute("aria-label", `Replay ${result.plannerLabel}`);
    const name = document.createElement("strong");
    name.textContent = shortPlanner(result.plannerLabel);
    const status = document.createElement("span");
    status.className = "result-status";
    status.classList.toggle("is-failure", result.status !== "success");
    status.textContent = result.status === "success" ? "Reached goal" : result.status;
    const details = document.createElement("span");
    details.className = "result-details";
    for (const [label, value] of [
      ["Time", result.completionTimeS === null ? "—" : `${result.completionTimeS.toFixed(1)} s`],
      ["Path", `${result.executedPathLengthM.toFixed(0)} m`],
      ["Replans", result.replans.toLocaleString()],
    ]) {
      const item = document.createElement("span");
      const number = document.createElement("b");
      number.textContent = value!;
      item.append(document.createTextNode(`${label} `), number);
      details.append(item);
    }
    card.append(name, status, details);
    card.addEventListener("click", () => {
      select.value = planner.id;
      select.dispatchEvent(new Event("change", { bubbles: true }));
    });
    body.append(card);
  }
}

function highlightPlanner(plannerId: string): void {
  document.querySelectorAll<HTMLButtonElement>("[data-planner]").forEach((button) => {
    const active = button.dataset.planner === plannerId;
    button.classList.toggle("is-active", active);
    button.setAttribute("aria-pressed", String(active));
  });
}

function renderFrameMetrics(frame: DynamicFrame, index: number, total: number): void {
  element("#frame-metrics-body").replaceChildren(
    metric("Active zones", String(frame.activeTemporaryZoneIds.length)),
    metric("Moving obstacles", String(frame.movingSpheres.length)),
    metric("Planning work", frame.workUsed.toLocaleString(), "Algorithm-specific work for this frame; work units differ between planners"),
    metric("Changed edges", frame.changedEdges.toLocaleString()),
  );
  element("#frame-summary").textContent = `Frame ${index + 1} / ${total}`;
  element("#vehicle-position").textContent = frame.vehicle.map((coordinate) => coordinate.toFixed(1)).join("  /  ");
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
    viewerHost.textContent = "3D preview unavailable. Replay telemetry remains available.";
    document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((button) => { button.disabled = true; });
    console.error(error);
  }

  let currentScenario: DynamicScenario;
  let currentRun: DynamicRun;
  let frameIndex = 0;
  let currentTimeS = 0;
  let lastMetricsFrame: DynamicFrame | null = null;
  let routes: OverviewRoute[] = [];
  let animationFrame: number | null = null;
  const playbackClock = new PlaybackClock();

  for (const scenario of bundle.scenarios) {
    const option = document.createElement("option");
    option.value = scenario.id;
    option.textContent = scenario.label;
    scenarioSelect.append(option);
  }

  const announce = (message: string): void => {
    element("#live-region").textContent = message;
  };

  const duration = (): number => overviewDuration(routes);
  const taskNotice = new TaskArrivalNotice(viewerHost);
  const flightHud = new FlightHud(element(".stage-viewport"), element(".playback-bar"));
  const routeInteraction = new RouteInteraction(viewerHost, viewer, scenarioSelect, () => routes);
  const routeStates = new RouteStatusStrip();
  const flightAnnouncements = new FlightAnnouncements(element(".stage-viewport"));
  const avoidanceCache = new WeakMap<DynamicRun, AvoidanceEvent[]>();
  let avoidanceEvents: AvoidanceEvent[] = [];
  const renderTime = (timeS: number, shouldAnnounce = false, notify = false): void => {
    const previousTimeS = currentTimeS;
    currentTimeS = Math.max(0, Math.min(timeS, duration()));
    frameIndex = waypointIndex(currentRun.frames, currentTimeS);
    const frame = currentRun.frames[frameIndex]!;
    timeline.value = String(currentTimeS);
    timelineValue.value = `${currentTimeS.toFixed(1)} s`;
    previous.disabled = currentTimeS === 0;
    next.disabled = currentTimeS === duration();
    viewer?.setTime(currentTimeS);
    if (notify) taskNotice.advance(routes.find(route => route.id === viewer?.followedRouteId), previousTimeS, currentTimeS);
    else taskNotice.reset();
    const route = routes.find(route => route.id === currentScenario.id)!;
    if (notify) flightAnnouncements.advance(route.id, routes.indexOf(route), avoidanceEvents, previousTimeS, currentTimeS);
    else flightAnnouncements.reset();
    flightHud.update(route, currentTimeS, routes.indexOf(route), !notify);
    routeInteraction.update(currentTimeS, !notify);
    routeStates.update(routes, currentTimeS, !notify);
    showPlaybackState(element("#playback-state"), "reactive", playbackAction(route.timedPath!, currentTimeS, route.mission, frame), false, flightPhase(route, currentTimeS));
    if (frame !== lastMetricsFrame) {
      renderFrameMetrics(frame, frameIndex, currentRun.frames.length);
      lastMetricsFrame = frame;
    }
    if (shouldAnnounce) announce(`All ${routes.length} missions at ${currentTimeS.toFixed(1)} seconds`);
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
    if (currentTimeS >= duration()) {
      setPlaying(false);
      announce("Playback complete");
      return;
    }
    animationFrame = requestAnimationFrame(playbackTick);
  };

  const beginPlayback = (): void => {
    if (currentTimeS >= duration() - 1e-8) renderTime(0);
    playbackClock.start(currentTimeS);
    setPlaying(true);
    animationFrame = requestAnimationFrame(playbackTick);
  };

  const updateRun = (plannerId: string, shouldAnnounce = true): void => {
    const resume = animationFrame !== null;
    setPlaying(false);
    const run = currentScenario.runs.find((candidate) => candidate.plannerId === plannerId);
    if (!run) throw new Error(`Missing ${plannerId} run in ${currentScenario.id}`);
    currentRun = run;
    avoidanceEvents = avoidanceCache.get(run) ?? dynamicAvoidanceEvents(currentScenario, run);
    avoidanceCache.set(run, avoidanceEvents);
    routes = dynamicRoutes(bundle.scenarios, plannerId);
    viewer?.setRoutes(routes, currentScenario.id);
    timeline.max = String(duration());
    renderOutcome(run);
    renderTime(currentTimeS);
    highlightPlanner(plannerId);
    const planner = bundle.planners.find((candidate) => candidate.id === plannerId);
    if (shouldAnnounce) announce(`${planner?.label ?? plannerId} trace loaded`);
    if (resume) beginPlayback();
  };

  const updateScenario = (): void => {
    const resume = animationFrame !== null;
    setPlaying(false);
    const scenario = bundle.scenarios.find((candidate) => candidate.id === scenarioSelect.value);
    if (!scenario) throw new Error(`Unknown scenario: ${scenarioSelect.value}`);
    currentScenario = scenario;
    viewer?.setScenario(scenario);
    viewer?.focusRoute(scenario.id);
    const previousPlanner = plannerSelect.value;
    plannerSelect.replaceChildren();
    for (const planner of bundle.planners) {
      const option = document.createElement("option");
      option.value = planner.id;
      option.textContent = shortPlanner(planner.label);
      option.selected = planner.id === previousPlanner;
      plannerSelect.append(option);
    }
    if (!plannerSelect.value) plannerSelect.value = bundle.planners[0]!.id;
    renderComparison(bundle, scenario);
    updateRun(plannerSelect.value, false);
    viewerHost.setAttribute(
      "aria-label",
      `${routes.length} tasks share one obstacle world and clock. Focus: ${scenario.label}: ${scenario.buildings.length} buildings, ` +
        `${scenario.staticNoFlyZones.length} static zones, ` +
        `${scenario.temporaryNoFlyZones.length} temporary zones, and ` +
        `${scenario.movingSpheres.length} moving obstacles.`,
    );
    announce(`${scenario.label} loaded`);
    if (resume) beginPlayback();
  };

  scenarioSelect.addEventListener("change", updateScenario);
  plannerSelect.addEventListener("change", () => updateRun(plannerSelect.value));
  timeline.addEventListener("input", () => {
    setPlaying(false);
    renderTime(Number(timeline.value), true);
  });
  previous.addEventListener("click", () => {
    setPlaying(false);
    renderTime(currentTimeS - 2, true);
  });
  next.addEventListener("click", () => {
    setPlaying(false);
    renderTime(currentTimeS + 2, true);
  });
  playPause.addEventListener("click", () => {
    if (animationFrame === null) beginPlayback();
    else {
      setPlaying(false);
      announce("Playback paused");
    }
  });
  mountPlaybackSpeedControls(playbackClock, () => currentTimeS, rate => announce(`Playback speed ${rate} times`));
  document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((button) => {
    button.addEventListener("click", () => {
      const view = button.dataset.view;
      if (view === "isometric" || view === "top" || view === "reset") viewer?.setView(view);
      const activeView = view === "reset" ? "isometric" : view;
      document.querySelectorAll<HTMLButtonElement>("[data-view]:not([data-view='reset'])").forEach((candidate) => {
        candidate.setAttribute("aria-pressed", String(candidate.dataset.view === activeView));
      });
    });
  });

  scenarioSelect.value = bundle.scenarios[0]!.id;
  currentScenario = bundle.scenarios[0]!;
  currentRun = currentScenario.runs[0]!;
  updateScenario();
  mountWorkspace({
    scenarios: bundle.scenarios,
    select: scenarioSelect,
  });
  mountInspector();
  mountTabs();
  routeStates.update(routes, currentTimeS, true);
  mountFollowControls(viewer, scenarioSelect, enabled => { taskNotice.reset(); flightAnnouncements.reset(); routeInteraction.followChanged(enabled); });
  element("#load-state").remove();
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) setPlaying(false);
  });
  mountBenchmark({ isPlaying: () => animationFrame !== null, pause: () => setPlaying(false), resume: beginPlayback });
  mountPageLifecycle({ pause: () => setPlaying(false), restore: () => renderTime(currentTimeS), dispose: () => {
      routeInteraction.dispose();
      taskNotice.dispose();
      flightAnnouncements.dispose();
      flightHud.dispose();
      viewer?.dispose();
  } });
}

start().catch((error: unknown) => {
  const state = element("#load-state");
  state.classList.add("is-error");
  state.textContent = error instanceof Error ? error.message : "Dynamic traces could not load.";
  console.error(error);
});
