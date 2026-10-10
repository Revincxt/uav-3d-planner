import "./styles.css";

import { loadDemoBundle } from "./data";
import { SceneViewer } from "./scene-viewer";
import type { DemoBundle, DemoScenario, PlannerId } from "./schema";
import { mountInspector, mountWorkspace } from "./workspace";
import "./workspace.css";
import "./simulator-ui.css";
import { mountBenchmark } from "./benchmark-panel";
import { mountPageLifecycle } from "./page-lifecycle";
import { RouteInteraction } from "./route-interaction";
import { FlightHud } from "./flight-hud";
import { overviewDuration, staticRoutes } from "./route-overview";
import { mountFollowControls } from "./drone-follow";
import { TaskArrivalNotice } from "./task-arrival-notice";
import { PlaybackClock, mountPlaybackSpeedControls } from "./playback-clock";
import { playbackAction, showPlaybackButton, showPlaybackState } from "./playback-state";
import { encounterView, mountEncounterControl } from "./encounter-view";
import { flightPhase, RouteStatusStrip } from "./trajectory-semantics";

const element = <T extends HTMLElement>(selector: string): T => {
  const value = document.querySelector<T>(selector);
  if (!value) throw new Error(`Missing required element: ${selector}`);
  return value;
};

const formatMetric = (value: number | null, digits = 1): string =>
  value === null ? "—" : value.toFixed(digits);

function renderPlannerResults(
  bundle: DemoBundle,
  scenario: DemoScenario,
  visible: Set<PlannerId>,
  onVisibilityChange: (plannerId: PlannerId, checked: boolean) => void,
): void {
  const container = element("#planner-results");
  container.replaceChildren();
  for (const result of scenario.results) {
    const planner = bundle.planners.find((item) => item.id === result.plannerId);
    const plannerLabel = planner?.label ?? result.plannerId;
    const card = document.createElement("article");
    card.className = `planner-card${visible.has(result.plannerId) ? "" : " is-hidden"}`;
    card.style.setProperty("--planner-color", "#64748b");
    const header = document.createElement("div");
    header.className = "planner-card-heading";
    const label = document.createElement("label");
    label.className = "planner-toggle";
    const input = document.createElement("input");
    input.type = "radio";
    input.name = "overview-planner";
    input.checked = visible.has(result.plannerId);
    input.value = result.plannerId;
    input.setAttribute("aria-label", `Use ${plannerLabel} for all mission routes`);
    const swatch = document.createElement("span");
    swatch.className = "planner-swatch";
    swatch.setAttribute("aria-hidden", "true");
    const name = document.createElement("strong");
    name.textContent = plannerLabel;
    const toggle = document.createElement("span");
    toggle.className = "path-visibility";
    toggle.setAttribute("aria-hidden", "true");
    label.append(input, swatch, name, toggle);
    input.addEventListener("change", () => {
      card.classList.toggle("is-hidden", !input.checked);
      onVisibilityChange(result.plannerId, input.checked);
    });
    header.append(label);

    const metric = document.createElement("div");
    metric.className = "path-length";
    const value = document.createElement("strong");
    value.textContent = formatMetric(result.metrics.smoothedLengthM);
    const unit = document.createElement("span");
    unit.textContent = "m";
    const metricLabel = document.createElement("span");
    metricLabel.className = "path-length-label";
    metricLabel.textContent = "Path length";
    metric.append(value, unit, metricLabel);

    const metrics = document.createElement("dl");
    metrics.className = "planner-metrics";
    for (const [labelText, valueText] of [
      ["Planning", `${formatMetric(result.metrics.planningTimeMs)} ms`],
      ["Clearance", result.metrics.minClearanceM === null ? "—" : `${formatMetric(result.metrics.minClearanceM, 2)} m`],
    ]) {
      const item = document.createElement("div");
      const term = document.createElement("dt");
      const definition = document.createElement("dd");
      term.textContent = labelText ?? "";
      definition.textContent = valueText ?? "";
      item.append(term, definition);
      metrics.append(item);
    }
    const status = document.createElement("span");
    status.className = `planner-status status-${result.status}`;
    status.textContent = result.status === "success" ? "Collision-free" : result.status === "no-path" ? "No route found" : "Invalid route";
    if (result.failureReason) status.title = result.failureReason;
    card.title = `${plannerLabel} · ${result.searchEffort.value.toLocaleString()} ${result.searchEffort.kind}`;
    card.append(header, metric, metrics, status);
    container.append(card);
  }
}

async function start(): Promise<void> {
  const bundle = await loadDemoBundle();
  const select = element<HTMLSelectElement>("#scene-select");
  const plannerSelect = element<HTMLSelectElement>("#static-planner-select");
  const viewerElement = element("#scene-viewer");
  let selectedPlanner: PlannerId = "lazy-theta-star";
  const visible = new Set<PlannerId>([selectedPlanner]);
  let cameraView: "isometric" | "top" = "isometric";
  let viewer: SceneViewer | null = null;
  const timeline = element<HTMLInputElement>("#timeline");
  const playPause = element<HTMLButtonElement>("#play-pause");
  let routes = staticRoutes(bundle.scenarios, selectedPlanner);
  let currentTimeS = 0;
  const playbackClock = new PlaybackClock();
  let animationFrame: number | null = null;
  for (const planner of bundle.planners) {
    const option = document.createElement("option"); option.value = planner.id;
    option.textContent = planner.label; plannerSelect.append(option);
  }
  plannerSelect.value = selectedPlanner;

  bundle.scenarios.forEach((scenario) => {
    const option = document.createElement("option");
    option.value = scenario.id;
    option.textContent = scenario.label;
    option.selected = scenario.id === bundle.defaultScenarioId;
    select.append(option);
  });
  mountWorkspace({
    scenarios: bundle.scenarios,
    select,
  });
  mountInspector();

  try {
    viewer = new SceneViewer(viewerElement);
  } catch (error) {
    viewerElement.classList.add("viewer-error");
    viewerElement.textContent = "3D preview unavailable. Select a scenario to explore its recorded paths.";
    document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((button) => { button.disabled = true; });
    console.error(error);
  }

  const selectedScenario = (): DemoScenario => {
    const scenario = bundle.scenarios.find((item) => item.id === select.value);
    if (!scenario) throw new Error(`Unknown scenario: ${select.value}`);
    return scenario;
  };

  const taskNotice = new TaskArrivalNotice(viewerElement);
  const flightHud = new FlightHud(element(".stage-viewport"), element(".playback-bar"));
  const routeInteraction = new RouteInteraction(viewerElement, viewer, select, () => routes);
  const routeStates = new RouteStatusStrip();
  const renderTime = (timeS: number, notify = false): void => {
    const previousTimeS = currentTimeS;
    const duration = overviewDuration(routes);
    currentTimeS = Math.max(0, Math.min(timeS, duration));
    timeline.value = String(currentTimeS);
    element<HTMLOutputElement>("#timeline-value").value = `${currentTimeS.toFixed(1)} s`;
    element<HTMLButtonElement>("#previous-frame").disabled = currentTimeS === 0;
    element<HTMLButtonElement>("#next-frame").disabled = currentTimeS === duration;
    viewer?.setTime(currentTimeS);
    if (notify) taskNotice.advance(routes.find(route => route.id === viewer?.followedRouteId), previousTimeS, currentTimeS);
    else taskNotice.reset();
    const route = routes.find(route => route.id === select.value)!;
    flightHud.update(route, currentTimeS, routes.indexOf(route), !notify);
    routeInteraction.update(currentTimeS, !notify);
    routeStates.update(routes, currentTimeS, !notify);
    showPlaybackState(element("#playback-state"), "fixed", playbackAction(route.timedPath!, currentTimeS, route.mission), false, flightPhase(route, currentTimeS));
  };
  const setPlaying = (playing: boolean): void => {
    viewer?.setPlaying(playing);
    if (!playing && animationFrame !== null) { cancelAnimationFrame(animationFrame); animationFrame = null; }
    showPlaybackButton(playPause, playing);
  };
  mountEncounterControl(() => encounterView(routes.find(route => route.id === select.value)!), view => {
    setPlaying(false); renderTime(view.timeS); viewer?.observeEncounter(view.position);
  });
  const playbackTick = (now: number): void => {
    renderTime(playbackClock.sample(now), true);
    if (currentTimeS >= overviewDuration(routes) - 1e-8) { setPlaying(false); return; }
    animationFrame = requestAnimationFrame(playbackTick);
  };
  const beginPlayback = (): void => {
    if (currentTimeS >= overviewDuration(routes) - 1e-8) renderTime(0);
    playbackClock.start(currentTimeS);
    setPlaying(true);
    animationFrame = requestAnimationFrame(playbackTick);
  };
  const refreshRoutes = (): void => {
    setPlaying(false);
    routes = staticRoutes(bundle.scenarios, selectedPlanner);
    timeline.max = String(overviewDuration(routes));
    viewer?.setRoutes(routes, select.value);
    renderTime(currentTimeS);
  };

  const renderResults = (): void => {
    renderPlannerResults(bundle, selectedScenario(), visible, (plannerId, checked) => {
      if (!checked) return;
      const resume = animationFrame !== null;
      selectedPlanner = plannerId;
      plannerSelect.value = plannerId;
      visible.clear();
      visible.add(plannerId);
      refreshRoutes();
      renderResults();
      if (resume) beginPlayback();
      element("#live-region").textContent = `All ${routes.length} routes use ${bundle.planners.find((planner) => planner.id === plannerId)?.label ?? plannerId}`;
    });
  };

  const update = (): void => {
    const resume = animationFrame !== null;
    const scenario = selectedScenario();
    viewer?.setScenario(scenario);
    viewer?.focusRoute(scenario.id);
    refreshRoutes();
    renderResults();
    element("#altitude-limit").textContent = `${scenario.constraints.maxAltitudeM} m`;
    element("#safety-margin").textContent = `${scenario.constraints.safetyMarginM} m`;
    viewerElement.setAttribute("aria-label", `${routes.length} tasks share one Manhattan obstacle world. Focus: ${scenario.label}.`);
    element("#live-region").textContent = `${scenario.label} loaded`;
    if (resume) beginPlayback();
  };

  select.addEventListener("change", update);
  plannerSelect.addEventListener("change", () => {
    const resume = animationFrame !== null;
    selectedPlanner = plannerSelect.value as PlannerId;
    visible.clear(); visible.add(selectedPlanner);
    refreshRoutes();
    renderResults();
    if (resume) beginPlayback();
  });
  document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((button) => {
    button.addEventListener("click", () => {
      const view = button.dataset.view;
      if (view !== "isometric" && view !== "top" && view !== "reset") return;
      cameraView = view === "top" ? "top" : "isometric";
      viewer?.setView(view);
      document.querySelectorAll<HTMLButtonElement>('[data-view="isometric"], [data-view="top"]').forEach((item) => item.setAttribute("aria-pressed", String(item.dataset.view === cameraView)));
    });
  });
  timeline.addEventListener("input", () => { setPlaying(false); renderTime(Number(timeline.value)); });
  playPause.addEventListener("click", () => { if (animationFrame === null) beginPlayback(); else setPlaying(false); });
  element("#previous-frame").addEventListener("click", () => { setPlaying(false); renderTime(currentTimeS - 2); });
  element("#next-frame").addEventListener("click", () => { setPlaying(false); renderTime(currentTimeS + 2); });
  mountPlaybackSpeedControls(playbackClock, () => currentTimeS);
  mountPageLifecycle({ pause: () => setPlaying(false), restore: () => renderTime(currentTimeS),
    dispose: () => { routeInteraction.dispose(); taskNotice.dispose(); flightHud.dispose(); viewer?.dispose(); } });
  document.addEventListener("visibilitychange", () => { if (document.hidden) setPlaying(false); });
  update();
  mountFollowControls(viewer, select, enabled => { taskNotice.reset(); routeInteraction.followChanged(enabled); });
  mountBenchmark({ isPlaying: () => animationFrame !== null, pause: () => setPlaying(false), resume: beginPlayback });
}

start().catch((error: unknown) => {
  const viewer = element("#scene-viewer");
  viewer.classList.add("viewer-error");
  viewer.textContent = error instanceof Error ? error.message : "The recorded dataset could not load.";
  console.error(error);
});
