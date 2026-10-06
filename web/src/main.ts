import "./styles.css";

import { loadDemoBundle } from "./data";
import { SceneViewer } from "./scene-viewer";
import type { DemoBundle, DemoScenario, PlannerId } from "./schema";
import { missionCaption, mountInspector, mountWorkspace } from "./workspace";
import "./workspace.css";
import "./simulator-ui.css";
import { FlightHud } from "./flight-hud";
import { overviewDuration, staticRoutes } from "./route-overview";
import { mountFollowControls } from "./drone-follow";
import { TaskArrivalNotice } from "./task-arrival-notice";
import { PlaybackClock, mountPlaybackSpeedControls } from "./playback-clock";
import { playbackAction, showPlaybackButton, showPlaybackState } from "./playback-state";
import { encounterView, mountEncounterControl, showChallenge } from "./encounter-view";

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
  const pathMode = "smoothed";
  let cameraView: "isometric" | "top" = "isometric";
  let viewer: SceneViewer | null = null;
  const timeline = element<HTMLInputElement>("#timeline");
  const playPause = element<HTMLButtonElement>("#play-pause");
  let routes = staticRoutes(bundle.scenarios, selectedPlanner, pathMode);
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
    scenarios: bundle.scenarios.map((scenario) => ({
      ...scenario,
      group: "Manhattan",
      summary: scenario.mission ? `${scenario.mission.origin} → ${scenario.mission.destination}` : scenario.description,
    })),
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
    showPlaybackState(element("#playback-state"), "fixed", playbackAction(route.timedPath!, currentTimeS, route.mission));
    showChallenge(document.querySelector("#scene-challenge"), route.mission);
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
    routes = staticRoutes(bundle.scenarios, selectedPlanner, pathMode);
    timeline.max = String(overviewDuration(routes));
    viewer?.setRoutes(routes, select.value);
    renderTime(currentTimeS);
  };

  const updateComparisonCaption = (): void => {
    const preservesAltitude = selectedScenario().results.every(
      (result) => result.smoothing.altitudePolicy === "preserve-raw-altitude-profile-v1",
    );
    element("#comparison-caption").textContent = preservesAltitude
      ? `${routes.length} routes · XY optimized · Z preserved`
      : `${routes.length} routes`;
  };
  const updateSceneCaption = (): void => {
    const scenario = selectedScenario();
    element("#scene-caption").textContent = missionCaption(scenario, "city");
  };
  const renderResults = (): void => {
    renderPlannerResults(bundle, selectedScenario(), visible, (plannerId, checked) => {
      if (!checked) return;
      selectedPlanner = plannerId;
      plannerSelect.value = plannerId;
      visible.clear();
      visible.add(plannerId);
      viewer?.setPlannerVisibility(visible);
      refreshRoutes();
      renderResults();
      element("#live-region").textContent = `All ${routes.length} routes use ${bundle.planners.find((planner) => planner.id === plannerId)?.label ?? plannerId}`;
    });
    updateComparisonCaption();
  };

  const update = (): void => {
    const resume = animationFrame !== null;
    const scenario = selectedScenario();
    viewer?.setScenario(scenario, visible, pathMode);
    viewer?.focusRoute(scenario.id);
    refreshRoutes();
    renderResults();
    element("#scene-title").textContent = "Manhattan";
    updateSceneCaption();
    element("#altitude-limit").textContent = `${scenario.constraints.maxAltitudeM} m`;
    element("#safety-margin").textContent = `${scenario.constraints.safetyMarginM} m`;
    const startCoordinate = element("#start-coordinate");
    const goalCoordinate = element("#goal-coordinate");
    startCoordinate.textContent = scenario.mission?.origin ?? scenario.start.map((value) => value.toFixed(1)).join(", ");
    goalCoordinate.textContent = scenario.mission?.destination ?? scenario.goal.map((value) => value.toFixed(1)).join(", ");
    startCoordinate.title = `ENU / m: ${scenario.start.map((value) => value.toFixed(1)).join(", ")}`;
    goalCoordinate.title = `ENU / m: ${scenario.goal.map((value) => value.toFixed(1)).join(", ")}`;
    viewerElement.setAttribute("aria-label", `${routes.length} tasks share one Manhattan obstacle world. Focus: ${scenario.label}.`);
    element("#live-region").textContent = `${scenario.label} loaded`;
    if (resume) beginPlayback();
  };

  select.addEventListener("change", update);
  plannerSelect.addEventListener("change", () => {
    selectedPlanner = plannerSelect.value as PlannerId;
    visible.clear(); visible.add(selectedPlanner);
    viewer?.setPlannerVisibility(visible);
    refreshRoutes();
    renderResults();
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
  window.addEventListener("pagehide", () => { setPlaying(false); taskNotice.dispose(); flightHud.dispose(); viewer?.dispose(); }, { once: true });
  update();
  mountFollowControls(viewer, select, () => taskNotice.reset());
}

start().catch((error: unknown) => {
  const viewer = element("#scene-viewer");
  viewer.classList.add("viewer-error");
  viewer.textContent = error instanceof Error ? error.message : "The recorded dataset could not load.";
  console.error(error);
});
