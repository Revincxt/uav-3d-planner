import * as THREE from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { enuToThree } from "./coordinates";
import type { CityMission } from "./city-schema";
import type { DemoScenario, PlannerId } from "./schema";
import type { DynamicRun, DynamicScenario } from "./dynamic-schema";
import type { PredictiveScenario } from "./predictive-schema";
import { finalFlight } from "./final-flight";
import { createEndpointMarker } from "./scene-style";
import { createDrone, orientDrone, sizeDrone } from "./drone-model";
import { disposeRenderObject } from "./render-resources";
import type { DisplayCamera } from "./camera-scale";
import { staticPlaybackPath } from "./static-playback";
import { revealReplayLine, revealReplayWindow } from "./replay-line";
import { flightPhase, planMeaning } from "./trajectory-semantics";
import { metresPerPixelAt } from "./camera-scale";
import type { PlaybackKind } from "./playback-state";
import type { DynamicFrame } from "./dynamic-schema";
import { missionGateGeometry, missionGateMaterial, missionGateMatrix, missionGateYaw } from "./mission-gate";
import { pickVisibleRoutes, type RoutePick } from "./route-picking";

export type RoutePoint = [number, number, number];
export interface RouteWaypoint { timeS: number; position: RoutePoint }
export interface OverviewRoute {
  id: string;
  plannerId?: string;
  label: string;
  points: RoutePoint[];
  timedPath?: RouteWaypoint[];
  mission?: CityMission;
  playbackKind?: PlaybackKind;
  plannedFrames?: DynamicFrame[];
  cruiseSpeedMps?: number;
  maxClimbRateMps?: number;
  waits?: { startTimeS: number; endTimeS: number; reason: string; position: RoutePoint }[];
}

// Color identifies the mission, never the planner or qualification status.
export const ROUTE_COLORS = [0x009b87, 0x2563eb, 0xe78a08, 0x9333ea, 0xe34273, 0x008da8, 0x76951c, 0x594bc9] as const;
export function routeColor(index: number): number { return ROUTE_COLORS[index % ROUTE_COLORS.length]!; }
export function routeColorCSS(index: number): string { return `#${routeColor(index).toString(16).padStart(6, "0")}`; }

/** Last recorded knot at/before t; duplicate boundary timestamps resolve to the last knot. */
export function waypointIndex(path: readonly { timeS: number }[], timeS: number): number {
  if (!path.length) return -1;
  let low = 0, high = path.length;
  while (low < high) {
    const middle = (low + high) >>> 1;
    if (path[middle]!.timeS <= timeS) low = middle + 1;
    else high = middle;
  }
  return Math.max(0, low - 1);
}

export function timedPosition(path: readonly RouteWaypoint[], timeS: number, target?: RoutePoint): RoutePoint {
  if (!path.length) throw new Error("Cannot sample an empty mission trace");
  const index = waypointIndex(path, timeS);
  const left = path[index]!, right = path[index + 1];
  if (timeS <= left.timeS || !right) {
    if (!target) return left.position;
    for (let axis = 0; axis < 3; axis++) target[axis] = left.position[axis]!;
    return target;
  }
  const fraction = (timeS - left.timeS) / (right.timeS - left.timeS);
  const result = target ?? [0, 0, 0];
  for (let axis = 0; axis < 3; axis++) result[axis] = left.position[axis]! + (right.position[axis]! - left.position[axis]!) * fraction;
  return result as RoutePoint;
}

/** A recorded local plan is consumed by actual travel, not by time spent hovering. */
function flightDuration(a: RoutePoint, b: RoutePoint, speed: number, climb?: number): number {
  return Math.max(Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) / speed,
    climb ? Math.abs(b[2] - a[2]) / climb : 0);
}

type MotionRoute = Pick<OverviewRoute, "timedPath" | "cruiseSpeedMps" | "maxClimbRateMps">;
interface MotionIndex {
  path: RouteWaypoint[]; speed: number; climb?: number; cumulative: Float64Array;
}
// Routes are retained immutable replay inputs. A changed path or flight envelope
// replaces the index; WeakMap ownership releases it with the route, not the page.
const motionIndices = new WeakMap<MotionRoute, MotionIndex>();

export function localPlanClock(route: Pick<OverviewRoute, "timedPath" | "cruiseSpeedMps" | "maxClimbRateMps">, fromS: number, timeS: number): number {
  const path = route.timedPath;
  if (!path?.length || timeS <= fromS) return fromS;
  const speed = route.cruiseSpeedMps || 15, climb = route.maxClimbRateMps;
  let index = motionIndices.get(route);
  if (!index || index.path !== path || index.cumulative.length !== path.length || index.speed !== speed || index.climb !== climb) {
    const cumulative = new Float64Array(path.length);
    for (let i = 1; i < path.length; i++) {
      cumulative[i] = cumulative[i - 1]! + flightDuration(path[i - 1]!.position, path[i]!.position, speed, climb);
    }
    index = { path, speed, climb, cumulative }; motionIndices.set(route, index);
  }
  const progressAt = (clock: number): number => {
    const knot = waypointIndex(path, clock), left = path[knot]!, right = path[knot + 1];
    const fraction = !right || clock <= left.timeS ? 0 : (clock - left.timeS) / (right.timeS - left.timeS);
    return index.cumulative[knot]! + (right ? fraction * (index.cumulative[knot + 1]! - index.cumulative[knot]!) : 0);
  };
  return fromS + Math.max(0, progressAt(timeS) - progressAt(fromS));
}

/** Deterministic horizontal flight heading, including seeks into a wait, climb or finished goal. */
export function flightHeading(path: readonly RouteWaypoint[], timeS: number): THREE.Vector3 {
  const index = waypointIndex(path, timeS);
  const direction = (left: RoutePoint, right: RoutePoint) => new THREE.Vector3(right[0] - left[0], 0, left[1] - right[1]);
  if (index >= 0 && index + 1 < path.length) {
    const current = direction(path[index]!.position, path[index + 1]!.position);
    if (current.lengthSq() > 1e-10) return current.normalize();
  }
  for (let i = index; i > 0; i--) {
    const previous = direction(path[i - 1]!.position, path[i]!.position);
    if (previous.lengthSq() > 1e-10) return previous.normalize();
  }
  for (let i = Math.max(0, index); i + 1 < path.length; i++) {
    const next = direction(path[i]!.position, path[i + 1]!.position);
    if (next.lengthSq() > 1e-10) return next.normalize();
  }
  return new THREE.Vector3(0, 0, -1);
}

/** Display attitude only: a short symmetric window suppresses tiny-knot yaw flicker.
 * Coordinates and clocks are untouched; waits and vertical legs retain an actual
 * substantial incoming bearing instead of amplifying sub-centimetre XY noise. */
export function displayHeading(path: readonly RouteWaypoint[], timeS: number): THREE.Vector3 {
  const index = waypointIndex(path, timeS), left = path[index], right = path[index + 1];
  if (left && right) {
    const horizontal = Math.hypot(right.position[0] - left.position[0], right.position[1] - left.position[1]);
    if (horizontal / Math.max(1e-9, right.timeS - left.timeS) > 0.2) {
      const a = timedPosition(path, timeS - 0.4), b = timedPosition(path, timeS + 0.4);
      const direction = new THREE.Vector3(b[0] - a[0], 0, a[1] - b[1]);
      if (direction.lengthSq() > 0.01) return direction.normalize();
    }
  }
  for (const [from, to, step] of [[index, 0, -1], [Math.max(1, index + 1), path.length - 1, 1]]) {
    for (let i = from!; step! < 0 ? i > to! : i <= to!; i += step!) {
      const a = path[i - 1]?.position, b = path[i]?.position;
      if (!a || !b) continue;
      const direction = new THREE.Vector3(b[0] - a[0], 0, a[1] - b[1]);
      if (direction.lengthSq() > 0.25) return direction.normalize();
    }
  }
  return flightHeading(path, timeS);
}

const staticRouteCache = new WeakMap<DemoScenario, Map<string, OverviewRoute>>();
export function staticRoutes(scenarios: DemoScenario[], planner: PlannerId): OverviewRoute[] {
  return scenarios.map(s => {
    let cache = staticRouteCache.get(s);
    if (!cache) { cache = new Map(); staticRouteCache.set(s, cache); }
    const cached = cache.get(planner);
    if (cached) return cached;
    const result = s.results.find(r => r.plannerId === planner);
    if (!result?.paths || result.status !== "success") throw new Error(`Missing successful ${planner} route in ${s.id}`);
    const points = result.paths.smoothed;
    const route: OverviewRoute = { id: s.id, plannerId: planner, label: s.label, mission: s.mission, points,
      timedPath: staticPlaybackPath(points, s.mission, 15, 3), playbackKind: "fixed" };
    cache.set(planner, route);
    return route;
  });
}

const dynamicRouteCache = new WeakMap<DynamicScenario, Map<string, OverviewRoute>>();
/** Recover within-frame corners from the cumulative execution, never join telemetry by a chord. */
export function reactiveTrace(run: Pick<DynamicRun, "frames" | "executionTimedPath">): RouteWaypoint[] {
  if (run.executionTimedPath) return run.executionTimedPath;
  const first = run.frames[0];
  if (!first) throw new Error("Empty reactive trace");
  const trace: RouteWaypoint[] = [{ timeS: first.timeS, position: first.vehicle }];
  for (let index = 1; index < run.frames.length; index += 1) {
    const left = run.frames[index - 1]!, right = run.frames[index]!;
    const points = right.executedPath.slice(left.executedPath.length - 1);
    const lengths = points.slice(1).map((point, i) => Math.hypot(...point.map((value, axis) => value - points[i]![axis]!)));
    const total = lengths.reduce((sum, length) => sum + length, 0);
    if (total === 0) { trace.push({ timeS: right.timeS, position: right.vehicle }); continue; }
    let traversed = 0;
    points.slice(1).forEach((position, i) => {
      traversed += lengths[i]!;
      trace.push({ timeS: i === lengths.length - 1 ? right.timeS : left.timeS + traversed / total * (right.timeS - left.timeS), position });
    });
  }
  return trace;
}

export function dynamicRoutes(scenarios: DynamicScenario[], planner: string): OverviewRoute[] {
  return scenarios.map(s => {
    let cache = dynamicRouteCache.get(s);
    if (!cache) { cache = new Map(); dynamicRouteCache.set(s, cache); }
    const cached = cache.get(planner);
    if (cached) return cached;
    const run = s.runs.find(r => r.plannerId === planner);
    if (!run || run.status !== "success") throw new Error(`Missing successful ${planner} trace in ${s.id}`);
    const timedPath = reactiveTrace(run);
    const route: OverviewRoute = { id: s.id, plannerId: planner, label: s.label, mission: s.mission, points: run.frames.at(-1)!.executedPath, timedPath,
      playbackKind: "reactive", plannedFrames: run.frames, cruiseSpeedMps: Number(run.parameters.cruiseSpeedMps),
      maxClimbRateMps: run.parameters.maxClimbRateMps };
    cache.set(planner, route);
    return route;
  });
}

const predictiveRouteCache = new WeakMap<PredictiveScenario, Map<string, OverviewRoute>>();
export function predictiveRoutes(scenarios: PredictiveScenario[], planner: string): OverviewRoute[] {
  return scenarios.map(s => {
    let cache = predictiveRouteCache.get(s);
    if (!cache) { cache = new Map(); predictiveRouteCache.set(s, cache); }
    const cached = cache.get(planner);
    if (cached) return cached;
    const run = s.runs.find(r => r.plannerId === planner);
    if (!run) throw new Error(`Missing ${planner} trace in ${s.id}`);
    const flight = finalFlight(run), timedPath = flight.path;
    const route: OverviewRoute = { id: s.id, plannerId: planner, label: s.label, mission: s.mission, points: timedPath.map(w => w.position), timedPath,
      playbackKind: run.predictive ? "predictive" : "reactive", waits: flight.waits };
    cache.set(planner, route);
    return route;
  });
}

export function overviewDuration(routes: readonly OverviewRoute[]): number {
  return Math.max(0, ...routes.map(r => r.timedPath?.at(-1)?.timeS ?? 0));
}

interface RouteVisual {
  route: OverviewRoute; group: THREE.Group; line: Line2; halo: Line2; vehicle?: THREE.Group;
  clock?: { value: number }; pending?: Line2; planFrame?: DynamicFrame;
  window?: ReturnType<typeof revealReplayWindow>; hold?: THREE.Group;
  tasks?: THREE.InstancedMesh; start: THREE.Object3D; goal: THREE.Object3D;
}

/** Mission queries in one obstacle world. Geometry is retained during playback. */
export class RouteOverview {
  readonly group = new THREE.Group();
  readonly bounds = new THREE.Box3();
  private visuals: RouteVisual[] = [];
  private timeS = 0;
  private playing = false;
  private selectedId: string | null = null;
  private readonly position: RoutePoint = [0, 0, 0];
  private readonly point = new THREE.Vector3();
  private readonly screenUp = new THREE.Vector3();
  constructor() { this.group.name = "mission-route-overview"; this.group.userData.presentation = "result-preview"; }

  setRoutes(routes: OverviewRoute[], activeId: string, width: number, height: number): void {
    if (new Set(routes.map(r => r.id)).size !== routes.length) throw new Error("Duplicate overview mission IDs");
    if (routes.length === this.visuals.length && routes.every((route, index) => {
      const previous = this.visuals[index]!.route;
      return route.id === previous.id && route.plannerId === previous.plannerId && route.mission === previous.mission
        && route.playbackKind === previous.playbackKind && route.plannedFrames === previous.plannedFrames
        && route.cruiseSpeedMps === previous.cruiseSpeedMps && route.maxClimbRateMps === previous.maxClimbRateMps && route.waits === previous.waits
        && (route.timedPath ? route.timedPath === previous.timedPath : route.points === previous.points);
    })) {
      this.setFocus(activeId);
      this.resize(width, height);
      return;
    }
    this.clear();
    routes.forEach((route, index) => {
      if (route.points.length < 2) throw new Error(`Empty overview route: ${route.id}`);
      const color = routeColor(index), group = new THREE.Group();
      group.name = `mission-route-${route.id}`;
      group.userData = { missionId: route.id, plannerId: route.plannerId, sharedWorldId: route.mission?.sharedWorld?.id };
      const geometry = new LineGeometry();
      const reveal = route.timedPath && route.playbackKind;
      geometry.setPositions((reveal ? route.timedPath!.map(w => w.position) : route.points).flatMap(enuToThree));
      const material = new LineMaterial({ linewidth: 4, worldUnits: false, color,
        depthTest: true, depthWrite: false, alphaToCoverage: true });
      material.resolution.set(Math.max(1, width), Math.max(1, height));
      const haloMaterial = new LineMaterial({ linewidth: 6.5, worldUnits: false, color: 0xffffff,
        depthTest: true, depthWrite: false, alphaToCoverage: true });
      haloMaterial.resolution.copy(material.resolution);
      const halo = new Line2(geometry, haloMaterial);
      halo.name = `overview-outline-${route.id}`; halo.renderOrder = 2;
      group.add(halo);
      const line = new Line2(geometry, material);
      line.name = `overview-trajectory-${route.id}`;
      line.renderOrder = 3;
      group.add(line);
      const clock = reveal ? revealReplayLine(geometry, route.timedPath!.map(w => w.timeS), [material, haloMaterial]) : undefined;
      let pending: Line2 | undefined;
      let window: RouteVisual["window"];
      if (route.playbackKind) {
        const meaning = planMeaning(route);
        const pendingGeometry = new LineGeometry();
        if (!route.plannedFrames) pendingGeometry.setPositions(route.timedPath!.flatMap(w => enuToThree(w.position)));
        if (route.plannedFrames) pendingGeometry.setPositions([0, 0, 0, 1, 0, 0]);
        const pendingMaterial = new LineMaterial({ linewidth: 3, worldUnits: false,
          color, dashed: meaning !== "fixed", dashSize: 14, gapSize: 9,
          transparent: true, opacity: 0.85, depthTest: true, depthWrite: false });
        pendingMaterial.resolution.copy(material.resolution);
        pending = new Line2(pendingGeometry, pendingMaterial);
        pending.name = `overview-plan-${route.id}`; pending.renderOrder = 1;
        pending.userData.meaning = meaning;
        pending.computeLineDistances();
        window = revealReplayWindow(pendingGeometry, route.plannedFrames ? [0, 1] : route.timedPath!.map(w => w.timeS), pendingMaterial);
        group.add(pending);
      }
      const endpoints = {} as { start: THREE.Object3D; goal: THREE.Object3D };
      for (const role of ["start", "goal"] as const) {
        const point = role === "start" ? route.points[0]! : route.points.at(-1)!;
        const marker = createEndpointMarker(role, true, color);
        marker.position.fromArray(enuToThree(point)); marker.name = `overview-${role}-${route.id}`;
        marker.rotation.y = missionGateYaw(route.points, point);
        group.add(marker);
        endpoints[role] = marker;
      }
      const tasks = route.mission?.taskPoints ?? [];
      let markers: THREE.InstancedMesh | undefined;
      if (tasks.length) {
        markers = new THREE.InstancedMesh(missionGateGeometry(), missionGateMaterial(color), tasks.length);
        markers.name = `overview-tasks-${route.id}`;
        tasks.forEach((task, index) => {
          markers!.setMatrixAt(index, missionGateMatrix(route.points, task.position));
        });
        markers.computeBoundingSphere();
        group.add(markers);
      }
      const vehicle = route.timedPath ? createDrone(color) : undefined;
      if (vehicle) {
        vehicle.position.fromArray(enuToThree(route.points[0]!));
        vehicle.name = `overview-vehicle-${route.id}`; group.add(vehicle);
      }
      const hold = vehicle ? new THREE.Group() : undefined;
      if (hold) {
        hold.name = `overview-hold-${route.id}`; hold.visible = false;
        const material = new THREE.MeshBasicMaterial({ color: 0xf5bb60, depthTest: true, depthWrite: false });
        hold.add(new THREE.Mesh(new THREE.TorusGeometry(1, 0.07, 5, 24), material));
        for (const x of [-0.23, 0.23]) {
          const bar = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.8, 0.08), material);
          bar.position.x = x; hold.add(bar);
        }
        group.add(hold);
      }
      this.visuals.push({ route, group, line, halo, vehicle, clock, pending, window, hold, tasks: markers, ...endpoints });
      this.group.add(group);
      for (const point of route.points) this.bounds.expandByPoint(this.point.set(point[0], point[2], -point[1]));
    });
    this.setFocus(activeId);
    this.setTime(this.timeS);
  }

  setFocus(activeId: string): void {
    for (const visual of this.visuals) {
      const focused = visual.route.id === activeId;
      visual.group.userData.focused = focused;
      this.styleSelection(visual);
      const tasks = visual.tasks;
      if (tasks) tasks.visible = !focused; // The focused mission already has numbered markers.
    }
  }

  setSelection(id: string | null): void {
    this.selectedId = id;
    for (const visual of this.visuals) this.styleSelection(visual);
  }
  private styleSelection(visual: RouteVisual): void {
    const selected = visual.route.id === this.selectedId, focused = visual.group.userData.focused;
    visual.line.material.linewidth = selected ? 6.2 : focused ? 4.8 : 4;
    visual.halo.material.linewidth = selected ? 9.2 : focused ? 7.3 : 6.5;
    if (visual.pending) visual.pending.material.linewidth = selected ? 4.5 : 3;
    visual.group.userData.selected = selected;
  }
  pick(camera: DisplayCamera, x: number, y: number, width: number, height: number,
    occluded: (point: THREE.Vector3) => boolean, radius = 7): RoutePick | null {
    return pickVisibleRoutes(this.visuals.map(v => ({ id: v.route.id, lines: [v.line, v.pending] })), camera, x, y, width, height, occluded, radius);
  }

  setTime(timeS: number): void {
    this.timeS = timeS;
    const preview = !this.playing && timeS <= 0;
    this.group.userData.presentation = preview ? "result-preview" : this.playing ? "playback" : "paused";
    for (const visual of this.visuals) {
      const { route, vehicle, clock, pending, window, hold } = visual;
      // Only the initial map is a result preview. Pause/seek retain the actual
      // elapsed/future split, rather than revealing all eight futures as flown.
      if (clock) clock.value = preview ? route.timedPath!.at(-1)!.timeS : timeS;
      if (pending) pending.visible = !preview && timeS < route.timedPath!.at(-1)!.timeS;
      if (window) { window.start.value = timeS; window.end.value = route.timedPath!.at(-1)!.timeS; }
      if (pending && route.plannedFrames) {
        const frame = route.plannedFrames[waypointIndex(route.plannedFrames, timeS)]!;
        pending.visible &&= frame.path.length >= 2;
        if (pending.visible && frame !== visual.planFrame) {
          pending.geometry.setPositions(frame.path.flatMap(enuToThree));
          pending.computeLineDistances();
          let clock = frame.timeS;
          const times = frame.path.map((point, i) => {
            if (i) clock += flightDuration(frame.path[i - 1]!, point, route.cruiseSpeedMps || 15, route.maxClimbRateMps);
            return clock;
          });
          visual.window = revealReplayWindow(pending.geometry, times, pending.material);
          visual.window.start.value = timeS;
          visual.window.end.value = times.at(-1)!;
          visual.planFrame = frame;
        }
        if (visual.window) {
          visual.window.start.value = localPlanClock(route, frame.timeS, timeS);
          visual.window.end.value = 1e9;
        }
      }
      if (!vehicle || !route.timedPath) continue;
      // Finished missions remain at their real goal; no looping or invented continuation.
      const position = timedPosition(route.timedPath, timeS, this.position);
      vehicle.position.set(position[0], position[2], -position[1]);
      orientDrone(vehicle, displayHeading(route.timedPath, timeS));
      if (hold) {
        hold.visible = !preview && flightPhase(route, timeS).kind === "waiting";
        hold.position.copy(vehicle.position);
      }
    }
  }

  /** Switch presentation without rebuilding geometry or changing the vehicle clock. */
  setPlaying(playing: boolean): void {
    if (this.playing === playing) return;
    this.playing = playing;
    this.group.userData.presentation = playing ? "playback" : "result-preview";
    this.setTime(this.timeS);
  }

  resize(width: number, height: number): void {
    for (const visual of this.visuals) {
      visual.line.material.resolution.set(width, height);
      visual.halo.material.resolution.set(width, height);
      visual.pending?.material.resolution.set(width, height);
    }
  }

  vehicle(id: string): THREE.Group | undefined { return this.visuals.find(v => v.route.id === id)?.vehicle; }
  heading(id: string, timeS: number): THREE.Vector3 {
    return displayHeading(this.visuals.find(v => v.route.id === id)?.route.timedPath ?? [], timeS);
  }
  updateSymbols(camera: DisplayCamera, height: number, followedId: string | null = null): void {
    this.screenUp.set(0, 1, 0).applyQuaternion(camera.quaternion);
    for (const { vehicle, group, route, hold, tasks, start, goal } of this.visuals) if (vehicle) {
      const followed = route.id === followedId;
      vehicle.visible = true;
      sizeDrone(vehicle, camera, height, followed ? Math.min(88, Math.max(24, height * 0.18)) : group.userData.focused ? 32 : 24,
        followed ? 0 : 0.8);
      if (hold?.visible) {
        const scale = metresPerPixelAt(camera, vehicle.position, height) * 11;
        hold.scale.setScalar(scale); hold.quaternion.copy(camera.quaternion);
        hold.position.copy(vehicle.position).addScaledVector(this.screenUp, scale * 2.7);
      }
      if (tasks) tasks.visible = !group.userData.focused && route.id !== followedId;
      // The hollow finish stays visible; only the start marker yields to the body.
      start.visible = start.position.distanceTo(vehicle.position) > Math.max(15, vehicle.scale.x);
      goal.visible = true;
    }
  }

  private clear(): void {
    disposeRenderObject(this.group);
    this.group.clear();
    this.visuals = [];
    this.bounds.makeEmpty();
  }
  dispose(): void { this.clear(); this.group.removeFromParent(); }
}
