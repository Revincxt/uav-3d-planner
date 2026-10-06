import * as THREE from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { enuToThree } from "./coordinates";
import type { CityMission } from "./city-schema";
import type { DemoScenario, PlannerId } from "./schema";
import type { DynamicRun, DynamicScenario } from "./dynamic-schema";
import type { PredictivePathMode, PredictiveRun, PredictiveScenario } from "./predictive-schema";
import { createEndpointMarker } from "./scene-style";
import { createDrone, orientDrone, sizeDrone } from "./drone-model";
import { disposeRenderObject } from "./render-resources";
import type { DisplayCamera } from "./camera-scale";
import { staticPlaybackPath } from "./static-playback";
import { revealReplayLine } from "./replay-line";
import type { PlaybackKind } from "./playback-state";
import type { DynamicFrame } from "./dynamic-schema";
import { missionGateGeometry, missionGateMaterial, missionGateMatrix, missionGateYaw } from "./mission-gate";

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

export function pathPrefix(path: readonly RouteWaypoint[], timeS: number): RoutePoint[] {
  if (!path.length) return [];
  const index = waypointIndex(path, timeS);
  const points = path.slice(0, index + 1).map(waypoint => waypoint.position);
  if (timeS > path[index]!.timeS && index + 1 < path.length) points.push(timedPosition(path, timeS));
  return points;
}

export function predictivePath(run: PredictiveRun, mode: PredictivePathMode): RouteWaypoint[] {
  if (mode === "raw") return run.rawTimedPath;
  if (mode === "execution") {
    if (!run.executionTimedPath) throw new Error(`No qualified execution trace for ${run.plannerId}`);
    return run.executionTimedPath;
  }
  if (!run.smoothing.certified) throw new Error(`No certified geometry for ${run.plannerId}`);
  return run.geometryTimedPath;
}

const staticRouteCache = new WeakMap<DemoScenario, Map<string, OverviewRoute>>();
export function staticRoutes(scenarios: DemoScenario[], planner: PlannerId, mode: "raw" | "smoothed"): OverviewRoute[] {
  return scenarios.map(s => {
    let cache = staticRouteCache.get(s);
    if (!cache) { cache = new Map(); staticRouteCache.set(s, cache); }
    const key = `${planner}/${mode}`, cached = cache.get(key);
    if (cached) return cached;
    const result = s.results.find(r => r.plannerId === planner);
    if (!result?.paths || result.status !== "success") throw new Error(`Missing successful ${planner} route in ${s.id}`);
    const points = result.paths[mode];
    const route: OverviewRoute = { id: s.id, plannerId: planner, label: s.label, mission: s.mission, points,
      timedPath: staticPlaybackPath(points, s.mission), playbackKind: "fixed" };
    cache.set(key, route);
    return route;
  });
}

const dynamicRouteCache = new WeakMap<DynamicScenario, Map<string, OverviewRoute>>();
/** Recover within-frame corners from the cumulative execution, never join telemetry by a chord. */
export function reactiveTrace(run: Pick<DynamicRun, "frames">): RouteWaypoint[] {
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
      playbackKind: "reactive", plannedFrames: run.frames };
    cache.set(planner, route);
    return route;
  });
}

const predictiveRouteCache = new WeakMap<PredictiveScenario, Map<string, OverviewRoute>>();
export function predictiveRoutes(scenarios: PredictiveScenario[], planner: string, mode: PredictivePathMode): OverviewRoute[] {
  return scenarios.map(s => {
    let cache = predictiveRouteCache.get(s);
    if (!cache) { cache = new Map(); predictiveRouteCache.set(s, cache); }
    const key = `${planner}/${mode}`, cached = cache.get(key);
    if (cached) return cached;
    const run = s.runs.find(r => r.plannerId === planner);
    if (!run) throw new Error(`Missing ${planner} trace in ${s.id}`);
    const timedPath = predictivePath(run, mode);
    const route: OverviewRoute = { id: s.id, plannerId: planner, label: s.label, mission: s.mission, points: timedPath.map(w => w.position), timedPath,
      playbackKind: run.predictive ? "predictive" : "reactive" };
    cache.set(key, route);
    return route;
  });
}

export function overviewDuration(routes: readonly OverviewRoute[]): number {
  return Math.max(0, ...routes.map(r => r.timedPath?.at(-1)?.timeS ?? 0));
}

interface RouteVisual {
  route: OverviewRoute; group: THREE.Group; line: Line2; halo: Line2; vehicle?: THREE.Group;
  clock?: { value: number }; pending?: Line2; planFrame?: DynamicFrame;
}

/** Mission queries in one obstacle world. Geometry is retained during playback. */
export class RouteOverview {
  readonly group = new THREE.Group();
  readonly bounds = new THREE.Box3();
  private visuals: RouteVisual[] = [];
  private timeS = 0;
  private playing = false;
  private readonly position: RoutePoint = [0, 0, 0];
  private readonly before: RoutePoint = [0, 0, 0];
  private readonly after: RoutePoint = [0, 0, 0];
  private readonly direction = new THREE.Vector3();
  constructor() { this.group.name = "mission-route-overview"; this.group.userData.presentation = "result-preview"; }

  setRoutes(routes: OverviewRoute[], activeId: string, width: number, height: number): void {
    if (new Set(routes.map(r => r.id)).size !== routes.length) throw new Error("Duplicate overview mission IDs");
    if (routes.length === this.visuals.length && routes.every((route, index) => {
      const previous = this.visuals[index]!.route;
      return route.id === previous.id && route.plannerId === previous.plannerId && route.mission === previous.mission
        && route.playbackKind === previous.playbackKind && route.plannedFrames === previous.plannedFrames
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
      const reveal = route.timedPath && route.playbackKind && route.playbackKind !== "fixed";
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
      if (route.playbackKind === "predictive" || route.plannedFrames) {
        const pendingGeometry = route.playbackKind === "predictive" ? geometry : new LineGeometry();
        if (route.plannedFrames) pendingGeometry.setPositions([0, 0, 0, 1, 0, 0]);
        const pendingMaterial = new LineMaterial({ linewidth: 2.8, worldUnits: false, color, dashed: true,
          dashSize: 14, gapSize: 9, transparent: true, opacity: 0.75, depthTest: true, depthWrite: false });
        pendingMaterial.resolution.copy(material.resolution);
        pending = new Line2(pendingGeometry, pendingMaterial);
        pending.name = `overview-plan-${route.id}`; pending.renderOrder = 1;
        pending.computeLineDistances();
        group.add(pending);
      }
      for (const role of ["start", "goal"] as const) {
        const point = role === "start" ? route.points[0]! : route.points.at(-1)!;
        const marker = createEndpointMarker(role, true, color);
        marker.position.fromArray(enuToThree(point)); marker.name = `overview-${role}-${route.id}`;
        if (role === "goal") marker.rotation.y = missionGateYaw(route.points, point);
        group.add(marker);
      }
      const tasks = route.mission?.taskPoints ?? [];
      if (tasks.length) {
        const markers = new THREE.InstancedMesh(missionGateGeometry(), missionGateMaterial(color), tasks.length);
        markers.name = `overview-tasks-${route.id}`;
        tasks.forEach((task, index) => {
          markers.setMatrixAt(index, missionGateMatrix(route.points, task.position));
        });
        markers.computeBoundingSphere();
        group.add(markers);
      }
      const vehicle = route.timedPath ? createDrone(color) : undefined;
      if (vehicle) {
        vehicle.position.fromArray(enuToThree(route.points[0]!));
        vehicle.name = `overview-vehicle-${route.id}`; group.add(vehicle);
      }
      this.visuals.push({ route, group, line, halo, vehicle, clock, pending });
      this.group.add(group);
      for (const point of route.points) this.bounds.expandByPoint(new THREE.Vector3(...enuToThree(point)));
    });
    this.setFocus(activeId);
    this.setTime(this.timeS);
  }

  setFocus(activeId: string): void {
    for (const visual of this.visuals) {
      const focused = visual.route.id === activeId;
      visual.group.userData.focused = focused;
      visual.line.material.linewidth = focused ? 4.8 : 4;
      visual.halo.material.linewidth = focused ? 7.3 : 6.5;
      const tasks = visual.group.getObjectByName(`overview-tasks-${visual.route.id}`);
      if (tasks) tasks.visible = !focused; // The focused mission already has numbered markers.
    }
  }

  setTime(timeS: number): void {
    this.timeS = timeS;
    for (const visual of this.visuals) {
      const { route, vehicle, clock, pending } = visual;
      // A paused map previews the complete recorded result, not a fictitious live
      // forecast. During playback, reactive histories are revealed at their real clock.
      if (clock) clock.value = this.playing ? timeS : route.timedPath!.at(-1)!.timeS;
      if (pending) pending.visible = this.playing && timeS < route.timedPath!.at(-1)!.timeS;
      if (pending && route.plannedFrames) {
        const frame = route.plannedFrames[waypointIndex(route.plannedFrames, timeS)]!;
        pending.visible &&= frame.path.length >= 2;
        if (pending.visible && frame !== visual.planFrame) {
          pending.geometry.setPositions(frame.path.flatMap(enuToThree));
          pending.computeLineDistances();
          visual.planFrame = frame;
        }
      }
      if (!vehicle || !route.timedPath) continue;
      // Finished missions remain at their real goal; no looping or invented continuation.
      const position = timedPosition(route.timedPath, timeS, this.position);
      vehicle.position.set(position[0], position[2], -position[1]);
      const before = timedPosition(route.timedPath, timeS - 0.03, this.before);
      const after = timedPosition(route.timedPath, timeS + 0.03, this.after);
      this.direction.set(after[0] - before[0], after[2] - before[2], before[1] - after[1]);
      orientDrone(vehicle, this.direction);
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
    return flightHeading(this.visuals.find(v => v.route.id === id)?.route.timedPath ?? [], timeS);
  }
  updateSymbols(camera: DisplayCamera, height: number, followedId: string | null = null): void {
    for (const { vehicle, group, route } of this.visuals) if (vehicle) {
      const followed = route.id === followedId;
      vehicle.visible = true;
      sizeDrone(vehicle, camera, height, followed ? Math.min(88, Math.max(24, height * 0.18)) : group.userData.focused ? 32 : 24,
        followed ? 0 : 0.8);
      const tasks = group.getObjectByName(`overview-tasks-${route.id}`);
      if (tasks) tasks.visible = !group.userData.focused && route.id !== followedId;
      for (const role of ["start", "goal"]) {
        const marker = group.getObjectByName(`overview-${role}-${route.id}`);
        // The hollow finish target never occupies the body; keep the flag visible on arrival.
        if (marker) marker.visible = role === "goal" || marker.position.distanceTo(vehicle.position) > Math.max(15, vehicle.scale.x);
      }
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
