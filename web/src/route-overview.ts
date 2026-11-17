import * as THREE from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { enuToThree } from "./coordinates";
import type { CityMission } from "./city-schema";
import type { DemoScenario, PlannerId } from "./schema";
import type { DynamicScenario } from "./dynamic-schema";
import type { PredictiveScenario } from "./predictive-schema";
import { finalFlight } from "./final-flight";
import { createEndpointMarker } from "./scene-style";
import { createDrone, orientDrone, sizeDrone } from "./drone-model";
import { setDroneRotorTime } from "./drone-rotors";
import { disposeRenderObject } from "./render-resources";
import type { DisplayCamera } from "./camera-scale";
import { staticPlaybackPath } from "./static-playback";
import { revealReplayLine, revealReplayWindow } from "./replay-line";
import { flightPhase, planMeaning } from "./trajectory-semantics";
import { dynamicAvoidanceEvents } from "./flight-announcements";
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
  rerouteTimes?: readonly number[];
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

interface HeadingIndex { yaw: Float64Array; target: Float64Array }
const headingIndices = new WeakMap<readonly RouteWaypoint[], HeadingIndex>();
const HEADING_RESPONSE = 2;

/** Continuous display attitude from local tangents, independent of playback/seek history.
 * Unwrapped angles remain well-defined at reversals; waits and vertical legs freeze
 * the arriving attitude. Recorded positions, timestamps and physical headings stay intact. */
export function displayHeading(path: readonly RouteWaypoint[], timeS: number): THREE.Vector3 {
  if (!path.length) return new THREE.Vector3(0, 0, -1);
  let headings = headingIndices.get(path);
  if (!headings || headings.yaw.length !== path.length) {
    const yaw = new Float64Array(path.length), target = new Float64Array(path.length);
    let raw = 0, turnSign = 1;
    for (let i = 1; i < path.length; i++) {
      const a = path[i - 1]!.position, b = path[i]!.position;
      if (Math.hypot(b[0] - a[0], b[1] - a[1]) > .001) { raw = Math.atan2(a[0] - b[0], b[1] - a[1]); break; }
    }
    yaw[0] = raw;
    for (let i = 0; i + 1 < path.length; i++) {
      const a = path[i]!, b = path[i + 1]!, east = b.position[0] - a.position[0], north = b.position[1] - a.position[1];
      if (Math.hypot(east, north) > .001) {
        const difference = Math.atan2(-east, north) - raw;
        let turn = Math.atan2(Math.sin(difference), Math.cos(difference));
        if (Math.abs(Math.abs(turn) - Math.PI) < 1e-6) turn = turnSign * Math.PI;
        if (Math.abs(turn) > .001) turnSign = Math.sign(turn);
        raw += turn;
        target[i] = raw;
      } else target[i] = yaw[i]!;
      yaw[i + 1] = yaw[i]! + (target[i]! - yaw[i]!) * -Math.expm1(-Math.max(0, b.timeS - a.timeS) * HEADING_RESPONSE);
    }
    target[path.length - 1] = yaw[path.length - 1]!;
    headings = { yaw, target }; headingIndices.set(path, headings);
  }
  const index = waypointIndex(path, timeS), elapsed = Math.max(0, timeS - path[index]!.timeS);
  const yaw = headings.yaw[index]! + (headings.target[index]! - headings.yaw[index]!) * -Math.expm1(-elapsed * HEADING_RESPONSE);
  return new THREE.Vector3(-Math.sin(yaw), 0, -Math.cos(yaw));
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
export function dynamicRoutes(scenarios: DynamicScenario[], planner: string): OverviewRoute[] {
  return scenarios.map(s => {
    let cache = dynamicRouteCache.get(s);
    if (!cache) { cache = new Map(); dynamicRouteCache.set(s, cache); }
    const cached = cache.get(planner);
    if (cached) return cached;
    const run = s.runs.find(r => r.plannerId === planner);
    if (!run || run.status !== "success") throw new Error(`Missing successful ${planner} trace in ${s.id}`);
    const timedPath = run.executionTimedPath;
    if (!timedPath?.length) throw new Error(`Missing exact ${planner} trace in ${s.id}`);
    const route: OverviewRoute = { id: s.id, plannerId: planner, label: s.label, mission: s.mission, points: run.frames.at(-1)!.executedPath, timedPath,
      playbackKind: "reactive", plannedFrames: run.frames, cruiseSpeedMps: Number(run.parameters.cruiseSpeedMps),
      maxClimbRateMps: run.parameters.maxClimbRateMps,
      rerouteTimes: dynamicAvoidanceEvents(s, run).filter(event => event.kind === "reroute").map(event => event.timeS) };
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
  planTimes?: Float64Array;
  window?: ReturnType<typeof revealReplayWindow>; hold?: THREE.Group;
  lead?: Line2; leadWindow?: ReturnType<typeof revealReplayWindow>;
  ticks?: THREE.InstancedMesh<THREE.RingGeometry, THREE.MeshBasicMaterial>;
  preview?: boolean;
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
  private readonly tickMatrix = new THREE.Matrix4();
  private readonly tickScale = new THREE.Vector3();
  constructor() { this.group.name = "mission-route-overview"; this.group.userData.presentation = "result-preview"; }

  setRoutes(routes: OverviewRoute[], activeId: string, width: number, height: number): void {
    if (new Set(routes.map(r => r.id)).size !== routes.length) throw new Error("Duplicate overview mission IDs");
    if (routes.length === this.visuals.length && routes.every((route, index) => {
      const previous = this.visuals[index]!.route;
      return route.id === previous.id && route.plannerId === previous.plannerId && route.mission === previous.mission
        && route.playbackKind === previous.playbackKind && route.plannedFrames === previous.plannedFrames
        && route.cruiseSpeedMps === previous.cruiseSpeedMps && route.maxClimbRateMps === previous.maxClimbRateMps && route.waits === previous.waits
        && route.rerouteTimes === previous.rerouteTimes
        && (route.timedPath ? route.timedPath === previous.timedPath : route.points === previous.points);
    })) {
      this.setFocus(activeId);
      this.resize(width, height);
      return;
    }
    this.clear();
    let gateGeometry: THREE.BufferGeometry | undefined;
    let tickGeometry: THREE.RingGeometry | undefined;
    routes.forEach((route, index) => {
      if (route.points.length < 2) throw new Error(`Empty overview route: ${route.id}`);
      const color = routeColor(index), group = new THREE.Group();
      group.name = `mission-route-${route.id}`;
      group.userData = { missionId: route.id, plannerId: route.plannerId, sharedWorldId: route.mission?.sharedWorld?.id };
      const geometry = new LineGeometry();
      const reveal = route.timedPath && route.playbackKind;
      geometry.setPositions((reveal ? route.timedPath!.map(w => w.position) : route.points).flatMap(enuToThree));
      const material = new LineMaterial({ linewidth: 4, worldUnits: false, color, transparent: true,
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
      let lead: Line2 | undefined, leadWindow: RouteVisual["leadWindow"], ticks: RouteVisual["ticks"];
      const planTimes = route.plannedFrames
        ? new Float64Array(Math.max(2, ...route.plannedFrames.map(frame => frame.path.length))) : undefined;
      if (route.playbackKind) {
        const meaning = planMeaning(route);
        const pendingGeometry = new LineGeometry();
        if (!route.plannedFrames) pendingGeometry.setPositions(route.timedPath!.flatMap(w => enuToThree(w.position)));
        if (planTimes) {
          pendingGeometry.setPositions(new Float32Array(planTimes.length * 3));
          (pendingGeometry.getAttribute("instanceStart") as THREE.InterleavedBufferAttribute).data.setUsage(THREE.DynamicDrawUsage);
        }
        const pendingMaterial = new LineMaterial({ linewidth: meaning === "fixed" ? 3 : 2.6, worldUnits: false,
          color, dashed: meaning !== "fixed", dashSize: meaning === "recorded" ? 2 : meaning === "scheduled" ? 26 : 18,
          gapSize: meaning === "recorded" ? 18 : 20,
          transparent: true, opacity: meaning === "fixed" ? .68 : meaning === "recorded" ? .72 : .5,
          depthTest: true, depthWrite: false });
        pendingMaterial.resolution.copy(material.resolution);
        pending = new Line2(pendingGeometry, pendingMaterial);
        pending.name = `overview-plan-${route.id}`; pending.renderOrder = 1;
        pending.userData.meaning = meaning;
        pending.computeLineDistances();
        window = revealReplayWindow(pendingGeometry, planTimes ?? route.timedPath!.map(w => w.timeS), pendingMaterial);
        group.add(pending);
        if (meaning === "local" || meaning === "scheduled") {
          const leadMaterial = new LineMaterial({ color, linewidth: 4.8, worldUnits: false,
            dashed: true, dashSize: pendingMaterial.dashSize, gapSize: pendingMaterial.gapSize,
            transparent: true, opacity: 1, depthTest: true, depthWrite: false, alphaToCoverage: true });
          leadMaterial.resolution.copy(material.resolution);
          lead = new Line2(pendingGeometry, leadMaterial);
          lead.name = `overview-lookahead-${route.id}`; lead.renderOrder = 2;
          leadWindow = revealReplayWindow(pendingGeometry, planTimes ?? route.timedPath!.map(w => w.timeS), leadMaterial);
          group.add(lead);
          if (meaning === "scheduled") {
            ticks = new THREE.InstancedMesh(tickGeometry ??= new THREE.RingGeometry(.58, 1, 16),
              new THREE.MeshBasicMaterial({ color, transparent: true, opacity: .95, side: THREE.DoubleSide,
                depthTest: true, depthWrite: false }), 4);
            ticks.name = `overview-time-marks-${route.id}`; ticks.visible = false;
            ticks.instanceMatrix.setUsage(THREE.DynamicDrawUsage); ticks.frustumCulled = false;
            ticks.renderOrder = 4; group.add(ticks);
          }
        }
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
        markers = new THREE.InstancedMesh(gateGeometry ??= missionGateGeometry(), missionGateMaterial(color), tasks.length);
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
      // Most routes never hover. Keep their hidden geometry and per-frame phase
      // lookup out of the scene; only positive-duration recorded holds need it.
      const hold = route.timedPath?.some((waypoint, i, path) => {
        if (!i || waypoint.timeS <= path[i - 1]!.timeS) return false;
        const a = path[i - 1]!.position, b = waypoint.position;
        return Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) < 1e-8;
      }) ? new THREE.Group() : undefined;
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
      this.visuals.push({ route, group, line, halo, vehicle, clock, pending, window, planTimes, lead, leadWindow, ticks,
        hold, tasks: markers, ...endpoints });
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
    const selected = visual.route.id === this.selectedId, primary = this.highlighted(visual);
    const meaning = planMeaning(visual.route), resultPreview = visual.preview && meaning === "local";
    visual.line.material.linewidth = resultPreview ? primary ? 1.9 : 1.1 : primary ? selected ? 4.4 : 3.8 : 1.7;
    visual.line.material.opacity = resultPreview ? primary ? .24 : .17 : primary ? 1 : .34;
    visual.line.material.alphaToCoverage = primary && !resultPreview;
    visual.halo.visible = primary && !resultPreview;
    visual.halo.material.linewidth = selected ? 6 : 5.2;
    if (visual.pending) {
      visual.pending.material.linewidth = primary ? meaning === "fixed" ? 2.6 : 2.2 : 1.2;
      visual.pending.material.opacity = primary ? meaning === "fixed" || meaning === "recorded" ? .72 : .45 : .28;
    }
    visual.group.userData.selected = selected;
    this.updateLookahead(visual);
  }
  private highlighted(visual: RouteVisual): boolean {
    return this.selectedId ? visual.route.id === this.selectedId : Boolean(visual.group.userData.focused);
  }
  private updateLookahead(visual: RouteVisual): void {
    const { lead, leadWindow, route } = visual;
    if (!lead || !leadWindow) return;
    lead.visible = this.highlighted(visual) && Boolean(visual.pending?.visible) && leadWindow.end.value > leadWindow.start.value;
    // The bright near segment and faint far context occupy disjoint windows.
    if (visual.window) visual.window.start.value = lead.visible ? leadWindow.end.value : leadWindow.start.value;
    let emphasis = 0;
    for (let i = (route.rerouteTimes?.length ?? 0) - 1; i >= 0; i--) {
      const age = this.timeS - route.rerouteTimes![i]!;
      if (age >= 0) { emphasis = Math.max(0, 1 - age / 8); break; }
    }
    lead.material.linewidth = (visual.group.userData.selected ? 4.2 : 3.6) + emphasis;
    if (visual.ticks) visual.ticks.visible = lead.visible;
  }
  pick(camera: DisplayCamera, x: number, y: number, width: number, height: number,
    occluded: (point: THREE.Vector3) => boolean, radius = 7): RoutePick | null {
    return pickVisibleRoutes(this.visuals.map(v => ({ id: v.route.id, lines: [v.line, v.pending, v.lead] })), camera, x, y, width, height, occluded, radius);
  }

  setTime(timeS: number): void {
    this.timeS = timeS;
    const preview = !this.playing && timeS <= 0;
    this.group.userData.presentation = preview ? "result-preview" : this.playing ? "playback" : "paused";
    for (const visual of this.visuals) {
      const { route, vehicle, clock, pending, window, hold, lead, leadWindow } = visual;
      const meaning = planMeaning(route), resultPreview = preview && meaning === "local";
      // A reactive page can preview its complete recorded result, but that faint
      // context is distinct from the current plan available to the aircraft.
      if (visual.preview !== preview) {
        visual.preview = preview;
        this.styleSelection(visual);
      }
      if (clock) {
        clock.value = resultPreview ? route.timedPath!.at(-1)!.timeS : timeS;
        // Skip the unreached suffix before vertex shading. Keep a boundary segment
        // for Float32 clock rounding; the existing shader still trims it exactly.
        visual.line.geometry.instanceCount = Math.min(route.timedPath!.length - 1, waypointIndex(route.timedPath!, clock.value) + 2);
      }
      if (pending) pending.visible = timeS < route.timedPath!.at(-1)!.timeS;
      if (window) { window.start.value = timeS; window.end.value = route.timedPath!.at(-1)!.timeS; }
      if (pending && route.plannedFrames) {
        const frame = route.plannedFrames[waypointIndex(route.plannedFrames, timeS)]!;
        pending.visible &&= frame.path.length >= 2;
        if (pending.visible && frame !== visual.planFrame) {
          this.updatePlanPositions(pending.geometry, frame.path);
          let clock = frame.timeS;
          const times = visual.planTimes!;
          for (let i = 0; i < frame.path.length; i++) {
            if (i) clock += flightDuration(frame.path[i - 1]!, frame.path[i]!, route.cruiseSpeedMps || 15, route.maxClimbRateMps);
            times[i] = clock;
          }
          times.fill(clock, frame.path.length);
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
      if (lead && leadWindow && visual.window) {
        leadWindow.start.value = visual.window.start.value;
        leadWindow.end.value = Math.min(visual.window.end.value, leadWindow.start.value + (meaning === "scheduled" ? 20 : 30));
        this.updateLookahead(visual);
      }
      if (!vehicle || !route.timedPath) continue;
      // Finished missions remain at their real goal; no looping or invented continuation.
      const position = timedPosition(route.timedPath, timeS, this.position);
      vehicle.position.set(position[0], position[2], -position[1]);
      orientDrone(vehicle, displayHeading(route.timedPath, timeS));
      setDroneRotorTime(vehicle, timeS);
      if (hold) {
        hold.visible = !preview && flightPhase(route, timeS).kind === "waiting";
        hold.position.copy(vehicle.position);
      }
    }
  }

  /** Fixed mission capacity keeps positions, dash distances and clocks on the GPU. */
  private updatePlanPositions(geometry: LineGeometry, points: readonly RoutePoint[]): void {
    const start = geometry.getAttribute("instanceStart") as THREE.InterleavedBufferAttribute;
    const end = geometry.getAttribute("instanceEnd");
    const distanceStart = geometry.getAttribute("instanceDistanceStart") as THREE.InterleavedBufferAttribute;
    const distanceEnd = geometry.getAttribute("instanceDistanceEnd");
    let distance = 0;
    for (let i = 0; i < start.count; i++) {
      const a = points[Math.min(i, points.length - 1)]!, b = points[Math.min(i + 1, points.length - 1)]!;
      start.setXYZ(i, a[0], a[2], -a[1]); end.setXYZ(i, b[0], b[2], -b[1]);
      const dx = end.getX(i) - start.getX(i), dy = end.getY(i) - start.getY(i), dz = end.getZ(i) - start.getZ(i);
      distanceStart.setX(i, distance);
      distanceEnd.setX(i, distance + Math.sqrt(dx * dx + dy * dy + dz * dz));
      distance = distanceEnd.getX(i);
    }
    geometry.instanceCount = points.length - 1;
    start.data.needsUpdate = true;
    distanceStart.data.needsUpdate = true;
    geometry.computeBoundingBox(); geometry.computeBoundingSphere();
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
      visual.lead?.material.resolution.set(width, height);
    }
  }

  vehicle(id: string): THREE.Group | undefined { return this.visuals.find(v => v.route.id === id)?.vehicle; }
  timedPath(id: string): readonly RouteWaypoint[] | undefined { return this.visuals.find(v => v.route.id === id)?.route.timedPath; }
  heading(id: string, timeS: number): THREE.Vector3 {
    return displayHeading(this.visuals.find(v => v.route.id === id)?.route.timedPath ?? [], timeS);
  }
  updateSymbols(camera: DisplayCamera, height: number, followedId: string | null = null): void {
    this.screenUp.set(0, 1, 0).applyQuaternion(camera.quaternion);
    for (const { vehicle, group, route, hold, tasks, start, goal, ticks, pending, lead } of this.visuals) if (vehicle) {
      const followed = route.id === followedId;
      vehicle.visible = true;
      sizeDrone(vehicle, camera, height, followed ? Math.min(128, Math.max(24, height * 0.18)) : group.userData.focused ? 32 : 24,
        followed ? 0 : 0.8);
      // In chase views use the street ahead, rather than the nearby aircraft,
      // as the dash scale reference; distant curves then retain readable gaps.
      if (camera instanceof THREE.PerspectiveCamera) this.point.set(0, 0, -80).applyQuaternion(camera.quaternion).add(camera.position);
      else this.point.copy(vehicle.position);
      const dashScale = 1 / Math.max(.01, metresPerPixelAt(camera, this.point, height));
      if (pending?.material.dashed) pending.material.dashScale = dashScale;
      if (lead) lead.material.dashScale = dashScale;
      if (hold?.visible) {
        const scale = metresPerPixelAt(camera, vehicle.position, height) * 11;
        hold.scale.setScalar(scale); hold.quaternion.copy(camera.quaternion);
        hold.position.copy(vehicle.position).addScaledVector(this.screenUp, scale * 2.7);
      }
      if (tasks) tasks.visible = !group.userData.focused && route.id !== followedId;
      if (ticks?.visible) {
        let count = 0;
        for (let ahead = 5; ahead <= 20; ahead += 5) {
          if (this.timeS + ahead >= route.timedPath!.at(-1)!.timeS) break;
          const position = timedPosition(route.timedPath!, this.timeS + ahead, this.position);
          this.point.set(position[0], position[2], -position[1]);
          this.tickScale.setScalar(metresPerPixelAt(camera, this.point, height) * 3.6);
          this.tickMatrix.compose(this.point, camera.quaternion, this.tickScale);
          ticks.setMatrixAt(count++, this.tickMatrix);
        }
        ticks.count = count; ticks.instanceMatrix.needsUpdate = true;
      }
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
