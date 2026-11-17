import * as THREE from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { describe, expect, it, vi } from "vitest";
import { auditMissionTaskVisits } from "../src/city-schema";
import { validateBundle } from "../src/static-validation";
import { validateDynamicBundle } from "../src/dynamic-validation";
import { validatePredictiveBundle } from "../src/predictive-validation";
import { RetainedCity } from "../src/retained-city";
import {
  RouteOverview, dynamicRoutes, overviewDuration, predictiveRoutes,
  displayHeading, flightHeading, routeColor, routeColorCSS, staticRoutes, timedPosition, waypointIndex,
  type OverviewRoute, type RouteWaypoint,
} from "../src/route-overview";
import type { DynamicRun } from "../src/dynamic-schema";
import { readStudyData } from '../scripts/study-reader.mjs';

const read = (file: string) => readStudyData(new URL(`../public/${file}`, import.meta.url));
const staticBundle = validateBundle(await read("demo-data.json"));
const dynamicBundle = validateDynamicBundle(await read("dynamic-data.json"));
const predictiveBundle = validatePredictiveBundle(await read("predictive-data.json"));

function fixtureRoutes(): OverviewRoute[] {
  return Array.from({ length: 4 }, (_, index) => {
    const path: RouteWaypoint[] = [
      { timeS: 0, position: [index * 10, 0, 30] },
      { timeS: 10 + index * 5, position: [index * 10, 100, 50] },
    ];
    return { id: `route-${index}`, label: `Route ${index}`, points: path.map(w => w.position), timedPath: path };
  });
}

describe("continuous display attitude", () => {
  it("turns continuously at a horizontal reversal and gives the same pose after seeking", () => {
    const path: RouteWaypoint[] = [
      { timeS: 0, position: [0, 0, 20] }, { timeS: 4, position: [20, 0, 20] },
      { timeS: 8, position: [0, 0, 30] },
    ];
    const original = structuredClone(path), expected = displayHeading(path, 4.4).toArray();
    let previous = displayHeading(path, 3);
    for (let time = 3 + 4 / 60; time <= 5; time += 4 / 60) {
      const next = displayHeading(path, time);
      expect(previous.angleTo(next) * 180 / Math.PI).toBeLessThan(25);
      previous = next;
    }
    expect(displayHeading(path, 4 - 1e-7).angleTo(displayHeading(path, 4 + 1e-7))).toBeLessThan(1e-5);
    displayHeading(path, 8); displayHeading(path, 0);
    expect(displayHeading(path, 4.4).toArray()).toEqual(expected);
    expect(path).toEqual(original);
  });
  it("retains the local bearing through a slow climb instead of reviving a distant chord", () => {
    const path: RouteWaypoint[] = [
      { timeS: 0, position: [0, 0, 20] }, { timeS: 10, position: [0, 10, 20] },
      { timeS: 11, position: [1, 10, 20] }, { timeS: 12, position: [1, 10.1, 23] },
      { timeS: 13, position: [1, 10.2, 26] }, { timeS: 15, position: [1, 10.2, 26] },
      { timeS: 17, position: [1, 10.2, 30] },
    ];
    expect(displayHeading(path, 12.2).dot(new THREE.Vector3(0, 0, -1))).toBeGreaterThan(.97);
    expect(displayHeading(path, 14).toArray()).toEqual(displayHeading(path, 13).toArray());
    expect(displayHeading(path, 16).toArray()).toEqual(displayHeading(path, 13).toArray());
  });
  it("does not rescan a dense near-vertical trace on every rendered frame", () => {
    const points: RouteWaypoint[] = [{ timeS: 0, position: [0, 0, 20] }, { timeS: 1, position: [10, 0, 20] }];
    for (let i = 1; i <= 8192; i++) points.push({ timeS: 1 + i * .01, position: [10 + i * .00001, 0, 20 + i * .001] });
    let reads = 0;
    const path = new Proxy(points, { get(target, key, receiver) {
      if (typeof key === "string" && /^\d+$/.test(key)) reads++;
      return Reflect.get(target, key, receiver);
    } });
    displayHeading(path, 80); reads = 0;
    const heading = displayHeading(path, 81);
    expect(reads).toBeLessThan(100);
    expect(heading.x).toBeCloseTo(1); expect(heading.y).toBe(0);
  });
});

describe("shared mission clock", () => {
  const path: RouteWaypoint[] = [
    { timeS: 0, position: [0, 0, 30] }, { timeS: 2, position: [10, 0, 40] },
    { timeS: 8, position: [10, 0, 40] }, { timeS: 10, position: [10, 10, 50] },
  ];
  it("interpolates real movement while preserving service waits and altitude", () => {
    expect(timedPosition(path, 1)).toEqual([5, 0, 35]);
    expect(timedPosition(path, 6)).toEqual([10, 0, 40]);
    expect(timedPosition(path, 9)).toEqual([10, 5, 45]);
  });
  it("clamps individual starts and goals, and uses the longest mission duration", () => {
    expect(timedPosition(path, -20)).toEqual(path[0]!.position);
    expect(timedPosition(path, 100)).toEqual(path.at(-1)!.position);
    expect(overviewDuration(fixtureRoutes())).toBe(25);
  });
  it("can sample into a retained scratch buffer without modifying native coordinates", () => {
    const original = structuredClone(path), scratch: [number, number, number] = [0, 0, 0];
    for (const time of [-20, 0, 1, 2, 6, 9, 10, 100]) {
      expect(timedPosition(path, time, scratch)).toBe(scratch);
      expect(scratch).toEqual(timedPosition(path, time));
    }
    expect(path).toEqual(original);
  });
  it("uses a binary lookup with exact boundary, duplicate and empty behavior", () => {
    expect(waypointIndex(path, 2)).toBe(1);
    expect(waypointIndex([{ timeS: 0 }, { timeS: 2 }, { timeS: 2 }, { timeS: 4 }], 2)).toBe(2);
    expect(waypointIndex([], 10)).toBe(-1);
    expect(() => timedPosition([], 10)).toThrow("empty");
  });
  it("derives forward headings from records and keeps arrival heading during service stops", () => {
    expect(flightHeading(path, 0).toArray()).toEqual([1, 0, 0]);
    expect(flightHeading(path, 6).toArray()).toEqual([1, 0, 0]);
    expect(flightHeading(path, 9).toArray()).toEqual([0, 0, -1]);
    expect(flightHeading(path, 100).toArray()).toEqual([0, 0, -1]);
    expect(flightHeading(path, 6).toArray()).toEqual([1, 0, 0]); // Independent of seek history.
  });
  it("handles vertical starts, climbs and duplicate knots without pitching or losing heading", () => {
    const vertical: RouteWaypoint[] = [
      { timeS: 0, position: [0, 0, 20] }, { timeS: 2, position: [0, 0, 40] },
      { timeS: 4, position: [10, 0, 40] }, { timeS: 4, position: [10, 0, 40] },
      { timeS: 6, position: [10, 0, 60] }, { timeS: 8, position: [10, 10, 60] },
    ];
    expect(flightHeading(vertical, 1).toArray()).toEqual([1, 0, 0]);
    expect(flightHeading(vertical, 5).toArray()).toEqual([1, 0, 0]);
    expect(flightHeading(vertical, 7).toArray()).toEqual([0, 0, -1]);
    expect(flightHeading([], 0).toArray()).toEqual([0, 0, -1]);
    expect(flightHeading(vertical.slice(0, 2), 1).toArray()).toEqual([0, 0, -1]);
  });
});

describe("four routes, one world", () => {
  it("reuses predictive final-flight arrays for an unchanged planner without mixing algorithms", () => {
    const planner = predictiveBundle.planners[0]!.id;
    const first = predictiveRoutes(predictiveBundle.scenarios, planner);
    const second = predictiveRoutes(predictiveBundle.scenarios, planner);
    first.forEach((route, index) => expect(second[index]).toBe(route));
    const other = predictiveRoutes(predictiveBundle.scenarios, predictiveBundle.planners[1]!.id);
    first.forEach((route, index) => expect(other[index]).not.toBe(route));
  });
  it("creates exactly four depth-tested trajectories with separate mission colors", () => {
    const overview = new RouteOverview(), routes = fixtureRoutes();
    overview.setRoutes(routes, routes[0]!.id, 800, 600);
    expect(overview.group.children).toHaveLength(4);
    const lines: Line2[] = [];
    overview.group.traverse(o => { if (o instanceof Line2 && o.name.startsWith("overview-trajectory-")) lines.push(o); });
    expect(lines).toHaveLength(4);
    lines.forEach((line, index) => {
      expect(line.material.color.getHex()).toBe(routeColor(index));
      expect(line.material.depthTest).toBe(true);
      expect(line.material.depthWrite).toBe(false);
      expect(line.material.worldUnits).toBe(false);
      expect(line.material.linewidth).toBeGreaterThan(0);
      expect(line.material.opacity).toBeGreaterThan(0);
      expect(line.material.resolution.toArray()).toEqual([800, 600]);
    });
    for (const line of lines.slice(1)) {
      expect(lines[0]!.material.linewidth).toBeGreaterThan(line.material.linewidth);
      expect(lines[0]!.material.opacity).toBeGreaterThan(line.material.opacity);
    }
    expect(routes.map(route => overview.group.getObjectByName(`overview-outline-${route.id}`)!.visible))
      .toEqual([true, false, false, false]);
    expect(new Set(lines.map(l => l.material.color.getHex())).size).toBe(4);
    expect(routeColorCSS(0)).toBe("#009b87");
    overview.dispose();
  });
  it("does not recreate or dispose trajectory GPU buffers on focus or 100 time samples", () => {
    const overview = new RouteOverview(), routes = fixtureRoutes();
    routes[0]!.playbackKind = "reactive";
    routes[0]!.plannedFrames = Array.from({ length: 40 }, (_, i) => ({ timeS: i / 4,
      path: i % 2 ? routes[0]!.points : [routes[0]!.points[0], [0, 40, 30], routes[0]!.points[1]],
    })) as DynamicRun["frames"];
    overview.setRoutes(routes, routes[0]!.id, 800, 600);
    const line = overview.group.getObjectByName(`overview-trajectory-${routes[0]!.id}`) as Line2;
    const pending = overview.group.getObjectByName(`overview-plan-${routes[0]!.id}`) as Line2;
    const lead = overview.group.getObjectByName(`overview-lookahead-${routes[0]!.id}`) as Line2;
    expect(lead.geometry).toBe(pending.geometry);
    const disposePlan = vi.spyOn(pending.geometry, "dispose"), planUpload = vi.spyOn(pending.geometry, "setPositions");
    const planPositions = pending.geometry.getAttribute("instanceStart");
    const planBuffers = ["instanceEnd", "instanceDistanceStart", "instanceDistanceEnd", "instanceTimeStart", "instanceTimeEnd"]
      .map(name => [name, pending.geometry.getAttribute(name)] as const);
    const dispose = vi.spyOn(line.geometry, "dispose"), upload = vi.spyOn(line.geometry, "setPositions");
    for (let i = 0; i < 100; i++) { overview.setTime(i / 4); overview.setFocus(routes[i % 4]!.id); }
    overview.setRoutes(routes.map(route => ({ ...route })), routes[2]!.id, 600, 400);
    expect(overview.group.getObjectByName(line.name) === line).toBe(true);
    expect(dispose).not.toHaveBeenCalled();
    expect(upload).not.toHaveBeenCalled(); expect(planUpload).not.toHaveBeenCalled();
    expect(pending.geometry.getAttribute("instanceStart")).toBe(planPositions);
    for (const [name, buffer] of planBuffers) expect(pending.geometry.getAttribute(name)).toBe(buffer);
    expect(overview.group.children.every(group => group.visible)).toBe(true);
    overview.dispose();
    expect(dispose).toHaveBeenCalledOnce(); expect(disposePlan).toHaveBeenCalledOnce();
  });
  it("separates reactive result context from local knowledge and only emphasizes causal reroutes", () => {
    const timedPath: RouteWaypoint[] = [
      { timeS: 0, position: [0, 0, 30] }, { timeS: 10, position: [0, 0, 30] },
      { timeS: 20, position: [100, 0, 30] }, { timeS: 60, position: [500, 0, 30] },
    ];
    const local: OverviewRoute = { id: "local", label: "Local", points: timedPath.map(w => w.position), timedPath,
      playbackKind: "reactive", cruiseSpeedMps: 10, rerouteTimes: [20], plannedFrames: [
        { timeS: 0, path: [[0, 0, 30], [500, 0, 30]], replanned: true, replanReason: "initial" },
        { timeS: 20, path: [[100, 0, 30], [100, 200, 30], [500, 0, 30]], replanned: true, replanReason: "safety-gate" },
        { timeS: 30, path: [[200, 0, 30], [500, 0, 30]], replanned: true, replanReason: "scheduled" },
      ] as NonNullable<OverviewRoute["plannedFrames"]> };
    const original = structuredClone(timedPath), overview = new RouteOverview();
    overview.setRoutes([local], local.id, 800, 600);
    const history = overview.group.getObjectByName("overview-trajectory-local") as Line2;
    const halo = overview.group.getObjectByName("overview-outline-local")!;
    const pending = overview.group.getObjectByName("overview-plan-local") as Line2;
    const lead = overview.group.getObjectByName("overview-lookahead-local") as Line2;
    expect(history.material.uniforms.replayTime!.value).toBe(60);
    expect(history.material.opacity).toBeLessThan(.5); expect(halo.visible).toBe(false);
    expect(pending.visible && lead.visible).toBe(true); expect(lead.geometry).toBe(pending.geometry);
    const positions = pending.geometry.getAttribute("instanceStart");
    overview.setPlaying(true); overview.setTime(5);
    expect(history.material.uniforms.replayTime!.value).toBe(5);
    expect(history.material.opacity).toBe(1); expect(halo.visible).toBe(true);
    // A genuine hold does not consume the spatial plan's 30-second lead.
    expect(lead.material.uniforms.windowStart!.value).toBe(0);
    expect(lead.material.uniforms.windowEnd!.value).toBe(30);
    const normalWidth = lead.material.linewidth;
    overview.setTime(20); const rerouteWidth = lead.material.linewidth;
    expect(rerouteWidth).toBeGreaterThan(normalWidth);
    expect(lead.material.uniforms.windowStart!.value).toBe(20);
    expect(lead.material.uniforms.windowEnd!.value).toBe(50);
    overview.setTime(24); expect(lead.material.linewidth).toBeLessThan(rerouteWidth);
    expect(lead.material.linewidth).toBeGreaterThan(normalWidth);
    overview.setTime(28); expect(lead.material.linewidth).toBe(normalWidth);
    overview.setTime(30); expect(lead.material.linewidth).toBe(normalWidth); // Routine replanning is not an obstacle event.
    overview.setTime(5); expect(lead.material.uniforms.windowStart!.value).toBe(0);
    expect(pending.geometry.getAttribute("instanceStart")).toBe(positions);
    overview.setTime(60); expect(pending.visible || lead.visible).toBe(false);
    overview.setPlaying(false); overview.setTime(0);
    expect(history.material.opacity).toBeLessThan(.5); expect(halo.visible).toBe(false);
    expect(pending.visible && lead.visible).toBe(true); expect(timedPath).toEqual(original);
    overview.dispose();
  });
  it("places scheduled time marks at real future positions and retains them through selection and rewind", () => {
    const timedPath: RouteWaypoint[] = [
      { timeS: 0, position: [0, 0, 30] }, { timeS: 5, position: [0, 0, 30] },
      { timeS: 5, position: [0, 0, 30] }, { timeS: 15, position: [100, 0, 30] },
      { timeS: 30, position: [100, 150, 45] }, { timeS: 50, position: [200, 150, 45] },
    ];
    const route: OverviewRoute = { id: "scheduled", label: "Scheduled", playbackKind: "predictive",
      points: timedPath.map(w => w.position), timedPath };
    const original = structuredClone(timedPath), overview = new RouteOverview(), other = { ...route, id: "other" };
    overview.setRoutes([route, other], route.id, 800, 600);
    const marks = overview.group.getObjectByName("overview-time-marks-scheduled") as THREE.InstancedMesh;
    const otherMarks = overview.group.getObjectByName("overview-time-marks-other") as THREE.InstancedMesh;
    const lead = overview.group.getObjectByName("overview-lookahead-scheduled") as Line2;
    const matrixBuffer = marks.instanceMatrix.array, geometry = marks.geometry;
    expect((marks.material as THREE.MeshBasicMaterial).depthTest).toBe(true);
    expect((marks.material as THREE.MeshBasicMaterial).depthWrite).toBe(false);
    const disposed = vi.spyOn(geometry, "dispose"), matrix = new THREE.Matrix4(), position = new THREE.Vector3();
    const camera = new THREE.OrthographicCamera(-1000, 1000, 750, -750, -10000, 10000);
    camera.position.set(600, 800, 1000); camera.lookAt(100, 30, -50); camera.updateMatrixWorld(true);
    expect(otherMarks.geometry).toBe(geometry); expect(otherMarks.visible).toBe(false);
    for (const timeS of [0, 12, 32, 46, 50, 0]) {
      overview.setTime(timeS); overview.updateSymbols(camera, 600);
      expect(lead.material.uniforms.windowStart!.value).toBe(timeS);
      expect(lead.material.uniforms.windowEnd!.value).toBe(Math.min(50, timeS + 20));
      expect(marks.visible).toBe(timeS < 50);
      if (marks.visible) {
        const aheadTimes = [5, 10, 15, 20].filter(ahead => timeS + ahead < 50);
        expect(marks.count).toBe(aheadTimes.length);
        aheadTimes.forEach((ahead, index) => {
          const point = timedPosition(timedPath, timeS + ahead);
          marks.getMatrixAt(index, matrix); position.setFromMatrixPosition(matrix);
          expect(position.x).toBeCloseTo(point[0], 4);
          expect(position.y).toBeCloseTo(point[2], 4);
          expect(position.z).toBeCloseTo(-point[1], 4);
        });
      }
      expect(marks.instanceMatrix.array).toBe(matrixBuffer);
    }
    overview.setSelection(other.id); overview.updateSymbols(camera, 600);
    expect(marks.visible).toBe(false); expect(otherMarks.visible).toBe(true);
    overview.setSelection(null); overview.updateSymbols(camera, 600);
    expect(marks.visible).toBe(true); expect(otherMarks.visible).toBe(false);
    expect(timedPath).toEqual(original);
    overview.dispose(); expect(disposed).toHaveBeenCalledOnce();
  });
  it("keeps the near future clickable and restores plan context when changing selection while paused", () => {
    const routes: OverviewRoute[] = [0, 200].map((north, index) => {
      const timedPath: RouteWaypoint[] = [
        { timeS: 0, position: [0, north, 30] }, { timeS: 100, position: [1000, north, 30] },
      ];
      return { id: `scheduled-${index}`, label: `Scheduled ${index}`, playbackKind: "predictive",
        points: timedPath.map(point => point.position), timedPath };
    });
    const overview = new RouteOverview();
    overview.setRoutes(routes, routes[0]!.id, 800, 600);
    overview.setPlaying(true); overview.setTime(10); overview.setPlaying(false);
    const plans = routes.map(route => overview.group.getObjectByName(`overview-plan-${route.id}`) as Line2);
    const leads = routes.map(route => overview.group.getObjectByName(`overview-lookahead-${route.id}`) as Line2);
    const starts = () => plans.map(plan => plan.material.uniforms.windowStart!.value);
    const camera = new THREE.OrthographicCamera(-100, 1100, 300, -300, .1, 10000);
    camera.position.set(0, 1000, 0); camera.up.set(0, 0, -1); camera.lookAt(0, 0, 0); camera.updateMatrixWorld(true);
    const pickNear = (index: number) => {
      const screen = new THREE.Vector3(150, 30, -index * 200).project(camera);
      return overview.pick(camera, (screen.x + 1) * 400, (1 - screen.y) * 300, 800, 600, () => false)?.id;
    };
    expect(starts()).toEqual([30, 10]);
    expect(leads.map(lead => lead.visible)).toEqual([true, false]);
    expect(pickNear(0)).toBe(routes[0]!.id); // History ends at 100 m; context starts at 300 m.
    overview.setSelection(routes[1]!.id);
    expect(starts()).toEqual([10, 30]);
    expect(leads.map(lead => lead.visible)).toEqual([false, true]);
    expect(pickNear(0)).toBe(routes[0]!.id); expect(pickNear(1)).toBe(routes[1]!.id);
    overview.setSelection(null);
    expect(starts()).toEqual([30, 10]);
    expect(leads.map(lead => lead.visible)).toEqual([true, false]);
    expect(pickNear(0)).toBe(routes[0]!.id);
    overview.dispose();
  });
  it("moves all four vehicles at once and keeps finished ones at their own goals", () => {
    const overview = new RouteOverview(), routes = fixtureRoutes();
    overview.setRoutes(routes, routes[0]!.id, 800, 600);
    overview.setTime(20);
    routes.forEach(route => {
      const vehicle = overview.group.getObjectByName(`overview-vehicle-${route.id}`)!;
      const p = timedPosition(route.timedPath!, 20);
      expect(vehicle.position.toArray()).toEqual([p[0], p[2], -p[1]]);
    });
    overview.setTime(0);
    expect(overview.group.getObjectByName("overview-vehicle-route-0")!.position.toArray()).toEqual([0, 30, -0]);
    overview.dispose();
  });
  it("allocates hold symbols only for real timed holds and preserves their replay boundaries", () => {
    const overview = new RouteOverview(), routes = fixtureRoutes();
    routes[1]!.timedPath!.splice(1, 0, { timeS: 0, position: routes[1]!.points[0]! });
    routes[2]!.playbackKind = "reactive";
    routes[2]!.timedPath!.splice(1, 0, { timeS: 5, position: routes[2]!.points[0]! });
    routes[3]!.playbackKind = "predictive";
    routes[3]!.timedPath!.splice(1, 0, { timeS: 5, position: [30 + 5e-9, 0, 30] });
    overview.setRoutes(routes, routes[0]!.id, 800, 600);
    for (const route of routes.slice(0, 2)) expect(overview.group.getObjectByName(`overview-hold-${route.id}`)).toBeUndefined();
    const holds = routes.slice(2).map(route => overview.group.getObjectByName(`overview-hold-${route.id}`)!);
    expect(holds.every(hold => hold && !hold.visible)).toBe(true);
    overview.setPlaying(true);
    for (const [timeS, visible] of [[-1, false], [0, true], [2.5, true], [5, false], [25, false]] as const) {
      overview.setTime(timeS);
      expect(holds.map(hold => hold.visible)).toEqual([visible, visible]);
    }
    overview.dispose();
  });
  it("keeps the followed aircraft visible and prominent, then restores its overview symbol", () => {
    const overview = new RouteOverview(), routes = fixtureRoutes();
    overview.setRoutes(routes, routes[0]!.id, 800, 600);
    const camera = new THREE.OrthographicCamera(-160, 160, 80, -80);
    const followed = overview.vehicle(routes[2]!.id)!;
    const position = followed.position.clone();
    overview.updateSymbols(camera, 600);
    const normal = followed.scale.x;
    overview.updateSymbols(camera, 600, routes[2]!.id);
    expect(followed.visible).toBe(true);
    expect(followed.scale.x / normal).toBeCloseTo(108 / 24);
    expect(overview.vehicle(routes[0]!.id)!.visible).toBe(true);
    expect(overview.vehicle(routes[1]!.id)!.visible).toBe(true);
    overview.updateSymbols(camera, 600, routes[1]!.id);
    expect(followed.visible).toBe(true);
    expect(overview.vehicle(routes[1]!.id)!.visible).toBe(true);
    expect(followed.scale.x).toBe(normal);
    expect(followed.position.distanceTo(position)).toBe(0);
    overview.updateSymbols(camera, 600, null);
    expect(followed.scale.x).toBe(normal);
    expect(routes.every(route => overview.vehicle(route.id)!.visible)).toBe(true);
    overview.dispose();
  });
  it("rejects duplicate missions instead of silently displaying the wrong route count", () => {
    expect(() => new RouteOverview().setRoutes([fixtureRoutes()[0]!, fixtureRoutes()[0]!], "route-0", 800, 600)).toThrow("Duplicate");
  });
  it("replaces and releases old trajectories when the planner really changes", () => {
    const overview = new RouteOverview(), routes = fixtureRoutes();
    overview.setRoutes(routes, routes[0]!.id, 800, 600);
    const old = overview.group.getObjectByName("overview-trajectory-route-0") as Line2;
    const disposed = vi.spyOn(old.geometry, "dispose");
    overview.setRoutes(fixtureRoutes(), "route-0", 800, 600);
    expect(disposed).toHaveBeenCalledOnce();
    expect(overview.group.getObjectByName(old.name) === old).toBe(false);
    overview.dispose();
  });
});

describe("real eight-route records", () => {
  it("loads every selected algorithm's eight complete results, including predictive-page reactive baselines", () => {
    const cohorts = [
      staticBundle.planners.map(planner => staticRoutes(staticBundle.scenarios, planner.id)),
      dynamicBundle.planners.map(planner => dynamicRoutes(dynamicBundle.scenarios, planner.id)),
      predictiveBundle.planners.map(planner => predictiveRoutes(predictiveBundle.scenarios, planner.id)),
    ];
    let flights = 0;
    for (const choices of cohorts) {
      const overview = new RouteOverview();
      for (const [choiceIndex, routes] of [...choices, choices[0]!].entries()) {
        overview.setPlaying(false); overview.setTime(0); overview.setRoutes(routes, routes[0]!.id, 800, 600);
        expect(overview.group.children).toHaveLength(8);
        for (const route of routes) {
          const group = overview.group.getObjectByName(`mission-route-${route.id}`)!;
          const line = group.getObjectByName(`overview-trajectory-${route.id}`) as Line2;
          expect(group.userData.plannerId).toBe(route.plannerId);
          expect(line.visible).toBe(true); expect(line.geometry.getAttribute("instanceStart").count).toBeGreaterThan(1);
          if (line.material.uniforms.replayTime) expect(line.material.uniforms.replayTime.value)
            .toBe(route.plannedFrames ? route.timedPath!.at(-1)!.timeS : 0);
          const pending = group.getObjectByName(`overview-plan-${route.id}`) as Line2;
          expect(pending.visible).toBe(true);
          expect(line.material.opacity).toBeGreaterThan(0);
          expect(line.material.opacity).toBeLessThanOrEqual(1);
          if (route.plannedFrames) expect(line.material.opacity).toBeLessThan(.5);
          if (route.id !== routes[0]!.id) {
            const primary = overview.group.getObjectByName(`overview-trajectory-${routes[0]!.id}`) as Line2;
            expect(line.material.opacity).toBeLessThan(primary.material.opacity);
          }
          const first = line.geometry.getAttribute("instanceStart");
          expect(first.getX(0)).toBeCloseTo(route.points[0]![0], 3);
          expect(first.getY(0)).toBeCloseTo(route.points[0]![2], 3);
          expect(first.getZ(0)).toBeCloseTo(-route.points[0]![1], 3);
          if (choiceIndex < choices.length) flights++;
        }
      }
      overview.dispose();
    }
    expect(flights).toBe(80);
  });
  it("uses the same rounded racing gates for original and added missions on every page", () => {
    const cohorts = [
      staticRoutes(staticBundle.scenarios, "lazy-theta-star"),
      dynamicRoutes(dynamicBundle.scenarios, "repeated-astar-3d"),
      predictiveRoutes(predictiveBundle.scenarios, "space-time-astar-4d"),
    ];
    for (const routes of cohorts) {
      const overview = new RouteOverview();
      overview.setRoutes(routes, routes[0]!.id, 800, 600);
      const gates = routes.map(route => overview.group.getObjectByName(`overview-tasks-${route.id}`) as THREE.InstancedMesh);
      const reference = gates[0]!.geometry, disposed = vi.spyOn(reference, "dispose");
      for (const [index, gate] of gates.entries()) {
        expect(gate).toBeInstanceOf(THREE.InstancedMesh);
        expect(gate.count).toBe(routes[index]!.mission!.taskPoints!.length);
        expect(gate.geometry).toBe(reference);
        const material = gate.material as THREE.MeshStandardMaterial;
        expect(material.userData.style).toBe("racing-gate");
        expect(material.color.getHex()).toBe(routeColor(index));
        expect(material.depthTest && material.depthWrite).toBe(true);
      }
      overview.dispose();
      expect(disposed).toHaveBeenCalledOnce();
    }
  });
  it.each(staticBundle.planners)("retains all static stops and original data for $id", planner => {
    const routes = staticRoutes(staticBundle.scenarios, planner.id);
    expect(routes).toHaveLength(8);
    routes.forEach((route, i) => {
      expect(route.points).toBe(staticBundle.scenarios[i]!.results.find(r => r.plannerId === planner.id)!.paths!.smoothed);
      auditMissionTaskVisits(route.points, route.mission);
    });
  });
  it.each(dynamicBundle.planners)("retains actual reactive execution and service intervals for $id", planner => {
    const routes = dynamicRoutes(dynamicBundle.scenarios, planner.id);
    expect(routes).toHaveLength(8);
    routes.forEach((route, i) => {
      const run = dynamicBundle.scenarios[i]!.runs.find(r => r.plannerId === planner.id)!;
      expect(route.points === run.frames.at(-1)!.executedPath).toBe(true);
      const timed = route.timedPath!;
      expect(timed).toBe(run.executionTimedPath);
      auditMissionTaskVisits(timed.map(w => w.position), route.mission, timed.map(w => w.timeS));
      expect(timed.every((w, i) => !i || w.timeS > timed[i - 1]!.timeS)).toBe(true);
      for (const frame of run.frames) {
        expect(Math.hypot(...timedPosition(timed, frame.timeS).map((v, axis) => v - frame.vehicle[axis]!))).toBeLessThan(1e-8);
      }
    });
    expect(dynamicRoutes(dynamicBundle.scenarios, planner.id)[0] === routes[0]).toBe(true);
  });
  it.each(predictiveBundle.planners)("shows only the validated final flight for $id", planner => {
    const routes = predictiveRoutes(predictiveBundle.scenarios, planner.id);
    expect(routes).toHaveLength(8);
    routes.forEach((route, index) => {
      const run = predictiveBundle.scenarios[index]!.runs.find(r => r.plannerId === planner.id)!;
      expect(route.timedPath).toBe(run.executionTimedPath);
      expect(route.waits).toBe(run.executionWaitIntervals);
      expect(route.plannedFrames).toBeUndefined();
      expect(route.playbackKind).toBe(run.predictive ? "predictive" : "reactive");
      auditMissionTaskVisits(route.timedPath!.map(w => w.position), route.mission, route.timedPath!.map(w => w.timeS));
    });
  });
  it("does not render intermediate geometry when the final predictive flight is missing or unqualified", () => {
    const scenario = predictiveBundle.scenarios[0]!, run = scenario.runs[0]!;
    expect(() => predictiveRoutes([{ ...scenario, runs: [{ ...run, executionTimedPath: null }] }], run.plannerId)).toThrow("validated flight");
    const unqualified = { ...run, smoothing: { ...run.smoothing, execution: { ...run.smoothing.execution, qualified: false } } };
    expect(() => predictiveRoutes([{ ...scenario, runs: [unqualified] }], run.plannerId)).toThrow("validated flight");
  });
});

describe("retained source city", () => {
  it("shares exact city geometry across missions but rebuilds when source or bounds change", () => {
    const host = new THREE.Group(), cache = new RetainedCity();
    const scene = { id: "one", bounds: { min: [0, 0, 0], max: [100, 100, 60] },
      city: staticBundle.scenarios[0]!.city!, buildings: [{ min: [10, 10, 0], max: [20, 20, 30] }] };
    cache.mount(host, scene);
    const context = cache.group.getObjectByName("city-context")!;
    cache.mount(host, { ...scene, id: "two" });
    expect(cache.group.getObjectByName("city-context") === context).toBe(true);
    cache.mount(host, { ...scene, bounds: { ...scene.bounds, max: [110, 100, 60] } });
    expect(cache.group.getObjectByName("city-context") === context).toBe(false);
    cache.dispose();
    expect(host.children).toHaveLength(0);
  });
});
