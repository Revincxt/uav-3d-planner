import * as THREE from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { describe, expect, it, vi } from "vitest";
import { auditMissionTaskVisits } from "../src/city-schema";
import { validateBundle } from "../src/data";
import { validateDynamicBundle } from "../src/dynamic-data";
import { validatePredictiveBundle } from "../src/predictive-data";
import { RetainedCity } from "../src/retained-city";
import {
  RouteOverview, dynamicRoutes, overviewDuration, pathPrefix, predictivePath, predictiveRoutes,
  flightHeading, reactiveTrace, routeColor, routeColorCSS, staticRoutes, timedPosition, waypointIndex,
  type OverviewRoute, type RouteWaypoint,
} from "../src/route-overview";
import type { DynamicRun } from "../src/dynamic-schema";
import type { PredictiveRun } from "../src/predictive-schema";

const filesystemModule = "node:fs";
const { readFileSync } = await import(filesystemModule);
const read = (file: string) => JSON.parse(readFileSync(new URL(`../public/${file}`, import.meta.url), "utf8"));
const staticBundle = validateBundle(read("demo-data.json"));
const dynamicBundle = validateDynamicBundle(read("dynamic-data.json"));
const predictiveBundle = validatePredictiveBundle(read("predictive-data.json"));

function fixtureRoutes(): OverviewRoute[] {
  return Array.from({ length: 4 }, (_, index) => {
    const path: RouteWaypoint[] = [
      { timeS: 0, position: [index * 10, 0, 30] },
      { timeS: 10 + index * 5, position: [index * 10, 100, 50] },
    ];
    return { id: `route-${index}`, label: `Route ${index}`, points: path.map(w => w.position), timedPath: path };
  });
}

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
  it("keeps every stop knot in a replay prefix and rewinds deterministically", () => {
    expect(pathPrefix(path, 9)).toEqual([[0, 0, 30], [10, 0, 40], [10, 0, 40], [10, 5, 45]]);
    expect(pathPrefix(path, 0)).toEqual([[0, 0, 30]]);
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
  it("retains within-frame corners instead of cutting a diagonal across an obstacle", () => {
    const frames = [
      { timeS: 0, vehicle: [0, 0, 20], executedPath: [[0, 0, 20]] },
      { timeS: 2, vehicle: [10, 10, 20], executedPath: [[0, 0, 20], [10, 0, 20], [10, 10, 20]] },
      { timeS: 8, vehicle: [10, 10, 20], executedPath: [[0, 0, 20], [10, 0, 20], [10, 10, 20]] },
    ] as DynamicRun["frames"];
    const trace = reactiveTrace({ frames });
    expect(trace.map(w => w.timeS)).toEqual([0, 1, 2, 8]);
    expect(timedPosition(trace, 1)).toEqual([10, 0, 20]);
    expect(timedPosition(trace, 5)).toEqual([10, 10, 20]);
  });
});

describe("four routes, one world", () => {
  it("reuses predictive route and point arrays for unchanged planner/mode without mixing algorithms", () => {
    const planner = predictiveBundle.planners[0]!.id;
    const first = predictiveRoutes(predictiveBundle.scenarios, planner, "execution");
    const second = predictiveRoutes(predictiveBundle.scenarios, planner, "execution");
    first.forEach((route, index) => expect(second[index]).toBe(route));
    const other = predictiveRoutes(predictiveBundle.scenarios, predictiveBundle.planners[1]!.id, "execution");
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
      expect(line.material.linewidth).toBeGreaterThanOrEqual(4);
      expect(line.material.resolution.toArray()).toEqual([800, 600]);
    });
    expect(new Set(lines.map(l => l.material.color.getHex())).size).toBe(4);
    expect(routeColorCSS(0)).toBe("#009b87");
    overview.dispose();
  });
  it("does not recreate or dispose trajectory GPU buffers on focus or 100 time samples", () => {
    const overview = new RouteOverview(), routes = fixtureRoutes();
    overview.setRoutes(routes, routes[0]!.id, 800, 600);
    const line = overview.group.getObjectByName(`overview-trajectory-${routes[0]!.id}`) as Line2;
    const dispose = vi.spyOn(line.geometry, "dispose"), upload = vi.spyOn(line.geometry, "setPositions");
    for (let i = 0; i < 100; i++) { overview.setTime(i / 4); overview.setFocus(routes[i % 4]!.id); }
    overview.setRoutes(routes.map(route => ({ ...route })), routes[2]!.id, 600, 400);
    expect(overview.group.getObjectByName(line.name) === line).toBe(true);
    expect(dispose).not.toHaveBeenCalled();
    expect(upload).not.toHaveBeenCalled();
    expect(overview.group.children.every(group => group.visible)).toBe(true);
    overview.dispose();
    expect(dispose).toHaveBeenCalledOnce();
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
    expect(followed.scale.x / normal).toBeCloseTo(88 / 24);
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
      staticBundle.planners.map(planner => staticRoutes(staticBundle.scenarios, planner.id, "smoothed")),
      dynamicBundle.planners.map(planner => dynamicRoutes(dynamicBundle.scenarios, planner.id)),
      predictiveBundle.planners.map(planner => predictiveRoutes(predictiveBundle.scenarios, planner.id, "execution")),
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
          if (line.material.uniforms.replayTime) expect(line.material.uniforms.replayTime.value).toBe(route.timedPath!.at(-1)!.timeS);
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
      staticRoutes(staticBundle.scenarios, "lazy-theta-star", "smoothed"),
      dynamicRoutes(dynamicBundle.scenarios, "repeated-astar-3d"),
      predictiveRoutes(predictiveBundle.scenarios, "space-time-astar-4d", "execution"),
    ];
    for (const routes of cohorts) {
      const overview = new RouteOverview();
      overview.setRoutes(routes, routes[0]!.id, 800, 600);
      const gates = routes.map(route => overview.group.getObjectByName(`overview-tasks-${route.id}`) as THREE.InstancedMesh);
      const reference = gates[0]!.geometry.getAttribute("position");
      for (const [index, gate] of gates.entries()) {
        expect(gate).toBeInstanceOf(THREE.InstancedMesh);
        expect(gate.count).toBe(routes[index]!.mission!.taskPoints!.length);
        expect(gate.geometry.getAttribute("position").array).toEqual(reference.array);
        const material = gate.material as THREE.MeshStandardMaterial;
        expect(material.userData.style).toBe("racing-gate");
        expect(material.color.getHex()).toBe(routeColor(index));
        expect(material.depthTest && material.depthWrite).toBe(true);
      }
      overview.dispose();
    }
  });
  it.each(staticBundle.planners)("retains all static stops and original data for $id", planner => {
    for (const mode of ["raw", "smoothed"] as const) {
      const routes = staticRoutes(staticBundle.scenarios, planner.id, mode);
      expect(routes).toHaveLength(8);
      routes.forEach((route, i) => {
        expect(route.points === staticBundle.scenarios[i]!.results.find(r => r.plannerId === planner.id)!.paths![mode]).toBe(true);
        auditMissionTaskVisits(route.points, route.mission);
      });
    }
  });
  it.each(dynamicBundle.planners)("retains actual reactive execution and service intervals for $id", planner => {
    const routes = dynamicRoutes(dynamicBundle.scenarios, planner.id);
    expect(routes).toHaveLength(8);
    routes.forEach((route, i) => {
      const run = dynamicBundle.scenarios[i]!.runs.find(r => r.plannerId === planner.id)!;
      expect(route.points === run.frames.at(-1)!.executedPath).toBe(true);
      const timed = route.timedPath!;
      auditMissionTaskVisits(timed.map(w => w.position), route.mission, timed.map(w => w.timeS));
      expect(timed.every((w, i) => !i || w.timeS > timed[i - 1]!.timeS)).toBe(true);
      for (const frame of run.frames) {
        expect(Math.hypot(...timedPosition(timed, frame.timeS).map((v, axis) => v - frame.vehicle[axis]!))).toBeLessThan(1e-8);
      }
    });
    expect(dynamicRoutes(dynamicBundle.scenarios, planner.id)[0] === routes[0]).toBe(true);
  });
  it.each(predictiveBundle.planners)("keeps raw/geometry/execution separate for $id", planner => {
    for (const mode of ["raw", "geometry", "execution"] as const) {
      const routes = predictiveRoutes(predictiveBundle.scenarios, planner.id, mode);
      expect(routes).toHaveLength(8);
      routes.forEach(route => auditMissionTaskVisits(route.timedPath!.map(w => w.position), route.mission, route.timedPath!.map(w => w.timeS)));
    }
  });
  it("fails closed when the requested predictive evidence is missing", () => {
    const run = predictiveBundle.scenarios[0]!.runs[0]!;
    expect(() => predictivePath({ ...run, executionTimedPath: null }, "execution")).toThrow("qualified");
    expect(() => predictivePath({ ...run, smoothing: { ...run.smoothing, certified: false } } as PredictiveRun, "geometry")).toThrow("certified");
  });
});

describe("exact reactive clock", () => {
  it("uses native climb-constrained timing instead of redistributing time by distance", () => {
    const executionTimedPath: NonNullable<DynamicRun["executionTimedPath"]> = [
      { timeS: 0, position: [0, 0, 0], action: "start" },
      { timeS: 10, position: [0, 0, 30], action: "move" },
      { timeS: 12, position: [30, 0, 30], action: "move" },
    ];
    expect(reactiveTrace({ frames: [], executionTimedPath })).toBe(executionTimedPath);
    expect(timedPosition(executionTimedPath, 5)).toEqual([0, 0, 15]);
    expect(timedPosition(executionTimedPath, 11)).toEqual([15, 0, 30]);
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
