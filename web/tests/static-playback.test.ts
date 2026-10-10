import { describe, expect, it } from "vitest";
import { staticPlaybackPath } from "../src/static-playback";
import { playbackAction } from "../src/playback-state";
import { auditMissionTaskVisits } from "../src/city-schema";
import { staticRoutes, timedPosition } from "../src/route-overview";
import { validateBundle } from "../src/data";
import type { CityMission } from "../src/city-schema";
import type { RoutePoint } from "../src/route-overview";

const filesystemModule = "node:fs";
const { readFileSync } = await import(filesystemModule);
const bundle = validateBundle(JSON.parse(readFileSync(new URL("../public/demo-data.json", import.meta.url), "utf8")));

describe("static route playback", () => {
  it("uses the physical climb rate without creating an artificial stop", () => {
    const path = staticPlaybackPath([[0, 0, 10], [30, 0, 50], [90, 0, 50]], undefined, 15, 3);
    expect(path[1]!.timeS).toBeCloseTo(40 / 3);
    expect(path[2]!.timeS).toBeCloseTo(40 / 3 + 4);
    expect(timedPosition(path, 20 / 3)).toEqual([15, 0, 30]);
    expect(() => staticPlaybackPath([[0, 0, 0]], undefined, 15, 0)).toThrow("climb");
  });
  it("adds a display clock without moving, rounding or dropping any original point", () => {
    const points: RoutePoint[] = [[0, 0, 10], [30, 0, 50], [30, 60, 50]];
    const original = structuredClone(points), path = staticPlaybackPath(points, undefined, 10);
    expect(path.map(w => w.timeS)).toEqual([0, 5, 11]);
    expect(path.map(w => w.position)).toEqual(points);
    expect(path[1]!.position).toBe(points[1]);
    expect(points).toEqual(original);
    expect(timedPosition(path, 2.5)).toEqual([15, 0, 30]);
    expect(timedPosition(path, 100)).toEqual(points.at(-1));
  });

  it.each(bundle.planners)("passes every required point without dwell for $id", planner => {
    const routes = staticRoutes(bundle.scenarios, planner.id, "smoothed");
    expect(routes).toHaveLength(8);
    routes.forEach((route, index) => {
      expect(route.playbackKind).toBe("fixed");
      expect(route.points).toBe(bundle.scenarios[index]!.results.find(r => r.plannerId === planner.id)!.paths!.smoothed);
      const path = route.timedPath!;
      auditMissionTaskVisits(path.map(w => w.position), route.mission, path.map(w => w.timeS));
      expect(path.every((w, i) => !i || w.timeS > path[i - 1]!.timeS)).toBe(true);
      for (const task of route.mission!.taskPoints!) {
        const arrival = path.findIndex(w => w.position.every((v, axis) => Math.abs(v - task.position[axis]!) < 1e-6));
        const a = path[arrival]!, b = path[arrival + 1]!;
        expect(task.visitMode).toBe("fly-through");
        expect(task.serviceDurationS).toBe(0);
        expect(b.position).not.toEqual(a.position);
        expect(timedPosition(path, (a.timeS + b.timeS) / 2)).not.toEqual(a.position);
        expect(playbackAction(path, (a.timeS + b.timeS) / 2, route.mission)).toBe("Flying");
      }
      expect(playbackAction(path, 0, route.mission)).toBe("Ready");
      expect(playbackAction(path, path.at(-1)!.timeS, route.mission)).toBe("Arrived");
    });
    expect(staticRoutes(bundle.scenarios, planner.id, "smoothed")[0]).toBe(routes[0]);
  });

  it("refuses to invent movement through a missing task point", () => {
    const mission = { taskPoints: [{ position: [5, 5, 10], serviceDurationS: 6 }] } as CityMission;
    expect(() => staticPlaybackPath([[0, 0, 10], [10, 0, 10]], mission)).toThrow("skip");
  });

  it("skips duplicate movement knots without inventing a wait", () => {
    const path = staticPlaybackPath([[0, 0, 10], [0, 0, 10], [30, 0, 10]]);
    expect(path.map(w => w.timeS)).toEqual([0, 2]);
  });

  it.each([0, -1, NaN, Infinity])("rejects invalid playback speed %s", speed => {
    expect(() => staticPlaybackPath([[0, 0, 10]], undefined, speed)).toThrow("Invalid");
  });
});
