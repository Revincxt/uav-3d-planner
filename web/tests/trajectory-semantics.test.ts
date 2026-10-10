import { describe, expect, it, vi } from "vitest";
import { Line2 } from "three/addons/lines/Line2.js";
import { RouteOverview, displayHeading, localPlanClock, routeColor, type OverviewRoute } from "../src/route-overview";
import { PLAN_LABELS, RouteStatusStrip, flightPhase, phaseLabel, planMeaning } from "../src/trajectory-semantics";

const route: OverviewRoute = { id: "a", label: "A", points: [], playbackKind: "predictive", timedPath: [
  { timeS: 0, position: [0, 0, 10] }, { timeS: 2, position: [0, 0, 10] },
  { timeS: 4, position: [0, 0, 10] }, { timeS: 6, position: [20, 0, 10] },
  { timeS: 8, position: [20, 0, 20] }, { timeS: 10, position: [20, 0, 10] },
], waits: [
  { startTimeS: 0, endTimeS: 2, position: [0, 0, 10], reason: "forecast-aware waiting action" },
  { startTimeS: 2, endTimeS: 4, position: [0, 0, 10], reason: "forecast-aware waiting action" },
] };

describe("flight semantics", () => {
  it("consumes a climb-constrained plan by its motion clock, not Euclidean cruise time", () => {
    const route: OverviewRoute = { id: "climb", label: "Climb", points: [], cruiseSpeedMps: 15, maxClimbRateMps: 3,
      timedPath: [{ timeS: 0, position: [0, 0, 10] }, { timeS: 2, position: [0, 0, 10] },
        { timeS: 12, position: [0, 0, 40] }, { timeS: 16, position: [60, 0, 40] }] };
    expect(localPlanClock(route, 0, 1)).toBe(0);
    expect(localPlanClock(route, 0, 7)).toBe(5);
    expect(localPlanClock(route, 2, 7)).toBe(7);
    expect(localPlanClock(route, 12, 14)).toBe(14);
  });
  it("reports a real initial hold, coalesces its countdown and uses exact boundaries", () => {
    expect(flightPhase(route, 0)).toEqual({ kind: "waiting", remainingS: 4, reason: "forecast-aware waiting action" });
    expect(phaseLabel(flightPhase(route, 2.5))).toBe("Await slot · 2 s");
    expect(flightPhase(route, 4).kind).toBe("flying");
    expect(flightPhase(route, 10).kind).toBe("arrived");
    expect(flightPhase(route, 100).kind).toBe("arrived");
  });
  it("does not confuse climb, descent or a finished mission with a hold", () => {
    expect(flightPhase(route, 7).kind).toBe("climbing");
    expect(flightPhase(route, 9).kind).toBe("descending");
    expect(flightPhase(route, -1).kind).toBe("ready");
    expect(flightPhase({ timedPath: [] }, 0).kind).toBe("ready");
  });
  it("distinguishes algorithm knowledge from a future recorded result", () => {
    expect(planMeaning(route)).toBe("scheduled");
    expect(planMeaning({ ...route, playbackKind: "reactive" })).toBe("recorded");
    expect(planMeaning({ ...route, playbackKind: "reactive", plannedFrames: [] })).toBe("local");
    expect(planMeaning({ ...route, playbackKind: "fixed" })).toBe("fixed");
  });
  it("aligns remaining-route appearance without inventing scheduled knowledge for reactive algorithms", () => {
    const overview = new RouteOverview();
    for (const playbackKind of ["reactive", "predictive"] as const) {
      overview.setRoutes([{ ...route, playbackKind, points: route.timedPath!.map(w => w.position) }], route.id, 800, 600);
      const pending = overview.group.getObjectByName(`overview-plan-${route.id}`) as Line2;
      expect(pending.material.color.getHex()).toBe(routeColor(0)); expect(pending.material.linewidth).toBe(3); expect(pending.material.dashed).toBe(true); expect(pending.material.dashSize).toBe(14);
      expect(pending.userData.meaning).toBe(playbackKind === "predictive" ? "scheduled" : "recorded");
    }
    expect(PLAN_LABELS.scheduled).toBe(PLAN_LABELS.recorded); overview.dispose();
  });
  it("stabilizes a tiny backtracking knot without changing coordinates or time", () => {
    const path = [
      { timeS: 0, position: [0, 0, 10] as [number, number, number] },
      { timeS: 2, position: [20, 0, 10] as [number, number, number] },
      { timeS: 2.01, position: [19.999, 0.001, 10] as [number, number, number] },
      { timeS: 4, position: [40, 0, 10] as [number, number, number] },
    ];
    const original = structuredClone(path);
    expect(displayHeading(path, 2.005).x).toBeGreaterThan(.99);
    expect(displayHeading(path, 2.005)).toEqual(displayHeading(path, 2.005));
    expect(path).toEqual(original);
  });
  it.each(["fixed", "reactive", "predictive"] as const)("retains the elapsed/future split when pausing %s", playbackKind => {
    const overview = new RouteOverview(), displayed = { ...route, playbackKind, points: route.timedPath!.map(w => w.position) };
    overview.setRoutes([displayed], route.id, 800, 600);
    const flown = overview.group.getObjectByName(`overview-trajectory-${route.id}`) as Line2;
    const pending = overview.group.getObjectByName(`overview-plan-${route.id}`) as Line2;
    expect(overview.group.userData.presentation).toBe("result-preview");
    expect(flown.material.uniforms.replayTime!.value).toBe(10);
    expect(pending.visible).toBe(false);
    overview.setPlaying(true); overview.setTime(5); overview.setPlaying(false);
    expect(overview.group.userData.presentation).toBe("paused");
    expect(flown.material.uniforms.replayTime!.value).toBe(5);
    expect(pending.material.uniforms.windowStart!.value).toBe(5);
    expect(pending.material.uniforms.windowEnd!.value).toBe(10);
    expect(pending.visible).toBe(true);
    expect(pending.geometry).not.toBe(flown.geometry);
    overview.setTime(10);
    expect(pending.visible).toBe(false);
    overview.setTime(0);
    expect(overview.group.userData.presentation).toBe("result-preview");
    expect(flown.material.uniforms.replayTime!.value).toBe(10);
    overview.dispose();
  });
  it("marks only genuine holds, not a finished mission or a vertical climb", () => {
    const overview = new RouteOverview();
    overview.setRoutes([{ ...route, points: route.timedPath!.map(w => w.position) }], route.id, 800, 600); overview.setPlaying(true);
    const hold = overview.group.getObjectByName(`overview-hold-${route.id}`)!;
    overview.setTime(1); expect(hold.visible).toBe(true);
    overview.setTime(7); expect(hold.visible).toBe(false);
    overview.setTime(12); expect(hold.visible).toBe(false);
    overview.dispose();
  });
  it("does not consume the local plan during a genuine safety hold", () => {
    const reactive = { ...route, cruiseSpeedMps: 10 };
    expect(localPlanClock(reactive, 0, 3)).toBe(0);
    expect(localPlanClock(reactive, 0, 5)).toBe(1);
    expect(localPlanClock(reactive, 4, 5)).toBe(5);
    expect(localPlanClock(reactive, 6, 7)).toBe(6.5);
    expect(localPlanClock(reactive, 6, 6)).toBe(6);
  });
  it("retains the indexed motion clock across arbitrary seeks and envelope/path replacements", () => {
    const indexed = { ...route, cruiseSpeedMps: 10, maxClimbRateMps: 2 };
    const reference = (from: number, to: number) => {
      if (to <= from) return from;
      const sample = (clock: number) => {
        const path = indexed.timedPath!;
        if (clock <= path[0]!.timeS) return path[0]!.position;
        for (let i = 1; i < path.length; i++) if (clock < path[i]!.timeS) {
          const a = path[i - 1]!, b = path[i]!, f = (clock - a.timeS) / (b.timeS - a.timeS);
          return a.position.map((p, axis) => p + (b.position[axis]! - p) * f);
        }
        return path.at(-1)!.position;
      };
      const clocks = [from, ...indexed.timedPath!.filter(p => p.timeS > from && p.timeS <= to).map(p => p.timeS), to];
      let result = from;
      for (let i = 1; i < clocks.length; i++) {
        const a = sample(clocks[i - 1]!), b = sample(clocks[i]!);
        result += Math.max(Math.hypot(...b.map((v, axis) => v - a[axis]!)) / indexed.cruiseSpeedMps,
          Math.abs(b[2]! - a[2]!) / indexed.maxClimbRateMps);
      }
      return result;
    };
    for (const from of [-2, 0, 2.3, 4, 6.5, 9, 10, 20]) for (const to of [30, 10, 8.9, 7, 4, 1, -1])
      expect(localPlanClock(indexed, from, to)).toBeCloseTo(reference(from, to), 10);
    indexed.cruiseSpeedMps = 20; indexed.maxClimbRateMps = 3;
    expect(localPlanClock(indexed, 3, 9)).toBeCloseTo(reference(3, 9), 10);
    indexed.timedPath = indexed.timedPath!.slice(0, 3);
    expect(localPlanClock(indexed, 0, 9)).toBeCloseTo(reference(0, 9), 10);
  });
  it("does not rescan earlier coordinates on every indexed playback sample", () => {
    let reads = 0;
    const path = Array.from({ length: 1000 }, (_, i) => ({ timeS: i,
      get position(): [number, number, number] { reads++; return [i, 0, 20]; } }));
    const indexed = { timedPath: path, cruiseSpeedMps: 10 };
    expect(localPlanClock(indexed, 0, 900)).toBeCloseTo(90, 10);
    reads = 0;
    for (let i = 0; i < 120; i++) localPlanClock(indexed, 400, 900 + i / 120);
    expect(reads).toBe(0);
  });
  it("initializes mission status buttons even if the workspace mounts them later", () => {
    const buttons: { dataset: Record<string, string>; title: string }[] = [];
    const query = vi.fn(() => buttons);
    vi.stubGlobal("document", { querySelectorAll: query });
    try {
      const strip = new RouteStatusStrip();
      strip.update([route], 0, true);
      const button = { dataset: { route: route.id } as Record<string, string>, title: "" };
      buttons.push(button);
      strip.update([route], 1, true);
      expect(button.dataset.flightPhase).toBe("waiting");
      expect(button.title).toBe("A · Await slot · 3 s");
      strip.update([route], 12, true);
      expect(button.dataset.flightPhase).toBe("arrived");
      expect(query).toHaveBeenCalledTimes(2);
    } finally { vi.unstubAllGlobals(); }
  });
});
