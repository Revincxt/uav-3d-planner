import { afterEach, describe, expect, it, vi } from "vitest";
import { dynamicAvoidanceEvents, predictiveAvoidanceEvents, FlightAnnouncements, type AvoidanceEvent } from "../src/flight-announcements";
import type { DynamicFrame, DynamicRun, DynamicScenario, Vec3 } from "../src/dynamic-schema";
import type { PredictiveRun, WaitInterval } from "../src/predictive-schema";

const straight: Vec3[] = [[0, 0, 10], [100, 0, 10]];
const detour: Vec3[] = [[0, 0, 10], [40, 20, 10], [60, 20, 10], [100, 0, 10]];
const aircraft = { id: "cargo-1", radiusM: 5, position: [50, 0, 10] as Vec3 };
const zone = { id: "zone-1", center: [50, 0] as [number, number], radiusM: 5, zMinM: 0, zMaxM: 20, activeFromS: 0, activeUntilS: 100 };
const scenario = { constraints: { vehicleRadiusM: 1, safetyMarginM: 1 }, temporaryNoFlyZones: [zone] } as DynamicScenario;
const frame = (overrides: Partial<DynamicFrame> = {}): DynamicFrame => ({
  timeS: 0, vehicle: [0, 0, 10], path: straight, executedPath: [], activeTemporaryZoneIds: [], movingSpheres: [],
  event: null, replanned: false, replanReason: null, plannerSuccess: null, planningTimeMs: null, workUsed: 0, changedEdges: 0,
  ...overrides,
});
const dynamic = (update: Partial<DynamicFrame> = {}, previous: Partial<DynamicFrame> = {}): DynamicRun => ({ frames: [
  frame(previous), frame({ timeS: 10, path: detour, replanned: true, replanReason: "scheduled", plannerSuccess: true, ...update }),
  frame({ timeS: 12, vehicle: [10, 0, 10] }),
] } as DynamicRun);
const wait = (reason: string, startTimeS = 0, endTimeS = 8): WaitInterval => ({ reason, startTimeS, endTimeS, position: [1, 2, 10] });
const predictive = (waits: WaitInterval[], isPredictive = true): PredictiveRun => ({
  plannerId: "fixture", predictive: isPredictive,
  executionTimedPath: [{ timeS: 0, position: [1, 2, 10] }], executionMetrics: {}, executionFrames: [], executionWaitIntervals: waits,
  smoothing: { execution: { qualified: true, collisionCertified: true, status: "qualified", qualification: { qualified: true } } },
} as unknown as PredictiveRun);

afterEach(() => { vi.useRealTimers(); vi.unstubAllGlobals(); });

describe("causal dynamic avoidance announcements", () => {
  it("reports a recorded detour around a cargo UAV", () => {
    expect(dynamicAvoidanceEvents(scenario, dynamic({ movingSpheres: [aircraft] }))).toEqual([
      { id: "replan-1", timeS: 10, kind: "reroute", message: "避让障碍无人机，航路已调整" },
    ]);
  });
  it("reports an active airspace detour, not the activation alone", () => {
    const events = dynamicAvoidanceEvents(scenario, dynamic({ activeTemporaryZoneIds: [zone.id] }));
    expect(events[0]!.message).toBe("避让动态空域，航路已调整");
    expect(dynamicAvoidanceEvents(scenario, dynamic())).toEqual([]);
    expect(dynamicAvoidanceEvents(scenario, dynamic({ activeTemporaryZoneIds: [zone.id], path: straight }))).toEqual([]);
  });
  it("ignores ordinary scheduled updates and failed plans", () => {
    expect(dynamicAvoidanceEvents(scenario, dynamic({ path: straight, movingSpheres: [aircraft] }))).toEqual([]);
    expect(dynamicAvoidanceEvents(scenario, dynamic({ plannerSuccess: false, movingSpheres: [aircraft] }))).toEqual([]);
    expect(dynamicAvoidanceEvents(scenario, dynamic({ replanReason: "initial", movingSpheres: [aircraft] }))).toEqual([]);
  });
  it("does not mistake a different mission leg for an obstacle detour", () => {
    expect(dynamicAvoidanceEvents(scenario, dynamic({ movingSpheres: [aircraft], path: detour.slice(0, -1) }))).toEqual([]);
  });
  it("does not blame consumed geometry behind the UAV", () => {
    const run = dynamic({ vehicle: [80, 0, 10], path: [[80, 0, 10], [100, 0, 10]], movingSpheres: [aircraft] });
    expect(dynamicAvoidanceEvents(scenario, run)).toEqual([]);
  });
  it("respects the height of both airspace and spherical obstacles", () => {
    expect(dynamicAvoidanceEvents(scenario, dynamic({ movingSpheres: [{ ...aircraft, position: [50, 0, 40] }] }))).toEqual([]);
    const lowZone = { ...scenario, temporaryNoFlyZones: [{ ...zone, zMaxM: 5 }] };
    expect(dynamicAvoidanceEvents(lowZone, dynamic({ activeTemporaryZoneIds: [zone.id] }))).toEqual([]);
  });
  it("detects entry into a vertical cylinder slab on a climb", () => {
    const run = dynamic({ activeTemporaryZoneIds: [zone.id], path: [[0, 0, 0], [50, 25, 15], [100, 0, 30]] },
      { path: [[0, 0, 0], [100, 0, 30]] });
    expect(dynamicAvoidanceEvents(scenario, run)[0]!.message).toContain("动态空域");
  });
  it("uses native safety-gate provenance even when a snapshot cannot explain a forecast conflict", () => {
    const events = dynamicAvoidanceEvents(scenario, dynamic({ replanReason: "safety-gate" }));
    expect(events[0]).toMatchObject({ kind: "reroute", message: "动态避障，航路已调整" });
  });
  it("distinguishes an actual protection hold from a moving detour", () => {
    const run = dynamic({ replanReason: "safety-gate" });
    run.frames[2]!.vehicle = [0, 0, 10];
    expect(dynamicAvoidanceEvents(scenario, run)[0]).toMatchObject({ kind: "hold", message: "避障保护，等待安全通行" });
  });
  it("coalesces repeated safety checks into one continuing episode", () => {
    const run = { frames: [frame(), ...[10, 12, 14].map(timeS => frame({ timeS, replanReason: "safety-gate", vehicle: [timeS, 0, 10] })),
      frame({ timeS: 16, vehicle: [16, 0, 10] })] } as DynamicRun;
    expect(dynamicAvoidanceEvents(scenario, run)).toHaveLength(1);
  });
});

describe("qualified predictive avoidance announcements", () => {
  it("does not treat time-lattice alignment or checkpoint service as avoidance", () => {
    expect(predictiveAvoidanceEvents(predictive([wait("time-lattice alignment"), wait("task service")]))).toEqual([]);
  });
  it("announces forecast holds and their actual resume time", () => {
    expect(predictiveAvoidanceEvents(predictive([wait("forecast-aware waiting action")]))).toEqual([
      { id: "avoidance-wait-0", timeS: 0, kind: "hold", message: "预测避让，等待动态空域窗口" },
      { id: "avoidance-resume-0", timeS: 8, kind: "resume", message: "恢复飞行，继续任务" },
    ]);
  });
  it("merges contiguous safety holds without modifying the native records", () => {
    const waits = [wait("reactive safety hold", 10, 12), wait("reactive safety hold", 12, 14)];
    const before = JSON.stringify(waits);
    const events = predictiveAvoidanceEvents(predictive(waits, false));
    expect(events).toHaveLength(2); expect(events[1]!.timeS).toBe(14);
    expect(events[0]!.message).toBe("避障保护，等待安全通行");
    expect(JSON.stringify(waits)).toBe(before);
  });
  it("keeps separate safety episodes and ignores zero-length holds", () => {
    expect(predictiveAvoidanceEvents(predictive([wait("safety", 0, 0), wait("safety", 10, 12), wait("safety", 14, 16)]))).toHaveLength(4);
  });
  it("never substitutes an unqualified intermediate flight", () => {
    const run = predictive([]); run.smoothing.execution.qualified = false;
    expect(() => predictiveAvoidanceEvents(run)).toThrow("No validated flight");
  });
});

describe("focused-UAV visual notices without audio", () => {
  const events: AvoidanceEvent[] = [
    { id: "first", timeS: 0, kind: "hold", message: "等待安全通行" },
    { id: "second", timeS: 10, kind: "reroute", message: "航路已调整" },
    { id: "third", timeS: 20, kind: "resume", message: "恢复飞行" },
  ];
  function fixture() {
    vi.useFakeTimers();
    const root = { hidden: false, className: "", dataset: {} as Record<string, string>, innerHTML: "", setAttribute: vi.fn(), append: vi.fn(), remove: vi.fn() };
    const copy = { textContent: "" };
    vi.stubGlobal("document", { createElement: vi.fn().mockReturnValueOnce(root).mockReturnValueOnce(copy) });
    const speak = vi.fn(); vi.stubGlobal("speechSynthesis", { speak });
    const append = vi.fn();
    return { notice: new FlightAnnouncements({ append } as unknown as HTMLElement), root, copy, append, speak };
  }
  it("does not announce paused time zero, but does announce a true initial avoidance hold on playback", () => {
    const f = fixture(); f.notice.advance("a", 0, events, 0, 0);
    expect(f.root.hidden).toBe(true);
    f.notice.advance("a", 0, events, 0, .1);
    expect(f.copy.textContent).toBe("UAV 01 · 等待安全通行");
    expect(f.root.hidden).toBe(false); expect(f.speak).not.toHaveBeenCalled();
  });
  it("reports a crossed edge exactly once and expires without another animation loop", () => {
    const f = fixture(); f.notice.advance("a", 7, events, 9.9, 10);
    expect(f.copy.textContent).toBe("UAV 08 · 航路已调整");
    expect(f.root.dataset).toMatchObject({ timeS: "10", routeId: "a", kind: "reroute" });
    vi.advanceTimersByTime(4200); expect(f.root.hidden).toBe(true);
    f.notice.advance("a", 7, events, 9.9, 10); expect(f.root.hidden).toBe(true);
    expect(f.append).toHaveBeenCalledOnce(); expect(f.speak).not.toHaveBeenCalled();
  });
  it("keeps only the most recent crossed event after a long rendering frame", () => {
    const f = fixture(); f.notice.advance("a", 1, events, 0, 21);
    expect(f.copy.textContent).toBe("UAV 02 · 恢复飞行");
  });
  it("clears on explicit seek or route change; pausing preserves deduplication", () => {
    const f = fixture(); f.notice.advance("a", 0, events, 9, 10); f.notice.suspend();
    f.notice.advance("a", 0, events, 9, 10); expect(f.root.hidden).toBe(true);
    f.notice.reset(); f.notice.advance("a", 0, events, 9, 10); expect(f.root.hidden).toBe(false);
    f.notice.advance("b", 2, events, 11, 12); expect(f.root.hidden).toBe(true);
  });
  it("rejects reverse or non-finite playback and disposes timers and nodes", () => {
    const f = fixture(); f.notice.advance("a", 0, events, 9, 10);
    f.notice.advance("a", 0, events, 10, 9); expect(f.root.hidden).toBe(true);
    f.notice.advance("a", 0, events, 9, 10); f.notice.advance("a", 0, events, NaN, 11);
    expect(f.root.hidden).toBe(true); f.notice.dispose(); expect(f.root.remove).toHaveBeenCalledOnce();
    expect(vi.getTimerCount()).toBe(0); expect(f.speak).not.toHaveBeenCalled();
  });
});
