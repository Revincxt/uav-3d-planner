import { afterEach, describe, expect, it, vi } from "vitest";
import { crossedTaskArrivals, taskArrivals, TaskArrivalNotice } from "../src/task-arrival-notice";
import type { OverviewRoute } from "../src/route-overview";

const route: OverviewRoute = { id: "uav-1", label: "One", points: [[0,0,30],[10,0,30],[20,0,30],[30,0,30]],
  timedPath: [{ timeS: 0, position: [0,0,30] }, { timeS: 10, position: [10,0,30] },
    { timeS: 20, position: [20,0,30] }, { timeS: 30, position: [30,0,30] }],
  mission: { origin: "A", destination: "B", purpose: "Fly through", taskPoints: [
    { id: "one", order: 1, position: [10,0,30], label: "One", action: "Fly through", serviceDurationS: 0, visitMode: "fly-through", buildingId: "one" },
    { id: "two", order: 2, position: [20,0,30], label: "Two", action: "Fly through", serviceDurationS: 0, visitMode: "fly-through", buildingId: "two" },
  ] } };

function fixture() {
  const root = { hidden: true, textContent: "", dataset: {} as Record<string,string>, className: "", id: "",
    setAttribute: vi.fn(), remove: vi.fn() };
  vi.stubGlobal("document", { createElement: () => root });
  const notice = new TaskArrivalNotice({ append: vi.fn() } as unknown as HTMLElement);
  return { root, notice };
}
afterEach(() => { vi.useRealTimers(); vi.unstubAllGlobals(); });

describe("Follow-only checkpoint arrival notices", () => {
  it("uses the exact ordered knots and excludes an already crossed left boundary", () => {
    expect(taskArrivals(route).map(a => a.timeS)).toEqual([10,20]);
    expect(taskArrivals(route)).toBe(taskArrivals(route));
    expect(crossedTaskArrivals(route, 9.9, 10).map(a => a.task.id)).toEqual(["one"]);
    expect(crossedTaskArrivals(route, 10, 10.1)).toEqual([]);
    expect(crossedTaskArrivals(route, 20, 10)).toEqual([]);
  });
  it("does not infer an arrival from a nearby visual marker", () => {
    const invalid = structuredClone(route); invalid.timedPath![1]!.position[0] += 0.01;
    expect(() => taskArrivals(invalid)).toThrow("No actual arrival");
  });
  it("never shows a notice without a followed route", () => {
    const { root, notice } = fixture(); notice.advance(undefined, 9, 11);
    expect(root.hidden).toBe(true); expect(root.textContent).toBe("");
    notice.dispose(); expect(root.remove).toHaveBeenCalledOnce();
  });
  it("shows one concise notice per pass and clears it automatically", () => {
    vi.useFakeTimers(); const { root, notice } = fixture();
    notice.advance(route, 9, 11); expect(root.hidden).toBe(false);
    expect(root.textContent).toBe("✓ 任务点 01 已通过");
    vi.advanceTimersByTime(1700); notice.advance(route, 9, 11);
    vi.advanceTimersByTime(100); expect(root.hidden).toBe(true);
  });
  it("clears immediately on exit, seek or target reset and permits a new replay pass", () => {
    vi.useFakeTimers(); const { root, notice } = fixture();
    notice.advance(route, 9, 11); notice.reset(); expect(root.hidden).toBe(true);
    notice.advance(route, 9, 11); expect(root.hidden).toBe(false);
    notice.advance(undefined, 11, 12); expect(root.hidden).toBe(true);
    notice.advance(route, 19, 21); expect(root.dataset.taskId).toBe("two");
    notice.advance(route, 21, 9); expect(root.hidden).toBe(true);
  });
});
