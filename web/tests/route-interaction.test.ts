import { describe, expect, it } from "vitest";
import { PointerClickGuard, RouteClickState, positionRouteCard } from "../src/route-interaction-state";
import { routeTaskSummary } from "../src/route-interaction";
const pointer = (pointerId = 1, x = 100, y = 100, pointerType = "mouse", button = 0) => ({ pointerId, clientX: x, clientY: y, pointerType, button });
describe("trajectory selection and task card", () => {
  it("selects on the first click, follows on the next and resets on a different route or dismiss", () => {
    const state = new RouteClickState(); expect(state.click("a")).toBe("select"); expect(state.click("a")).toBe("follow");
    expect(state.click("b")).toBe("select"); expect(state.click(null)).toBe("clear"); expect(state.click("b")).toBe("select"); state.clear(); expect(state.click("b")).toBe("select");
  });
  it("does not misinterpret an orbit drag, even when the pointer returns to its origin", () => {
    const guard = new PointerClickGuard(); guard.down(pointer()); guard.move(pointer(1, 120)); guard.move(pointer()); expect(guard.up(pointer())).toBe(false);
    guard.down(pointer()); expect(guard.up(pointer(1, 103, 101))).toBe(true);
    guard.down(pointer(1, 100, 100, "mouse", 2)); expect(guard.up(pointer(1, 100, 100, "mouse", 2))).toBe(false);
  });
  it("accepts a touch tap but rejects pinches, cancellations and late releases", () => {
    const guard = new PointerClickGuard(); guard.down(pointer(1, 100, 100, "touch")); expect(guard.up(pointer(1, 104, 102, "touch"))).toBe(true);
    guard.down(pointer(1, 100, 100, "touch")); guard.down(pointer(2, 140, 100, "touch"));
    expect(guard.up(pointer(1, 100, 100, "touch"))).toBe(false); expect(guard.up(pointer(2, 140, 100, "touch"))).toBe(false);
    guard.down(pointer()); guard.cancel(1); expect(guard.up(pointer())).toBe(false);
    guard.down(pointer()); guard.reset(); expect(guard.up(pointer())).toBe(false);
  });
  it("keeps the card on the map and away from controls when a free quadrant is available", () => {
    const blockers = [{ x: 0, y: 0, width: 320, height: 120 }, { x: 20, y: 410, width: 280, height: 230 }];
    const position = positionRouteCard({ x: 300, y: 500 }, 272, 220, 320, 640, blockers);
    expect(position.x).toBeGreaterThanOrEqual(12); expect(position.x + 272).toBeLessThanOrEqual(308);
    expect(position.y).toBeGreaterThanOrEqual(120); expect(position.y + 220).toBeLessThanOrEqual(410);
  });
  it("does not cover the clicked curve, so the second click still reaches the canvas", () => {
    for (const point of [{ x: 1410, y: 840 }, { x: 20, y: 20 }, { x: 380, y: 800 }]) {
      const position = positionRouteCard(point, 294, 300, 1440, 900, []);
      expect(point.x >= position.x && point.x <= position.x + 294 && point.y >= position.y && point.y <= position.y + 300).toBe(false);
    }
  });
  it("fits the compact card between mobile controls and the imagery attribution after resizing", () => {
    const blockers = [{ x: 0, y: 0, width: 320, height: 191 }, { x: 120, y: 406, width: 188, height: 16 },
      { x: 12, y: 425, width: 296, height: 203 }];
    const position = positionRouteCard({ x: 916, y: 210 }, 272, 204.16, 320, 640, blockers);
    expect(position.y).toBeGreaterThanOrEqual(191); expect(position.y + 204.16).toBeLessThanOrEqual(406);
  });
  it("uses the actual flown trace and physical mission clock, including genuine holds", () => {
    const route = { id: "mission", label: "Task", points: [[0, 0, 0], [9999, 0, 0]] as [number, number, number][], timedPath: [
      { timeS: 10, position: [0, 0, 0] as [number, number, number] }, { timeS: 30, position: [0, 0, 0] as [number, number, number] },
      { timeS: 80, position: [300, 400, 0] as [number, number, number] },
    ] };
    expect(routeTaskSummary(route)).toEqual({ lengthM: 500, durationS: 70 }); expect(routeTaskSummary(route)).toBe(routeTaskSummary(route));
  });
});
