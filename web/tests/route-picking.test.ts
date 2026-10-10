import { OrthographicCamera, PerspectiveCamera, Vector3 } from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { describe, expect, it } from "vitest";
import { pickVisibleRoutes } from "../src/route-picking";
import { revealReplayLine, revealReplayWindow } from "../src/replay-line";
import { RouteOverview, type OverviewRoute } from "../src/route-overview";
import type { DisplayCamera } from "../src/camera-scale";
const camera = () => { const c = new OrthographicCamera(-10, 10, 10, -10, .1, 100); c.position.set(0, 0, 20); c.lookAt(0, 0, 0); c.updateMatrixWorld(); return c; };
function line(points = [-8, 0, 0, 8, 0, 0]): Line2 { const l = new Line2(new LineGeometry().setPositions(points), new LineMaterial({ linewidth: 4 })); l.updateMatrixWorld(); return l; }
const pixel = (point: Vector3, c: DisplayCamera) => { const p = point.clone().project(c); return { x: (p.x + 1) * 200, y: (1 - p.y) * 200 }; };
describe("visible trajectory picking", () => {
  it("uses screen pixels at any zoom and picks the real closest world point", () => {
    const c = camera(), l = line();
    const picked = pickVisibleRoutes([{ id: "mission", lines: [l] }], c, 250, 204, 400, 400);
    expect(picked?.id).toBe("mission"); expect(picked?.position.x).toBeCloseTo(2.5); expect(picked?.screenDistancePx).toBeCloseTo(4);
    expect(pickVisibleRoutes([{ id: "mission", lines: [l] }], c, 200, 208, 400, 400)).toBeNull();
    c.zoom = 2; c.updateProjectionMatrix(); expect(pickVisibleRoutes([{ id: "mission", lines: [l] }], c, 200, 204, 400, 400)?.id).toBe("mission");
  });
  it("selects only the GPU-visible flown prefix, including a partial current segment", () => {
    const l = line(), c = camera(), clock = revealReplayLine(l.geometry, [0, 10], [l.material]); clock.value = 5;
    expect(pickVisibleRoutes([{ id: "flight", lines: [l] }], c, 100, 200, 400, 400)?.id).toBe("flight");
    expect(pickVisibleRoutes([{ id: "flight", lines: [l] }], c, 300, 200, 400, 400)).toBeNull();
    clock.value = 0; expect(pickVisibleRoutes([{ id: "flight", lines: [l] }], c, 100, 200, 400, 400)).toBeNull();
  });
  it("clips both sides of a remaining/local plan without altering geometry or uniforms", () => {
    const l = line(), c = camera(), window = revealReplayWindow(l.geometry, [0, 10], l.material); window.start.value = 3; window.end.value = 7;
    const before = Array.from(l.geometry.getAttribute("instanceStart").array);
    expect(pickVisibleRoutes([{ id: "plan", lines: [l] }], c, 200, 200, 400, 400)?.id).toBe("plan");
    for (const x of [50, 350]) expect(pickVisibleRoutes([{ id: "plan", lines: [l] }], c, x, 200, 400, 400)).toBeNull();
    expect(Array.from(l.geometry.getAttribute("instanceStart").array)).toEqual(before); expect(window.start.value).toBe(3); expect(window.end.value).toBe(7);
  });
  it("respects hidden parents, empty segments and the viewport boundary", () => {
    const l = line(), c = camera(); l.visible = false;
    expect(pickVisibleRoutes([{ id: "hidden", lines: [l] }], c, 200, 200, 400, 400)).toBeNull();
    expect(pickVisibleRoutes([{ id: "zero", lines: [line([0, 0, 0, 0, 0, 0])] }], c, 200, 200, 400, 400)).toBeNull();
    expect(pickVisibleRoutes([{ id: "outside", lines: [line()] }], c, -1, 200, 400, 400)).toBeNull();
  });
  it("prefers the front route at a crossing and rejects hidden buildings", () => {
    const c = camera(), back = line(), front = line([-8, 0, 5, 8, 0, 5]);
    const routes = [{ id: "back", lines: [back] }, { id: "front", lines: [front] }];
    expect(pickVisibleRoutes(routes, c, 200, 200, 400, 400)?.id).toBe("front");
    expect(pickVisibleRoutes(routes, c, 200, 200, 400, 400, p => p.z > 1)?.id).toBe("back");
    expect(pickVisibleRoutes(routes, c, 200, 200, 400, 400, () => true)).toBeNull();
  });
  it("uses perspective-correct interpolation and clips near-plane crossings", () => {
    const c = new PerspectiveCamera(60, 1, 1, 100); c.updateMatrixWorld();
    const l = line([-2, 0, -4, 8, 0, -20]), world = new Vector3(1, 0, -8.8), p = pixel(world, c);
    expect(pickVisibleRoutes([{ id: "perspective", lines: [l] }], c, p.x, p.y, 400, 400)?.position.distanceTo(world)).toBeLessThan(1e-5);
    const hidden = line([-1, 0, 4, 1, 0, 5]); expect(pickVisibleRoutes([{ id: "behind", lines: [hidden] }], c, 200, 200, 400, 400)).toBeNull();
    const crossing = line([0, 0, .5, 0, 0, -10]); expect(pickVisibleRoutes([{ id: "cross", lines: [crossing] }], c, 200, 200, 400, 400)?.id).toBe("cross");
  });
  it("highlights a selection without changing other routes, clocks, buffers or follow targets", () => {
    const route: OverviewRoute = { id: "r", label: "Task", points: [[-8, 0, 0], [8, 0, 0]], playbackKind: "predictive", timedPath: [{ timeS: 0, position: [-8, 0, 0] }, { timeS: 10, position: [8, 0, 0] }] };
    const overview = new RouteOverview(); overview.setRoutes([route], route.id, 400, 400);
    const flown = overview.group.getObjectByName("overview-trajectory-r") as Line2, geometry = flown.geometry;
    overview.setSelection("r"); expect(flown.material.linewidth).toBe(6.2); expect(flown.geometry).toBe(geometry);
    expect(overview.pick(camera(), 200, 200, 400, 400, () => false)?.id).toBe("r");
    overview.setPlaying(true); overview.setTime(5); overview.setSelection(null);
    expect(flown.material.uniforms.replayTime!.value).toBe(5); expect(flown.material.linewidth).toBe(4.8); expect(flown.geometry).toBe(geometry); overview.dispose();
  });
});
