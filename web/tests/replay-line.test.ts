import { describe, expect, it, vi } from "vitest";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { revealReplayLine, revealReplayWindow } from "../src/replay-line";
import { RouteOverview, type OverviewRoute } from "../src/route-overview";

describe("faithful temporal route presentation", () => {
  it("clips both ends of known forecasts without reuploading buffers", () => {
    const geometry = new LineGeometry(); geometry.setPositions([0, 10, 0, 100, 10, 0, 100, 10, -100]);
    const material = new LineMaterial({ depthTest: true, depthWrite: false });
    const window = revealReplayWindow(geometry, [0, 10, 20], material);
    const positions = geometry.getAttribute("instanceStart");
    const upload = vi.spyOn(geometry, "setPositions");
    for (const t of [0, 5, 10, 15, 0]) { window.start.value = t; window.end.value = Math.min(20, t + 5); }
    expect(upload).not.toHaveBeenCalled(); expect(geometry.getAttribute("instanceStart")).toBe(positions);
    expect(material.uniforms.windowStart).toBe(window.start); expect(material.uniforms.windowEnd).toBe(window.end);
    expect(material.vertexShader).toContain("vec4( clippedStart, 1.0 )");
    expect(material.vertexShader).toContain("vec4( clippedEnd, 1.0 )");
    expect(material.fragmentShader).toContain("windowVisible < 0.5) discard");
    expect(material.depthTest).toBe(true);
    geometry.dispose(); material.dispose();
  });
  it("clips in world space using matching segment clocks and normal depth testing", () => {
    const geometry = new LineGeometry(); geometry.setPositions([0, 10, 0, 100, 10, 0, 100, 10, -100]);
    const line = new LineMaterial({ depthTest: true }), halo = new LineMaterial({ depthTest: true });
    const clock = revealReplayLine(geometry, [0, 10, 20], [line, halo]);
    expect(Array.from(geometry.getAttribute("instanceTimeStart").array)).toEqual([0, 10]);
    expect(Array.from(geometry.getAttribute("instanceTimeEnd").array)).toEqual([10, 20]);
    expect(line.uniforms.replayTime).toBe(clock);
    expect(halo.uniforms.replayTime).toBe(clock);
    expect(line.vertexShader).toContain("vec4( replayEnd, 1.0 )");
    expect(line.fragmentShader).toContain("replayVisible < 0.5) discard");
    expect(line.depthTest && halo.depthTest).toBe(true);
    geometry.dispose(); line.dispose(); halo.dispose();
  });

  it.each(["fixed", "reactive", "predictive"] as const)("keeps %s geometry retained across playback and rewind", kind => {
    const route: OverviewRoute = { id: "one", label: "One", points: [[0, 0, 10], [100, 0, 10], [100, 100, 10]],
      timedPath: [{ timeS: 0, position: [0, 0, 10] }, { timeS: 10, position: [100, 0, 10] }, { timeS: 20, position: [100, 100, 10] }],
      playbackKind: kind };
    const overview = new RouteOverview(); overview.setRoutes([route], route.id, 800, 600);
    const line = overview.group.getObjectByName("overview-trajectory-one") as Line2;
    const upload = vi.spyOn(line.geometry, "setPositions");
    for (const t of [5, 10, 15, 20, 0]) overview.setTime(t);
    expect(upload).not.toHaveBeenCalled();
    expect(Boolean(line.material.uniforms.replayTime)).toBe(kind !== "fixed");
    expect(Boolean(overview.group.getObjectByName("overview-plan-one"))).toBe(kind === "predictive");
    expect(overview.vehicle(route.id)!.position.toArray().map(v => v === 0 ? 0 : v)).toEqual([0, 10, 0]);
    overview.dispose();
  });

  it.each(["reactive", "predictive"] as const)("shows the complete %s result at time zero and restores the real clock on playback", kind => {
    const route: OverviewRoute = { id: "one", label: "One", plannerId: "selected-planner", points: [[0, 0, 10], [100, 0, 10]],
      timedPath: [{ timeS: 0, position: [0, 0, 10] }, { timeS: 10, position: [100, 0, 10] }], playbackKind: kind };
    const overview = new RouteOverview(); overview.setRoutes([route], route.id, 800, 600);
    const line = overview.group.getObjectByName("overview-trajectory-one") as Line2;
    const clock = line.material.uniforms.replayTime!, pending = overview.group.getObjectByName("overview-plan-one");
    const upload = vi.spyOn(line.geometry, "setPositions");
    expect(clock.value).toBe(10); expect(overview.group.userData.presentation).toBe("result-preview");
    if (pending) expect(pending.visible).toBe(false);
    overview.setPlaying(true); expect(clock.value).toBe(0);
    if (pending) expect(pending.visible).toBe(true);
    overview.setTime(4); expect(clock.value).toBe(4);
    const position = overview.vehicle(route.id)!.position.clone();
    overview.setPlaying(false); expect(clock.value).toBe(10);
    expect(overview.vehicle(route.id)!.position.equals(position)).toBe(true);
    if (pending) expect(pending.visible).toBe(false);
    overview.setPlaying(true); expect(clock.value).toBe(4);
    overview.setTime(10); if (pending) expect(pending.visible).toBe(false);
    expect(upload).not.toHaveBeenCalled(); overview.dispose();
  });

  it("invalidates presentation caches when the algorithm or policy changes despite shared waypoint arrays", () => {
    const route: OverviewRoute = { id: "one", label: "One", plannerId: "a", points: [[0, 0, 10], [100, 0, 10]],
      timedPath: [{ timeS: 0, position: [0, 0, 10] }, { timeS: 10, position: [100, 0, 10] }], playbackKind: "reactive" };
    const overview = new RouteOverview(); overview.setRoutes([route], route.id, 800, 600);
    const before = overview.group.getObjectByName("overview-trajectory-one") as Line2;
    const dispose = vi.spyOn(before.geometry, "dispose");
    overview.setRoutes([{ ...route, plannerId: "b", playbackKind: "predictive" }], route.id, 800, 600);
    expect(dispose).toHaveBeenCalledOnce(); expect(overview.group.getObjectByName(before.name)).not.toBe(before);
    expect(overview.group.children[0]!.userData.plannerId).toBe("b");
    expect(overview.group.getObjectByName("overview-plan-one")).toBeDefined();
    overview.dispose();
  });
});
