import { afterEach, describe, expect, it, vi } from "vitest";
import { FlightHud, flightReadout, altitudeTrend } from "../src/flight-hud";
import type { OverviewRoute } from "../src/route-overview";

const route: OverviewRoute = {
  id: "mission-1", label: "First mission", points: [[0, 0, 30], [30, 0, 70], [30, 40, 70]],
  timedPath: [
    { timeS: 5, position: [0, 0, 30] }, { timeS: 10, position: [30, 0, 70] },
    { timeS: 14, position: [30, 0, 70] }, { timeS: 18, position: [30, 40, 70] },
  ],
  mission: { origin: "Roof A", destination: "Roof B", purpose: "Inspection", taskPoints: [
    { id: "gate-1", order: 1, position: [30, 0, 70], label: "First", action: "Fly through", serviceDurationS: 0, visitMode: "fly-through", buildingId: "building-1" },
    { id: "gate-2", order: 2, position: [30, 40, 70], label: "Second", action: "Fly through", serviceDurationS: 0, visitMode: "fly-through", buildingId: "building-2" },
  ] },
};

afterEach(() => { vi.unstubAllGlobals(); });

describe("recorded flight instruments", () => {
  it("identifies actual climbs and descents and clears at level flight, waits and goals", () => {
    expect(altitudeTrend(route, 7.5)).toBe("climb");
    expect(altitudeTrend(route, 12, "climb")).toBeNull();
    expect(altitudeTrend(route, 16)).toBeNull();
    expect(altitudeTrend(route, 100, "climb")).toBeNull();
    expect(altitudeTrend(route, 5)).toBeNull();
    expect(altitudeTrend(route, NaN)).toBeNull();
    const descent = { ...route, timedPath: route.timedPath!.map(p => ({ ...p, position: [p.position[0], p.position[1], 100 - p.position[2]] as [number, number, number] })) };
    expect(altitudeTrend(descent, 7.5)).toBe("descent");
  });
  it("ignores small height oscillations and uses hysteresis for an established slow climb", () => {
    const slow: OverviewRoute = { id: "slow", label: "Slow climb", points: [], timedPath: [
      { timeS: 0, position: [0, 0, 30] }, { timeS: 5, position: [0, 0, 31.2] },
    ] };
    expect(altitudeTrend(slow, 2)).toBeNull();
    expect(altitudeTrend(slow, 2, "climb")).toBe("climb");
    expect(altitudeTrend(slow, 2, "descent")).toBeNull();
    const noise: OverviewRoute = { ...slow, timedPath: [
      { timeS: 0, position: [0, 0, 30] }, { timeS: 1, position: [1, 0, 30] },
      { timeS: 1.001, position: [1.001, 0, 30.04] }, { timeS: 2, position: [2, 0, 30] },
    ] };
    expect(altitudeTrend(noise, 1.0005, "climb")).toBeNull();
    expect(altitudeTrend(noise, 1.5)).toBeNull();
  });
  it("uses interpolated ENU altitude and physical 3D speed, not replay speed", () => {
    expect(flightReadout(route, 7.5)).toEqual({ altitudeM: 50, speedMps: 10, headingDeg: 90 });
    expect(flightReadout(route, 16)).toEqual({ altitudeM: 70, speedMps: 10, headingDeg: 0 });
  });
  it("shows stationary records and finished flights as zero speed without losing bearing", () => {
    expect(flightReadout(route, 12)).toMatchObject({ speedMps: 0, headingDeg: 90 });
    expect(flightReadout(route, 100)).toEqual({ speedMps: 0, headingDeg: 0, altitudeM: 70 });
    expect(flightReadout(route, -100)).toMatchObject({ speedMps: 0, altitudeM: 30 });
  });
  it("updates instruments at exact motion boundaries and supports rewind", () => {
    expect(flightReadout(route, 9.999)!.speedMps).toBe(10);
    expect(flightReadout(route, 10)!.speedMps).toBe(0);
    expect(flightReadout(route, 18)!.speedMps).toBe(0);
    expect(flightReadout(route, 6)!.altitudeM).toBe(38);
  });
  it("keeps the horizontal compass bearing during a vertical segment", () => {
    const climb: OverviewRoute = { id: "climb", label: "Climb", points: [], timedPath: [
      { timeS: 0, position: [0, 0, 20] }, { timeS: 2, position: [0, 0, 40] }, { timeS: 4, position: [-10, 0, 40] },
    ] };
    expect(flightReadout(climb, 1)).toEqual({ speedMps: 10, headingDeg: 270, altitudeM: 30 });
  });
  it("does not invent instruments for missing or non-finite records", () => {
    expect(flightReadout({ ...route, timedPath: [] }, 0)).toBeNull();
    expect(flightReadout({ ...route, timedPath: undefined }, 0)).toBeNull();
    expect(flightReadout(route, NaN)).toBeNull();
    expect(flightReadout(route, Infinity)).toBeNull();
  });
  it("handles a one-knot stationary mission", () => {
    const one = { id: "one", label: "One", points: [], timedPath: [{ timeS: 0, position: [1, 2, 30] as [number, number, number] }] };
    expect(flightReadout(one, 0)).toEqual({ altitudeM: 30, speedMps: 0, headingDeg: 0 });
  });
});

describe("retained flight HUD", () => {
  function fixture(playback?: HTMLElement) {
    const fields = new Map([".hud-aircraft", ".hud-route-key", ".hud-key-plan", ".hud-key-flown", ".hud-altitude-trend", ...["altitude", "speed", "heading"].map(key => `[data-readout="${key}"]`)]
      .map(key => [key, { textContent: "", hidden: true, dataset: {} as Record<string, string> }]));
    const root = { hidden: false, dataset: {} as Record<string, string>, title: "", className: "", innerHTML: "",
      style: { setProperty: vi.fn() }, setAttribute: vi.fn(), querySelector: (key: string) => fields.get(key), append: vi.fn(), remove: vi.fn() };
    const createElement = vi.fn(() => root), append = vi.fn();
    let now = 0;
    vi.stubGlobal("document", { createElement }); vi.stubGlobal("performance", { now: () => now });
    return { root, fields, createElement, append, hud: new FlightHud({ append } as unknown as HTMLElement, playback), advance: (ms: number) => { now += ms; } };
  }
  it("retains one instrument tree and throttles playback updates to 10 Hz", () => {
    const f = fixture(); f.hud.update(route, 7.5, 0);
    expect(f.fields.get('[data-readout="altitude"]')!.textContent).toBe("50.0");
    expect(f.root.style.setProperty).toHaveBeenCalledWith("--speed-angle", "-67.5deg");
    expect(f.root.style.setProperty).toHaveBeenCalledWith("--altitude-angle", "-118.125deg");
    f.advance(50); f.hud.update(route, 16, 0);
    expect(f.fields.get('[data-readout="altitude"]')!.textContent).toBe("50.0");
    f.advance(50); f.hud.update(route, 16, 0);
    expect(f.fields.get('[data-readout="altitude"]')!.textContent).toBe("70.0");
    expect(f.createElement).toHaveBeenCalledOnce(); expect(f.append).toHaveBeenCalledOnce();
    f.hud.dispose(); expect(f.root.remove).toHaveBeenCalledOnce();
  });
  it("updates immediately on seek or aircraft switch and hides absent telemetry", () => {
    const f = fixture(); f.hud.update(route, 7.5, 0);
    f.hud.update(route, 18, 0, true);
    expect(f.fields.get('[data-readout="altitude"]')!.textContent).toBe("70.0");
    f.hud.update({ ...route, id: "mission-8" }, 6, 7);
    expect(f.fields.get(".hud-aircraft")!.textContent).toBe("UAV 08");
    expect(f.root.dataset.routeId).toBe("mission-8");
    f.hud.update({ ...route, timedPath: [] }, 0, 0);
    expect(f.root.hidden).toBe(true);
  });
  it("retains one concise altitude hint and clears it immediately when seeking into a wait", () => {
    const f = fixture(), hint = f.fields.get(".hud-altitude-trend")!;
    f.hud.update(route, 7.5, 0);
    expect(hint.hidden).toBe(false); expect(hint.textContent).toBe("Climb");
    expect(hint.dataset.direction).toBe("climb");
    f.hud.update(route, 12, 0, true);
    expect(hint.hidden).toBe(true); expect(hint.textContent).toBe("");
    expect(hint.dataset.direction).toBeUndefined();
    expect(f.root.style.setProperty).toHaveBeenCalledWith("--speed-angle", "-135deg");
    expect(f.fields.get('[data-readout="altitude"]')!.textContent).toBe("70.0");
    expect(f.createElement).toHaveBeenCalledOnce();
  });
  it("moves the existing playback controls into the instrument without recreating them", () => {
    const timeline = { min: "0", max: "100", value: "25", style: { setProperty: vi.fn() } };
    const playback = { id: "playback", querySelector: () => timeline } as unknown as HTMLElement;
    const f = fixture(playback);
    f.hud.update(route, 7.5, 0); f.hud.update(route, 18, 0, true);
    expect(f.root.append).toHaveBeenCalledExactlyOnceWith(playback);
    expect(timeline.style.setProperty).toHaveBeenLastCalledWith("--timeline-progress", "25%");
    timeline.value = "50"; f.hud.update(route, 18, 0);
    expect(timeline.style.setProperty).toHaveBeenLastCalledWith("--timeline-progress", "50%");
    expect(f.createElement).toHaveBeenCalledOnce();
    expect(f.root.innerHTML).toContain('class="hud-telemetry"');
    expect(f.root.innerHTML).toContain('class="hud-compass"');
    expect(f.root.innerHTML).not.toMatch(/gates/i);
    for (const cardinal of ["N", "E", "S", "W"]) expect(f.root.innerHTML).toContain(`>${cardinal}</text>`);
    for (const label of ["ALT <small>m</small>", "SPD <small>m/s</small>", "HDG <small>°</small>"]) {
      expect(f.root.innerHTML).toContain(label);
    }
  });
});
