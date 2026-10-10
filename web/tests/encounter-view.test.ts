import { describe, expect, it } from "vitest";
import { OrthographicCamera, PerspectiveCamera, Vector3 } from "three";
import { encounterBounds, encounterView } from "../src/encounter-view";
import { createTrafficAircraft, sizeTrafficAircraft, updateTrafficAircraft } from "../src/traffic-aircraft";
import { playbackAction } from "../src/playback-state";
import { validateExecutionSequence } from "../shared/execution-sequence.mjs";
import { parseCityMission } from "../src/city-schema";
import type { OverviewRoute } from "../src/route-overview";
import type { MovingSphereDefinition } from "../src/dynamic-schema";

const aircraft: MovingSphereDefinition = { id: "cargo", radiusM: 18, keyframes: [
  { timeS: 0, position: [50, -80, 100] }, { timeS: 20, position: [50, -80, 100] },
  { timeS: 40, position: [50, 80, 100] }, { timeS: 100, position: [50, 80, 280] },
] };
const route: OverviewRoute = { id: "one", label: "One", points: [[0, 0, 100], [100, 0, 100]],
  timedPath: [{ timeS: 0, position: [0, 0, 100] }, { timeS: 30, position: [50, 0, 100] }, { timeS: 60, position: [100, 0, 100] }],
  mission: { origin: "Roof", destination: "Roof", purpose: "Cargo", challenge: {
    kind: "dynamic", title: "Crossing", focusPosition: [50, 0, 100], startTimeS: 10, endTimeS: 70,
  } },
};

describe("Physical traffic and event-local observation", () => {
  it("uses an eighteen-metre cargo airframe and independent source-defined safety ring", () => {
    const drone = createTrafficAircraft(aircraft);
    expect(drone.userData).toEqual({ kind: "cargo-drone", airframeSpanM: 18, separationRadiusM: 18 });
    expect(drone.getObjectByName("traffic-separation-envelope")).toBeDefined();
    drone.traverse(node => {
      expect(node.castShadow).toBe(false);
      if ("material" in node) expect((node.material as { depthTest: boolean }).depthTest).toBe(true);
    });
    updateTrafficAircraft(drone, aircraft, 30);
    expect(drone.position.toArray()).toEqual([50, 100, -0]);
    expect(drone.scale.toArray()).toEqual([1, 1, 1]);
  });
  it("keeps the cargo symbol readable in overview and physical-sized up close", () => {
    const drone = createTrafficAircraft(aircraft), body = drone.getObjectByName("traffic-airframe")!;
    const ring = drone.getObjectByName("traffic-separation-envelope")!;
    const camera = new OrthographicCamera(-2000, 2000, 2000, -2000);
    sizeTrafficAircraft(drone, camera, 900);
    expect(body.userData.displaySpanM / (4000 / 900)).toBeCloseTo(32);
    expect(ring.scale.toArray()).toEqual([1, 1, 1]);
    const chase = new PerspectiveCamera(60, 1, 0.1, 1000); chase.position.set(0, 0, 12); chase.lookAt(drone.position);
    sizeTrafficAircraft(drone, chase, 900);
    expect(body.scale.x * 2.1).toBe(18);
    expect(drone.userData.separationRadiusM).toBe(18);
  });
  it("focuses an actual active encounter, not decorative map bounds", () => {
    expect(encounterView(route, [aircraft])).toEqual({ timeS: 22, position: [50, 0, 100] });
    expect(encounterBounds([50, 0, 100]).getSize(new Vector3()).toArray()).toEqual([360, 80, 360]);
  });
  it("has no invented observation for legacy tasks without a challenge", () => {
    expect(encounterView({ ...route, mission: undefined })).toBeNull();
  });
  it("rejects malformed/out-of-bounds scene challenges", () => {
    expect(() => parseCityMission({ origin: "A", destination: "B", purpose: "P", challenge: {
      ...route.mission!.challenge, focusPosition: [999, 0, 20],
    } }, { min: [-100, -100, 0], max: [100, 100, 300] })).toThrow("challenge");
  });
  it("distinguishes operational waiting from completed task service", () => {
    const path = [{ timeS: 0, position: [0, 0, 100] as [number, number, number] },
      { timeS: 10, position: [50, 0, 100] as [number, number, number] },
      { timeS: 16, position: [50, 0, 100] as [number, number, number] },
      { timeS: 40, position: [50, 0, 100] as [number, number, number] },
      { timeS: 50, position: [100, 0, 100] as [number, number, number] }];
    const mission = { origin: "A", destination: "B", purpose: "P", taskPoints: [{
      id: "roof", order: 1, label: "Roof", action: "Service", buildingId: "roof", position: [50, 0, 100] as [number, number, number], serviceDurationS: 6,
    }] };
    expect(playbackAction(path, 12, mission)).toBe("Service");
    expect(playbackAction(path, 24, mission)).toBe("Waiting");
  });
});

describe("Shared execution-sequence gate", () => {
  const geometry = [{ timeS: 0, position: [0, 0, 100] }, { timeS: 10, position: [50, 0, 100] },
    { timeS: 16, position: [50, 0, 100] }, { timeS: 26, position: [100, 0, 120] }];
  const execution = [geometry[0]!, { timeS: 4, position: [0, 0, 100] },
    { timeS: 14, position: [50, 0, 100] }, { timeS: 20, position: [50, 0, 100] },
    { timeS: 26, position: [50, 0, 100] }, { timeS: 40, position: [100, 0, 120] }];
  it("allows only declared stationary additions, retaining every height knot and original hold", () => {
    expect(() => validateExecutionSequence(geometry, execution, true)).not.toThrow();
    expect(() => validateExecutionSequence(geometry, execution, false)).toThrow();
  });
  it("rejects spatial drift disguised as an extra hold", () => {
    const invalid = structuredClone(execution); invalid[1]!.position[2] = 101;
    expect(() => validateExecutionSequence(geometry, invalid, true)).toThrow();
  });
  it("rejects shortened original service or movement", () => {
    const invalid = structuredClone(execution); invalid[3]!.timeS = 19;
    expect(() => validateExecutionSequence(geometry, invalid, true)).toThrow();
    const fast = structuredClone(execution); fast[2]!.timeS = 12;
    expect(() => validateExecutionSequence(geometry, fast, true)).toThrow();
  });
});
