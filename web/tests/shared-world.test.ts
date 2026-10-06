import { describe, expect, it } from "vitest";
import { auditSharedWorld, sameSharedWorld } from "../src/shared-world";
import { parseCityMission } from "../src/city-schema";

const world = { id: "world", fingerprint: `sha256:${"a".repeat(64)}`, missionCount: 4,
  clockOriginS: 0, airframeSpanM: 18, missionDeconfliction: "not-jointly-optimized" as const };
const mission = { origin: "A", destination: "B", purpose: "Cargo", sharedWorld: world };
const fixtures = () => Array.from({ length: 4 }, () => ({ mission: structuredClone(mission),
  bounds: { min: [0,0,0], max: [100,100,100] }, buildings: [], constraints: { radius: 1 },
  staticNoFlyZones: [], temporaryNoFlyZones: [], movingSpheres: [{ id: "cargo", radiusM: 24,
    keyframes: [{ timeS: 0, position: [0,0,50] }, { timeS: 10, position: [100,100,50] }] }] }));

describe("One physical world for four mission queries", () => {
  it("accepts different tasks with identical physical constraints and one clock", () => {
    const scenes = fixtures(); scenes[1]!.mission.origin = "Another roof";
    expect(() => auditSharedWorld(scenes)).not.toThrow();
    expect(sameSharedWorld(scenes[0]!.mission, scenes[1]!.mission)).toBe(true);
  });
  it("rejects a shared declaration when one mission has different traffic", () => {
    const scenes = fixtures(); scenes[2]!.movingSpheres[0]!.keyframes[1]!.timeS = 12;
    expect(() => auditSharedWorld(scenes)).toThrow("identical obstacles");
  });
  it("rejects missing members, undeclared worlds and changed safety margins", () => {
    expect(() => auditSharedWorld(fixtures().slice(1))).toThrow("all declared");
    const scenes = fixtures(); scenes[1]!.constraints.radius = 2;
    expect(() => auditSharedWorld(scenes)).toThrow("identical obstacles");
    expect(sameSharedWorld(undefined, mission)).toBe(false);
  });
  it("does not confuse a different world's fingerprint with a task focus change", () => {
    expect(sameSharedWorld(mission, { ...mission, sharedWorld: { ...world, fingerprint: `sha256:${"b".repeat(64)}` } })).toBe(false);
  });
  it("validates the shared-clock declaration without accepting joint-safety claims", () => {
    const bounds = { min: [0,0,0], max: [100,100,100] };
    expect(parseCityMission(mission, bounds)?.sharedWorld?.missionCount).toBe(4);
    expect(parseCityMission({ ...mission, sharedWorld: { ...world, missionCount: 8 } }, bounds)?.sharedWorld?.missionCount).toBe(8);
    for (const missionCount of [0, -1, 2.5, "8", Infinity]) {
      expect(() => parseCityMission({ ...mission, sharedWorld: { ...world, missionCount } }, bounds)).toThrow("sharedWorld");
    }
    expect(() => parseCityMission({ ...mission, sharedWorld: { ...world, clockOriginS: 10 } }, bounds)).toThrow("sharedWorld");
    expect(() => parseCityMission({ ...mission, sharedWorld: { ...world, missionDeconfliction: "certified" } }, bounds)).toThrow("sharedWorld");
  });
});
