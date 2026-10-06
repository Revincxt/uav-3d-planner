import type { CityMission } from "./city-schema";

interface WorldMission {
  mission?: CityMission;
  bounds: unknown;
  buildings: unknown;
  noFlyZones?: unknown;
  staticNoFlyZones?: unknown;
  temporaryNoFlyZones?: unknown;
  movingSpheres?: unknown;
  constraints: unknown;
}

/** A declaration is not sufficient: compare the actual physical input arrays too. */
export function auditSharedWorld(scenarios: readonly WorldMission[]): void {
  if (!scenarios.some(scenario => scenario.mission?.sharedWorld)) return;
  const first = scenarios[0]!, world = first.mission?.sharedWorld;
  if (!world || scenarios.length !== world.missionCount) throw new Error("Shared world requires all declared mission queries");
  const physical = (scenario: WorldMission) => ({ bounds: scenario.bounds,
    buildings: scenario.buildings, constraints: scenario.constraints,
    static: scenario.noFlyZones ?? scenario.staticNoFlyZones,
    temporary: scenario.temporaryNoFlyZones, traffic: scenario.movingSpheres });
  const expected = physical(first);
  const declaration = JSON.stringify(world);
  for (const scenario of scenarios) {
    const actual = physical(scenario);
    if (JSON.stringify(scenario.mission?.sharedWorld) !== declaration ||
        (Object.keys(expected) as (keyof typeof expected)[]).some(key =>
          actual[key] !== expected[key] && JSON.stringify(actual[key]) !== JSON.stringify(expected[key])))
      throw new Error("Shared world missions must use identical obstacles and absolute schedules");
  }
}

export function sameSharedWorld(left?: CityMission, right?: CityMission): boolean {
  return Boolean(left?.sharedWorld && right?.sharedWorld &&
    left.sharedWorld.id === right.sharedWorld.id && left.sharedWorld.fingerprint === right.sharedWorld.fingerprint);
}
