import type { DemoBundle, DemoScenario, PlannerResult, Vec3 } from "./schema";
import { auditSharedWorld } from "./shared-world";
import { auditMissionTaskVisits, parseBuildingFootprint, parseCityMetadata, parseCityMission } from "./city-schema";

function isFiniteVec3(value: unknown): value is Vec3 {
  return (
    Array.isArray(value) &&
    value.length === 3 &&
    value.every((component) => typeof component === "number" && Number.isFinite(component))
  );
}

function validateResult(result: PlannerResult, scenario: DemoScenario): void {
  if (!result.runId || !result.plannerId || !Number.isFinite(result.metrics.planningTimeMs)) {
    throw new Error(`Invalid planner record in ${scenario.id}`);
  }
  if (result.status === "success") {
    if (!result.paths || result.paths.raw.length < 2 || result.paths.smoothed.length < 2) {
      throw new Error(`Successful record has no trajectory: ${result.runId}`);
    }
    for (const path of [result.paths.raw, result.paths.smoothed]) {
      if (!path.every(isFiniteVec3)) throw new Error(`Non-finite path coordinate: ${result.runId}`);
      auditMissionTaskVisits(path, scenario.mission);
      const start = path[0];
      const goal = path[path.length - 1];
      if (!start || !goal || start.some((value, index) => value !== scenario.start[index])) {
        throw new Error(`Trajectory start mismatch: ${result.runId}`);
      }
      if (goal.some((value, index) => value !== scenario.goal[index])) {
        throw new Error(`Trajectory goal mismatch: ${result.runId}`);
      }
    }
  } else if (result.paths !== null) {
    throw new Error(`Failed record unexpectedly contains a trajectory: ${result.runId}`);
  }
}

export function validateBundle(value: unknown): DemoBundle {
  if (!value || typeof value !== "object") throw new Error("Demo data must be an object");
  const bundle = value as DemoBundle;
  if (bundle.schemaVersion !== 1 || bundle.verificationStatus !== "DEMO_NON_CONFIRMATORY") {
    throw new Error("Unsupported or unlabelled demo dataset");
  }
  if (!Array.isArray(bundle.scenarios) || bundle.scenarios.length < 1) {
    throw new Error("Demo dataset contains no scenarios");
  }
  const ids = new Set<string>();
  const checkedBuildings = new WeakSet<object>();
  for (const scenario of bundle.scenarios) {
    if (ids.has(scenario.id)) throw new Error(`Duplicate scenario ID: ${scenario.id}`);
    ids.add(scenario.id);
    if (!isFiniteVec3(scenario.start) || !isFiniteVec3(scenario.goal)) {
      throw new Error(`Invalid endpoint in ${scenario.id}`);
    }
    scenario.city = parseCityMetadata(scenario.city, `${scenario.id}.city`);
    scenario.mission = parseCityMission(scenario.mission, scenario.bounds, `${scenario.id}.mission`);
    if (!checkedBuildings.has(scenario.buildings)) {
      scenario.buildings.forEach((building) => {
        building.footprint = parseBuildingFootprint(building.footprint, building, `${scenario.id}.${building.id}.footprint`);
      });
      checkedBuildings.add(scenario.buildings);
    }
    if (scenario.city) for (const building of scenario.buildings) {
      if (!building.footprint) throw new Error(`City building has no source footprint: ${building.id}`);
    }
    scenario.results.forEach((result) => validateResult(result, scenario));
  }
  if (!ids.has(bundle.defaultScenarioId)) throw new Error("Default scenario does not exist");
  auditSharedWorld(bundle.scenarios);
  return bundle;
}
