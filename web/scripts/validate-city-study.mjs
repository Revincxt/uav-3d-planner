import { createHash } from "node:crypto";
import { readFile } from "node:fs/promises";
import { resolve } from "node:path";

const source = JSON.parse(await readFile(resolve(process.argv[2] ?? "../data/manhattan/city.json"), "utf8"));
const publicDirectory = resolve(process.argv[3] ?? "public");
const canonical = value => Array.isArray(value) ? value.map(canonical)
  : value && typeof value === "object"
    ? Object.fromEntries(Object.keys(value).sort().map(key => [key, canonical(value[key])])) : value;
const digest = (buildings) => createHash("sha256").update(JSON.stringify(buildings.map((building) =>
  [building.id, building.min, building.max, building.footprint]))).digest("hex");
const expectedGeometry = digest(source.buildings);
const expectedSource = source.metadata.sourceSha256.replace(/^sha256:/, "");
let missionCount = 0;
let runCount = 0;
let taskCount = 0;
const samePoint = (a, b) => Math.hypot(...a.map((value, axis) => value - b[axis])) <= 1e-5;
const auditVisits = (positions, tasks, times, label) => {
  let cursor = 0;
  for (const task of tasks) {
    while (cursor < positions.length && !samePoint(positions[cursor], task.position)) cursor += 1;
    if (cursor === positions.length) throw new Error(`${label}: omitted or unordered task ${task.id}`);
    if (times) {
      let departure = cursor;
      while (departure + 1 < positions.length && samePoint(positions[departure + 1], task.position)) departure += 1;
      if (times[departure] - times[cursor] < task.serviceDurationS - 1e-5) throw new Error(`${label}: missing service duration at ${task.id}`);
      if (task.visitMode === "fly-through" && times[departure] - times[cursor] > 1e-5) throw new Error(`${label}: fly-through point has a dwell at ${task.id}`);
      cursor = departure;
    }
    cursor += 1;
  }
};
for (const track of ["demo", "dynamic", "predictive"]) {
  const bundle = JSON.parse(await readFile(resolve(publicDirectory, `${track}-data.json`), "utf8"));
  if (bundle.scenarios.length !== 8) throw new Error(`${track}: exactly eight new Manhattan missions are required`);
  if (track !== "demo") for (const scenario of bundle.scenarios) {
    if (scenario.movingSpheres.length !== 7) throw new Error(`${scenario.id}: requires seven shared cargo aircraft`);
    for (const aircraft of scenario.movingSpheres) {
      const frames = aircraft.keyframes;
      if (frames[0].timeS !== 0 || frames.at(-1).timeS < bundle.protocol.maxTimeS) throw new Error(`${aircraft.id}: incomplete patrol horizon`);
      for (let i = 1; i < frames.length; i++) {
        if (frames[i].timeS <= frames[i - 1].timeS || samePoint(frames[i].position, frames[i - 1].position)) throw new Error(`${aircraft.id}: stopped patrol`);
      }
    }
  }
  for (const scenario of bundle.scenarios) {
    if (scenario.city?.id !== source.metadata.id || scenario.city.sourceKind !== "nyc-open-data") {
      throw new Error(`${scenario.id}: not the official Manhattan city`);
    }
    if (scenario.city.sourceSha256.replace(/^sha256:/, "") !== expectedSource) throw new Error(`${scenario.id}: source digest mismatch`);
    if (JSON.stringify(canonical(scenario.city.planningRegion)) !== JSON.stringify(canonical(source.metadata.planningRegion))) throw new Error(`${scenario.id}: declared planning region differs from the physical city`);
    if (digest(scenario.buildings) !== expectedGeometry) throw new Error(`${scenario.id}: physical buildings or source polygons differ from the complete official extract`);
    if (JSON.stringify([scenario.bounds.min, scenario.bounds.max]) !== JSON.stringify([source.bounds.min, source.bounds.max])) throw new Error(`${scenario.id}: city bounds mismatch`);
    if (scenario.city.collisionModel !== "conservative-aabb") throw new Error(`${scenario.id}: undeclared physical model`);
    if (!scenario.mission?.origin || !scenario.mission.destination || !scenario.mission.purpose) throw new Error(`${scenario.id}: missing urban mission context`);
    const tasks = scenario.mission.taskPoints;
    if (!Array.isArray(tasks) || tasks.length < 6 || tasks.length > 8) throw new Error(`${scenario.id}: requires 6–8 intermediate checkpoints`);
    if (tasks.some((task, index) => task.order !== index + 1 || !source.buildings.some(building => building.id === task.buildingId)
      || (task.visitMode !== undefined && !["fly-through", "service"].includes(task.visitMode))
      || !Number.isFinite(task.serviceDurationS) || (task.visitMode === "fly-through" ? task.serviceDurationS !== 0 : task.serviceDurationS <= 0)
      || !Array.isArray(task.position) || task.position.length !== 3 || task.position.some((value, axis) => !Number.isFinite(value) || value < source.bounds.min[axis] || value > source.bounds.max[axis]))) throw new Error(`${scenario.id}: invalid source-backed ordered task`);
    if (new Set(tasks.map(task => task.id)).size !== tasks.length) throw new Error(`${scenario.id}: duplicate task IDs`);
    if (scenario.mission.planningScale?.id !== "cross-district-multistop-v2"
      || scenario.mission.planningScale.horizontalDistanceM < 2400 || scenario.mission.planningScale.cityAxisCoverage < 0.55) throw new Error(`${scenario.id}: missing cross-district planning evidence`);
    taskCount += tasks.length;
    const records = scenario.results ?? scenario.runs;
    for (const run of records) {
      if (run.status !== "success") throw new Error(`${scenario.id}: displayed mission did not succeed`);
      const length = run.metrics?.rawLengthM ?? run.plannerMetrics?.executedPathLengthM ?? run.metrics?.executedPathLengthM;
      if (!Number.isFinite(length) || length < 2400) throw new Error(`${scenario.id}: missing recomputed city-scale path evidence`);
      if (track === "demo") {
        for (const [layer, positions] of Object.entries(run.paths)) auditVisits(positions, tasks, null, `${scenario.id}/${run.plannerId}/${layer}`);
      } else if (track === "dynamic") {
        auditVisits(run.frames.map(frame => frame.vehicle), tasks, run.frames.map(frame => frame.timeS), `${scenario.id}/${run.plannerId}`);
        auditVisits(run.frames.at(-1).executedPath, tasks, null, `${scenario.id}/${run.plannerId}/executed`);
      } else {
        for (const layer of ["rawTimedPath", "geometryTimedPath", "executionTimedPath"]) {
          if (!run[layer]?.length) throw new Error(`${scenario.id}/${run.plannerId}: missing ${layer}`);
          auditVisits(run[layer].map(point => point.position), tasks, run[layer].map(point => point.timeS), `${scenario.id}/${run.plannerId}/${layer}`);
        }
        if (run.smoothing.execution.status !== "qualified") throw new Error(`${scenario.id}/${run.plannerId}: execution not qualified`);
      }
      runCount += 1;
    }
    missionCount += 1;
  }
}
console.log(`Validated one official Manhattan city, ${source.buildings.length} physical buildings, ${missionCount} multistop missions, ${taskCount} required stops and ${runCount} freshly computed runs.`);
