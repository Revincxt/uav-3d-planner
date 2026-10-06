import { readFile } from "node:fs/promises";
import { createHash } from "node:crypto";
import { resolve } from "node:path";

const file = process.argv[2] ? resolve(process.argv[2]) : new URL("../public/demo-data.json", import.meta.url);
const data = JSON.parse(await readFile(file, "utf8"));

const fail = (message) => {
  throw new Error(`demo-data.json: ${message}`);
};
const vec3 = (value) =>
  Array.isArray(value) && value.length === 3 && value.every((item) => Number.isFinite(item));
const samePoint = (a, b) => a.every((value, index) => Math.abs(value - b[index]) < 1e-8);
const length = (path) =>
  path.slice(1).reduce((sum, point, index) => {
    const previous = path[index];
    return sum + Math.hypot(...point.map((value, axis) => value - previous[axis]));
  }, 0);

const validateAltitudeProfile = (result) => {
  const policy = result.smoothing.altitudePolicy;
  if (policy === undefined) return; // Historical exports retain their original 3D policy.
  if (policy !== "preserve-raw-altitude-profile-v1"
    || JSON.stringify(result.smoothing.optimizationAxes) !== '["x","y"]') {
    fail(`${result.runId} must declare XY-only altitude-preserving smoothing`);
  }
  const raw = result.paths.raw;
  const candidate = result.paths.smoothed;
  const outputProgress = result.smoothing.altitudeProfileProgress;
  if (!Array.isArray(outputProgress) || outputProgress.length !== candidate.length
    || outputProgress.some((value, index) => !Number.isFinite(value) || value < 0 || value > 1
      || (index > 0 && value <= outputProgress[index - 1]))
    || Math.abs(outputProgress[0]) > 1e-9 || Math.abs(outputProgress.at(-1) - 1) > 1e-9
    || !Number.isFinite(result.smoothing.altitudeProfileMaxErrorM)
    || result.smoothing.altitudeProfileMaxErrorM < 0
    || result.smoothing.altitudeProfileMaxErrorM > 1e-6) {
    fail(`${result.runId} has an invalid altitude-preservation progress certificate`);
  }
  const cumulative = [0];
  for (let index = 1; index < raw.length; index += 1) {
    cumulative.push(cumulative.at(-1) + Math.hypot(...raw[index].map((value, axis) => value - raw[index - 1][axis])));
  }
  const total = cumulative.at(-1);
  if (total <= 0) fail(`${result.runId} raw altitude reference must have positive length`);
  const rawProgress = cumulative.map((value) => value / total);
  const heightAt = (points, progress, value) => {
    let index = 0;
    while (index + 1 < progress.length && progress[index + 1] <= value) index += 1;
    if (index === progress.length - 1) return points[index][2];
    const fraction = (value - progress[index]) / (progress[index + 1] - progress[index]);
    return points[index][2] + fraction * (points[index + 1][2] - points[index][2]);
  };
  // Checking the union of both profiles' knots also catches a missing raw peak or slope change.
  for (const value of new Set([...rawProgress, ...outputProgress])) {
    if (Math.abs(heightAt(raw, rawProgress, value) - heightAt(candidate, outputProgress, value)) > 1e-6) {
      fail(`${result.runId} XY smoothing changed its original altitude profile`);
    }
  }
};

if (data.schemaVersion !== 1) fail("schemaVersion must be 1");
if (data.verificationStatus !== "DEMO_NON_CONFIRMATORY") fail("verification label is missing");
if (!Array.isArray(data.scenarios) || data.scenarios.length < 4) fail("expected at least four scenes");
const realCity = data.sourceCommit?.startsWith("local-snapshot:");
if (realCity) {
  if (data.scenarios.length !== 8) fail("expected eight current Manhattan missions");
  const provenance = data.sourceProvenance;
  if (!/^local-snapshot:sha256:[0-9a-f]{64}$/.test(data.sourceCommit)
    || provenance?.kind !== "local-snapshot" || provenance.sha256 !== data.sourceCommit.slice(15)
    || !Array.isArray(provenance.files) || provenance.files.length === 0) {
    fail("real-city data must identify its local source snapshot");
  }
  if (provenance.files.some((entry) => typeof entry.path !== "string" || entry.path.startsWith("/")
    || entry.path.split("/").includes("..") || !/^sha256:[0-9a-f]{64}$/.test(entry.sha256 ?? ""))
    || new Set(provenance.files.map((entry) => entry.path)).size !== provenance.files.length) {
    fail("source snapshot requires unique relative file paths and SHA-256 digests");
  }
  const digest = `sha256:${createHash("sha256").update(JSON.stringify(provenance.files)).digest("hex")}`;
  if (digest !== provenance.sha256) fail("source snapshot manifest digest disagrees");
}
const intersectsBox = (a, b, box, padding) => {
  let near = 0;
  let far = 1;
  for (let axis = 0; axis < 3; axis += 1) {
    const delta = b[axis] - a[axis];
    const lower = box.min[axis] - padding;
    const upper = box.max[axis] + padding;
    if (Math.abs(delta) < 1e-9) {
      if (a[axis] < lower || a[axis] > upper) return false;
    } else {
      const left = (lower - a[axis]) / delta;
      const right = (upper - a[axis]) / delta;
      near = Math.max(near, Math.min(left, right));
      far = Math.min(far, Math.max(left, right));
      if (near > far + 1e-9) return false;
    }
  }
  return true;
};
const intersectsZone = (a, b, zone, padding) => {
  const dz = b[2] - a[2];
  let near = 0;
  let far = 1;
  if (Math.abs(dz) < 1e-9) {
    if (a[2] < zone.zMinM - padding || a[2] > zone.zMaxM + padding) return false;
  } else {
    const left = (zone.zMinM - padding - a[2]) / dz;
    const right = (zone.zMaxM + padding - a[2]) / dz;
    near = Math.max(0, Math.min(left, right));
    far = Math.min(1, Math.max(left, right));
    if (near > far + 1e-9) return false;
  }
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const ox = a[0] - zone.center[0];
  const oy = a[1] - zone.center[1];
  const t = dx * dx + dy * dy <= 1e-9 ? near
    : Math.max(near, Math.min(far, -(ox * dx + oy * dy) / (dx * dx + dy * dy)));
  return (ox + t * dx) ** 2 + (oy + t * dy) ** 2 <= (zone.radiusM + padding) ** 2 + 1e-9;
};
const citySources = new Set();
const ids = new Set();
for (const scene of data.scenarios) {
  if (ids.has(scene.id)) fail(`duplicate scene ${scene.id}`);
  ids.add(scene.id);
  if (!vec3(scene.start) || !vec3(scene.goal)) fail(`invalid endpoints in ${scene.id}`);
  if (!scene.fingerprint.startsWith("sha256:")) fail(`invalid fingerprint in ${scene.id}`);
  if (realCity) {
    if (scene.city?.sourceKind !== "nyc-open-data" || scene.city.collisionModel !== "conservative-aabb"
      || !/^(?:sha256:)?[0-9a-f]{64}$/.test(scene.city.sourceSha256 ?? "")
      || !scene.city.sourceUrl?.startsWith("https://services6.arcgis.com/")) {
      fail(`${scene.id} must retain its official NYC source and collision model`);
    }
    citySources.add(scene.city.sourceSha256);
    const width = scene.bounds.max[0] - scene.bounds.min[0], depth = scene.bounds.max[1] - scene.bounds.min[1];
    const regionId = scene.city.planningRegion?.id;
    const completeExtent = regionId === "midtown-expanded-v3"
      ? width >= 3500 && width <= 3700 && depth >= 3700 && depth <= 3900
      : regionId === "midtown-landscape-v2"
        ? width >= 3500 && depth >= 1800 && width / depth >= 1.7 && width / depth <= 2
        : width >= 2000 && depth >= 2000;
    if (!Array.isArray(scene.buildings) || scene.buildings.length < 1000
      || scene.city.buildingCount !== scene.buildings.length
      || !completeExtent) {
      fail(`${scene.id} must plan on the full physical Manhattan district`);
    }
    const buildingIds = new Set();
    for (const building of scene.buildings) {
      if (buildingIds.has(building.id) || !vec3(building.min) || !vec3(building.max)
        || building.min.some((coordinate, axis) => coordinate >= building.max[axis]
          || coordinate < scene.bounds.min[axis] - 1e-6 || building.max[axis] > scene.bounds.max[axis] + 1e-6)) {
        fail(`${scene.id} contains an invalid physical building`);
      }
      buildingIds.add(building.id);
      if (!Array.isArray(building.footprint) || building.footprint.length === 0) {
        fail(`${scene.id}/${building.id} is missing its original footprint`);
      }
      for (const ring of building.footprint) {
        if (!Array.isArray(ring) || ring.length < 4 || ring.some((point) => !Array.isArray(point)
          || point.length !== 2 || point.some((coordinate, axis) => !Number.isFinite(coordinate)
            || coordinate < building.min[axis] - 1e-5 || coordinate > building.max[axis] + 1e-5))
          || ring[0].some((value, axis) => value !== ring.at(-1)[axis])) {
          fail(`${scene.id}/${building.id} contains an invalid source footprint ring`);
        }
      }
    }
  }
  if (!Array.isArray(scene.results) || scene.results.length !== 3) {
    fail(`expected three planner results in ${scene.id}`);
  }
  for (const result of scene.results) {
    if (result.status !== "success") fail(`${result.runId} is not a successful recorded run`);
    for (const kind of ["raw", "smoothed"]) {
      const path = result.paths?.[kind];
      if (!Array.isArray(path) || path.length < 2 || !path.every(vec3)) {
        fail(`${result.runId} has an invalid ${kind} path`);
      }
      if (!samePoint(path[0], scene.start) || !samePoint(path.at(-1), scene.goal)) {
        fail(`${result.runId} ${kind} endpoints do not match`);
      }
      const recorded = result.metrics[kind === "raw" ? "rawLengthM" : "smoothedLengthM"];
      if (Math.abs(length(path) - recorded) > 0.02) {
        fail(`${result.runId} ${kind} length does not match its metric`);
      }
      const clearance = scene.constraints.vehicleRadiusM + scene.constraints.safetyMarginM;
      for (const [index, point] of path.entries()) {
        if (point.some((coordinate, axis) => coordinate < scene.bounds.min[axis] + clearance - 1e-7
          || coordinate > scene.bounds.max[axis] - clearance + 1e-7)) {
          fail(`${result.runId} ${kind} point ${index} violates the declared bounds clearance`);
        }
        if (index === 0) continue;
        const previous = path[index - 1];
        if (scene.buildings.some((building) => intersectsBox(previous, point, building, clearance))
          || scene.noFlyZones.some((zone) => intersectsZone(previous, point, zone, clearance))) {
          fail(`${result.runId} ${kind} segment ${index} intersects an actual planning obstacle`);
        }
      }
    }
    if (!result.smoothing.collisionFree) fail(`${result.runId} was not certified`);
    validateAltitudeProfile(result);
  }
}
if (realCity && citySources.size !== 1) fail("Manhattan missions must share the same official source geometry");
if (!ids.has(data.defaultScenarioId)) fail("defaultScenarioId is missing");
console.log(`validated ${data.scenarios.length} scenes and ${data.scenarios.length * 3} runs`);
