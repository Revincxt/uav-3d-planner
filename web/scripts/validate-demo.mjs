import { readFile } from "node:fs/promises";

const file = new URL("../public/demo-data.json", import.meta.url);
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

if (data.schemaVersion !== 1) fail("schemaVersion must be 1");
if (data.verificationStatus !== "DEMO_NON_CONFIRMATORY") fail("verification label is missing");
if (!Array.isArray(data.scenarios) || data.scenarios.length < 4) fail("expected four scenes");
const ids = new Set();
for (const scene of data.scenarios) {
  if (ids.has(scene.id)) fail(`duplicate scene ${scene.id}`);
  ids.add(scene.id);
  if (!vec3(scene.start) || !vec3(scene.goal)) fail(`invalid endpoints in ${scene.id}`);
  if (!scene.fingerprint.startsWith("sha256:")) fail(`invalid fingerprint in ${scene.id}`);
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
    }
    if (!result.smoothing.collisionFree) fail(`${result.runId} was not certified`);
  }
}
if (!ids.has(data.defaultScenarioId)) fail("defaultScenarioId is missing");
console.log(`validated ${data.scenarios.length} scenes and ${data.scenarios.length * 3} runs`);

