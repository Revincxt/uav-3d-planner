import { describe, expect, it } from "vitest";
import { finalFlight } from "../src/final-flight";
import { validatePredictiveBundle } from "../src/predictive-validation";
import type { PredictiveRun } from "../src/predictive-schema";

const filesystemModule = "node:fs";
const { readFileSync } = await import(filesystemModule);
const bundle = validatePredictiveBundle(JSON.parse(
  readFileSync(new URL("../public/predictive-data.json", import.meta.url), "utf8"),
));
const runs = bundle.scenarios.flatMap(scenario => scenario.runs);

describe("one final flight", () => {
  it("uses matching path, clock, events, metrics and waits for all 32 current flights", () => {
    expect(runs).toHaveLength(32);
    for (const run of runs) {
      const before = JSON.stringify(run);
      const flight = finalFlight(run);
      expect(flight.path).toBe(run.executionTimedPath);
      expect(flight.metrics).toBe(run.executionMetrics);
      expect(flight.frames).toBe(run.executionFrames);
      expect(flight.waits).toBe(run.executionWaitIntervals);
      expect(flight.path.at(-1)!.timeS).toBeCloseTo(flight.metrics.arrivalTimeS!, 6);
      expect(flight.path).not.toBe(run.rawTimedPath);
      expect(flight.path).not.toBe(run.geometryTimedPath);
      expect(JSON.stringify(run)).toBe(before);
    }
  });

  it.each([
    "executionTimedPath", "executionMetrics", "executionFrames", "executionWaitIntervals",
  ] as const)("rejects a missing %s instead of falling back to intermediate results", field => {
    const run = { ...runs[0]!, [field]: null } as PredictiveRun;
    expect(() => finalFlight(run)).toThrow("No validated flight");
  });

  it.each(["qualified", "collisionCertified"] as const)("rejects a failed %s check", field => {
    const run = structuredClone(runs[0]!);
    run.smoothing.execution[field] = false;
    expect(() => finalFlight(run)).toThrow("No validated flight");
  });

  it("rejects an empty path or a missing flight qualification", () => {
    const run = structuredClone(runs[0]!);
    run.executionTimedPath = [];
    expect(() => finalFlight(run)).toThrow("No validated flight");
    run.executionTimedPath = runs[0]!.executionTimedPath;
    run.smoothing.execution.qualification = null;
    expect(() => finalFlight(run)).toThrow("No validated flight");
  });
});
