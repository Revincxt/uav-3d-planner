import { describe, expect, it } from "vitest";
import { observedMedian, predictiveObservation, summarizeObservedMissions, type MissionObservation } from "../src/results-data";
import type { PredictiveRunMetrics } from "../src/predictive-schema";
function mission(success: boolean, routeM: number | null, arrivalTimeS: number | null): MissionObservation {
  return { success, routeM, planningTimeMs: null, arrivalTimeS, waitTimeS: 0, work: 30, workUnit: "expanded-nodes" };
}
describe("observed Manhattan mission results", () => {
  it("keeps zero waits and excludes missing values without inventing zero observations", () => {
    expect(observedMedian([null, Number.NaN])).toBeNull(); expect(observedMedian([0, 0, 2, 4])).toBe(1); expect(observedMedian([3, null, 1])).toBe(2);
  });
  it("shows failed attempts but uses completed missions only for route and arrival summaries", () => {
    const summary = summarizeObservedMissions([mission(true, 1200, 80), mission(true, 1800, 120), mission(false, 9000, 600)]);
    expect(summary.attempts).toBe(3); expect(summary.completed).toBe(2); expect(summary.routeM).toBe(1500); expect(summary.arrivalTimeS).toBe(100); expect(summary.planningTimeMs).toBeNull();
  });
  it("summarizes the final predictive flight shown on the map, not intermediate paths", () => {
    const plannerMetrics = { success: true, executedPathLengthM: 1800, arrivalTimeS: 120, waitTimeS: 2, expandedStates: 100, workUnit: "expanded-spacetime-states", failureReason: null } as PredictiveRunMetrics;
    const run = { plannerMetrics, geometryMetrics: { ...plannerMetrics, executedPathLengthM: 1500, arrivalTimeS: 100 }, executionMetrics: { ...plannerMetrics, executedPathLengthM: 1500, arrivalTimeS: 400 }, status: "invalid" };
    const observation = predictiveObservation(run);
    expect(observation.routeM).toBe(1500); expect(observation.arrivalTimeS).toBe(400); expect(observation.waitTimeS).toBe(2); expect(observation.success).toBe(true); expect(observation.planningTimeMs).toBeNull();
  });
  it("does not substitute raw metrics when a final flight is missing", () => {
    const observation = predictiveObservation({ executionMetrics: null });
    expect(observation.success).toBe(false); expect(observation.routeM).toBeNull();
    expect(observation.arrivalTimeS).toBeNull(); expect(observation.waitTimeS).toBeNull();
    for (const field of ["id", "label", "failureReason"]) expect(observation).not.toHaveProperty(field);
  });
});
