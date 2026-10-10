import { describe, expect, it } from "vitest";
import { observedMedian, predictiveObservation, summarizeObservedMissions, type MissionObservation } from "../src/results";
import type { PredictiveRunMetrics } from "../src/predictive-schema";
function mission(success: boolean, routeM: number | null, arrivalTimeS: number | null): MissionObservation {
  return { id: "test", label: "Test mission", success, routeM, planningTimeMs: null, arrivalTimeS, waitTimeS: 0, work: 30, workUnit: "expanded-nodes", failureReason: success ? null : "timeout" };
}
describe("observed Manhattan mission results", () => {
  it("removes the old study heading, counter, per-mission lists and their styles", async () => {
    const filesystemModule = "node:fs";
    const { readFileSync } = await import(filesystemModule);
    const read = (path: string) => readFileSync(new URL(`../${path}`, import.meta.url), "utf8");
    expect(read("results.html")).not.toMatch(/Mission studies|case-count|sidebar-heading/);
    expect(read("src/benchmark-panel.ts")).toContain('charts.id = "comparison-charts"');
    expect(read("results.html")).toContain('id="benchmark-map"');
    for (const path of ["src/results.ts", "src/results.css", "src/simulator-ui.css"]) {
      expect(read(path)).not.toMatch(/mission-outcomes|mission-outcome|outcome-values|outcome-name|median-heading|summary-completion/);
    }
  });
  it("uses charts alone without counts, inspector controls or repeated summary cards", async () => {
    const filesystemModule = "node:fs";
    const { readFileSync } = await import(filesystemModule);
    const read = (path: string) => readFileSync(new URL(`../${path}`, import.meta.url), "utf8");
    expect(read("results.html")).not.toMatch(/summary-body|inspector|dataset-metrics|official-source/);
    expect(read("src/results.ts")).not.toMatch(/mountInspector|renderCards|renderDataset|missions ·|planners`/);
    expect(read("src/workspace.ts")).not.toContain("copy.append(title, subtitle)");
    for (const path of ["src/results.css", "src/simulator-ui.css"]) {
      expect(read(path)).not.toMatch(/planner-summary|summary-measure/);
    }
  });
  it("shows all study charts together without a sidebar or study selector", async () => {
    const filesystemModule = "node:fs";
    const { readFileSync } = await import(filesystemModule);
    const read = (path: string) => readFileSync(new URL(`../${path}`, import.meta.url), "utf8");
    expect(read("results.html")).not.toMatch(/case-sidebar|case-library|scenario-select|analysis-title/);
    expect(read("src/results.ts")).not.toMatch(/mountWorkspace|select\.value|select\.addEventListener/);
    expect(read("src/benchmark-panel.ts")).toContain("analyses.map(renderStudy)");
    expect(read("src/results.css")).toContain("grid-template-rows: repeat(3, minmax(0, 1fr))");
  });
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
    const observation = predictiveObservation({ id: "mission", label: "Mission" }, run);
    expect(observation.routeM).toBe(1500); expect(observation.arrivalTimeS).toBe(400); expect(observation.waitTimeS).toBe(2); expect(observation.success).toBe(true); expect(observation.planningTimeMs).toBeNull();
  });
  it("does not substitute raw metrics when a final flight is missing", () => {
    const observation = predictiveObservation({ id: "mission", label: "Mission" }, { executionMetrics: null });
    expect(observation.success).toBe(false); expect(observation.routeM).toBeNull();
    expect(observation.arrivalTimeS).toBeNull(); expect(observation.waitTimeS).toBeNull();
  });
});
