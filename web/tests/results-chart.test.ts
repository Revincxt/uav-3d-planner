import { describe, expect, it } from "vitest";
import { barFraction, comparisonGroups, comparisonTicks, metricComparisons, metricNumber, plannerStyle, type ComparisonPlanner } from "../src/results-chart";
const planner = (id = "astar-3d", workUnit = "expanded-nodes", work: number | null = 4000): ComparisonPlanner => ({
  id, label: id, workUnit, summary: { routeM: 2500, planningTimeMs: 40.5, arrivalTimeS: 310, waitTimeS: 0, work },
});

describe("three overall algorithm comparisons", () => {
  it.each([
    ["static", ["routeM", "planningTimeMs", "work"]], ["dynamic", ["routeM", "arrivalTimeS", "work"]],
    ["predictive", ["routeM", "arrivalTimeS", "waitTimeS"]],
  ] as const)("preserves the existing %s three-metric cohort", (kind, keys) => {
    expect(metricComparisons(kind, [planner()]).map(metric => metric.key)).toEqual(keys);
  });
  it("uses kilometres on plots, but does not rescale seconds or work counts", () => {
    const metrics = metricComparisons("predictive", [planner()]);
    expect(metrics[0]!.entries[0]).toMatchObject({ value: 2.5, unit: "km" });
    expect(metrics[1]!.entries[0]).toMatchObject({ value: 310, unit: "s" });
    expect(metrics[2]!.entries[0]).toMatchObject({ value: 0, unit: "s" });
  });
  it("renders vertical columns with one value label per algorithm and no parallel readout", async () => {
    const filesystemModule = "node:fs";
    const { readFileSync } = await import(filesystemModule);
    const source = readFileSync(new URL("../src/results-chart.ts", import.meta.url), "utf8");
    const css = readFileSync(new URL("../src/results.css", import.meta.url), "utf8");
    expect(source).toContain('"--bar-height"');
    expect(css).toContain("height: var(--bar-height)");
    expect(source).not.toMatch(/comparison-row|comparison-track|comparison-full-value|comparison-compact-value/);
    expect(source.match(/node\("output", "comparison-value"/g)).toHaveLength(1);
  });
  it("never compares sample counts, queue pops and node expansions against one shared scale", () => {
    const metric = metricComparisons("static", [planner(), planner("lazy-theta-star", "expanded-nodes", 1800), planner("rrt-star", "samples", 1200)])[2]!;
    const groups = comparisonGroups(metric);
    expect(groups.map(group => group.unit)).toEqual(["nodes", "samples"]);
    expect(groups[0]!.entries.map(entry => entry.id)).toEqual(["astar-3d", "lazy-theta-star"]);
    expect(groups.map(group => group.maximum)).toEqual([5000, 2000]);
    expect(barFraction(1800, groups[0]!.maximum)).toBe(.36);
    const dynamic = metricComparisons("dynamic", [planner(), planner("dstar-lite-3d", "queue-pops", 200)])[2]!;
    expect(comparisonGroups(dynamic).map(group => group.unit)).toEqual(["nodes", "queue pops"]);
  });
  it("preserves true zero waits without drawing a fictitious minimum bar", () => {
    const metric = metricComparisons("predictive", [planner()])[2]!;
    expect(comparisonGroups(metric)[0]!.maximum).toBe(0);
    expect(metricNumber(metric.entries[0]!.value, 1)).toBe("0");
    expect(barFraction(0, 0)).toBe(0);
  });
  it("keeps missing or invalid observations absent rather than showing zero measurements", () => {
    for (const value of [null, NaN, Infinity, -1]) {
      const metric = metricComparisons("static", [planner("a", "samples", value)])[2]!;
      expect(metric.entries[0]!.value).toBeNull(); expect(barFraction(metric.entries[0]!.value, 10)).toBe(0);
      expect(metricNumber(metric.entries[0]!.value, 1)).toBe("—");
    }
  });
  it("uses honest round scale endpoints and never rounds a sub-unit axis to zero", () => {
    const small = planner(); small.summary.routeM = 230;
    const metric = metricComparisons("static", [small])[0]!;
    expect(comparisonGroups(metric)[0]!.maximum).toBe(.5);
    expect(metricNumber(.5, 2, true)).toBe("0.5");
    expect(metricNumber(2000, 0, true)).toBe("2K"); expect(metricNumber(1200.5, 1)).toBe("1,200.5");
  });
  it("keeps algorithm colors stable between independent studies", () => {
    expect(plannerStyle("astar-3d", "A*").color).toBe(plannerStyle("repeated-astar-3d", "Repeated A*").color);
    expect(plannerStyle("lazy-theta-star", "Theta*").color).toBe(plannerStyle("repeated-lazy-theta-star", "Repeated Theta*").color);
    expect(new Set(["repeated-astar-3d", "dstar-lite-reset-3d", "dstar-lite-reuse-3d", "space-time-astar-4d"].map(id => plannerStyle(id, id).color)).size).toBe(4);
  });
  it("uses an explicit logarithmic timing scale only for a hundredfold spread", () => {
    const fast = planner(), slow = planner("rrt-star", "samples"); slow.summary.planningTimeMs = 25000;
    const timing = comparisonGroups(metricComparisons("static", [fast, slow])[1]!)[0]!;
    expect(timing.scale).toBe("log1p"); expect(timing.maximum).toBe(50000);
    expect(barFraction(40.5, timing.maximum, timing.scale)).toBeGreaterThan(.3);
    expect(barFraction(25000, timing.maximum, timing.scale)).toBeGreaterThan(.9);
    slow.summary.planningTimeMs = 400;
    expect(comparisonGroups(metricComparisons("static", [fast, slow])[1]!)[0]!.scale).toBe("linear");
    expect(comparisonGroups(metricComparisons("static", [fast, slow])[2]!)[0]!.scale).toBe("linear");
  });
  it("keeps log-axis zero and missing values honest and aligns ticks with bar heights", () => {
    expect(barFraction(0, 50000, "log1p")).toBe(0); expect(barFraction(null, 50000, "log1p")).toBe(0);
    const ticks = comparisonTicks(50000, "log1p");
    expect(ticks.map(tick => tick.value)).toEqual([0, 10, 100, 1000, 10000, 50000]);
    for (const tick of ticks) expect(tick.fraction).toBe(barFraction(tick.value, 50000, "log1p"));
    expect(comparisonTicks(0, "linear")).toEqual([{ value: 0, fraction: 0 }]);
  });
});
