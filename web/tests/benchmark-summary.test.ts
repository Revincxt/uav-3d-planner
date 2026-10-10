import { afterEach, describe, expect, it, vi } from "vitest";
import { fetchBenchmarkSummary, validateBenchmarkSummary, type BenchmarkSummary } from "../src/benchmark-summary";
const data = (): BenchmarkSummary => ({ schema: "uav-benchmark-v1", citySha256: "a".repeat(64),
  sources: { "demo-data": "b".repeat(64), "dynamic-data": "c".repeat(64), "predictive-data": "d".repeat(64) },
  analyses: (["static", "dynamic", "predictive"] as const).map(id => ({ id, label: id, planners: [{ id: "astar", label: "A*", workUnit: "nodes", summary: {
    attempts: 8, completed: 8, routeM: 2300, planningTimeMs: null, arrivalTimeS: 145, waitTimeS: 0, work: 200,
  } }] })) });
afterEach(() => vi.unstubAllGlobals());
describe("compact build-audited benchmark", () => {
  it("preserves missing observations, real zeros, cohorts and actual metrics", () => {
    const value = data(); expect(validateBenchmarkSummary(value)).toEqual(value.analyses);
    expect(validateBenchmarkSummary(value)[2]!.planners[0]!.summary.waitTimeS).toBe(0);
  });
  it.each(["NaN", "negative", "missing", "cohort", "source", "city", "duplicate", "order"])("rejects %s summaries", kind => {
    const value = data(), summary = value.analyses[0]!.planners[0]!.summary;
    if (kind === "NaN") summary.routeM = NaN;
    if (kind === "negative") summary.routeM = -1;
    if (kind === "missing") delete (summary as Partial<typeof summary>).routeM;
    if (kind === "cohort") summary.completed = 9;
    if (kind === "source") value.sources["demo-data"] = "stale";
    if (kind === "city") value.citySha256 = "";
    if (kind === "duplicate") value.analyses[0]!.planners.push(value.analyses[0]!.planners[0]!);
    if (kind === "order") value.analyses.reverse();
    expect(() => validateBenchmarkSummary(value)).toThrow("Invalid benchmark");
  });
  it("opens with one tiny request, without fetching any trajectory datasets", async () => {
    const fetcher = vi.fn().mockResolvedValue(new Response(JSON.stringify(data()))); vi.stubGlobal("fetch", fetcher);
    await expect(fetchBenchmarkSummary("https://example.test/uav/")).resolves.toEqual(data().analyses);
    expect(fetcher).toHaveBeenCalledTimes(1); expect(String(fetcher.mock.calls[0]![0])).toBe("https://example.test/uav/benchmark-summary.json");
  });
  it("falls back only for a missing legacy asset, never for corruption or server failure", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue(new Response("", { status: 404 })));
    await expect(fetchBenchmarkSummary("https://example.test/")).resolves.toBeNull();
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue(new Response("{}")));
    await expect(fetchBenchmarkSummary("https://example.test/")).rejects.toThrow("Invalid benchmark");
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue(new Response("", { status: 503 })));
    await expect(fetchBenchmarkSummary("https://example.test/")).rejects.toThrow("503");
  });
});
