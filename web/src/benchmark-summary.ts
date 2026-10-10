import type { Analysis } from "./results-data";
export interface BenchmarkSummary {
  schema: "uav-benchmark-v1";
  citySha256: string;
  sources: Record<"demo-data" | "dynamic-data" | "predictive-data", string>;
  analyses: Analysis[];
}
/** Small build-audited summaries, not a second implementation of metric aggregation. */
export function validateBenchmarkSummary(value: unknown): Analysis[] {
  const fail = (): never => { throw new Error("Invalid benchmark summary"); };
  if (!value || typeof value !== "object") return fail();
  const data = value as BenchmarkSummary;
  if (data.schema !== "uav-benchmark-v1" || !/^([a-f\d]{64}|sha256:[a-f\d]{64})$/i.test(data.citySha256)
    || !data.sources || !["demo-data", "dynamic-data", "predictive-data"].every(name => /^[a-f\d]{64}$/i.test(data.sources[name as keyof BenchmarkSummary["sources"]]))
    || !Array.isArray(data.analyses) || data.analyses.length !== 3) return fail();
  for (const [index, analysis] of data.analyses.entries()) {
    if (!analysis || analysis.id !== ["static", "dynamic", "predictive"][index] || typeof analysis.label !== "string"
      || !Array.isArray(analysis.planners) || !analysis.planners.length || new Set(analysis.planners.map(p => p?.id)).size !== analysis.planners.length) return fail();
    for (const planner of analysis.planners) {
      const s = planner?.summary;
      if (!planner || typeof planner.id !== "string" || typeof planner.label !== "string" || typeof planner.workUnit !== "string"
        || !s || !Number.isInteger(s.attempts) || s.attempts < 0 || !Number.isInteger(s.completed) || s.completed < 0 || s.completed > s.attempts) return fail();
      for (const key of ["routeM", "planningTimeMs", "arrivalTimeS", "waitTimeS", "work"] as const) {
        if (s[key] !== null && (typeof s[key] !== "number" || !Number.isFinite(s[key]) || s[key]! < 0)) return fail();
      }
    }
  }
  return data.analyses;
}
/** A missing summary can be rebuilt from validated runtime data; corruption fails closed. */
export async function fetchBenchmarkSummary(base: string): Promise<Analysis[] | null> {
  const response = await fetch(new URL("benchmark-summary.json", base));
  if (response.status === 404) return null;
  if (!response.ok) throw new Error(`Benchmark request failed (${response.status})`);
  return validateBenchmarkSummary(await response.json());
}
