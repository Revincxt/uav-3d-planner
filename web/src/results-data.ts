import type { DemoBundle } from "./schema";
import type { DynamicBundleV1 } from "./dynamic-schema";
import type { PredictiveBundleV3, PredictiveRun, PredictiveScenario } from "./predictive-schema";

export interface MissionObservation {
  id: string; label: string; success: boolean; routeM: number | null;
  planningTimeMs: number | null; arrivalTimeS: number | null; waitTimeS: number | null;
  work: number; workUnit: string; failureReason: string | null;
}
interface ObservedSummary {
  attempts: number; completed: number; routeM: number | null; planningTimeMs: number | null;
  arrivalTimeS: number | null; waitTimeS: number | null; work: number | null;
}
interface PlannerSummary {
  id: string; label: string; workUnit: string; summary: ObservedSummary;
}
export interface Analysis {
  id: "static" | "dynamic" | "predictive"; label: string;
  planners: PlannerSummary[];
}
export function observedMedian(values: readonly (number | null)[]): number | null {
  const observed = values.filter((value): value is number => value !== null && Number.isFinite(value)).sort((a, b) => a - b);
  if (!observed.length) return null;
  const middle = Math.floor(observed.length / 2);
  return observed.length % 2 ? observed[middle]! : (observed[middle - 1]! + observed[middle]!) / 2;
}
/** Failures remain visible; missing paths never become zero-length observations. */
export function summarizeObservedMissions(missions: readonly MissionObservation[]): ObservedSummary {
  const completed = missions.filter((mission) => mission.success);
  return { attempts: missions.length, completed: completed.length,
    routeM: observedMedian(completed.map((m) => m.routeM)), planningTimeMs: observedMedian(completed.map((m) => m.planningTimeMs)),
    arrivalTimeS: observedMedian(completed.map((m) => m.arrivalTimeS)), waitTimeS: observedMedian(completed.map((m) => m.waitTimeS)),
    work: observedMedian(completed.map((m) => m.work)) };
}
/** Summaries use the same final flight as the map; unavailable flights remain failures. */
export function predictiveObservation(scenario: Pick<PredictiveScenario, "id" | "label">, run: Pick<PredictiveRun, "executionMetrics">): MissionObservation {
  const m = run.executionMetrics;
  if (m === null) return { id: scenario.id, label: scenario.label, success: false, routeM: null,
    planningTimeMs: null, arrivalTimeS: null, waitTimeS: null, work: 0, workUnit: "expanded-nodes", failureReason: "No validated flight" };
  return { id: scenario.id, label: scenario.label, success: m.success, routeM: m.success ? m.executedPathLengthM : null,
    planningTimeMs: null, arrivalTimeS: m.arrivalTimeS, waitTimeS: m.waitTimeS, work: m.expandedStates, workUnit: m.workUnit, failureReason: m.failureReason };
}
function plannerSummary(id: string, label: string, missions: MissionObservation[]): PlannerSummary {
  return { id, label, workUnit: missions[0]?.workUnit ?? "", summary: summarizeObservedMissions(missions) };
}
export function analysesFromBundles(stat: DemoBundle, dyn: DynamicBundleV1, pred: PredictiveBundleV3): Analysis[] {
  const s = stat.scenarios[0]!, d = dyn.scenarios[0]!, p = pred.scenarios[0]!;
  if (!s.city || !d.city || !p.city) throw new Error("Results require the current official Manhattan city datasets.");
  if (new Set([s.city.sourceSha256, d.city.sourceSha256, p.city.sourceSha256].map((digest) => digest.replace(/^sha256:/, ""))).size !== 1) throw new Error("The studies use different Manhattan extracts.");
  return [staticAnalysis(stat), dynamicAnalysis(dyn), predictiveAnalysis(pred)];
}
export function staticAnalysis(stat: DemoBundle): Analysis {
  return { id: "static", label: "Static",
      planners: stat.planners.map((planner) => plannerSummary(planner.id, planner.label,
        stat.scenarios.flatMap((scenario) => scenario.results.filter((run) => run.plannerId === planner.id).map((run) => ({ id: scenario.id, label: scenario.label,
          success: run.status === "success", routeM: run.metrics.smoothedLengthM, planningTimeMs: run.metrics.planningTimeMs, arrivalTimeS: null, waitTimeS: null,
          work: run.searchEffort.value, workUnit: run.searchEffort.kind, failureReason: run.failureReason }))))) };
}
export function dynamicAnalysis(dyn: DynamicBundleV1): Analysis {
  return { id: "dynamic", label: "Dynamic",
      planners: dyn.planners.map((planner) => plannerSummary(planner.id, planner.label,
        dyn.scenarios.flatMap((scenario) => scenario.runs.filter((run) => run.plannerId === planner.id).map((run) => ({ id: scenario.id, label: scenario.label.replace(/ · reactive$/, ""),
          success: run.metrics.success, routeM: run.metrics.success ? run.metrics.executedPathLengthM : null, planningTimeMs: null, arrivalTimeS: run.metrics.completionTimeS,
          waitTimeS: null, work: run.metrics.totalPlanningWork, workUnit: run.metrics.workUnit, failureReason: run.metrics.failureReason }))))) };
}
export function predictiveAnalysis(pred: PredictiveBundleV3): Analysis {
  return { id: "predictive", label: "Predictive",
      planners: pred.planners.map((planner) => plannerSummary(planner.id, planner.label,
        pred.scenarios.flatMap((scenario) => scenario.runs.filter((run) => run.plannerId === planner.id).map((run) => predictiveObservation(scenario, run))))) };
}
