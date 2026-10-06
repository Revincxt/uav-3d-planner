import type { PredictiveRun } from "./predictive-schema";

export interface FinalFlight {
  path: NonNullable<PredictiveRun["executionTimedPath"]>;
  metrics: NonNullable<PredictiveRun["executionMetrics"]>;
  frames: NonNullable<PredictiveRun["executionFrames"]>;
  waits: NonNullable<PredictiveRun["executionWaitIntervals"]>;
}

/** The replay has one result. Never substitute an unchecked intermediate path. */
export function finalFlight(run: PredictiveRun): FinalFlight {
  const qualification = run.smoothing.execution;
  if (
    !qualification.qualified || !qualification.collisionCertified ||
    qualification.status !== "qualified" || !qualification.qualification?.qualified ||
    run.executionTimedPath === null || run.executionTimedPath.length === 0 ||
    run.executionMetrics === null || run.executionFrames === null ||
    run.executionWaitIntervals === null
  ) {
    throw new Error(`No validated flight available for ${run.plannerId}`);
  }
  return {
    path: run.executionTimedPath,
    metrics: run.executionMetrics,
    frames: run.executionFrames,
    waits: run.executionWaitIntervals,
  };
}
