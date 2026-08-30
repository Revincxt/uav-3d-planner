import { describe, expect, it } from "vitest";

import type {
  PredictiveMinimumSeparationWitness,
  PredictiveRun,
  PredictiveScenario,
} from "../src/predictive-schema";
import {
  isMinimumSeparationEvidenceTime,
  minimumSeparationEvidence,
} from "../src/predictive-viewer";

const scenario = {
  constraints: { vehicleRadiusM: 0.75, safetyMarginM: 1 },
} satisfies Pick<PredictiveScenario, "constraints">;

function runWithWitness(
  witness: PredictiveMinimumSeparationWitness | null,
): Pick<PredictiveRun, "plannerMetrics" | "geometryMetrics" | "executionMetrics"> {
  const metrics = { minimumSeparationWitness: witness } as PredictiveRun["geometryMetrics"];
  return {
    plannerMetrics: metrics,
    geometryMetrics: metrics,
    executionMetrics: metrics,
  };
}

function witness(exact: boolean): PredictiveMinimumSeparationWitness {
  return {
    separationM: 2.5,
    timeS: 12.3456789,
    vehiclePosition: [4, 5, 6],
    obstacleId: "hazard-1",
    obstacleKind: exact ? "moving-sphere" : "temporary-cylinder",
    obstaclePosition: [7, 5, 6],
    declaredSafetyMarginM: 1,
    method: exact ? "exact-relative-linear-motion" : "deterministic-convex-search",
    exact,
  };
}

describe("minimum-separation viewer evidence", () => {
  it("derives the declared-margin envelope and an exact solid connector", () => {
    const evidence = minimumSeparationEvidence(
      scenario,
      runWithWitness(witness(true)),
      "geometry",
    );

    expect(evidence).not.toBeNull();
    expect(evidence?.safetyEnvelopeRadiusM).toBe(1.75);
    expect(evidence?.connectorDashed).toBe(false);
    expect(isMinimumSeparationEvidenceTime(evidence, 12.3456789)).toBe(true);
    expect(isMinimumSeparationEvidenceTime(evidence, 12.34567895)).toBe(true);
    expect(isMinimumSeparationEvidenceTime(evidence, 12.3457)).toBe(false);
  });

  it("uses a dashed connector for approximate evidence", () => {
    const evidence = minimumSeparationEvidence(
      scenario,
      runWithWitness(witness(false)),
      "execution",
    );

    expect(evidence?.connectorDashed).toBe(true);
    expect(evidence?.witness.exact).toBe(false);
  });

  it("returns no renderable evidence when the run has no witness", () => {
    const evidence = minimumSeparationEvidence(scenario, runWithWitness(null), "raw");

    expect(evidence).toBeNull();
    expect(isMinimumSeparationEvidenceTime(evidence, 0)).toBe(false);
  });

  it("keeps witnesses scoped to the selected evidence layer", () => {
    const exact = witness(true);
    const metrics = { minimumSeparationWitness: exact } as PredictiveRun["geometryMetrics"];
    const run = {
      plannerMetrics: { minimumSeparationWitness: null } as PredictiveRun["plannerMetrics"],
      geometryMetrics: metrics,
      executionMetrics: null,
    };

    expect(minimumSeparationEvidence(scenario, run, "raw")).toBeNull();
    expect(minimumSeparationEvidence(scenario, run, "geometry")?.witness).toBe(exact);
    expect(minimumSeparationEvidence(scenario, run, "execution")).toBeNull();
  });
});
