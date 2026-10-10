import type { OverviewRoute, RouteWaypoint } from "./route-overview";

export type PlanMeaning = "fixed" | "local" | "scheduled" | "recorded";
export function planMeaning(route: OverviewRoute): PlanMeaning {
  return route.playbackKind === "fixed" ? "fixed" : route.plannedFrames ? "local"
    : route.playbackKind === "predictive" ? "scheduled" : "recorded";
}
export const PLAN_LABELS: Record<PlanMeaning, string> = {
  fixed: "Fixed route", local: "Local plan", scheduled: "Remaining route", recorded: "Remaining route",
};

export interface FlightPhase {
  kind: "ready" | "flying" | "climbing" | "descending" | "waiting" | "arrived";
  remainingS?: number;
  reason?: string;
}

/** Holds come from equal-position intervals, never from frame rate or low screen motion. */
export function flightPhase(route: Pick<OverviewRoute, "timedPath" | "waits">, timeS: number): FlightPhase {
  const path = route.timedPath;
  if (!path?.length || timeS < path[0]!.timeS) return { kind: "ready" };
  if (timeS >= path.at(-1)!.timeS) return { kind: "arrived" };
  let low = 0, high = path.length;
  while (low < high) {
    const middle = (low + high) >>> 1;
    if (path[middle]!.timeS <= timeS) low = middle + 1; else high = middle;
  }
  const index = Math.max(0, low - 1), left = path[index]!, right = path[index + 1]!;
  if (Math.hypot(...right.position.map((p, axis) => p - left.position[axis]!)) < 1e-8) {
    let end = index + 1;
    while (end + 1 < path.length && samePosition(left, path[end + 1]!)) end++;
    const wait = route.waits?.find(w => w.startTimeS <= timeS && timeS < w.endTimeS);
    return { kind: "waiting", remainingS: path[end]!.timeS - timeS, reason: wait?.reason ?? "Safety hold" };
  }
  const horizontal = Math.hypot(right.position[0] - left.position[0], right.position[1] - left.position[1]);
  const vertical = right.position[2] - left.position[2];
  if (Math.abs(vertical) > horizontal * 2) return { kind: vertical > 0 ? "climbing" : "descending" };
  return { kind: timeS === path[0]!.timeS ? "ready" : "flying" };
}
const samePosition = (a: RouteWaypoint, b: RouteWaypoint): boolean =>
  Math.hypot(...a.position.map((p, i) => p - b.position[i]!)) < 1e-8;

export function phaseLabel(phase: FlightPhase): string {
  if (phase.kind !== "waiting") return phase.kind[0]!.toUpperCase() + phase.kind.slice(1);
  const reason = /lattice|alignment/i.test(phase.reason ?? "") ? "Time alignment"
    : /brak/i.test(phase.reason ?? "") ? "Braking"
    : /forecast|departure|window/i.test(phase.reason ?? "") ? "Await slot" : "Safety hold";
  return `${reason} · ${Math.ceil(phase.remainingS ?? 0)} s`;
}

/** The eight mission chips distinguish completed aircraft from actual safety holds. */
export class RouteStatusStrip {
  private buttons: HTMLButtonElement[] = [];
  private lastMs = -Infinity;
  update(routes: readonly OverviewRoute[], timeS: number, force = false): void {
    // Dynamic/Predictive mount their mission buttons after the first time sample.
    if (!this.buttons.length) this.buttons = [...document.querySelectorAll<HTMLButtonElement>(".route-legend button[data-route]")];
    const now = performance.now();
    if (!force && now - this.lastMs < 100) return;
    this.lastMs = now;
    for (const button of this.buttons) {
      const route = routes.find(r => r.id === button.dataset.route);
      if (!route) continue;
      const phase = flightPhase(route, timeS);
      if (button.dataset.flightPhase !== phase.kind) button.dataset.flightPhase = phase.kind;
      const title = `${route.label} · ${phaseLabel(phase)}`;
      if (button.title !== title) button.title = title;
    }
  }
}
