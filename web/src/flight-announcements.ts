import type { DynamicRun, DynamicScenario, Vec3 } from "./dynamic-schema";
import type { PredictiveRun } from "./predictive-schema";
import { finalFlight } from "./final-flight";

export interface AvoidanceEvent {
  id: string;
  timeS: number;
  kind: "reroute" | "hold" | "resume";
  message: string;
}

const distance2 = (a: readonly number[], b: readonly number[]): number =>
  a.reduce((sum, value, i) => sum + (value - b[i]!) ** 2, 0);

function segmentDistance2(a: readonly number[], b: readonly number[], point: readonly number[]): number {
  const length = distance2(a, b);
  const fraction = length ? Math.max(0, Math.min(1,
    a.reduce((sum, value, i) => sum + (point[i]! - value) * (b[i]! - value), 0) / length)) : 0;
  return a.reduce((sum, value, i) => sum + (value + (b[i]! - value) * fraction - point[i]!) ** 2, 0);
}

/** Drop consumed geometry; a hazard behind the UAV cannot explain a new detour. */
function remainingPlan(path: Vec3[], position: Vec3): Vec3[] {
  let closest = Infinity, index = 0;
  for (let i = 0; i + 1 < path.length; i++) {
    const distance = segmentDistance2(path[i]!, path[i + 1]!, position);
    if (distance < closest) { closest = distance; index = i; }
  }
  return [position, ...path.slice(index + 1)];
}

function hitsCylinder(a: Vec3, b: Vec3, zone: DynamicScenario["temporaryNoFlyZones"][number], padding: number): boolean {
  const lowZ = zone.zMinM - padding, highZ = zone.zMaxM + padding, dz = b[2] - a[2];
  let from = 0, to = 1;
  if (Math.abs(dz) < 1e-10) {
    if (a[2] < lowZ || a[2] > highZ) return false;
  } else {
    const first = (lowZ - a[2]) / dz, last = (highZ - a[2]) / dz;
    from = Math.max(0, Math.min(first, last)); to = Math.min(1, Math.max(first, last));
    if (from > to) return false;
  }
  const xy = (fraction: number) => [a[0] + (b[0] - a[0]) * fraction, a[1] + (b[1] - a[1]) * fraction];
  return segmentDistance2(xy(from), xy(to), zone.center) <= (zone.radiusM + padding) ** 2;
}

/** Display-only causal evidence, never a replacement for the native collision certificate.
 * Scheduled updates count only when the remaining old plan is blocked by a current
 * dynamic volume and the newly recorded plan avoids that same volume. */
export function dynamicAvoidanceEvents(scenario: DynamicScenario, run: DynamicRun): AvoidanceEvent[] {
  const events: AvoidanceEvent[] = [];
  const padding = scenario.constraints.vehicleRadiusM + scenario.constraints.safetyMarginM;
  let lastEpisodeTime = -Infinity;
  for (let i = 1; i < run.frames.length; i++) {
    const frame = run.frames[i]!, previous = run.frames[i - 1]!;
    const protectedMove = frame.replanReason === "safety-gate";
    if (!protectedMove && (!frame.replanned || frame.plannerSuccess !== true || frame.replanReason !== "scheduled")) continue;
    const oldPlan = remainingPlan(previous.path, frame.vehicle), plan = frame.path;
    const intersects = (points: Vec3[], test: (a: Vec3, b: Vec3) => boolean): boolean =>
      points.slice(1).some((point, index) => test(points[index]!, point));
    let cause: "traffic" | "airspace" | undefined;
    // Do not mistake a new mission leg or numerical resampling for an obstacle detour.
    if (oldPlan.length > 1 && plan.length > 1 && distance2(oldPlan.at(-1)!, plan.at(-1)!) < 1e-6) {
      for (const aircraft of frame.movingSpheres) {
        const test = (a: Vec3, b: Vec3) => segmentDistance2(a, b, aircraft.position) <= (aircraft.radiusM + padding) ** 2;
        if (intersects(oldPlan, test) && !intersects(plan, test)) { cause = "traffic"; break; }
      }
      if (!cause) for (const zone of scenario.temporaryNoFlyZones) {
        if (!frame.activeTemporaryZoneIds.includes(zone.id)) continue;
        const test = (a: Vec3, b: Vec3) => hitsCylinder(a, b, zone, padding);
        if (intersects(oldPlan, test) && !intersects(plan, test)) { cause = "airspace"; break; }
      }
    }
    if (!protectedMove && !cause) continue;
    const moving = run.frames[i + 1] && distance2(frame.vehicle, run.frames[i + 1]!.vehicle) > 1e-8;
    const kind = moving ? "reroute" : "hold";
    const message = kind === "hold" ? "避障保护，等待安全通行"
      : cause === "traffic" ? "避让障碍无人机，航路已调整"
      : cause === "airspace" ? "避让动态空域，航路已调整" : "动态避障，航路已调整";
    const last = events.at(-1);
    if (last?.kind === kind && last.message === message && frame.timeS - lastEpisodeTime <= 6) {
      lastEpisodeTime = frame.timeS; continue;
    }
    events.push({ id: `replan-${i}`, timeS: frame.timeS, kind, message });
    lastEpisodeTime = frame.timeS;
  }
  return events;
}

/** Only explicit safety/forecast waits qualify. Time-lattice alignment is not avoidance. */
export function predictiveAvoidanceEvents(run: PredictiveRun): AvoidanceEvent[] {
  const waits = finalFlight(run).waits;
  const episodes: typeof waits = [];
  for (const wait of waits) {
    if (!/forecast|safety|obstacle|traffic|dynamic/i.test(wait.reason) || wait.endTimeS <= wait.startTimeS) continue;
    const previous = episodes.at(-1);
    if (previous && Math.abs(previous.endTimeS - wait.startTimeS) < 1e-6 && distance2(previous.position, wait.position) < 1e-8) {
      previous.endTimeS = wait.endTimeS;
    } else episodes.push({ ...wait });
  }
  return episodes.flatMap((wait, index): AvoidanceEvent[] => [
    { id: `avoidance-wait-${index}`, timeS: wait.startTimeS, kind: "hold",
      message: run.predictive ? "预测避让，等待动态空域窗口" : "避障保护，等待安全通行" },
    { id: `avoidance-resume-${index}`, timeS: wait.endTimeS, kind: "resume", message: "恢复飞行，继续任务" },
  ]).sort((a, b) => a.timeS - b.timeS);
}

/** Focused-UAV visual announcements only. Explicit seeks, pauses and route changes are silent. */
export class FlightAnnouncements {
  private readonly root = document.createElement("output");
  private readonly copy = document.createElement("span");
  private routeId = "";
  private seen = new Set<string>();
  private timer?: ReturnType<typeof setTimeout>;

  constructor(container: HTMLElement) {
    this.root.className = "flight-announcement"; this.root.hidden = true;
    this.root.setAttribute("role", "status"); this.root.setAttribute("aria-live", "polite");
    this.root.setAttribute("aria-atomic", "true");
    this.root.innerHTML = '<svg viewBox="0 0 20 20" fill="none" aria-hidden="true"><path d="m3 13 4-4 4 2 6-7m-5 0h5v5" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" /></svg>';
    this.root.append(this.copy); container.append(this.root);
  }

  advance(routeId: string, index: number, events: readonly AvoidanceEvent[], fromS: number, toS: number): void {
    if (!Number.isFinite(fromS) || !Number.isFinite(toS) || toS < fromS) { this.reset(); return; }
    if (toS === fromS) return;
    if (this.routeId !== routeId) { this.reset(); this.routeId = routeId; }
    const crossed = events.filter(event => (event.timeS > fromS || (fromS === 0 && event.timeS === 0)) && event.timeS <= toS && !this.seen.has(event.id));
    if (!crossed.length) return;
    crossed.forEach(event => this.seen.add(event.id));
    const event = crossed.at(-1)!;
    this.copy.textContent = `UAV ${String(index + 1).padStart(2, "0")} · ${event.message}`;
    this.root.dataset.kind = event.kind; this.root.dataset.routeId = routeId; this.root.dataset.timeS = String(event.timeS);
    this.root.hidden = false;
    if (this.timer) clearTimeout(this.timer);
    this.timer = setTimeout(() => this.suspend(), 4200);
  }

  suspend(): void {
    if (this.timer) clearTimeout(this.timer);
    this.timer = undefined; this.root.hidden = true; this.copy.textContent = "";
  }

  reset(): void {
    this.suspend(); this.seen.clear(); this.routeId = "";
    delete this.root.dataset.kind; delete this.root.dataset.routeId; delete this.root.dataset.timeS;
  }

  dispose(): void { this.reset(); this.root.remove(); }
}
