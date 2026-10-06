import type { OverviewRoute } from "./route-overview";
import type { MissionTaskPoint } from "./city-schema";
import "./task-arrival-notice.css";

interface Arrival { task: MissionTaskPoint; timeS: number }
const cache = new WeakMap<OverviewRoute, Arrival[]>();

/** Actual ordered trajectory knots, not proximity to a decorative gate. */
export function taskArrivals(route: OverviewRoute): Arrival[] {
  const previous = cache.get(route);
  if (previous) return previous;
  const arrivals: Arrival[] = [], path = route.timedPath ?? [];
  let cursor = 0;
  for (const task of route.mission?.taskPoints ?? []) {
    while (cursor < path.length && Math.hypot(...task.position.map((v, axis) => v - path[cursor]!.position[axis]!)) > 1e-5) cursor++;
    if (cursor === path.length) throw new Error(`No actual arrival for ${task.id}`);
    arrivals.push({ task, timeS: path[cursor]!.timeS }); cursor++;
  }
  cache.set(route, arrivals); return arrivals;
}

export function crossedTaskArrivals(route: OverviewRoute, fromS: number, toS: number): Arrival[] {
  if (!Number.isFinite(fromS) || !Number.isFinite(toS) || toS <= fromS) return [];
  return taskArrivals(route).filter(arrival => arrival.timeS > fromS && arrival.timeS <= toS);
}

/** Only forward playback in chase mode can announce a checkpoint; seeking is silent. */
export class TaskArrivalNotice {
  private readonly root: HTMLOutputElement;
  private timer?: ReturnType<typeof setTimeout>;
  private route?: OverviewRoute;
  private seen = new Set<string>();
  constructor(container: HTMLElement) {
    this.root = document.createElement("output"); this.root.className = "task-arrival-notice";
    this.root.id = "task-arrival-notice"; this.root.hidden = true;
    this.root.setAttribute("role", "status"); this.root.setAttribute("aria-live", "polite");
    this.root.setAttribute("aria-atomic", "true"); container.append(this.root);
  }
  advance(route: OverviewRoute | undefined, fromS: number, toS: number): void {
    if (!route || toS < fromS) { this.reset(); return; }
    if (this.route !== route) { this.reset(); this.route = route; }
    const arrivals = crossedTaskArrivals(route, fromS, toS).filter(arrival => !this.seen.has(arrival.task.id));
    if (!arrivals.length) return;
    arrivals.forEach(arrival => this.seen.add(arrival.task.id));
    const arrival = arrivals.at(-1)!;
    this.root.textContent = `✓ 任务点 ${String(arrival.task.order).padStart(2, "0")} 已通过`;
    this.root.dataset.taskId = arrival.task.id; this.root.dataset.routeId = route.id;
    this.root.hidden = false;
    if (this.timer) clearTimeout(this.timer);
    this.timer = setTimeout(() => { this.root.hidden = true; this.root.textContent = ""; this.timer = undefined; }, 1800);
  }
  reset(): void {
    if (!this.timer && !this.route && !this.seen.size && this.root.hidden && !this.root.textContent) return;
    if (this.timer) clearTimeout(this.timer);
    this.timer = undefined; this.route = undefined; this.seen.clear();
    this.root.hidden = true; this.root.textContent = "";
    delete this.root.dataset.taskId; delete this.root.dataset.routeId;
  }
  dispose(): void { this.reset(); this.root.remove(); }
}
