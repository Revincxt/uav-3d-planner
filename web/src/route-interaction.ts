import "./route-interaction.css";
import { routeColorCSS, type OverviewRoute } from "./route-overview";
import { flightPhase, phaseLabel } from "./trajectory-semantics";
import { taskArrivals } from "./task-arrival-notice";
import { uiSymbol } from "./ui-symbol";
import type { RoutePick } from "./route-picking";
import { PointerClickGuard, RouteClickState, positionRouteCard } from "./route-interaction-state";
interface RouteViewer {
  pickRouteAt(x: number, y: number, radius?: number): RoutePick | null;
  setRouteSelection(id: string | null): void;
  setFollowRoute(id: string | null): void;
  readonly followedRouteId: string | null;
}
const summaries = new WeakMap<OverviewRoute, { lengthM: number; durationS: number }>();
export function routeTaskSummary(route: OverviewRoute): { lengthM: number; durationS: number } {
  const previous = summaries.get(route); if (previous) return previous;
  const points = route.timedPath?.map(w => w.position) ?? route.points;
  let lengthM = 0; for (let i = 1; i < points.length; i++) lengthM += Math.hypot(...points[i]!.map((v, axis) => v - points[i - 1]![axis]!));
  const summary = { lengthM, durationS: route.timedPath?.length ? route.timedPath.at(-1)!.timeS - route.timedPath[0]!.timeS : 0 };
  summaries.set(route, summary); return summary;
}
const flightDuration = (seconds: number): string => `${Math.floor(seconds / 60)}:${String(Math.floor(seconds % 60)).padStart(2, "0")}`;

/** One retained card and pointer listeners; no playback loop, plan mutation or synthetic movement. */
export class RouteInteraction {
  private readonly root = document.createElement("section");
  private readonly state = new RouteClickState();
  private readonly gesture = new PointerClickGuard();
  private readonly events = new AbortController();
  private readonly resize: ResizeObserver;
  private readonly canvas: HTMLCanvasElement | null;
  private readonly overlay: HTMLElement;
  private route?: OverviewRoute;
  private anchor = { x: 0, y: 0 };
  private timeS = 0;
  private lastMs = -Infinity;
  private hover?: number;
  private lastHover = -Infinity;
  private hoverPoint?: { x: number; y: number };
  constructor(host: HTMLElement, private readonly viewer: RouteViewer | null,
    private readonly select: HTMLSelectElement, private readonly routes: () => readonly OverviewRoute[]) {
    this.root.className = "route-task-card"; this.root.hidden = true;
    this.root.setAttribute("role", "region"); this.root.setAttribute("aria-label", "Selected flight task");
    this.root.innerHTML = `<header class="route-card-header"><span class="route-card-aircraft"><strong data-field="aircraft"></strong></span><span class="route-card-phase"></span><button class="route-card-close" type="button" aria-label="Close task card" title="Close · Esc"></button></header>
      <h3 class="route-card-title"></h3><p class="route-card-purpose"></p>
      <div class="route-card-endpoints"><span data-field="origin"></span><span data-field="destination"></span></div>
      <dl class="route-card-metrics"><div><dt>Path</dt><dd data-field="length"></dd></div><div><dt>Flight</dt><dd data-field="duration"></dd></div><div><dt>Stops</dt><dd data-field="stops"></dd></div></dl>
      <div class="route-card-stops" aria-label="Required checkpoint progress"></div>
      <button type="button" class="route-card-follow"><span>Follow UAV</span></button>`;
    this.root.querySelector(".route-card-aircraft")!.prepend(uiSymbol("route"));
    this.root.querySelector(".route-card-close")!.append(uiSymbol("close"));
    this.root.querySelector(".route-card-follow")!.prepend(uiSymbol("follow"));
    this.overlay = host.closest<HTMLElement>(".stage-viewport") ?? host;
    this.overlay.append(this.root); this.canvas = host.querySelector("canvas");
    if (this.canvas && this.canvas.tabIndex < 0) this.canvas.tabIndex = 0;
    const options = { signal: this.events.signal };
    this.root.querySelector(".route-card-close")!.addEventListener("click", () => this.clear(), options);
    this.root.querySelector(".route-card-follow")!.addEventListener("click", () => this.follow(), options);
    this.select.addEventListener("change", () => { if (this.state.selectedId !== this.select.value) this.clear(); }, options);
    document.addEventListener("keydown", event => { if (event.key === "Escape" && !this.root.hidden) this.clear(); }, options);
    window.addEventListener("blur", () => this.gesture.reset(), options);
    if (this.canvas && viewer) {
      this.canvas.addEventListener("pointerdown", event => { this.cancelHover(); this.gesture.down(event); }, options);
      this.canvas.addEventListener("pointermove", event => {
        this.gesture.move(event);
        if (event.buttons || event.pointerType === "touch") return;
        this.hoverPoint = { x: event.clientX, y: event.clientY };
        if (this.hover === undefined && performance.now() - this.lastHover >= 60) this.hover = requestAnimationFrame(() => {
          this.hover = undefined; this.lastHover = performance.now();
          if (this.hoverPoint && this.canvas) this.canvas.style.cursor = viewer.pickRouteAt(this.hoverPoint.x, this.hoverPoint.y) ? "pointer" : "";
        });
      }, options);
      this.canvas.addEventListener("pointerup", event => {
        if (!this.gesture.up(event)) return;
        const picked = viewer.pickRouteAt(event.clientX, event.clientY, event.pointerType === "touch" ? 12 : 7);
        const action = this.state.click(picked?.id ?? null);
        if (action === "clear") { this.clear(); return; }
        if (action === "follow") { this.follow(); return; }
        if (viewer.followedRouteId) viewer.setFollowRoute(null);
        if (this.select.value !== picked!.id) { this.select.value = picked!.id; this.select.dispatchEvent(new Event("change", { bubbles: true })); }
        viewer.setRouteSelection(picked!.id);
        const rect = this.overlay.getBoundingClientRect(); this.anchor = { x: event.clientX - rect.left, y: event.clientY - rect.top };
        this.root.hidden = false; this.update(this.timeS, true); this.place();
      }, options);
      this.canvas.addEventListener("pointercancel", event => this.gesture.cancel(event.pointerId), options);
      this.canvas.addEventListener("pointerleave", () => { this.cancelHover(); this.canvas!.style.cursor = ""; }, options);
    }
    this.resize = new ResizeObserver(() => this.place()); this.resize.observe(this.overlay); this.resize.observe(this.root);
  }
  private cancelHover(): void { if (this.hover !== undefined) cancelAnimationFrame(this.hover); this.hover = undefined; this.hoverPoint = undefined; }
  private follow(): void {
    const id = this.state.selectedId; if (!id || !this.viewer) return;
    if (this.viewer.followedRouteId !== id) this.viewer.setFollowRoute(id);
    this.clear();
  }
  followChanged(enabled: boolean): void { if (enabled) this.clear(); }
  clear(): void {
    const returnFocus = this.root.contains(document.activeElement);
    this.state.clear(); this.root.hidden = true; this.route = undefined;
    this.viewer?.setRouteSelection(null);
    if (returnFocus) this.canvas?.focus({ preventScroll: true });
  }
  update(timeS: number, force = false): void {
    this.timeS = timeS; if (this.root.hidden) return;
    const route = this.routes().find(r => r.id === this.state.selectedId);
    if (!route) { this.clear(); return; }
    const now = performance.now(); if (!force && route === this.route && now - this.lastMs < 100) return;
    this.lastMs = now;
    if (this.route !== route) {
      this.route = route; const index = this.routes().indexOf(route), summary = routeTaskSummary(route);
      this.root.dataset.routeId = route.id; this.root.style.setProperty("--task-color", routeColorCSS(index));
      this.root.setAttribute("aria-label", `UAV ${index + 1}: ${route.label}`);
      const field = (name: string, text: string): void => { const node = this.root.querySelector<HTMLElement>(`[data-field="${name}"]`)!; node.textContent = text; node.title = text; };
      field("aircraft", `UAV ${String(index + 1).padStart(2, "0")}`); field("origin", route.mission?.origin ?? "Start"); field("destination", route.mission?.destination ?? "Goal");
      field("length", `${(summary.lengthM / 1000).toFixed(2)} km`); field("duration", route.timedPath?.length ? flightDuration(summary.durationS) : "—"); field("stops", String(route.mission?.taskPoints?.length ?? 0));
      this.root.querySelector<HTMLElement>(".route-card-title")!.textContent = route.label;
      const purpose = this.root.querySelector<HTMLElement>(".route-card-purpose")!; purpose.textContent = purpose.title = route.mission?.purpose ?? ""; purpose.hidden = !purpose.textContent;
      const stops = this.root.querySelector<HTMLElement>(".route-card-stops")!;
      stops.replaceChildren(...taskArrivals(route).map(({ task }) => {
        const chip = document.createElement("span"); chip.textContent = String(task.order).padStart(2, "0"); chip.title = task.label; chip.dataset.taskId = task.id; return chip;
      })); stops.hidden = !stops.children.length;
      this.root.querySelector<HTMLElement>(".route-card-follow")!.title = `Follow ${route.label} · Or click this route again`;
    }
    const phase = flightPhase(route, timeS), status = this.root.querySelector<HTMLElement>(".route-card-phase")!;
    status.textContent = phaseLabel(phase); status.title = phase.reason ?? phaseLabel(phase); status.dataset.phase = phase.kind;
    const arrivals = taskArrivals(route), next = arrivals.findIndex(a => a.timeS > timeS);
    this.root.querySelectorAll<HTMLElement>(".route-card-stops span").forEach((chip, i) => {
      const state = arrivals[i]!.timeS <= timeS ? "passed" : i === next ? "next" : "pending";
      chip.dataset.state = state; chip.setAttribute("aria-label", `${arrivals[i]!.task.label}: ${state}`);
    });
  }
  private place(): void {
    if (this.root.hidden) return;
    const host = this.overlay.getBoundingClientRect();
    const blockers = [...document.querySelectorAll<HTMLElement>(".study-nav, .route-legend, .playback-state, .stage-toolbar > :not([hidden]), .flight-hud, .inspector-open, .map-attribution")]
      .map(e => e.getBoundingClientRect()).filter(r => r.width > 0 && r.height > 0).map(r => ({ x: r.x - host.x, y: r.y - host.y, width: r.width, height: r.height }));
    const position = positionRouteCard(this.anchor, this.root.offsetWidth, this.root.offsetHeight, host.width, host.height, blockers);
    this.root.style.left = `${position.x}px`; this.root.style.top = `${position.y}px`;
  }
  dispose(): void { this.clear(); this.cancelHover(); this.events.abort(); this.resize.disconnect(); this.gesture.reset(); if (this.canvas) this.canvas.style.cursor = ""; this.root.remove(); }
}
