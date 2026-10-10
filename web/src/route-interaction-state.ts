interface PointerSample { pointerId: number; clientX: number; clientY: number; button: number; pointerType: string }
/** Two separate clicks, not a timed double-click; selecting another route starts afresh. */
export class RouteClickState {
  selectedId: string | null = null;
  click(id: string | null): "select" | "follow" | "clear" {
    if (!id) { this.selectedId = null; return "clear"; }
    if (this.selectedId === id) return "follow";
    this.selectedId = id; return "select";
  }
  clear(): void { this.selectedId = null; }
}
/** Dragging out and back, cancelled pointers, and pinch gestures can never select a route. */
export class PointerClickGuard {
  private pointers = new Map<number, { x: number; y: number; moved: boolean; tolerance: number }>();
  down(event: PointerSample): void {
    if (event.button !== 0) return;
    if (this.pointers.size) for (const pointer of this.pointers.values()) pointer.moved = true;
    this.pointers.set(event.pointerId, { x: event.clientX, y: event.clientY, moved: this.pointers.size > 0, tolerance: event.pointerType === "touch" ? 8 : 5 });
  }
  move(event: Pick<PointerSample, "pointerId" | "clientX" | "clientY">): void {
    const pointer = this.pointers.get(event.pointerId);
    if (pointer && Math.hypot(event.clientX - pointer.x, event.clientY - pointer.y) > pointer.tolerance) pointer.moved = true;
  }
  up(event: PointerSample): boolean {
    this.move(event); const pointer = this.pointers.get(event.pointerId);
    const click = !!pointer && !pointer.moved && this.pointers.size === 1 && event.button === 0;
    this.pointers.delete(event.pointerId); return click;
  }
  cancel(id: number): void { this.pointers.delete(id); }
  reset(): void { this.pointers.clear(); }
}
export interface ScreenRect { x: number; y: number; width: number; height: number }
/** Prefer a nearby free quadrant, clamped to the map, without covering navigation or telemetry. */
export function positionRouteCard(point: { x: number; y: number }, width: number, height: number,
  mapWidth: number, mapHeight: number, blockers: readonly ScreenRect[]): { x: number; y: number } {
  const gap = 14, controlGap = 6, padding = 12;
  const candidates = [[point.x + gap, point.y + gap], [point.x - width - gap, point.y + gap],
    [point.x + gap, point.y - height - gap], [point.x - width - gap, point.y - height - gap],
    [padding, padding], [mapWidth - width - padding, padding], [padding, mapHeight - height - padding], [mapWidth - width - padding, mapHeight - height - padding]];
  candidates.push([(mapWidth - width) / 2, (mapHeight - height) / 2]);
  for (const b of blockers) candidates.push([point.x - width / 2, b.y + b.height + controlGap], [point.x - width / 2, b.y - height - controlGap],
    [b.x - width - controlGap, point.y - height / 2], [b.x + b.width + controlGap, point.y - height / 2]);
  return candidates.map(([x, y]) => {
    x = Math.max(padding, Math.min(x!, mapWidth - width - padding)); y = Math.max(padding, Math.min(y!, mapHeight - height - padding));
    const overlap = blockers.reduce((sum, b) => sum + Math.max(0, Math.min(x! + width, b.x + b.width) - Math.max(x!, b.x)) * Math.max(0, Math.min(y! + height, b.y + b.height) - Math.max(y!, b.y)), 0);
    const coversClick = point.x >= x - 8 && point.x <= x + width + 8 && point.y >= y - 8 && point.y <= y + height + 8;
    return { x, y, score: overlap * 100 + (coversClick ? width * height * 1000 : 0) + Math.hypot(x - point.x, y - point.y) };
  }).sort((a, b) => a.score - b.score)[0]!;
}
