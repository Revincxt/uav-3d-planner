import type { CityMission } from "./city-schema";
import type { DynamicFrame } from "./dynamic-schema";
import { timedPosition, waypointIndex, type RouteWaypoint } from "./route-overview";

export type PlaybackKind = "fixed" | "reactive" | "predictive";
export type PlaybackAction = "Ready" | "Flying" | "Service" | "Waiting" | "Replanning" | "Arrived";

/** Keep the SVG icon mounted while exposing its current action to assistive technology. */
export function showPlaybackButton(button: HTMLButtonElement, playing: boolean): void {
  button.setAttribute("aria-pressed", String(playing));
  button.setAttribute("aria-label", playing ? "Pause" : "Play");
  button.title = playing ? "Pause" : "Play";
}

export function playbackAction(
  path: readonly RouteWaypoint[], timeS: number, mission?: CityMission, frame?: DynamicFrame,
): PlaybackAction {
  if (!path.length || timeS <= path[0]!.timeS) return "Ready";
  if (timeS >= path.at(-1)!.timeS) return "Arrived";
  const index = waypointIndex(path, timeS), left = path[index]!, right = path[index + 1]!;
  const distance = Math.hypot(...right.position.map((v, axis) => v - left.position[axis]!));
  if (distance <= 1e-8) {
    const position = timedPosition(path, timeS);
    const task = mission?.taskPoints?.find(task => Math.hypot(...position.map((v, axis) => v - task.position[axis]!)) <= 1e-6);
    let arrival = index;
    while (arrival > 0 && Math.hypot(...path[arrival - 1]!.position.map((v, axis) => v - position[axis]!)) <= 1e-6) arrival--;
    return task && timeS - path[arrival]!.timeS < task.serviceDurationS ? "Service" : "Waiting";
  }
  if (frame?.replanned && timeS >= frame.timeS && timeS - frame.timeS < 1) return "Replanning";
  return "Flying";
}

export function showPlaybackState(host: HTMLElement, kind: PlaybackKind, action: PlaybackAction, counterpart = false): void {
  const name = kind === "fixed" ? "Fixed route" : kind === "predictive" ? "Space-time" : counterpart ? "Reactive baseline" : "Live replanning";
  if (host.textContent === `${name} · ${action}` && host.dataset.kind === kind) return;
  host.textContent = `${name} · ${action}`;
  host.dataset.kind = kind;
  host.dataset.action = action.toLowerCase();
  host.ownerDocument.documentElement.dataset.playbackKind = kind;
  host.title = kind === "fixed"
    ? "Illustrative fly-through playback at 15 m/s; not a timed flight feasibility certificate."
    : kind === "predictive"
      ? "Dashed: the preplanned final route. Solid: the portion already flown."
      : "Solid: the portion already flown. Dashed: the current recorded local plan, when available.";
}
