import * as THREE from "three";
import type { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { positionWesternOverview } from "./map-navigation";
import { enuToThree } from "./coordinates";
import { timedPosition } from "./route-overview";
import type { OverviewRoute } from "./route-overview";
import type { MovingSphereDefinition, TemporaryNoFlyZone, Vec3 } from "./dynamic-schema";

/** Explicit local view; overview and chase framing remain unchanged. */
export function encounterBounds(position: Vec3): THREE.Box3 {
  const center = new THREE.Vector3(...enuToThree(position));
  return new THREE.Box3(new THREE.Vector3(center.x - 180, Math.max(0, center.y - 20), center.z - 180),
    new THREE.Vector3(center.x + 180, center.y + 60, center.z + 180));
}

export function positionEncounterCamera(camera: THREE.OrthographicCamera, controls: OrbitControls, position: Vec3): THREE.Box3 {
  const bounds = encounterBounds(position), center = bounds.getCenter(new THREE.Vector3());
  positionWesternOverview(camera, center, 360);
  camera.zoom = 1; controls.target.copy(center); controls.update();
  return bounds;
}

export interface EncounterView { timeS: number; position: Vec3 }
export function encounterView(route: OverviewRoute, traffic: MovingSphereDefinition[] = [], zones: TemporaryNoFlyZone[] = []): EncounterView | null {
  const challenge = route.mission?.challenge, path = route.timedPath;
  if (!challenge || !path?.length) return null;
  let best = Infinity, time = challenge.startTimeS, position = challenge.focusPosition;
  for (const waypoint of path) {
    if (challenge.kind === "static") {
      const separation = Math.hypot(...waypoint.position.map((v, i) => v - challenge.focusPosition[i]!));
      if (separation < best) { best = separation; time = waypoint.timeS; position = challenge.focusPosition; }
      continue;
    }
    for (const aircraft of traffic) {
      const index = aircraft.keyframes.findIndex((right, i) => i > 0 && waypoint.timeS >= aircraft.keyframes[i - 1]!.timeS && waypoint.timeS <= right.timeS &&
        Math.hypot(right.position[0] - aircraft.keyframes[i - 1]!.position[0], right.position[1] - aircraft.keyframes[i - 1]!.position[1]) > 1e-8);
      if (index < 0) continue;
      const other = timedPosition(aircraft.keyframes, waypoint.timeS);
      const separation = Math.hypot(...waypoint.position.map((v, i) => v - other[i]!)) - aircraft.radiusM;
      if (separation < best) { best = separation; time = waypoint.timeS; position = waypoint.position.map((v, i) => (v + other[i]!) / 2) as Vec3; }
    }
    for (const zone of zones) {
      if (waypoint.timeS < zone.activeFromS || waypoint.timeS >= zone.activeUntilS) continue;
      const separation = Math.hypot(waypoint.position[0] - zone.center[0], waypoint.position[1] - zone.center[1]) - zone.radiusM;
      if (separation < best) { best = separation; time = waypoint.timeS;
        position = [(waypoint.position[0] + zone.center[0]) / 2, (waypoint.position[1] + zone.center[1]) / 2, waypoint.position[2]]; }
    }
  }
  return { timeS: Math.max(0, time - 8), position };
}

export function mountEncounterControl(get: () => EncounterView | null, observe: (view: EncounterView) => void): void {
  const host = document.querySelector<HTMLElement>(".camera-controls, .stage-toolbar .view-controls:last-child");
  if (!host) return;
  const button = document.createElement("button");
  button.type = "button"; button.id = "observe-encounter";
  button.setAttribute("aria-label", "Observe encounter");
  button.innerHTML = '<svg viewBox="0 0 20 20" fill="none" aria-hidden="true"><circle cx="10" cy="10" r="5.5" stroke="currentColor" stroke-width="1.4" /><circle cx="10" cy="10" r="1.5" fill="currentColor" /><path d="M10 2v4m0 8v4M2 10h4m8 0h4" stroke="currentColor" stroke-width="1.4" stroke-linecap="round" /></svg>';
  button.title = "Close view of this mission's fixed constraint or closest active encounter";
  button.addEventListener("click", () => { const view = get(); if (view) observe(view); });
  host.append(button);
}

