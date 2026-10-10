import * as THREE from "three";
import type { LineMaterialParameters } from "three/addons/lines/LineMaterial.js";
import { createFinishMarker } from "./mission-finish";
import { createLaunchMarker } from "./mission-launch";

/** Display symbols, not collision envelopes. Keep their size independent of camera zoom. */
export function createEndpointMarker(role: "start" | "goal", city: boolean, color: THREE.ColorRepresentation): THREE.Group {
  return role === "goal" ? createFinishMarker(color, city) : createLaunchMarker(color, city);
}

/** Trajectories share the city's projection and depth buffer; they are not HUD overlays. */
export function sceneLineStyle(
  city: boolean,
  weight: number,
  dashed: boolean,
  opacity = 1,
): LineMaterialParameters {
  return {
    linewidth: weight * (city ? 1.25 : 0.2),
    worldUnits: true,
    dashed,
    dashSize: city ? 18 : 2.2,
    gapSize: city ? 10 : 1.4,
    transparent: true,
    opacity,
    depthTest: true,
    // Translucent annotations must not punch holes in other translucent volumes.
    depthWrite: false,
    alphaToCoverage: true,
  };
}
