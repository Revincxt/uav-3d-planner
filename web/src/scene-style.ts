import * as THREE from "three";
import type { LineMaterialParameters } from "three/addons/lines/LineMaterial.js";
import { createFinishMarker } from "./mission-finish";

/** Display symbols, not collision envelopes. Keep their size independent of camera zoom. */
export function createEndpointMarker(role: "start" | "goal", city: boolean, color: THREE.ColorRepresentation,
  emissive: THREE.ColorRepresentation = color, emissiveIntensity = 0.04): THREE.Mesh | THREE.Group {
  if (role === "goal") return createFinishMarker(color, city);
  const marker = new THREE.Mesh(new THREE.SphereGeometry(city ? 6 : 1.7, 20, 14),
    new THREE.MeshStandardMaterial({ color, emissive, emissiveIntensity, roughness: 0.65, depthTest: true, depthWrite: true }));
  marker.receiveShadow = true;
  return marker;
}

export function vehicleGeometry(city: boolean): THREE.ConeGeometry {
  return new THREE.ConeGeometry(city ? 4 : 1.35, city ? 10 : 3.8, 5);
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
