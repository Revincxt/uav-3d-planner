import * as THREE from "three";
import { createDrone, orientDrone } from "./drone-model";
import { enuToThree } from "./coordinates";
import { timedPosition, waypointIndex } from "./route-overview";
import { ringPoints } from "./viewer-geometry";
import type { MovingSphereDefinition } from "./dynamic-schema";
import { metresPerPixelAt, type DisplayCamera } from "./camera-scale";

/** Large cargo traffic; camera-sized body is a depth-tested display annotation. */
export function createTrafficAircraft(definition: MovingSphereDefinition): THREE.Group {
  const aircraft = new THREE.Group();
  // A camera-sized glyph must not cast an enlarged physical shadow or refresh
  // the entire city shadow map on every playback tick.
  aircraft.castShadow = false;
  aircraft.name = `traffic-aircraft:${definition.id}`;
  aircraft.userData = { kind: "cargo-drone", airframeSpanM: 18, separationRadiusM: definition.radiusM };
  const body = createDrone(0xff6a00);
  body.name = "traffic-airframe";
  body.scale.setScalar(18 / 2.1);
  body.userData.displaySymbol = true;
  body.traverse(node => {
    if (node instanceof THREE.Mesh && node.name === "drone-rotors" && node.material instanceof THREE.MeshStandardMaterial) {
      node.material.color.setHex(0xff9a25); node.material.emissive.setHex(0x963200);
      node.material.emissiveIntensity = 0.25;
    }
  });
  const cargo = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.3, 0.55), new THREE.MeshStandardMaterial({
    color: 0xd1a178, roughness: 0.85, depthTest: true, depthWrite: true,
  }));
  cargo.position.y = -0.32;
  body.add(cargo);
  body.traverse(object => { if (object instanceof THREE.Mesh) { object.castShadow = false; object.receiveShadow = true; } });
  aircraft.add(body);
  const material = new THREE.LineBasicMaterial({ color: 0xe46c15, transparent: true,
    opacity: 0.8, depthTest: true, depthWrite: false });
  const envelope = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(ringPoints(definition.radiusM, 0, 48)), material);
  envelope.name = "traffic-separation-envelope";
  aircraft.add(envelope);
  return aircraft;
}

/** Keep overview traffic legible without enlarging its recorded collision envelope. */
export function sizeTrafficAircraft(aircraft: THREE.Object3D, camera: DisplayCamera, height: number): void {
  const body = aircraft.getObjectByName("traffic-airframe");
  if (!body) return;
  const span = Math.max(18, Math.min(210, metresPerPixelAt(camera, aircraft.position, height) * 32));
  body.scale.setScalar(span / 2.1);
  body.userData.displaySpanM = span;
}

export function updateTrafficAircraft(aircraft: THREE.Object3D, definition: MovingSphereDefinition, timeS: number): void {
  aircraft.position.fromArray(enuToThree(timedPosition(definition.keyframes, timeS)));
  const index = waypointIndex(definition.keyframes, timeS);
  const frames = definition.keyframes;
  const moving = (i: number): boolean => frames[i]!.position.some((v, axis) => Math.abs(v - frames[i + 1]!.position[axis]!) > 1e-8);
  // Binary lookup, then normally one segment; legacy stationary records retain
  // their last real heading without allocating or scanning an entire schedule.
  let segment = Math.min(index, frames.length - 2);
  while (segment >= 0 && !moving(segment)) segment--;
  if (segment < 0) {
    segment = 0;
    while (segment < frames.length - 1 && !moving(segment)) segment++;
  }
  if (segment < frames.length - 1) orientDrone(aircraft, new THREE.Vector3().fromArray(enuToThree(frames[segment + 1]!.position))
    .sub(new THREE.Vector3().fromArray(enuToThree(frames[segment]!.position))));
}

