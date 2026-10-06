import * as THREE from "three";
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js";
import { metresPerPixelAt, type DisplayCamera } from "./camera-scale";

/** A depth-tested display symbol, never a replacement for the recorded collision radius. */
export function createDrone(color: number): THREE.Group {
  const drone = new THREE.Group();
  drone.userData = { kind: "quadcopter", rotorCount: 4, displaySymbol: true };
  const frame: THREE.BufferGeometry[] = [new THREE.BoxGeometry(0.52, 0.22, 0.72)];
  const rotors: THREE.BufferGeometry[] = [];
  for (const x of [-0.7, 0.7]) for (const z of [-0.7, 0.7]) {
    const arm = new THREE.BoxGeometry(Math.SQRT2 * 0.7, 0.1, 0.11);
    arm.rotateY(-Math.atan2(z, x));
    arm.translate(x / 2, 0, z / 2);
    frame.push(arm, new THREE.CylinderGeometry(0.09, 0.09, 0.22, 10).translate(x, 0.08, z));
    const ring = new THREE.TorusGeometry(0.34, 0.025, 5, 24);
    ring.rotateX(Math.PI / 2); ring.translate(x, 0.2, z);
    rotors.push(ring, new THREE.BoxGeometry(0.62, 0.018, 0.045).translate(x, 0.2, z));
  }
  for (const x of [-0.22, 0.22]) {
    frame.push(new THREE.BoxGeometry(0.04, 0.2, 0.04).translate(x, -0.18, 0.22),
      new THREE.BoxGeometry(0.04, 0.04, 0.65).translate(x, -0.28, 0));
  }
  const mesh = (parts: THREE.BufferGeometry[], tint: number, name: string): void => {
    const geometry = mergeGeometries(parts)!;
    parts.forEach(part => part.dispose());
    const object = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({
      color: tint, roughness: 0.55, metalness: 0.15, depthTest: true, depthWrite: true,
    }));
    object.name = name; drone.add(object);
  };
  mesh(frame, 0x182735, "drone-frame");
  mesh(rotors, 0xe6edf3, "drone-rotors");
  mesh([new THREE.BoxGeometry(0.46, 0.1, 0.5).translate(0, 0.15, 0),
    new THREE.SphereGeometry(0.08, 10, 8).translate(0, 0, -0.4)], color, "drone-shell");
  return drone;
}

/** Keep the airframe level during climbing and waiting; its nose is local -Z. */
export function orientDrone(drone: THREE.Object3D, direction: THREE.Vector3): void {
  if (direction.x * direction.x + direction.z * direction.z > 1e-10)
    drone.rotation.y = Math.atan2(-direction.x, -direction.z);
}

/** Camera-scaled annotation size, with the same world depth/occlusion as the city. */
export function sizeDrone(drone: THREE.Object3D, camera: DisplayCamera, height: number, pixels = 28, minimumScale = 0.8): void {
  const metresPerPixel = metresPerPixelAt(camera, drone.position, height);
  drone.scale.setScalar(Math.max(minimumScale, Math.min(90, metresPerPixel * pixels / 2.1)));
}
