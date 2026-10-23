import * as THREE from "three";

export type DisplayCamera = THREE.OrthographicCamera | THREE.PerspectiveCamera;
const direction = new THREE.Vector3();

/** World metres per screen pixel at an annotation's depth; both cameras use normal projection. */
export function metresPerPixelAt(camera: DisplayCamera, point: THREE.Vector3, height: number): number {
  if (camera instanceof THREE.OrthographicCamera)
    return (camera.top - camera.bottom) / (Math.max(1, height) * camera.zoom);
  camera.getWorldDirection(direction);
  const depth = Math.max(camera.near, (point.x - camera.position.x) * direction.x
    + (point.y - camera.position.y) * direction.y + (point.z - camera.position.z) * direction.z);
  return 2 * depth * Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2) / (Math.max(1, height) * camera.zoom);
}
