import * as THREE from "three";

export type DisplayCamera = THREE.OrthographicCamera | THREE.PerspectiveCamera;

/** World metres per screen pixel at an annotation's depth; both cameras use normal projection. */
export function metresPerPixelAt(camera: DisplayCamera, point: THREE.Vector3, height: number): number {
  if (camera instanceof THREE.OrthographicCamera)
    return (camera.top - camera.bottom) / (Math.max(1, height) * camera.zoom);
  const direction = camera.getWorldDirection(new THREE.Vector3());
  const depth = Math.max(camera.near, point.clone().sub(camera.position).dot(direction));
  return 2 * depth * Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2) / (Math.max(1, height) * camera.zoom);
}
