import * as THREE from "three";

export interface ZoneVisual {
  fill: THREE.Mesh<THREE.CylinderGeometry, THREE.MeshStandardMaterial>;
  outline: THREE.Group;
}
interface Zone { center: readonly number[]; radiusM: number; zMinM: number; zMaxM: number }

export function ringPoints(radius: number, height: number, segments = 64): THREE.Vector3[] {
  return Array.from({ length: segments }, (_, index) => {
    const angle = index / segments * Math.PI * 2;
    return new THREE.Vector3(Math.cos(angle) * radius, height, Math.sin(angle) * radius);
  });
}

/** One depth-tested exclusion-volume implementation for all planning views. */
export function addZoneVisual(host: THREE.Group, zone: Zone, temporary = false, color = 0xc84d52): ZoneVisual {
  const height = zone.zMaxM - zone.zMinM;
  const fill = new THREE.Mesh(new THREE.CylinderGeometry(zone.radiusM, zone.radiusM, height, 48),
    new THREE.MeshStandardMaterial({ color, transparent: true, opacity: temporary ? 0.025 : 0.18,
      side: THREE.DoubleSide, depthTest: true, depthWrite: false, roughness: 0.8 }));
  fill.position.set(zone.center[0]!, zone.zMinM + height / 2, -zone.center[1]!);
  const outline = new THREE.Group();
  outline.position.copy(fill.position);
  const material = new THREE.LineBasicMaterial({ color, transparent: true, opacity: temporary ? 0.18 : 0.52,
    depthTest: true, depthWrite: false });
  const points: THREE.Vector3[] = [];
  for (const level of [-height / 2, height / 2]) {
    const ring = ringPoints(zone.radiusM, level);
    for (let index = 0; index < ring.length; index++) points.push(ring[index]!, ring[(index + 1) % ring.length]!);
  }
  for (let index = 0; index < 4; index += 1) {
    const angle = index / 4 * Math.PI * 2, x = Math.cos(angle) * zone.radiusM, z = Math.sin(angle) * zone.radiusM;
    points.push(new THREE.Vector3(x, -height / 2, z), new THREE.Vector3(x, height / 2, z));
  }
  outline.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(points), material));
  host.add(fill, outline);
  return { fill, outline };
}

/** Project physical bounds into the current camera; decorative tiles never affect framing. */
export function fitMapCamera(camera: THREE.OrthographicCamera, bounds: THREE.Box3, width: number, height: number, padding: number): void {
  const aspect = width / height;
  camera.updateMatrixWorld(true);
  const projected = new THREE.Box3();
  for (const x of [bounds.min.x, bounds.max.x]) for (const y of [bounds.min.y, bounds.max.y]) for (const z of [bounds.min.z, bounds.max.z]) {
    projected.expandByPoint(new THREE.Vector3(x, y, z).applyMatrix4(camera.matrixWorldInverse));
  }
  const extent = projected.getSize(new THREE.Vector3()), center = projected.getCenter(new THREE.Vector3());
  const vertical = Math.max(extent.y, extent.x / aspect, 1) * padding;
  camera.left = center.x - vertical * aspect / 2;
  camera.right = center.x + vertical * aspect / 2;
  camera.top = center.y + vertical / 2;
  camera.bottom = center.y - vertical / 2;
  camera.near = 0.1;
  camera.far = Math.max(100, -projected.min.z + bounds.getSize(new THREE.Vector3()).length() * 2);
  camera.updateProjectionMatrix();
}
