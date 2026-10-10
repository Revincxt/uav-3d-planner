import * as THREE from "three";
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js";

function outline(half: number, cut: number): THREE.Vector2[] {
  return [[-half + cut, -half], [half - cut, -half], [half, -half + cut], [half, half - cut],
    [half - cut, half], [-half + cut, half], [-half, half - cut], [-half, -half + cut]]
    .map(([x, y]) => new THREE.Vector2(x!, y!));
}

function plate(half: number, depth: number, height: number, inner?: number): THREE.BufferGeometry {
  const shape = new THREE.Shape(outline(half, 1));
  if (inner !== undefined) shape.holes.push(new THREE.Path(outline(inner, 0.8).reverse()));
  return new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: true, bevelSegments: 1,
    steps: 1, bevelSize: 0.06, bevelThickness: 0.04 }).rotateX(-Math.PI / 2).translate(0, height, 0);
}

/** A low-profile, chamfered launch pad below the recorded origin. Visual only. */
export function createLaunchMarker(accent: THREE.ColorRepresentation, city = true): THREE.Group {
  const marker = new THREE.Group();
  marker.userData = { kind: "launch-marker", visualOnly: true };
  const add = (parts: THREE.BufferGeometry[], color: THREE.ColorRepresentation, name: string,
    metalness = 0.15): void => {
    const geometry = parts.length === 1 ? parts[0]! : mergeGeometries(parts)!;
    if (parts.length > 1) parts.forEach(part => part.dispose());
    const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color,
      roughness: 0.5, metalness, depthTest: true, depthWrite: true }));
    mesh.name = name; mesh.receiveShadow = true; marker.add(mesh);
  };
  add([plate(6.5, 0.28, -2.9)], 0x263b42, "launch-pad-base", 0.28);
  const accents = [plate(6.1, 0.12, -2.57, 5.65)];
  // Raised corner lights and a forward chevron identify departure without text.
  for (const x of [-5.55, 5.55]) for (const z of [-5.55, 5.55])
    accents.push(new THREE.BoxGeometry(0.44, 0.4, 0.44).toNonIndexed().translate(x, -2.3, z));
  for (const angle of [-Math.PI / 4, Math.PI / 4])
    accents.push(new THREE.BoxGeometry(1.6, 0.1, 0.28).toNonIndexed()
      .rotateY(angle).translate(Math.sign(angle) * 0.5, -2.5, 4.7));
  add(accents, accent, "launch-pad-frame");
  const markings = [new THREE.BoxGeometry(2.8, 0.09, 0.5).translate(0, -2.47, 0)];
  for (const x of [-1.65, 1.65])
    markings.push(new THREE.BoxGeometry(0.55, 0.09, 3.8).translate(x, -2.47, 0));
  add(markings, 0xf1f6f5, "launch-pad-markings");
  if (!city) marker.scale.setScalar(0.28);
  return marker;
}
