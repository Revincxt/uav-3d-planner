import * as THREE from "three";
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js";
import { RoundedBoxGeometry } from "three/addons/geometries/RoundedBoxGeometry.js";
import { metresPerPixelAt, type DisplayCamera } from "./camera-scale";
import { configureDroneRotors } from "./drone-rotors";

/** A depth-tested display symbol, never a replacement for the recorded collision radius. */
export function createDrone(color: number): THREE.Group {
  const drone = new THREE.Group();
  drone.userData = { kind: "quadcopter", rotorCount: 4, displaySymbol: true };
  const rounded = (w: number, h: number, d: number, r: number, segments = 1) => new RoundedBoxGeometry(w, h, d, segments, r);
  const paint = (geometry: THREE.BufferGeometry, tint: number): THREE.BufferGeometry => {
    const shade = new THREE.Color(tint), colors = new Float32Array(geometry.getAttribute("position").count * 3);
    for (let i = 0; i < colors.length; i += 3) shade.toArray(colors, i);
    geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    return geometry;
  };
  const frame: THREE.BufferGeometry[] = [rounded(.47, .12, .85, .055).translate(0, -.065, .035)];
  const shell: THREE.BufferGeometry[] = [
    rounded(.5, .26, .94, .11, 2).translate(0, .035, 0),
    paint(rounded(.34, .035, .53, .016).translate(0, .177, .08), color),
    paint(rounded(.16, .018, .035, .008).translate(0, .205, .25), 0xadc7bd),
  ];
  const rotors: THREE.BufferGeometry[] = [];
  // Opposite-handed two-blade propellers, pitched to push air down in either direction.
  const profile = new THREE.Shape().moveTo(.035, -.018).lineTo(.13, -.035)
    .quadraticCurveTo(.27, -.06, .342, -.005).quadraticCurveTo(.36, .022, .31, .036)
    .lineTo(.12, .045).quadraticCurveTo(.06, .04, .035, .018).closePath();
  const blade = new THREE.ExtrudeGeometry(profile, { depth: .012, bevelEnabled: false, curveSegments: 4 });
  blade.rotateX(Math.PI / 2); blade.translate(0, .006, 0);
  for (const sx of [-1, 1]) for (const sz of [-1, 1]) {
    const x = sx * .68, z = sz * .61;
    const dx = x - sx * .16, dz = z - sz * .14, length = Math.hypot(dx, dz), angle = -Math.atan2(dz, dx);
    frame.push(rounded(length, .085, .15, .035).rotateY(angle).translate(x - dx / 2, -.015, z - dz / 2),
      new THREE.CylinderGeometry(.086, .074, .14, 12).translate(x, .07, z),
      new THREE.CylinderGeometry(.072, .072, .018, 12).translate(x, .167, z));
    shell.push(rounded(length * .88, .035, .11, .016).rotateY(angle).translate(x - dx / 2, .041, z - dz / 2),
      new THREE.CylinderGeometry(.09, .09, .028, 16).translate(x, .145, z),
      paint(rounded(.085, .018, .14, .008).rotateY(angle).translate(x - dx * .2, .063, z - dz * .2), color),
      paint(new THREE.SphereGeometry(.025, 8, 6).scale(1, .6, 1).translate(x, .075, z + sz * .075), sz < 0 ? sx < 0 ? 0xe96257 : 0x6cc696 : color));
    // Viewed from above: front-left/rear-right CW, the other diagonal CCW.
    const direction = -sx * sz;
    const rotorPart = (geometry: THREE.BufferGeometry, disk: boolean): void => {
      const count = geometry.getAttribute("position").count, pivots = new Float32Array(count * 3);
      for (let i = 0; i < pivots.length; i += 3) { pivots[i] = x; pivots[i + 1] = z; pivots[i + 2] = direction; }
      geometry.setAttribute("rotorPivot", new THREE.BufferAttribute(pivots, 3));
      geometry.setAttribute("rotorDisk", new THREE.BufferAttribute(new Float32Array(count).fill(Number(disk)), 1));
      rotors.push(geometry);
    };
    for (const side of [0, 1]) rotorPart(blade.clone().rotateX((direction < 0 ? Math.PI : 0) + direction * .18)
      .rotateY(side * Math.PI + sx * .45 + sz * .6).translate(x, .195, z), false);
    rotorPart(new THREE.CircleGeometry(.35, 32).rotateX(-Math.PI / 2).translate(x, .195, z), true);
    frame.push(new THREE.CylinderGeometry(.052, .065, .045, 12).translate(x, .2, z));
  }
  blade.dispose();
  for (const sx of [-1, 1]) {
    for (const z of [-.23, .26]) shell.push(rounded(.04, .23, .05, .017).rotateZ(-sx * .16).translate(sx * .225, -.23, z));
    frame.push(rounded(.075, .045, .68, .021).translate(sx * .24, -.355, .03));
    // Recessed battery vents and forward stereo sensors.
    for (let i = 0; i < 4; i++) frame.push(new THREE.BoxGeometry(.012, .035, .027).translate(sx * .247, .035, .16 + i * .049));
    frame.push(new THREE.CylinderGeometry(.034, .034, .018, 12).rotateX(Math.PI / 2).translate(sx * .14, .045, -.477));
  }
  // Underslung three-axis camera, protected inside the landing-gear clearance.
  frame.push(rounded(.06, .11, .07, .02).translate(0, -.16, -.29),
    rounded(.21, .16, .16, .045).translate(0, -.235, -.39),
    new THREE.CylinderGeometry(.071, .071, .034, 16).rotateX(Math.PI / 2).translate(0, -.235, -.48));
  shell.push(new THREE.TorusGeometry(.063, .009, 6, 16).translate(0, -.235, -.5),
    paint(new THREE.CylinderGeometry(.051, .051, .008, 16).rotateX(Math.PI / 2).translate(0, -.235, -.504), 0x214657));

  const mesh = (parts: THREE.BufferGeometry[], tint: number, name: string, roughness: number, metalness: number, vertexColors = false): THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial> => {
    // Rounded surfaces and extrusions are non-indexed; normalize before batching.
    const normalized = parts.map(part => part.index ? part.toNonIndexed() : part);
    if (vertexColors) for (const part of normalized) if (!part.hasAttribute("color")) paint(part, 0xcbd4db);
    const geometry = mergeGeometries(normalized)!;
    normalized.forEach((part, index) => { if (part !== parts[index]) part.dispose(); });
    parts.forEach(part => part.dispose());
    const object = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({
      color: tint, roughness, metalness, vertexColors, depthTest: true, depthWrite: true,
    }));
    object.name = name; drone.add(object); return object;
  };
  mesh(frame, 0x26313c, "drone-frame", .48, .28);
  configureDroneRotors(drone, mesh(rotors, 0x263039, "drone-rotors", .64, .12));
  mesh(shell, 0xffffff, "drone-shell", .4, .18, true);
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
