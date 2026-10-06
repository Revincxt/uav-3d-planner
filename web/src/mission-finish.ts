import * as THREE from "three";

/** A physical checkered cloth, no billboard texture or filled marker at the flight endpoint. */
function finishFlagGeometry(): THREE.BufferGeometry {
  const geometry = new THREE.PlaneGeometry(4, 2.6, 6, 4).toNonIndexed();
  const positions = geometry.getAttribute("position"), colors = new Float32Array(positions.count * 3);
  const white = new THREE.Color(0xf4f8f6), dark = new THREE.Color(0x19352f);
  for (let first = 0; first < positions.count; first += 6) {
    let x = 0, y = 0;
    for (let i = first; i < first + 6; i++) { x += positions.getX(i); y += positions.getY(i); }
    const cellX = Math.floor((x / 6 + 2) / 4 * 6), cellY = Math.floor((y / 6 + 1.3) / 2.6 * 4);
    const color = (cellX + cellY) % 2 ? dark : white;
    for (let i = first; i < first + 6; i++) {
      color.toArray(colors, i * 3);
      const along = (positions.getX(i) + 2) / 4;
      positions.setZ(i, 0.16 * along * Math.sin(along * Math.PI * 2 + positions.getY(i) * 0.5));
    }
  }
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  geometry.computeVertexNormals();
  return geometry;
}

/** Open landing target below the vehicle, with a finish flag off to the side. Decorative only. */
export function createFinishMarker(accent: THREE.ColorRepresentation, city = true): THREE.Group {
  const marker = new THREE.Group(); marker.userData.kind = "finish-marker";
  const colored = new THREE.MeshStandardMaterial({ color: accent, emissive: accent, emissiveIntensity: 0.035,
    roughness: 0.42, metalness: 0.12, depthTest: true, depthWrite: true });
  const white = new THREE.MeshStandardMaterial({ color: 0xf4f8f6, roughness: 0.45, metalness: 0.12,
    depthTest: true, depthWrite: true });
  const dark = new THREE.MeshStandardMaterial({ color: 0x40514f, roughness: 0.45, metalness: 0.3,
    depthTest: true, depthWrite: true });
  const add = (geometry: THREE.BufferGeometry, material: THREE.Material, name: string, position: [number, number, number]): THREE.Mesh => {
    const mesh = new THREE.Mesh(geometry, material); mesh.name = name; mesh.position.fromArray(position);
    mesh.receiveShadow = true; marker.add(mesh); return mesh;
  };
  for (const [radius, material, name] of [[5.4, white, "finish-outer-ring"], [4.65, colored, "finish-inner-ring"]] as const) {
    add(new THREE.TorusGeometry(radius, 0.18, 8, 64), material, name, [0,-2.2,0]).rotation.x = Math.PI / 2;
  }
  const tickGeometry = new THREE.BoxGeometry(0.22, 0.12, 1.1);
  for (let i = 0; i < 4; i++) {
    const angle = i * Math.PI / 2;
    add(tickGeometry, colored, `finish-target-tick-${i}`, [Math.sin(angle) * 6.1,-2.2,Math.cos(angle) * 6.1]).rotation.y = angle;
  }
  add(new THREE.CylinderGeometry(0.8, 0.85, 0.4, 16), colored, "finish-flag-base", [6.2,-2.1,0]);
  add(new THREE.CylinderGeometry(0.1, 0.1, 7.2, 12), dark, "finish-flag-pole", [6.2,1.4,0]);
  add(new THREE.SphereGeometry(0.18, 12, 8), colored, "finish-flag-cap", [6.2,5.1,0]);
  add(finishFlagGeometry(), new THREE.MeshStandardMaterial({ color: 0xffffff, vertexColors: true, side: THREE.DoubleSide,
    roughness: 0.75, depthTest: true, depthWrite: true }), "finish-checkered-flag", [8.2,3.65,0]);
  if (!city) marker.scale.setScalar(0.28);
  return marker;
}
