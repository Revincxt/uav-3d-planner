import * as THREE from "three";
import { enuToThree } from "./coordinates";

export const GATE_SIZE_M = 12;
export const GATE_BEVEL_M = 0.14;

function roundedSquare(path: THREE.Shape | THREE.Path, half: number, radius: number): void {
  path.moveTo(-half + radius, -half);
  path.lineTo(half - radius, -half); path.quadraticCurveTo(half, -half, half, -half + radius);
  path.lineTo(half, half - radius); path.quadraticCurveTo(half, half, half - radius, half);
  path.lineTo(-half + radius, half); path.quadraticCurveTo(-half, half, -half, half - radius);
  path.lineTo(-half, -half + radius); path.quadraticCurveTo(-half, -half, -half + radius, -half);
  path.closePath();
}

/** Open square frame, not a filled billboard; its center is the actual flight knot. */
export function missionGateGeometry(): THREE.ExtrudeGeometry {
  const shape = new THREE.Shape(), hole = new THREE.Path();
  roundedSquare(shape, GATE_SIZE_M / 2, 0.9);
  roundedSquare(hole, GATE_SIZE_M / 2 - 0.9, 0.4);
  shape.holes.push(hole);
  const geometry = new THREE.ExtrudeGeometry(shape, { depth: 0.62, bevelEnabled: true,
    bevelSegments: 4, steps: 1, bevelSize: GATE_BEVEL_M, bevelThickness: GATE_BEVEL_M, curveSegments: 8 });
  geometry.translate(0, 0, -0.31);
  return geometry;
}

/** Paint the real frame: white soft rails, checkerboard corners and a colored inner rim. */
export function missionGateMaterial(accent: THREE.ColorRepresentation = 0x00ae95): THREE.MeshStandardMaterial {
  const material = new THREE.MeshStandardMaterial({ color: accent, emissive: accent, emissiveIntensity: 0.025,
    metalness: 0.05, roughness: 0.38, depthTest: true, depthWrite: true });
  material.userData.style = "racing-gate";
  material.onBeforeCompile = shader => {
    shader.vertexShader = `varying vec2 vRaceGateXY;\n${shader.vertexShader}`.replace("#include <begin_vertex>",
      "#include <begin_vertex>\nvRaceGateXY = position.xy;");
    shader.fragmentShader = `varying vec2 vRaceGateXY;\n${shader.fragmentShader}`.replace("#include <color_fragment>", `
      #include <color_fragment>
      vec3 gateAccent = diffuseColor.rgb;
      vec2 gateXY = abs(vRaceGateXY);
      float gateEdge = max(gateXY.x, gateXY.y);
      vec3 gatePaint = vec3(0.89, 0.94, 0.92);
      float cornerPatch = step(4.3, gateXY.x) * step(4.3, gateXY.y);
      float checker = mod(floor((gateXY.x - 4.3) / 0.45) + floor((gateXY.y - 4.3) / 0.45), 2.0);
      float checkerDetail = 1.0 - smoothstep(0.3, 0.9, max(fwidth(vRaceGateXY.x), fwidth(vRaceGateXY.y)));
      vec3 cornerPaint = mix(gatePaint, vec3(0.012, 0.025, 0.023), mix(0.5, checker, checkerDetail));
      gatePaint = mix(gatePaint, cornerPaint, cornerPatch);
      float innerRim = 1.0 - step(5.35, gateEdge);
      float topBadge = step(5.6, gateXY.y) * (1.0 - step(1.4, gateXY.x));
      float sideBands = step(5.6, gateXY.x) * (1.0 - step(1.1, gateXY.y));
      diffuseColor.rgb = mix(gatePaint, gateAccent, max(innerRim, max(topBadge, sideBands)));
    `);
  };
  material.customProgramCacheKey = () => "mission-racing-gate-v1";
  return material;
}

export function missionGateYaw(points: readonly (readonly number[])[], position: readonly number[]): number {
  const index = points.findIndex(point => Math.hypot(...point.map((v, axis) => v - position[axis]!)) <= 1e-5);
  const directions: THREE.Vector3[] = [];
  for (const step of [-1, 1]) {
    for (let i = index + step; index >= 0 && i >= 0 && i < points.length; i += step) {
      const p = points[i]!;
      const direction = new THREE.Vector3((p[0]! - position[0]!) * step, 0, -(p[1]! - position[1]!) * step);
      if (direction.lengthSq() > 1e-8) { directions.push(direction.normalize()); break; }
    }
  }
  const heading = directions.reduce((sum, direction) => sum.add(direction), new THREE.Vector3());
  if (heading.lengthSq() <= 1e-8) heading.copy(directions.at(-1) ?? new THREE.Vector3(0, 0, 1));
  return Math.atan2(heading.x, heading.z);
}

export function missionGateMatrix(points: readonly (readonly number[])[], position: [number, number, number]): THREE.Matrix4 {
  return new THREE.Matrix4().compose(new THREE.Vector3(...enuToThree(position)),
    new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), missionGateYaw(points, position)), new THREE.Vector3(1,1,1));
}
