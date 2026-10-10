import * as THREE from "three";
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js";
import type { CityMetadata, CityMission } from "./city-schema";
import { addMapBackground, MAP_PALETTE } from "./map-background";
import { addMapSurround } from "./map-surround";

type CityPoint = readonly number[];
export interface CityBox {
  min: CityPoint;
  max: CityPoint;
  footprint?: number[][][];
}

interface CityZone {
  center: CityPoint;
  radiusM: number;
}

/** Physical scene geometry; no generated or decorative buildings. */
export interface CityScenario {
  id: string;
  bounds: CityBox;
  city?: CityMetadata;
  mission?: CityMission;
  buildings?: readonly CityBox[];
  start?: CityPoint;
  goal?: CityPoint;
  noFlyZones?: readonly CityZone[];
  staticNoFlyZones?: readonly CityZone[];
  temporaryNoFlyZones?: readonly CityZone[];
  movingSpheres?: readonly {
    radiusM: number;
    keyframes: readonly { position: CityPoint }[];
  }[];
}


function facadeVertices(width: number, depth: number, height: number, index: number): number[] {
  const points: number[] = [];
  if (height < 9 || index % 4 === 0) return points;
  const bottom = -height / 2 + height * 0.035;
  const top = height / 2 - height * 0.05;
  const columns = Math.max(3, Math.min(13, Math.round(width / 1.8)));
  const sides = Math.max(3, Math.min(13, Math.round(depth / 1.8)));
  if (index % 3 !== 0) {
    for (let column = 1; column < columns; column += 1) {
      const x = -width / 2 + width * column / columns;
      for (const z of [-depth / 2, depth / 2]) points.push(x, bottom, z, x, top, z);
    }
    for (let side = 1; side < sides; side += 1) {
      const z = -depth / 2 + depth * side / sides;
      for (const x of [-width / 2, width / 2]) points.push(x, bottom, z, x, top, z);
    }
  } else {
    const floors = Math.max(3, Math.min(16, Math.round(height / 3.2)));
    for (let floor = 1; floor < floors; floor += 1) {
      const y = -height / 2 + height * floor / floors;
      points.push(-width / 2, y, -depth / 2, width / 2, y, -depth / 2,
        -width / 2, y, depth / 2, width / 2, y, depth / 2,
        -width / 2, y, -depth / 2, -width / 2, y, depth / 2,
        width / 2, y, -depth / 2, width / 2, y, depth / 2);
    }
  }
  return points;
}


/** Render the exact obstacle box; surface detailing never changes its envelope. */
export function addCityBuilding(
  host: THREE.Group,
  building: CityBox,
  index: number,
  detail = true,
): THREE.Mesh {
  const width = building.max[0]! - building.min[0]!;
  const depth = building.max[1]! - building.min[1]!;
  const height = building.max[2]! - building.min[2]!;
  const shades = [0xbcbcbc, 0xcdcdcd, 0xb1b1b1, 0xc4c4c4, 0xd2d2d2, 0xb8b8b8];
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(width, height, depth),
    new THREE.MeshStandardMaterial({ color: shades[Math.abs(index) % shades.length]!, roughness: 0.94, metalness: 0 }),
  );
  mesh.name = "city-building";
  mesh.position.set((building.min[0]! + building.max[0]!) / 2,
    (building.min[2]! + building.max[2]!) / 2,
    -(building.min[1]! + building.max[1]!) / 2);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  host.add(mesh);
  if (!detail) return mesh;
  const vertices = facadeVertices(width, depth, height, index);
  if (vertices.length > 0) {
    const facade = new THREE.LineSegments(
      new THREE.BufferGeometry().setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3)),
      new THREE.LineBasicMaterial({ color: 0x686868, transparent: true, opacity: 0.22, depthWrite: false }),
    );
    facade.name = "city-facade";
    mesh.add(facade);
  }
  if (index % 3 !== 1 && width > 4 && depth > 4) {
    const rooftop = new THREE.Mesh(
      new THREE.PlaneGeometry(width * 0.56, depth * 0.58),
      new THREE.MeshStandardMaterial({ color: 0xaaaaaa, roughness: 1,
        polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1 }),
    );
    rooftop.name = "city-rooftop";
    rooftop.rotation.x = -Math.PI / 2;
    rooftop.position.y = height / 2;
    mesh.add(rooftop);
  }
  return mesh;
}

/** Thousands of sourced footprint prisms share a handful of GPU draw calls. */
export function addCityBuildings(host: THREE.Group, buildings: readonly CityBox[]): void {
  if (!buildings.some((building) => building.footprint)) {
    buildings.forEach((building, index) => addCityBuilding(host, building, index));
    return;
  }
  const shades = [0xbcbcbc, 0xcdcdcd, 0xb1b1b1, 0xc4c4c4, 0xd2d2d2, 0xb8b8b8];
  const batches: THREE.BufferGeometry[][] = shades.map(() => []);
  const facadeVertices: number[] = [];
  buildings.forEach((building, index) => {
    let geometry: THREE.BufferGeometry;
    if (building.footprint) {
      const [outer, ...holes] = building.footprint;
      const shape = new THREE.Shape(outer!.map((point) => new THREE.Vector2(point[0]!, point[1]!)));
      shape.holes = holes.map((ring) => new THREE.Path(ring.map((point) => new THREE.Vector2(point[0]!, point[1]!))));
      geometry = new THREE.ExtrudeGeometry(shape, {
        depth: building.max[2]! - building.min[2]!, bevelEnabled: false, curveSegments: 1, steps: 1,
      });
      geometry.rotateX(-Math.PI / 2);
      geometry.translate(0, building.min[2]!, 0);
      // Architecture detail follows real exterior edges; it never invents an
      // obstacle, an ornamental volume, or a synthetic building outside the map.
      if (building.max[2]! - building.min[2]! > 30) {
        for (let edge = 1; edge < outer!.length; edge += 1) {
          const a = outer![edge - 1]!;
          const b = outer![edge]!;
          const length = Math.hypot(b[0]! - a[0]!, b[1]! - a[1]!);
          const columns = Math.floor(length / 9);
          for (let column = 1; column < columns; column += 1) {
            const fraction = column / columns;
            const east = a[0]! + (b[0]! - a[0]!) * fraction;
            const north = a[1]! + (b[1]! - a[1]!) * fraction;
            facadeVertices.push(east, building.min[2]! + 3, -north,
              east, building.max[2]! - 2, -north);
          }
        }
      }
    } else {
      geometry = new THREE.BoxGeometry(
        building.max[0]! - building.min[0]!, building.max[2]! - building.min[2]!, building.max[1]! - building.min[1]!,
      ).toNonIndexed();
      geometry.translate((building.min[0]! + building.max[0]!) / 2,
        (building.min[2]! + building.max[2]!) / 2, -(building.min[1]! + building.max[1]!) / 2);
    }
    geometry.deleteAttribute("uv");
    batches[index % shades.length]!.push(geometry);
  });
  batches.forEach((geometries, index) => {
    if (geometries.length === 0) return;
    const merged = mergeGeometries(geometries, false)!;
    geometries.forEach((geometry) => geometry.dispose());
    const mesh = new THREE.Mesh(merged, new THREE.MeshStandardMaterial({ color: shades[index], roughness: 0.94, metalness: 0 }));
    mesh.name = `planning-city-buildings-${index}`;
    mesh.userData.buildingCount = geometries.length;
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    host.add(mesh);
  });
  if (facadeVertices.length) {
    const facade = new THREE.LineSegments(
      new THREE.BufferGeometry().setAttribute("position", new THREE.Float32BufferAttribute(facadeVertices, 3)),
      new THREE.LineBasicMaterial({ color: 0x686868, transparent: true, opacity: 0.15, depthWrite: false }),
    );
    facade.name = "planning-city-facades";
    host.add(facade);
  }
}

/** Route-focused bounds are independent of the full metropolitan map. */
export function cityMissionBounds(scenario: CityScenario, points: Iterable<CityPoint>): THREE.Box3 {
  const bounds = new THREE.Box3(), scratch = new THREE.Vector3();
  const expand = (point: CityPoint) => bounds.expandByPoint(scratch.set(point[0]!, point[2]!, -point[1]!));
  // Long replay histories must not become a second multi-million-element array
  // or allocate a Vector3 for each historical point during camera framing.
  for (const point of points) expand(point);
  if (scenario.start) expand(scenario.start);
  if (scenario.goal) expand(scenario.goal);
  if (bounds.isEmpty()) {
    return new THREE.Box3(new THREE.Vector3(scenario.bounds.min[0]!, scenario.bounds.min[2]!, -scenario.bounds.max[1]!),
      new THREE.Vector3(scenario.bounds.max[0]!, scenario.bounds.max[2]!, -scenario.bounds.min[1]!));
  }
  const span = bounds.getSize(new THREE.Vector3());
  const margin = Math.max(100, Math.max(span.x, span.z) * 0.1);
  bounds.min.x -= margin;
  bounds.min.z -= margin;
  bounds.max.x += margin;
  bounds.max.z += margin;
  bounds.min.y = scenario.bounds.min[2]!;
  for (const building of scenario.buildings ?? []) {
    if (building.min[0]! <= bounds.max.x && building.max[0]! >= bounds.min.x
      && -building.max[1]! <= bounds.max.z && -building.min[1]! >= bounds.min.z) {
      bounds.max.y = Math.max(bounds.max.y, building.max[2]!);
    }
  }
  bounds.max.y += 20;
  return bounds;
}

/** Physical city ground with public imagery and source-backed offline roads. */
export function addCityContext(host: THREE.Group, scenario: CityScenario, onBackgroundReady?: (texture?: THREE.Texture) => void): THREE.Box3 {
  const { min, max } = scenario.bounds;
  const widthM = max[0]! - min[0]!, depthM = max[1]! - min[1]!;
  const group = new THREE.Group();
  group.name = "city-context";
  group.userData = { buildingCount: scenario.buildings?.length ?? 0, widthM, depthM, source: scenario.city?.sourceKind };
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(widthM, depthM, scenario.city ? 16 : 1, scenario.city ? 16 : 1).rotateX(-Math.PI / 2),
    new THREE.MeshStandardMaterial({ color: scenario.city ? MAP_PALETTE.ground : 0xffffff, roughness: 1 }),
  );
  ground.name = "city-ground";
  ground.position.set((min[0]! + max[0]!) / 2, min[2]! - 0.012, -(min[1]! + max[1]!) / 2);
  ground.receiveShadow = true;
  group.add(ground);
  if (scenario.city) {
    addMapBackground(group, scenario.city.id, scenario.bounds);
    addMapSurround(group, scenario.city.id, scenario.bounds, onBackgroundReady);
  }
  host.add(group);
  return new THREE.Box3(new THREE.Vector3(min[0]!, min[2]!, -max[1]!),
    new THREE.Vector3(max[0]!, max[2]!, -min[1]!));
}
