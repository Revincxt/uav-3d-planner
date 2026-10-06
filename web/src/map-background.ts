import * as THREE from "three";
import { MAP_BACKGROUND } from "./map-background-data";

type Point2 = readonly number[];
export interface MapBounds { min: readonly number[]; max: readonly number[] }
export interface MapRoad { name?: string; widthM: number; points: readonly Point2[] }
export const MAP_PALETTE = { ground: "#f6f7f5", sidewalk: "#ffffff", road: "#e0e5e8" };

/** Sourced centerlines are context only. Styled widths are not collision geometry. */
export function backgroundRoads(cityId: string | undefined): readonly MapRoad[] {
  return cityId === MAP_BACKGROUND.cityId ? MAP_BACKGROUND.roads : [];
}

/** Clip the actual ribbon, not just its centerline, to the original physical map extent. */
function clipPolygon(points: Point2[], bounds: MapBounds): Point2[] {
  let output = points;
  for (const [axis, limit, sign] of [
    [0, bounds.min[0]!, 1], [0, bounds.max[0]!, -1],
    [1, bounds.min[1]!, 1], [1, bounds.max[1]!, -1],
  ] as const) {
    const input = output;
    output = [];
    for (let index = 0; index < input.length; index += 1) {
      const previous = input[(index + input.length - 1) % input.length]!;
      const current = input[index]!;
      const previousInside = sign * (previous[axis]! - limit) >= 0;
      const currentInside = sign * (current[axis]! - limit) >= 0;
      if (previousInside !== currentInside) {
        const fraction = (limit - previous[axis]!) / (current[axis]! - previous[axis]!);
        const point = [
          previous[0]! + fraction * (current[0]! - previous[0]!),
          previous[1]! + fraction * (current[1]! - previous[1]!),
        ];
        point[axis] = limit;
        output.push(point);
      }
      if (currentInside) output.push(current);
    }
  }
  return output;
}

export function roadRibbonVertices(
  roads: readonly MapRoad[], bounds: MapBounds, height: number, sidewalkM = 0,
): number[] {
  const vertices: number[] = [];
  for (const road of roads) {
    if (!Number.isFinite(road.widthM) || road.widthM <= 0) continue;
    const halfWidth = (road.widthM + sidewalkM * 2) / 2;
    for (let index = 1; index < road.points.length; index += 1) {
      const a = road.points[index - 1]!;
      const b = road.points[index]!;
      if (![a[0], a[1], b[0], b[1]].every(Number.isFinite)) continue;
      const dx = b[0]! - a[0]!, dy = b[1]! - a[1]!;
      const length = Math.hypot(dx, dy);
      if (length < 1e-8) continue;
      const nx = -dy * halfWidth / length, ny = dx * halfWidth / length;
      const polygon = clipPolygon([
        [a[0]! + nx, a[1]! + ny], [a[0]! - nx, a[1]! - ny],
        [b[0]! - nx, b[1]! - ny], [b[0]! + nx, b[1]! + ny],
      ], bounds);
      for (let triangle = 1; triangle + 1 < polygon.length; triangle += 1) {
        for (const point of [polygon[0]!, polygon[triangle]!, polygon[triangle + 1]!]) {
          vertices.push(point[0]!, height, -point[1]!);
        }
      }
    }
  }
  return vertices;
}

/** Two batched, untextured ground meshes; scene disposal owns both buffers and materials. */
export function addMapBackground(
  host: THREE.Group, cityId: string, bounds: MapBounds,
): void {
  const roads = backgroundRoads(cityId);
  if (!roads.length) return;
  const background = new THREE.Group();
  background.name = "map-background";
  background.userData = { source: MAP_BACKGROUND.source, visualOnly: true, roadCount: roads.length };
  const base = bounds.min[2] ?? 0;
  for (const [name, color, height, sidewalk] of [
    ["map-sidewalks", MAP_PALETTE.sidewalk, base - 0.008, 3],
    ["map-roads", MAP_PALETTE.road, base - 0.004, 0],
  ] as const) {
    const vertices = roadRibbonVertices(roads, bounds, height, sidewalk);
    if (!vertices.length) continue;
    const geometry = new THREE.BufferGeometry()
      .setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
    geometry.computeVertexNormals();
    const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({
      color, roughness: 1, side: THREE.DoubleSide,
      polygonOffset: true, polygonOffsetFactor: name === "map-roads" ? -2 : -1,
      polygonOffsetUnits: name === "map-roads" ? -2 : -1,
    }));
    mesh.name = name;
    mesh.receiveShadow = true;
    background.add(mesh);
  }
  host.add(background);
}
