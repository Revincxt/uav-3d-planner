import { Box3, Ray, Vector3 } from "three";
import type { CityBox } from "./city-scene";
interface Entry { building: CityBox; bounds: Box3 }
interface Branch { bounds: Box3; entries?: Entry[]; left?: Branch; right?: Branch }
function build(entries: Entry[]): Branch {
  const bounds = new Box3(); for (const entry of entries) bounds.union(entry.bounds);
  if (entries.length <= 8) return { bounds, entries };
  const size = bounds.getSize(new Vector3()), axis = size.x >= size.y && size.x >= size.z ? "x" : size.y >= size.z ? "y" : "z";
  entries.sort((a, b) => a.bounds.min[axis] + a.bounds.max[axis] - b.bounds.min[axis] - b.bounds.max[axis]);
  const middle = Math.floor(entries.length / 2); return { bounds, left: build(entries.slice(0, middle)), right: build(entries.slice(middle)) };
}
function insideRing(x: number, y: number, ring: readonly number[][]): boolean {
  let inside = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const a = ring[i]!, b = ring[j]!;
    if ((a[1]! > y) !== (b[1]! > y) && x < (b[0]! - a[0]!) * (y - a[1]!) / (b[1]! - a[1]!) + a[0]!) inside = !inside;
  }
  return inside;
}
/** Roofs, walls and courtyard holes match the rendered footprint prism, not its conservative AABB. */
export function intersectsBuilding(ray: Ray, building: CityBox, limit: number): boolean {
  const rings = building.footprint;
  if (!rings?.length) {
    const bounds = new Box3(new Vector3(building.min[0], building.min[2], -building.max[1]!), new Vector3(building.max[0], building.max[2], -building.min[1]!));
    const hit = ray.intersectBox(bounds, new Vector3()); return bounds.containsPoint(ray.origin) || !!hit && hit.distanceTo(ray.origin) < limit;
  }
  const x = ray.origin.x, y = -ray.origin.z, z = ray.origin.y, dx = ray.direction.x, dy = -ray.direction.z, dz = ray.direction.y;
  const solid = (east: number, north: number) => insideRing(east, north, rings[0]!) && !rings.slice(1).some(ring => insideRing(east, north, ring));
  if (z > building.min[2]! && z < building.max[2]! && solid(x, y)) return true;
  if (Math.abs(dz) > 1e-12) for (const altitude of [building.min[2]!, building.max[2]!]) {
    const t = (altitude - z) / dz;
    if (t >= 0 && t < limit && solid(x + dx * t, y + dy * t)) return true;
  }
  for (const ring of rings) for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const a = ring[j]!, b = ring[i]!, ex = b[0]! - a[0]!, ey = b[1]! - a[1]!, determinant = dx * ey - dy * ex;
    if (Math.abs(determinant) < 1e-12) continue;
    const ox = a[0]! - x, oy = a[1]! - y, t = (ox * ey - oy * ex) / determinant, u = (ox * dy - oy * dx) / determinant;
    const altitude = z + t * dz;
    if (t >= 0 && t < limit && u >= 0 && u <= 1 && altitude >= building.min[2]! && altitude <= building.max[2]!) return true;
  }
  return false;
}
/** Lazy, retained BVH: pointer picking never raycasts millions of merged city triangles. */
export class BuildingOcclusion {
  private readonly root?: Branch;
  private readonly hit = new Vector3();
  constructor(buildings: readonly CityBox[]) {
    if (buildings.length) this.root = build(buildings.map(building => ({ building, bounds: new Box3(
      new Vector3(building.min[0], building.min[2], -building.max[1]!), new Vector3(building.max[0], building.max[2], -building.min[1]!),
    ) })));
  }
  blocks(ray: Ray, distance: number): boolean {
    const limit = distance - .01;
    const visit = (branch?: Branch): boolean => {
      if (!branch) return false;
      const point = ray.intersectBox(branch.bounds, this.hit);
      if (!branch.bounds.containsPoint(ray.origin) && (!point || point.distanceTo(ray.origin) >= limit)) return false;
      if (branch.entries) return branch.entries.some(entry => intersectsBuilding(ray, entry.building, limit));
      return visit(branch.left) || visit(branch.right);
    };
    return limit > 0 && visit(this.root);
  }
}
