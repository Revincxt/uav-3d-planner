import type { Vec3 } from "./schema";

/** Convert ENU [east, north, up] to Three.js [x, y, z]. */
export function enuToThree(point: Vec3): Vec3 {
  return [point[0], point[2], -point[1]];
}

