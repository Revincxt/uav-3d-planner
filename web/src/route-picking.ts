import { Vector3, Vector4 } from "three";
import type { Line2 } from "three/addons/lines/Line2.js";
import type { DisplayCamera } from "./camera-scale";

export interface RoutePick { id: string; position: Vector3; screenDistancePx: number }
interface PickableRoute { id: string; lines: readonly (Line2 | undefined)[] }

/** Match the GPU's time clipping; hidden history/futures must not be clickable. */
export function visibleSegmentWindow(line: Line2, index: number): [number, number] | null {
  const start = line.geometry.getAttribute("instanceTimeStart"), end = line.geometry.getAttribute("instanceTimeEnd");
  if (!start || !end) return [0, 1];
  const first = start.getX(index), last = end.getX(index), span = Math.max(last - first, .000001);
  const uniforms = line.material.uniforms;
  const clamp = (v: number) => Math.max(0, Math.min(1, v));
  if (uniforms.replayTime) {
    const elapsed = Number(uniforms.replayTime.value) - first;
    return elapsed > 0 ? [0, clamp(elapsed / span)] : null;
  }
  if (uniforms.windowStart && uniforms.windowEnd) {
    const from = clamp((Number(uniforms.windowStart.value) - first) / span), to = clamp((Number(uniforms.windowEnd.value) - first) / span);
    return to > from ? [from, to] : null;
  }
  return [0, 1];
}

/** Homogeneous clipping avoids false hits behind the camera, including near-plane crossings. */
function frustumWindow(a: Vector4, b: Vector4): [number, number] | null {
  let from = 0, to = 1;
  for (const axis of ["x", "y", "z"] as const) for (const sign of [-1, 1]) {
    const left = a.w + sign * a[axis], right = b.w + sign * b[axis];
    if (left < 0 && right < 0) return null;
    if (left < 0) from = Math.max(from, left / (left - right));
    else if (right < 0) to = Math.min(to, left / (left - right));
    if (from > to) return null;
  }
  return [from, to];
}
function visible(line: Line2): boolean {
  for (let parent: typeof line.parent | Line2 = line; parent; parent = parent.parent) if (!parent.visible) return false;
  return true;
}

/** Screen-pixel tolerance is zoom/DPR independent. Only a pointer event performs this work. */
export function pickVisibleRoutes(routes: readonly PickableRoute[], camera: DisplayCamera, x: number, y: number,
  width: number, height: number, occluded: (position: Vector3) => boolean = () => false, radius = 7): RoutePick | null {
  if (width <= 0 || height <= 0 || x < 0 || y < 0 || x > width || y > height) return null;
  camera.updateMatrixWorld();
  const candidates: Array<RoutePick & { depth: number }> = [];
  const a = new Vector3(), b = new Vector3(), ca = new Vector4(), cb = new Vector4();
  for (const route of routes) for (const line of route.lines) {
    if (!line || !visible(line)) continue;
    line.updateWorldMatrix(true, false);
    const starts = line.geometry.getAttribute("instanceStart"), ends = line.geometry.getAttribute("instanceEnd");
    if (!starts || !ends) continue;
    for (let i = 0; i < Math.min(starts.count, line.geometry.instanceCount); i++) {
      const window = visibleSegmentWindow(line, i);
      if (!window) continue;
      a.fromBufferAttribute(starts, i).applyMatrix4(line.matrixWorld); b.fromBufferAttribute(ends, i).applyMatrix4(line.matrixWorld);
      if (a.distanceToSquared(b) < 1e-12) continue;
      const ax = a.x, ay = a.y, az = a.z, bx = b.x, by = b.y, bz = b.z;
      a.set(ax + (bx - ax) * window[0], ay + (by - ay) * window[0], az + (bz - az) * window[0]);
      b.set(ax + (bx - ax) * window[1], ay + (by - ay) * window[1], az + (bz - az) * window[1]);
      ca.set(a.x, a.y, a.z, 1).applyMatrix4(camera.matrixWorldInverse).applyMatrix4(camera.projectionMatrix);
      cb.set(b.x, b.y, b.z, 1).applyMatrix4(camera.matrixWorldInverse).applyMatrix4(camera.projectionMatrix);
      const clip = frustumWindow(ca, cb);
      if (!clip) continue;
      if (clip[0] > 0 || clip[1] < 1) {
        const left = [ca.x, ca.y, ca.z, ca.w], right = [cb.x, cb.y, cb.z, cb.w];
        for (const [index, axis] of (["x", "y", "z", "w"] as const).entries()) {
          ca[axis] = left[index]! + (right[index]! - left[index]!) * clip[0]; cb[axis] = left[index]! + (right[index]! - left[index]!) * clip[1];
        }
      }
      const sx = (ca.x / ca.w + 1) * width / 2, sy = (1 - ca.y / ca.w) * height / 2;
      const ex = (cb.x / cb.w + 1) * width / 2, ey = (1 - cb.y / cb.w) * height / 2;
      if (x < Math.min(sx, ex) - radius || x > Math.max(sx, ex) + radius || y < Math.min(sy, ey) - radius || y > Math.max(sy, ey) + radius) continue;
      const dx = ex - sx, dy = ey - sy, squared = dx * dx + dy * dy;
      const t = squared > 1e-12 ? Math.max(0, Math.min(1, ((x - sx) * dx + (y - sy) * dy) / squared)) : 0;
      const distance = Math.hypot(x - sx - dx * t, y - sy - dy * t);
      if (distance > radius) continue;
      // Perspective-correct world parameter; linear screen interpolation is not a world point.
      const worldT = t * ca.w / ((1 - t) * cb.w + t * ca.w);
      const parameter = clip[0] + (clip[1] - clip[0]) * worldT;
      const position = a.clone().lerp(b, parameter);
      const depth = -position.clone().applyMatrix4(camera.matrixWorldInverse).z;
      candidates.push({ id: route.id, position, screenDistancePx: distance, depth });
    }
  }
  candidates.sort((a, b) => Math.floor(a.screenDistancePx * 4) - Math.floor(b.screenDistancePx * 4) || a.depth - b.depth);
  return candidates.find(candidate => !occluded(candidate.position)) ?? null;
}
