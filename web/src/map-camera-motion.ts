import { OrthographicCamera, Spherical, Vector3 } from "three";
import type { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { MIN_MAP_ELEVATION_RAD } from "./map-navigation";
interface Pose { position: Vector3; target: Vector3; left: number; right: number; top: number; bottom: number; near: number; far: number; zoom: number }
const fields = ["left", "right", "top", "bottom", "near", "far", "zoom"] as const;
/** Retargetable orbit interpolation with a matching orthographic frustum, never below 15°. */
export class MapCameraMotion {
  private initialized = false;
  private flight?: { from: Pose; to: Pose; start: number };
  constructor(private readonly camera: OrthographicCamera, private readonly controls: Pick<OrbitControls, "target" | "update">,
    private readonly request: () => void, private readonly reduced = () => typeof matchMedia !== "undefined" && matchMedia("(prefers-reduced-motion: reduce)").matches) {}
  private capture(): Pose {
    return { position: this.camera.position.clone(), target: this.controls.target.clone(), left: this.camera.left, right: this.camera.right,
      top: this.camera.top, bottom: this.camera.bottom, near: this.camera.near, far: this.camera.far, zoom: this.camera.zoom };
  }
  private apply(pose: Pose): void {
    this.camera.position.copy(pose.position); this.controls.target.copy(pose.target);
    for (const key of fields) this.camera[key] = pose[key];
    this.camera.updateProjectionMatrix(); this.controls.update();
  }
  transition(destination: () => void, now = performance.now()): void {
    if (this.flight) this.step(now);
    const from = this.capture(); destination(); const to = this.capture(); this.flight = undefined;
    if (!this.initialized || this.reduced()) { this.initialized = true; return; }
    this.apply(from); this.flight = { from, to, start: now }; this.request();
  }
  step(now = performance.now()): boolean {
    if (!this.flight) return false;
    const { from, to, start } = this.flight, fraction = Math.min(1, Math.max(0, (now - start) / 320));
    if (fraction === 1) { this.flight = undefined; this.apply(to); return false; }
    const t = fraction * fraction * (3 - 2 * fraction), a = new Spherical().setFromVector3(from.position.clone().sub(from.target));
    const b = new Spherical().setFromVector3(to.position.clone().sub(to.target));
    const delta = Math.atan2(Math.sin(b.theta - a.theta), Math.cos(b.theta - a.theta));
    const polar = Math.min(Math.PI / 2 - MIN_MAP_ELEVATION_RAD, Math.max(.000001, a.phi + (b.phi - a.phi) * t));
    const target = from.target.clone().lerp(to.target, t);
    const pose = { ...from, target, position: new Vector3().setFromSpherical(new Spherical(a.radius + (b.radius - a.radius) * t, polar, a.theta + delta * t)).add(target) };
    for (const key of fields) pose[key] = from[key] + (to[key] - from[key]) * t;
    this.apply(pose); return true;
  }
  cancel(): void { this.flight = undefined; }
}
