import { OrthographicCamera, Vector3 } from "three";
import { describe, expect, it, vi } from "vitest";
import { MapCameraMotion } from "../src/map-camera-motion";
function fixture(reduced = false) {
  const camera = new OrthographicCamera(), target = new Vector3(), request = vi.fn();
  const controls = { target, update: () => { camera.lookAt(target); return true; } };
  const motion = new MapCameraMotion(camera, controls, request, () => reduced);
  const west = () => { camera.position.set(-100, 27, 10); camera.left = -100; camera.right = 100; controls.update(); };
  const top = () => { camera.position.set(-.001, 200, 0); camera.left = -50; camera.right = 50; controls.update(); };
  motion.transition(west, 0); return { camera, target, request, motion, west, top };
}
describe("smooth safe map presets", () => {
  it("sets the first view immediately, then interpolates camera and matching frustum", () => {
    const f = fixture(); expect(f.camera.position.x).toBe(-100); f.motion.transition(f.top, 10);
    expect(f.camera.position.x).toBe(-100); expect(f.request).toHaveBeenCalled();
    expect(f.motion.step(170)).toBe(true); expect(f.camera.left).toBeCloseTo(-75);
    expect(f.camera.position.y).toBeGreaterThan(27); expect(f.camera.position.y).toBeLessThan(200);
    expect(f.motion.step(330)).toBe(false); expect(f.camera.position.y).toBe(200); expect(f.camera.left).toBe(-50);
  });
  it("never passes beneath the 15 degree orbit limit during transitions", () => {
    const f = fixture(); f.motion.transition(f.top, 0);
    for (let t = 0; t < 320; t += 4) { f.motion.step(t); const d = f.camera.position.clone().sub(f.target); expect(Math.asin(d.y / d.length()) * 180 / Math.PI).toBeGreaterThanOrEqual(15 - 1e-6); }
  });
  it("retargets rapidly without jumping back to the previous preset", () => {
    const f = fixture(); f.motion.transition(f.top, 0); f.motion.step(100); const position = f.camera.position.clone();
    f.motion.transition(f.west, 100); expect(f.camera.position.distanceTo(position)).toBeLessThan(1e-8);
    f.motion.step(420); expect(f.camera.position.x).toBe(-100);
  });
  it("allows user orbit to cancel motion and honors reduced-motion preferences", () => {
    const f = fixture(); f.motion.transition(f.top, 0); f.motion.step(100); const position = f.camera.position.clone(); f.motion.cancel();
    expect(f.motion.step(200)).toBe(false); expect(f.camera.position.equals(position)).toBe(true);
    const reduced = fixture(true); reduced.motion.transition(reduced.top, 0); expect(reduced.camera.position.y).toBe(200); expect(reduced.motion.step(100)).toBe(false);
  });
});
