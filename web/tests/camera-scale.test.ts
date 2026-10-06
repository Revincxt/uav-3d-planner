import * as THREE from "three";
import { describe, expect, it } from "vitest";
import { metresPerPixelAt } from "../src/camera-scale";
import { createDrone, sizeDrone } from "../src/drone-model";
import { disposeRenderObject } from "../src/render-resources";

describe("annotation scaling in overview and perspective follow", () => {
  it("keeps orthographic annotation size independent of depth", () => {
    const camera = new THREE.OrthographicCamera(-160, 160, 80, -80); camera.zoom = 2;
    expect(metresPerPixelAt(camera, new THREE.Vector3(0, 0, -100), 600)).toBeCloseTo(160 / 1200);
    expect(metresPerPixelAt(camera, new THREE.Vector3(0, 0, -500), 600)).toBeCloseTo(160 / 1200);
  });
  it("uses camera-space depth, field of view and zoom, not ground distance", () => {
    const camera = new THREE.PerspectiveCamera(70, 2, 0.15, 8000);
    const near = new THREE.Vector3(0, 0, -100), far = new THREE.Vector3(50, 0, -200);
    const size = 200 * Math.tan(35 * Math.PI / 180) / 600;
    expect(metresPerPixelAt(camera, near, 600)).toBeCloseTo(size);
    expect(metresPerPixelAt(camera, far, 600)).toBeCloseTo(size * 2);
    camera.lookAt(100, 0, 0);
    expect(metresPerPixelAt(camera, new THREE.Vector3(100, 0, 50), 600)).toBeCloseTo(size);
    camera.zoom = 2;
    expect(metresPerPixelAt(camera, new THREE.Vector3(100, 0, 50), 600)).toBeCloseTo(size / 2);
  });
  it("keeps finite positive sizes at the near plane and behind the perspective camera", () => {
    const camera = new THREE.PerspectiveCamera(70, 1, 0.15, 8000);
    for (const point of [new THREE.Vector3(), new THREE.Vector3(0, 0, 100)]) {
      const size = metresPerPixelAt(camera, point, 0);
      expect(Number.isFinite(size)).toBe(true); expect(size).toBeGreaterThan(0);
    }
  });
  it("renders other drones with depth-appropriate symbols without altering their positions", () => {
    const camera = new THREE.PerspectiveCamera(70, 2, 0.15, 8000), drone = createDrone(0x2563eb);
    drone.position.set(0, 0, -100); sizeDrone(drone, camera, 600, 24);
    const near = drone.scale.x;
    drone.position.z = -200; sizeDrone(drone, camera, 600, 24);
    expect(drone.scale.x).toBeCloseTo(near * 2);
    expect(drone.position.toArray()).toEqual([0, 0, -200]);
    disposeRenderObject(drone);
  });
  it("keeps the centered aircraft readable without oversized geometry when a wall shortens the boom", () => {
    const camera = new THREE.PerspectiveCamera(70, 2, 0.15, 8000), drone = createDrone(0x2563eb);
    drone.position.set(0, 0, -1);
    sizeDrone(drone, camera, 600, 88, 0);
    expect(drone.scale.x).toBeLessThan(0.8);
    expect(drone.scale.x * 2.1 / metresPerPixelAt(camera, drone.position, 600)).toBeCloseTo(88);
    expect(drone.position.toArray()).toEqual([0, 0, -1]);
    disposeRenderObject(drone);
  });
});
