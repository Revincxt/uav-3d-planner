import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { describe, expect, it, vi } from "vitest";
import { DroneFollow } from "../src/drone-follow";
import { createDrone, orientDrone, sizeDrone } from "../src/drone-model";
import { configureMapNavigation } from "../src/map-navigation";
import { disposeRenderObject } from "../src/render-resources";
import { avoidDroneMarkerOverlap } from "../src/mission-tasks";

function fixture() {
  const camera = new THREE.OrthographicCamera(-1000, 1000, 600, -600, -10000, 10000);
  camera.position.set(-2000, 1200, 1000); camera.zoom = 1.7;
  const controls = new OrbitControls(camera, null);
  configureMapNavigation(controls); controls.target.set(100, 80, -200); controls.update();
  const follow = new DroneFollow(controls);
  return { camera, controls, follow };
}

describe("quadcopter display", () => {
  it("has four rotors and three batched opaque, depth-tested parts, not a moving dot", () => {
    const drone = createDrone(0x2563eb);
    expect(drone.userData.rotorCount).toBe(4);
    expect(drone.children).toHaveLength(3);
    drone.traverse(object => {
      if (!(object instanceof THREE.Mesh)) return;
      expect(object.material.transparent).toBe(false);
      expect(object.material.depthTest).toBe(true);
      expect(object.material.depthWrite).toBe(true);
      expect(object.geometry.index!.count).toBeGreaterThan(30);
    });
    disposeRenderObject(drone);
  });
  it("uses horizontal heading, retains its level attitude and does not spin during holds", () => {
    const drone = createDrone(0x2563eb);
    orientDrone(drone, new THREE.Vector3(10, 200, 0));
    expect(drone.rotation.y).toBeCloseTo(-Math.PI / 2);
    expect(drone.rotation.x).toBe(0); expect(drone.rotation.z).toBe(0);
    orientDrone(drone, new THREE.Vector3(0, 20, 0));
    expect(drone.rotation.y).toBeCloseTo(-Math.PI / 2);
    disposeRenderObject(drone);
  });
  it("adjusts display scale for zoom without changing position or collision data", () => {
    const drone = createDrone(0x2563eb), { camera } = fixture();
    drone.position.set(200, 90, -300); sizeDrone(drone, camera, 600);
    const previous = drone.scale.x;
    camera.zoom *= 2; sizeDrone(drone, camera, 600);
    expect(drone.scale.x).toBeCloseTo(previous / 2);
    expect(drone.position.toArray()).toEqual([200, 90, -300]);
    expect(drone.userData.displaySymbol).toBe(true); disposeRenderObject(drone);
  });
  it("keeps the gate visible during a fly-through but hides the number near the drone", () => {
    const host = new THREE.Group(), tasks = new THREE.Group(), stop = new THREE.Group();
    tasks.name = "mission-task-points"; stop.position.set(100, 90, 100);
    const gate = new THREE.Mesh(), number = new THREE.Sprite();
    stop.add(gate, number); tasks.add(stop); host.add(tasks);
    const drone = createDrone(0x2563eb); drone.position.copy(stop.position);
    avoidDroneMarkerOverlap(host, drone); expect(gate.visible).toBe(true); expect(number.visible).toBe(false);
    drone.position.x += 100; avoidDroneMarkerOverlap(host, drone);
    expect(stop.children.every(marker => marker.visible)).toBe(true);
    disposeRenderObject(host);
    disposeRenderObject(drone);
  });
});

describe("centered close chase camera", () => {
  const north = new THREE.Vector3(0, 0, -1), east = new THREE.Vector3(1, 0, 0);
  it("uses perspective just behind the aircraft, centers it and retains the 15 degree view", () => {
    const { controls, follow } = fixture(), position = new THREE.Vector3(100, 90, -200);
    follow.start("uav-1", position, north, 1200, 600);
    expect(follow.routeId).toBe("uav-1"); expect(controls.enabled).toBe(false);
    expect(follow.camera).toBeInstanceOf(THREE.PerspectiveCamera);
    expect(follow.camera.aspect).toBe(2); expect(follow.camera.fov).toBe(70);
    expect(follow.camera.position.x).toBeCloseTo(100);
    expect(follow.camera.position.z).toBeCloseTo(-188);
    expect(follow.camera.position.y - position.y).toBeCloseTo(12 * Math.tan(Math.PI / 12));
    const center = position.clone().project(follow.camera);
    expect(center.x).toBeCloseTo(0); expect(center.y).toBeCloseTo(0);
    expect(center.z).toBeGreaterThan(-1); expect(center.z).toBeLessThan(1);
    const direction = follow.camera.getWorldDirection(new THREE.Vector3());
    expect(direction.x).toBeCloseTo(0); expect(direction.z).toBeLessThan(0);
    expect(Math.asin(-direction.y) * 180 / Math.PI).toBeCloseTo(15);
    expect(position.toArray()).toEqual([100, 90, -200]);
  });
  it("immediately re-centers at seeks and rewinds with the recorded heading", () => {
    const { follow } = fixture(), origin = new THREE.Vector3(100, 90, -200);
    follow.start("uav-1", origin, north, 1200, 600); follow.update(origin, north, 0, 1000);
    const next = new THREE.Vector3(500, 130, 500);
    follow.update(next, east, 80, 1016);
    expect(follow.camera.position.x).toBeCloseTo(488);
    expect(follow.camera.position.z).toBeCloseTo(500);
    expect(follow.camera.position.y - next.y).toBeCloseTo(12 * Math.tan(Math.PI / 12));
    expect(follow.camera.getWorldDirection(new THREE.Vector3()).x).toBeCloseTo(Math.cos(Math.PI / 12));
    follow.update(origin, north, 0, 1032);
    expect(follow.camera.position.x).toBeCloseTo(100);
    expect(follow.camera.position.z).toBeCloseTo(-188);
    expect(origin.clone().project(follow.camera).y).toBeCloseTo(0);
    expect(follow.camera.getWorldDirection(new THREE.Vector3()).x).toBeCloseTo(0);
  });
  it("smooths turns while keeping the moving aircraft centered, and settles while paused", () => {
    const { follow } = fixture(), origin = new THREE.Vector3(0, 90, 0), next = new THREE.Vector3(15, 90, 0);
    follow.start("uav-1", origin, north, 1200, 600); follow.update(origin, north, 0, 1000);
    expect(follow.update(next, east, 1, 1016)).toBe(true);
    expect(follow.camera.position.distanceTo(next)).toBeCloseTo(12 / Math.cos(Math.PI / 12));
    expect(next.clone().project(follow.camera).x).toBeCloseTo(0);
    expect(next.clone().project(follow.camera).y).toBeCloseTo(0);
    const direction = follow.camera.getWorldDirection(new THREE.Vector3());
    expect(direction.x).toBeGreaterThan(0); expect(direction.x).toBeLessThan(Math.cos(Math.PI / 12));
    expect(direction.y).toBeCloseTo(-Math.sin(Math.PI / 12));
    for (let frame = 2; frame < 160; frame++) follow.update(next, east, 1, 1000 + frame * 16);
    expect(follow.camera.getWorldDirection(new THREE.Vector3()).x).toBeCloseTo(Math.cos(Math.PI / 12));
    expect(follow.update(next, east, 1, 4000)).toBe(false);
  });
  it("turns across +/-180 degrees using the short arc, never rolling the horizon", () => {
    const { follow } = fixture(), origin = new THREE.Vector3(0, 90, 0);
    const heading = (yaw: number) => new THREE.Vector3(-Math.sin(yaw), 0, -Math.cos(yaw));
    follow.start("uav-1", origin, heading(179 * Math.PI / 180), 1200, 600);
    follow.update(origin, heading(179 * Math.PI / 180), 0, 1000);
    follow.update(origin, heading(-179 * Math.PI / 180), 1, 1016);
    expect(follow.camera.getWorldDirection(new THREE.Vector3()).z).toBeGreaterThan(0.96);
    expect(follow.camera.rotation.z).toBe(0);
  });
  it("leaves the overview pose/projection untouched across target changes, resize and exit", () => {
    const { camera, controls, follow } = fixture();
    const before = camera.position.clone(), target = controls.target.clone(), projection = camera.projectionMatrix.clone();
    const onChange = vi.fn(); follow.onChange = onChange;
    follow.start("uav-1", new THREE.Vector3(0, 90, 0), north, 1200, 600);
    follow.resize(390, 844); expect(follow.camera.aspect).toBeCloseTo(390 / 844);
    follow.start("uav-4", new THREE.Vector3(1000, 70, 400), east, 600, 600);
    expect(follow.camera.aspect).toBe(1); follow.stop();
    expect(camera.position.distanceTo(before)).toBe(0); expect(controls.target.distanceTo(target)).toBe(0);
    expect(camera.projectionMatrix.equals(projection)).toBe(true);
    expect(camera.zoom).toBe(1.7); expect(camera.right).toBe(1000);
    expect(controls.enabled).toBe(true); expect(follow.routeId).toBeNull();
    expect(onChange).toHaveBeenLastCalledWith(false); expect(follow.resize(600, 600)).toBe(false);
  });
  it("does not enable controls that were disabled before follow mode", () => {
    const { controls, follow } = fixture(); controls.enabled = false;
    follow.start("uav-1", new THREE.Vector3(0, 90, 0), north, 600, 600);
    follow.start("uav-2", new THREE.Vector3(0, 90, 0), east, 600, 600);
    follow.stop(); expect(controls.enabled).toBe(false);
  });
  it.each([[1440, 650], [1280, 470], [768, 800], [390, 600], [844, 120]])("keeps the aircraft centered at %ix%i", (width, height) => {
    const { follow } = fixture(), position = new THREE.Vector3(100, 90, -200);
    follow.start("uav-1", position, east, width, height);
    const center = position.clone().project(follow.camera);
    expect(center.x).toBeCloseTo(0); expect(center.y).toBeCloseTo(0);
    expect(follow.camera.aspect).toBeCloseTo(width / height);
  });
  it("shortens the boom before a building while keeping the aircraft centered and depth-tested", () => {
    const { follow } = fixture(), position = new THREE.Vector3(0, 30, 0);
    // ENU north -6..-4 is Three Z +4..+6, directly behind a northbound aircraft.
    const buildings = [{ min: [-2, -6, 0], max: [2, -4, 100] }];
    follow.setBuildings(buildings);
    follow.start("uav-1", position, north, 1200, 600);
    expect(follow.camera.position.z).toBeLessThan(4);
    expect(follow.camera.position.z).toBeGreaterThan(3);
    expect(position.clone().project(follow.camera).x).toBeCloseTo(0);
    expect(position.clone().project(follow.camera).y).toBeCloseTo(0);
    expect(follow.camera.position.distanceTo(position)).toBeCloseTo(4 / Math.cos(Math.PI / 12) - 0.5);
    follow.setBuildings([]); follow.update(position, north, 80, 1000);
    expect(follow.camera.position.z).toBeCloseTo(12);
    expect(position.toArray()).toEqual([0, 30, 0]);
  });
  it("ignores buildings below or beyond the short boom and supports negative grid coordinates", () => {
    const { follow } = fixture(), position = new THREE.Vector3(-64, 30, -64);
    follow.setBuildings([
      { min: [-66, 40, 0], max: [-62, 45, 100] }, // Beyond the 12 m boom.
      { min: [-66, 58, 0], max: [-62, 60, 10] }, // Below the aircraft.
    ]);
    follow.start("uav-1", position, north, 1200, 600);
    expect(follow.camera.position.z).toBeCloseTo(-52);
    follow.setBuildings([{ min: [-66, 58, 0], max: [-62, 60, 100] }]);
    follow.update(position, north, 80, 1000);
    expect(follow.camera.position.z).toBeLessThan(-60);
    expect(position.clone().project(follow.camera).x).toBeCloseTo(0);
    expect(position.clone().project(follow.camera).y).toBeCloseTo(0);
  });
  it("retracts before walls immediately but smoothly restores distance without losing center", () => {
    const { follow } = fixture(), position = new THREE.Vector3(0, 30, 0);
    const buildings = [{ min: [-2, -6, 0], max: [2, -4, 100] }];
    follow.setBuildings(buildings); follow.start("uav-1", position, north, 1200, 600);
    follow.update(position, north, 0, 1000);
    const near = follow.camera.position.distanceTo(position);
    follow.setBuildings([]);
    expect(follow.update(position, north, 1, 1016)).toBe(true);
    expect(follow.camera.position.distanceTo(position)).toBeGreaterThan(near);
    expect(follow.camera.position.distanceTo(position)).toBeLessThan(12 / Math.cos(Math.PI / 12));
    expect(position.clone().project(follow.camera).y).toBeCloseTo(0);
    for (let frame = 2; frame < 160; frame++) follow.update(position, north, 1, 1000 + frame * 16);
    expect(follow.camera.position.distanceTo(position)).toBeCloseTo(12 / Math.cos(Math.PI / 12));
    expect(follow.update(position, north, 1, 4000)).toBe(false);
    follow.setBuildings(buildings); follow.update(position, north, 1, 4016);
    expect(follow.camera.position.distanceTo(position)).toBeCloseTo(near);
    expect(position.clone().project(follow.camera).x).toBeCloseTo(0);
    expect(position.clone().project(follow.camera).y).toBeCloseTo(0);
  });
});
