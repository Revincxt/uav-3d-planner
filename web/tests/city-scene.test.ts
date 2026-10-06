import * as THREE from "three";
import { describe, expect, it } from "vitest";

import { addCityBuilding, addCityBuildings, addCityContext, cityMissionBounds, type CityScenario } from "../src/city-scene";

const scenario: CityScenario = {
  id: "city-context-test",
  bounds: { min: [0, 0, 0], max: [100, 80, 60] },
  start: [5, 10, 4],
  goal: [95, 72, 40],
  buildings: [{ min: [30, 25, 0], max: [44, 48, 30] }],
};

const cityScenario: CityScenario = {
  ...scenario,
  bounds: { min: [0, 0, 0], max: [2800, 2700, 480] },
  city: {
    id: "nyc-test", name: "Manhattan", sourceKind: "nyc-open-data", collisionModel: "conservative-aabb",
    sourceUrl: "https://data.cityofnewyork.us/City-Government/BUILDING/5zhs-2jue", sourceSha256: "1".repeat(64),
  },
};

describe("real planning city presentation", () => {
  it("never grows the physical map or invents peripheral buildings", () => {
    const host = new THREE.Group();
    addCityContext(host, cityScenario);
    expect(host.getObjectByName("city-round-tower")).toBeUndefined();
    expect(host.getObjectByName("city-mission-boundary")).toBeUndefined();
    expect(host.getObjectByName("city-street")).toBeUndefined();
  });

  it("batches sourced polygon prisms while retaining courtyard holes", () => {
    const host = new THREE.Group();
    addCityBuildings(host, [{ min: [0, 0, 0], max: [20, 20, 40], footprint: [
      [[0, 0], [20, 0], [20, 20], [0, 20], [0, 0]],
      [[7, 7], [7, 13], [13, 13], [13, 7], [7, 7]],
    ] }]);
    host.updateMatrixWorld(true);
    const mesh = host.getObjectByName("planning-city-buildings-0")!;
    const maximum = new THREE.Box3().setFromObject(mesh).max;
    expect(maximum.x).toBeCloseTo(20, 8);
    expect(maximum.y).toBeCloseTo(40, 8);
    expect(maximum.z).toBeCloseTo(0, 8);
    const ray = new THREE.Raycaster(new THREE.Vector3(10, 70, -10), new THREE.Vector3(0, -1, 0));
    expect(ray.intersectObject(mesh)).toHaveLength(0);
    ray.set(new THREE.Vector3(3, 70, -3), new THREE.Vector3(0, -1, 0));
    expect(ray.intersectObject(mesh).length).toBeGreaterThan(0);
  });

  it("focuses the mission rather than fitting the entire city's limits", () => {
    const bounds = cityMissionBounds(cityScenario, [[300, 400, 65], [600, 700, 95]]);
    expect(bounds.getSize(new THREE.Vector3()).x).toBeLessThan(1000);
    expect(bounds.containsPoint(new THREE.Vector3(600, 95, -700))).toBe(true);
  });
});

describe("physical map context", () => {
  it("keeps the exact obstacle envelope when facade and rooftop details are enabled", () => {
    const host = new THREE.Group();
    const building = scenario.buildings![0]!;
    const mesh = addCityBuilding(host, building, 2);
    expect(mesh.geometry).toBeInstanceOf(THREE.BoxGeometry);
    const envelope = new THREE.Box3().setFromObject(mesh);
    expect(envelope.min.toArray()).toEqual([30, 0, -48]);
    expect(envelope.max.toArray()).toEqual([44, 30, -25]);
    expect(mesh.castShadow).toBe(true);
    expect(mesh.getObjectByName("city-facade")).toBeDefined();
    expect(mesh.getObjectByName("city-rooftop")).toBeDefined();
  });

  it("names the context, reports its dimensions, and returns ENU-converted framing bounds", () => {
    const host = new THREE.Group();
    const bounds = addCityContext(host, scenario);
    const context = host.getObjectByName("city-context")!;
    expect(context.userData.widthM).toBe(100);
    expect(context.userData.depthM).toBe(80);
    expect(context.userData.buildingCount).toBe(1);
    expect(bounds.min.toArray()).toEqual([0, 0, -80]);
    expect(bounds.max.toArray()).toEqual([100, 60, -0]);
    expect(context.getObjectByName("city-ground")).toBeDefined();
    expect(context.getObjectByName("city-mission-boundary")).toBeUndefined();
  });

  it("does not add fake architecture to a non-city fixture either", () => {
    const host = new THREE.Group();
    addCityContext(host, scenario);
    expect(host.getObjectByName("city-round-tower")).toBeUndefined();
    expect(host.getObjectByName("city-building")).toBeUndefined();
    expect(host.getObjectByName("city-street")).toBeUndefined();
  });
});
