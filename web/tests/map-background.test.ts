import * as THREE from "three";
import { describe, expect, it } from "vitest";
import { addCityContext } from "../src/city-scene";
import { MAP_BACKGROUND } from "../src/map-background-data";
import { addMapBackground, backgroundRoads, roadRibbonVertices } from "../src/map-background";

const bounds = { min: [-240.46513, -186.36859, 0], max: [2542.36633, 2496.13957, 480] };

describe("simple sourced map background", () => {
  it("uses one shared offline road source only for its matching city", () => {
    expect(backgroundRoads(MAP_BACKGROUND.cityId)).toBe(MAP_BACKGROUND.roads);
    expect(backgroundRoads("some-other-city")).toEqual([]);
    expect(backgroundRoads(undefined)).toEqual([]);
    expect(MAP_BACKGROUND.source.url).toMatch(/^https:\/\/services6\.arcgis\.com\//);
    expect(MAP_BACKGROUND.source.projectionOriginWgs84).toEqual([-74.005, 40.742]);
    expect(MAP_BACKGROUND.roads.length).toBeGreaterThan(100);
    for (const road of MAP_BACKGROUND.roads) {
      expect(road.widthM).toBeGreaterThan(0);
      expect(road.points.length).toBeGreaterThanOrEqual(2);
      expect(road.points.flat().every(Number.isFinite)).toBe(true);
    }
  });

  it("clips full ribbons including sidewalk width at every edge and never grows the map", () => {
    const clippedBounds = { min: [0, 0, 0], max: [10, 10, 20] };
    const roads = [
      { widthM: 6, points: [[-10, -10], [20, 20]] },
      { widthM: 4, points: [[0, -10], [0, 20]] },
      { widthM: 8, points: [[-10, 10], [20, 10]] },
      { widthM: 2, points: [[-20, -20], [-10, -10]] },
    ];
    const vertices = roadRibbonVertices(roads, clippedBounds, -0.008, 3);
    expect(vertices.length).toBeGreaterThan(0);
    expect(vertices.length % 9).toBe(0);
    for (let index = 0; index < vertices.length; index += 3) {
      expect(vertices[index]).toBeGreaterThanOrEqual(0);
      expect(vertices[index]).toBeLessThanOrEqual(10);
      expect(vertices[index + 1]).toBe(-0.008);
      expect(-vertices[index + 2]!).toBeGreaterThanOrEqual(0);
      expect(-vertices[index + 2]!).toBeLessThanOrEqual(10);
    }
    expect(vertices).toEqual(roadRibbonVertices(roads, clippedBounds, -0.008, 3));
  });

  it("skips zero-length and invalid segments without creating NaNs", () => {
    expect(roadRibbonVertices([
      { widthM: 4, points: [[1, 1], [1, 1]] },
      { widthM: 4, points: [[Number.NaN, 1], [4, 4]] },
      { widthM: -4, points: [[1, 1], [4, 4]] },
    ], bounds, -0.004)).toEqual([]);
  });

  it("renders only two bounded ground buffers, not buildings, hazards or an image texture", () => {
    const host = new THREE.Group();
    addMapBackground(host, MAP_BACKGROUND.cityId, bounds);
    const background = host.getObjectByName("map-background")!;
    expect(background.userData.visualOnly).toBe(true);
    expect(background.children).toHaveLength(2);
    host.updateMatrixWorld(true);
    for (const child of background.children) {
      const mesh = child as THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>;
      expect(mesh.material.map).toBeNull();
      expect(mesh.material.depthTest).toBe(true);
      expect(mesh.receiveShadow).toBe(true);
      const box = new THREE.Box3().setFromObject(mesh);
      // Float32 GPU positions have sub-mm rounding at the kilometre-scale boundary.
      expect(box.min.x).toBeGreaterThanOrEqual(bounds.min[0]! - 0.001);
      expect(box.max.x).toBeLessThanOrEqual(bounds.max[0]! + 0.001);
      expect(-box.max.z).toBeGreaterThanOrEqual(bounds.min[1]! - 0.001);
      expect(-box.min.z).toBeLessThanOrEqual(bounds.max[1]! + 0.001);
      expect(box.max.y).toBeLessThan(bounds.min[2]!);
    }
  });

  it("leaves source city bounds and all planner inputs untouched", () => {
    const scenario = {
      id: "map-test", bounds, buildings: [{ min: [100, 100, 0], max: [140, 150, 40] }],
      start: [80, 80, 50], goal: [300, 300, 50],
      city: { id: MAP_BACKGROUND.cityId, name: "Midtown", sourceKind: "nyc-open-data" as const,
        sourceUrl: "https://data.cityofnewyork.us/City-Government/BUILDING/5zhs-2jue",
        sourceSha256: "1".repeat(64), collisionModel: "conservative-aabb" as const },
    };
    const before = JSON.stringify(scenario);
    const host = new THREE.Group();
    const viewBounds = addCityContext(host, scenario);
    expect(JSON.stringify(scenario)).toBe(before);
    expect(viewBounds.min.toArray()).toEqual([bounds.min[0], bounds.min[2], -bounds.max[1]!]);
    expect(viewBounds.max.toArray()).toEqual([bounds.max[0], bounds.max[2], -bounds.min[1]!]);
    expect(host.getObjectByName("map-background")).toBeDefined();
    expect(host.getObjectByName("city-round-tower")).toBeUndefined();
    expect(host.getObjectByName("city-street")).toBeUndefined();
  });

});
