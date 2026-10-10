import { Ray, Vector3 } from "three";
import { describe, expect, it } from "vitest";
import { BuildingOcclusion, intersectsBuilding } from "../src/building-occlusion";
import type { CityBox } from "../src/city-scene";
const building: CityBox = { min: [-5, -5, 0], max: [5, 5, 10], footprint: [[[-5, -5], [5, -5], [5, 5], [-5, 5], [-5, -5]]] };
describe("rendered city occlusion", () => {
  it("blocks walls and roofs before a route, not buildings behind it", () => {
    const city = new BuildingOcclusion([building]);
    expect(city.blocks(new Ray(new Vector3(-20, 5, 0), new Vector3(1, 0, 0)), 30)).toBe(true);
    expect(city.blocks(new Ray(new Vector3(-20, 15, 0), new Vector3(1, 0, 0)), 30)).toBe(false);
    expect(city.blocks(new Ray(new Vector3(0, 30, 0), new Vector3(0, -1, 0)), 25)).toBe(true);
    expect(city.blocks(new Ray(new Vector3(0, 30, 0), new Vector3(0, -1, 0)), 15)).toBe(false);
  });
  it("matches real footprint corners rather than blocking an empty AABB corner", () => {
    const triangle = { ...building, footprint: [[[-5, -5], [5, -5], [-5, 5], [-5, -5]]] };
    expect(new BuildingOcclusion([triangle]).blocks(new Ray(new Vector3(4, 30, -4), new Vector3(0, -1, 0)), 40)).toBe(false);
  });
  it("allows courtyard holes but blocks their side walls", () => {
    const courtyard = { ...building, footprint: [...building.footprint!, [[-2, -2], [-2, 2], [2, 2], [2, -2], [-2, -2]]] };
    const city = new BuildingOcclusion([courtyard]);
    expect(city.blocks(new Ray(new Vector3(0, 30, 0), new Vector3(0, -1, 0)), 40)).toBe(false);
    expect(city.blocks(new Ray(new Vector3(0, 5, 0), new Vector3(1, 0, 0)), 10)).toBe(true);
  });
  it("supports legacy boxes, empty cities, interior origins and roof-level precision", () => {
    const box = { min: building.min, max: building.max }, down = new Ray(new Vector3(0, 30, 0), new Vector3(0, -1, 0));
    expect(new BuildingOcclusion([]).blocks(down, 40)).toBe(false);
    expect(new BuildingOcclusion([box]).blocks(down, 40)).toBe(true);
    expect(new BuildingOcclusion([box]).blocks(down, 20)).toBe(false);
    expect(new BuildingOcclusion([box]).blocks(new Ray(new Vector3(0, 5, 0), new Vector3(1, 0, 0)), 10)).toBe(true);
  });
  it("has the same answer as exact brute force across BVH branches", () => {
    const boxes = Array.from({ length: 50 }, (_, i) => ({ min: [i % 10 * 15, Math.floor(i / 10) * 15, 0], max: [i % 10 * 15 + 10, Math.floor(i / 10) * 15 + 10, 10 + i] }));
    const city = new BuildingOcclusion(boxes); let seed = 71;
    const random = () => { seed = seed * 16807 % 2147483647; return seed / 2147483647; };
    for (let i = 0; i < 150; i++) {
      const ray = new Ray(new Vector3(random() * 180 - 20, random() * 80, -random() * 90), new Vector3(random() - .5, random() - .5, random() - .5).normalize());
      const distance = random() * 200;
      expect(city.blocks(ray, distance)).toBe(boxes.some(box => intersectsBuilding(ray, box, distance - .01)));
    }
  });
});
