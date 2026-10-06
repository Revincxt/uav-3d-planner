import { describe, expect, it } from "vitest";
import { hasCompleteCityExtent, type CityMetadata } from "../src/city-schema";

const city: CityMetadata = { id: "nyc-manhattan-midtown-official", name: "Midtown",
  sourceKind: "nyc-open-data", sourceUrl: "https://data.cityofnewyork.us/",
  sourceSha256: "a".repeat(64), collisionModel: "conservative-aabb",
  planningRegion: { id: "midtown-landscape-v2" } };
const extent = (width: number, depth: number) => ({ min: [0, 0, 0], max: [width, depth, 480] });

describe("declared physical city region", () => {
  it("accepts the expanded depth without widening or stretching the city", () => {
    const expanded = { ...city, planningRegion: { id: "midtown-expanded-v3" } };
    expect(hasCompleteCityExtent(expanded, extent(3576, 3800))).toBe(true);
  });
  it.each([[3576, 1962], [3576, 2500], [3000, 3800], [4000, 3800], [3576, 4200]])(
    "rejects the wrong extent for the expanded region: %s × %s", (width, depth) => {
      const expanded = { ...city, planningRegion: { id: "midtown-expanded-v3" } };
      expect(hasCompleteCityExtent(expanded, extent(width, depth))).toBe(false);
    });
  it("accepts the new real landscape extent and keeps legacy square regions compatible", () => {
    expect(hasCompleteCityExtent(city, extent(3575.59154, 1961.95673))).toBe(true);
    expect(hasCompleteCityExtent({ ...city, planningRegion: undefined }, extent(2782.83146, 2682.50816))).toBe(true);
  });
  it.each([[2000, 1900], [3575, 900], [5000, 1900], [3575, 3500]])(
    "rejects a truncated or incorrectly proportioned declared landscape region: %s × %s", (width, depth) => {
      expect(hasCompleteCityExtent(city, extent(width, depth))).toBe(false);
    });
  it("does not silently apply the redesigned checks to an undeclared legacy region", () => {
    expect(hasCompleteCityExtent({ ...city, planningRegion: undefined }, extent(3575, 1962))).toBe(false);
  });
});
