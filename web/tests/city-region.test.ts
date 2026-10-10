import { describe, expect, it } from "vitest";
import { hasCompleteCityExtent, type CityMetadata } from "../src/city-schema";

const city: CityMetadata = { id: "nyc-manhattan-midtown-official", name: "Midtown",
  sourceKind: "nyc-open-data", sourceUrl: "https://data.cityofnewyork.us/",
  sourceSha256: "a".repeat(64), collisionModel: "conservative-aabb",
  planningRegion: { id: "manhattan-south-expanded-v4" } };
const extent = (width: number, depth: number) => ({ min: [0, 0, 0], max: [width, depth, 480] });

describe("declared physical city region", () => {
  it('requires the full southern island extent without stretching the existing ENU axes', () => {
    expect(hasCompleteCityExtent(city, extent(4550, 7600))).toBe(true);
    expect(hasCompleteCityExtent(city, extent(3576, 3800))).toBe(false);
    expect(hasCompleteCityExtent(city, extent(4550, 5500))).toBe(false);
  });
  it.each([[4300, 7600], [4900, 7600], [4550, 7300], [4550, 8100], [3576, 3800]])(
    "rejects a truncated or stretched current region: %s × %s", (width, depth) => {
      expect(hasCompleteCityExtent(city, extent(width, depth))).toBe(false);
    });
  it.each(["midtown-expanded-v3", "midtown-landscape-v2", "unknown"])("rejects retired or unknown region %s", (id) => {
    expect(hasCompleteCityExtent({ ...city, planningRegion: { id } }, extent(4550, 7600))).toBe(false);
  });
  it("requires an explicitly declared planning region", () => {
    expect(hasCompleteCityExtent({ ...city, planningRegion: undefined }, extent(4550, 7600))).toBe(false);
    expect(hasCompleteCityExtent(undefined, extent(4550, 7600))).toBe(false);
  });
});
