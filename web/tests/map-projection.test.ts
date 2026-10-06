import { describe, expect, it } from "vitest";
import { lonLatToMapENU, mapENUToLonLat } from "../src/map-basemap";

describe("source-aligned WGS84 / ENU map projection", () => {
  it("retains the native NYC origin and measured Penn Station position", () => {
    expect(lonLatToMapENU(-74.005, 40.742)).toEqual([0, 0]);
    const p = lonLatToMapENU(-73.9935, 40.7506);
    expect(p[0]).toBeCloseTo(971.19250698153, 6);
    expect(p[1]).toBeCloseTo(955.0850703455951, 6);
  });
  it("round-trips the complete expanded city and both river banks", () => {
    for (const east of [-1000, -328.24036, 0, 3294.2203, 5000]) {
      for (const north of [-2000, -713.88319, 0, 3056.08779, 6000]) {
        const restored = lonLatToMapENU(...mapENUToLonLat(east, north));
        expect(restored[0]).toBeCloseTo(east, 5);
        expect(restored[1]).toBeCloseTo(north, 5);
      }
    }
  });
});
