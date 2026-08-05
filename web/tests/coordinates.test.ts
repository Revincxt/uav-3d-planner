import { describe, expect, it } from "vitest";

import { enuToThree } from "../src/coordinates";

describe("enuToThree", () => {
  it("maps east, north, up to x, negative z, y", () => {
    expect(enuToThree([12, 7, 3])).toEqual([12, 3, -7]);
  });
});

