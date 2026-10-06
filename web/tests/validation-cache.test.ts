import { describe, expect, it, vi } from "vitest";
import { ValidationCache } from "../src/validation-cache";
describe("invocation-scoped validation reuse", () => {
  it("reuses identical input only within a call and only for the same bounds/key", () => {
    const cache = new ValidationCache(), input = [1, 2, 3], parse = vi.fn(() => input.slice());
    cache.run(() => {
      expect(cache.memo(input, "vec3", parse)).toBe(cache.memo(input, "vec3", parse));
      cache.memo(input, "other-bounds", parse);
      expect(parse).toHaveBeenCalledTimes(2);
    });
    input[0] = 9;
    expect(cache.run(() => cache.memo(input, "vec3", parse))).toEqual([9, 2, 3]);
    expect(parse).toHaveBeenCalledTimes(3);
  });
  it("does not leave a cache active after failure or merge independent equal objects", () => {
    const cache = new ValidationCache(), input = {};
    const parse = vi.fn(() => ({}));
    expect(() => cache.run(() => { cache.memo(input, "x", parse); throw new Error("invalid"); })).toThrow();
    expect(cache.memo(input, "x", parse)).not.toBe(cache.memo(input, "x", parse));
    cache.run(() => expect(cache.memo({}, "x", parse)).not.toBe(cache.memo({}, "x", parse)));
  });
});
