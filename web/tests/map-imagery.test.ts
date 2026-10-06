import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { decodeImageryTile } from "../src/map-imagery";
import { MAP_SURROUND } from "../src/map-surround-data";

let dimensions = [MAP_SURROUND.tileSize, MAP_SURROUND.tileSize];
const decode = vi.fn(async () => {});
class MockImage {
  src = "";
  decoding = "auto";
  naturalWidth = dimensions[0];
  naturalHeight = dimensions[1];
  decode = decode;
}

beforeEach(() => {
  dimensions = [MAP_SURROUND.tileSize, MAP_SURROUND.tileSize]; decode.mockReset(); decode.mockResolvedValue(undefined);
  vi.stubGlobal("Image", MockImage);
  vi.spyOn(URL, "createObjectURL").mockReturnValue("blob:imagery-fixture");
  vi.spyOn(URL, "revokeObjectURL").mockImplementation(() => {});
});
afterEach(() => { vi.restoreAllMocks(); vi.unstubAllGlobals(); });

describe("real-color imagery decoding", () => {
  it("decodes the image natively without painting, whitening or flipping its source pixels", async () => {
    const blob = new Blob(["fixture"], { type: "image/jpeg" });
    const image = await decodeImageryTile(blob);
    expect(image).toBeInstanceOf(MockImage); expect(image.src).toBe("blob:imagery-fixture");
    expect(image.decoding).toBe("async"); expect(decode).toHaveBeenCalledOnce();
    expect(URL.createObjectURL).toHaveBeenCalledWith(blob);
    expect(URL.revokeObjectURL).toHaveBeenCalledWith("blob:imagery-fixture");
  });

  it.each(["text/html", "application/json", "image/svg+xml"])("rejects error pages and non-raster content: %s", async type => {
    await expect(decodeImageryTile(new Blob(["error"], { type }))).rejects.toThrow("Invalid imagery");
    expect(URL.createObjectURL).not.toHaveBeenCalled();
  });

  it("rejects empty or oversized raster payloads before decoding", async () => {
    for (const blob of [new Blob([], { type: "image/png" }), new Blob([new Uint8Array(2_000_001)], { type: "image/jpeg" })]) {
      await expect(decodeImageryTile(blob)).rejects.toThrow("Invalid imagery");
    }
    expect(URL.createObjectURL).not.toHaveBeenCalled();
  });

  it("releases the object URL even when native decoding fails", async () => {
    decode.mockRejectedValueOnce(new Error("corrupt image"));
    await expect(decodeImageryTile(new Blob(["invalid"], { type: "image/png" }))).rejects.toThrow("corrupt image");
    expect(URL.revokeObjectURL).toHaveBeenCalledOnce();
  });

  it("rejects images that do not match the registered tile grid", async () => {
    dimensions = [512, 256];
    await expect(decodeImageryTile(new Blob(["fixture"], { type: "image/jpeg" }))).rejects.toThrow("dimensions");
    expect(URL.revokeObjectURL).toHaveBeenCalledOnce();
  });
});
