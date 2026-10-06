import { MAP_SURROUND } from "./map-surround-data";

/** Native asynchronous decoding preserves the source colors and north-up orientation.
 * A normal image texture uses Three's default flipY; ImageBitmap would ignore that flag.
 */
export async function decodeImageryTile(blob: Blob): Promise<HTMLImageElement> {
  if (!/^image\/(jpeg|png|webp)$/i.test(blob.type) || !blob.size || blob.size > 2_000_000) {
    throw new Error("Invalid imagery tile response");
  }
  const url = URL.createObjectURL(blob);
  try {
    const image = new Image();
    image.decoding = "async";
    image.src = url;
    await image.decode();
    if (image.naturalWidth !== MAP_SURROUND.tileSize || image.naturalHeight !== MAP_SURROUND.tileSize) {
      throw new Error("Unexpected imagery tile dimensions");
    }
    return image;
  } finally {
    URL.revokeObjectURL(url);
  }
}
