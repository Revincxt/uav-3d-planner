/** Refresh visual-only Manhattan roads from the official NYC OTI centerline service.
 *
 * No keys, tiles, planning files, or public study artifacts are involved. Run with Node 22+.
 * The generated TypeScript is a local asset: application previews never call the service.
 */
import { createHash } from "node:crypto";
import { writeFile } from "node:fs/promises";
import { pathToFileURL, fileURLToPath } from "node:url";

const SOURCE_URL = "https://services6.arcgis.com/yG5s3afENB5iO9fj/arcgis/rest/services/Centerline_view/FeatureServer/0";
const METADATA_URL = "https://github.com/CityOfNewYork/nyc-geo-metadata/blob/main/Metadata/Metadata_StreetCenterline.md";
const ORIGIN = [-74.005, 40.742];
// Covers the expanded physical region, with a small margin for whole edge streets.
const BBOX = [-74.021, 40.6995, -73.9655, 40.7705];
const SIMPLIFICATION_M = 1;
const ROUNDING_M = 0.01;
const QUERY_SELECTION = "STATUS = '2' AND RW_TYPE IN (1, 2, 3, 9, 10) AND (FROM_LEVEL_CODE IS NULL OR FROM_LEVEL_CODE >= 13) AND (TO_LEVEL_CODE IS NULL OR TO_LEVEL_CODE >= 13)";

function ecef(longitude, latitude) {
  const lon = longitude * Math.PI / 180;
  const lat = latitude * Math.PI / 180;
  const eccentricitySquared = 6.6943799901413165e-3;
  const radius = 6378137 / Math.sqrt(1 - eccentricitySquared * Math.sin(lat) ** 2);
  return [radius * Math.cos(lat) * Math.cos(lon), radius * Math.cos(lat) * Math.sin(lon), radius * (1 - eccentricitySquared) * Math.sin(lat)];
}

/** Same WGS84 ECEF-to-ENU formula and origin as uav3d.manhattan_city; metres, no axis rotation. */
export function lonLatToENU(longitude, latitude) {
  const origin = ecef(...ORIGIN);
  const point = ecef(longitude, latitude);
  const [dx, dy, dz] = point.map((value, axis) => value - origin[axis]);
  const lon = ORIGIN[0] * Math.PI / 180;
  const lat = ORIGIN[1] * Math.PI / 180;
  return [-Math.sin(lon) * dx + Math.cos(lon) * dy, -Math.sin(lat) * Math.cos(lon) * dx - Math.sin(lat) * Math.sin(lon) * dy + Math.cos(lat) * dz];
}

function distanceToSegment(point, start, end) {
  const dx = end[0] - start[0];
  const dy = end[1] - start[1];
  const squared = dx ** 2 + dy ** 2;
  const parameter = squared === 0 ? 0 : Math.min(1, Math.max(0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / squared));
  return Math.hypot(point[0] - start[0] - parameter * dx, point[1] - start[1] - parameter * dy);
}

function simplify(points, tolerance) {
  if (points.length <= 2) return points;
  let farthest = -1;
  let maximum = tolerance;
  for (let index = 1; index < points.length - 1; index += 1) {
    const separation = distanceToSegment(points[index], points[0], points.at(-1));
    if (separation > maximum) { maximum = separation; farthest = index; }
  }
  if (farthest === -1) return [points[0], points.at(-1)];
  return [...simplify(points.slice(0, farthest + 1), tolerance).slice(0, -1), ...simplify(points.slice(farthest), tolerance)];
}

function canonicalLine(points) {
  const forward = JSON.stringify(points);
  const backward = JSON.stringify([...points].reverse());
  return forward < backward ? forward : backward;
}

async function readJSON(url) {
  const response = await fetch(url, { signal: AbortSignal.timeout(30_000) });
  if (!response.ok) throw new Error(`Official source returned ${response.status}: ${url}`);
  const raw = await response.text();
  const parsed = JSON.parse(raw);
  if (parsed.error) throw new Error(`Official source query failed: ${JSON.stringify(parsed.error)}`);
  return { parsed, raw };
}

export async function fetchMapBackground(output = new URL("../web/src/map-background-data.ts", import.meta.url)) {
  const { parsed: metadata } = await readJSON(`${SOURCE_URL}?f=json`);
  const fields = ["OBJECTID", "PHYSICALID", "FULL_STREET_NAME", "STREET_NAME", "FCC", "RW_TYPE", "STATUS"];
  if (fields.some((name) => !metadata.fields.some((field) => field.name === name))) {
    throw new Error("Official centerline fields changed; inspect the service before regenerating");
  }
  const pageSize = Math.min(metadata.maxRecordCount, 2000);
  const checksum = createHash("sha256");
  const features = [];
  let pages = 0;
  for (let offset = 0; ; offset += pageSize) {
    const parameters = new URLSearchParams({
      f: "json", where: QUERY_SELECTION,
      geometry: BBOX.join(","), geometryType: "esriGeometryEnvelope",
      inSR: "4326", outSR: "4326", spatialRel: "esriSpatialRelIntersects",
      outFields: fields.join(","), returnGeometry: "true",
      orderByFields: "OBJECTID", resultOffset: String(offset), resultRecordCount: String(pageSize),
    });
    const { parsed, raw } = await readJSON(`${SOURCE_URL}/query?${parameters}`);
    if (parsed.spatialReference?.wkid !== 4326 || !Array.isArray(parsed.features)) {
      throw new Error("Official service must return complete WGS84 polyline features");
    }
    checksum.update(raw);
    pages += 1;
    features.push(...parsed.features);
    if (!parsed.exceededTransferLimit) break;
    if (pages >= 10) throw new Error("Unexpectedly large query; refusing an incomplete visual layer");
  }
  const seen = new Set();
  const roads = [];
  let sourceVertexCount = 0;
  let removedDuplicates = 0;
  let maxDeviationM = 0;
  for (const feature of features) {
    const attributes = feature.attributes;
    for (const path of feature.geometry?.paths ?? []) {
      if (path.length < 2 || path.some((point) => point.length < 2 || !Number.isFinite(point[0]) || !Number.isFinite(point[1]))) {
        throw new Error(`Malformed official centerline geometry: ${attributes.OBJECTID}`);
      }
      const projected = path.map(([longitude, latitude]) => lonLatToENU(longitude, latitude));
      const points = simplify(projected, SIMPLIFICATION_M).map((point) => point.map((value) => Math.round(value / ROUNDING_M) * ROUNDING_M));
      // Round for a compact source asset while avoiding binary decimal tails in generated JSON.
      for (const point of points) for (let axis = 0; axis < 2; axis += 1) point[axis] = Number(point[axis].toFixed(2));
      const key = canonicalLine(points);
      if (seen.has(key)) { removedDuplicates += 1; continue; }
      seen.add(key);
      sourceVertexCount += projected.length;
      for (const original of projected) {
        const deviation = Math.min(...points.slice(1).map((end, index) => distanceToSegment(original, points[index], end)));
        maxDeviationM = Math.max(maxDeviationM, deviation);
      }
      const name = attributes.FULL_STREET_NAME || attributes.STREET_NAME;
      roads.push({ ...(name ? { name } : {}), widthM: 12, points });
    }
  }
  if (roads.length < 100 || maxDeviationM > SIMPLIFICATION_M + Math.SQRT2 * ROUNDING_M / 2 + 1e-6) {
    throw new Error("Road coverage or simplification audit failed; do not replace the existing visual layer");
  }
  const source = {
    url: SOURCE_URL, metadataUrl: METADATA_URL,
    publisher: "New York City Office of Technology and Innovation (OTI)",
    dataset: "Citywide Street Centerline (CSCL)", serviceItemId: metadata.serviceItemId,
    retrievedAt: new Date().toISOString(),
    sourceLastEditedAt: new Date(metadata.editingInfo.dataLastEditDate).toISOString(),
    sha256: `sha256:${checksum.digest("hex")}`,
    sha256Scope: "concatenated unmodified ArcGIS query response bytes in OBJECTID order",
    requestedBoundsWgs84: BBOX,
    projectionOriginWgs84: ORIGIN,
    projection: "WGS84 ECEF to local east/north tangent plane; identical to uav3d.manhattan_city.lonlat_to_enu; ground flattened to z=0",
    widthPolicy: "visual-only",
    widthDescription: "Uniform 12 m illustrative width, not measured road widths, functional classes or planning corridors",
    selection: QUERY_SELECTION,
    simplificationToleranceM: SIMPLIFICATION_M,
    coordinateRoundingM: ROUNDING_M,
    maximumSourceVertexDeviationM: Number(maxDeviationM.toFixed(6)),
    sourceFeatureCount: features.length,
    sourceVertexCount,
    removedExactForwardReverseDuplicates: removedDuplicates,
    backgroundOnly: true,
    modifiesPlanningGeometry: false,
  };
  const content = `// Generated by scripts/fetch_map_background.mjs from official NYC OTI data.\n// Visual ground background only; never use these stylized widths for planning or safety.\nexport const MAP_BACKGROUND = ${JSON.stringify({ cityId: "nyc-manhattan-midtown-official", source, roads })};\n`;
  if (Buffer.byteLength(content) > 600_000) throw new Error(`Background asset exceeds the 600 KB budget: ${Buffer.byteLength(content)} bytes / ${roads.length} road pieces`);
  await writeFile(output, content, "utf8");
  console.log(JSON.stringify({ output: fileURLToPath(output), bytes: Buffer.byteLength(content), roads: roads.length, sourceFeatureCount: features.length, sourceVertexCount, outputVertexCount: roads.reduce((total, road) => total + road.points.length, 0), maxDeviationM, removedDuplicates, sha256: source.sha256 }));
  return { cityId: "nyc-manhattan-midtown-official", source, roads };
}

if (process.argv[1] && pathToFileURL(process.argv[1]).href === import.meta.url) {
  await fetchMapBackground();
}
