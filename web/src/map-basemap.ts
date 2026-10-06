/** Shared WGS84 ↔ local ENU projection for source buildings and online imagery. */
const ORIGIN = [-74.005, 40.742] as const;
const EARTH_RADIUS = 6378137;

function ecef(longitude: number, latitude: number): number[] {
  const lon = longitude * Math.PI / 180, lat = latitude * Math.PI / 180;
  const radius = EARTH_RADIUS / Math.sqrt(1 - 6.6943799901413165e-3 * Math.sin(lat) ** 2);
  return [radius * Math.cos(lat) * Math.cos(lon), radius * Math.cos(lat) * Math.sin(lon),
    radius * (1 - 6.6943799901413165e-3) * Math.sin(lat)];
}

const ORIGIN_ECEF = ecef(...ORIGIN);
const ORIGIN_LON = ORIGIN[0] * Math.PI / 180, ORIGIN_LAT = ORIGIN[1] * Math.PI / 180;
const SIN_LON = Math.sin(ORIGIN_LON), COS_LON = Math.cos(ORIGIN_LON);
const SIN_LAT = Math.sin(ORIGIN_LAT), COS_LAT = Math.cos(ORIGIN_LAT);

/** Identical horizontal projection to the physical NYC building dataset. */
export function lonLatToMapENU(longitude: number, latitude: number): [number, number] {
  const origin = ORIGIN_ECEF, point = ecef(longitude, latitude);
  const dx = point[0]! - origin[0]!, dy = point[1]! - origin[1]!, dz = point[2]! - origin[2]!;
  return [-SIN_LON * dx + COS_LON * dy,
    -SIN_LAT * COS_LON * dx - SIN_LAT * SIN_LON * dy + COS_LAT * dz];
}

/** Invert the same local tangent projection, rather than stretching a north-up screenshot. */
export function mapENUToLonLat(east: number, north: number): [number, number] {
  let lon = ORIGIN[0] + east / (111320 * COS_LAT);
  let lat = ORIGIN[1] + north / 111132;
  const step = 1e-6;
  for (let iteration = 0; iteration < 4; iteration += 1) {
    const p = lonLatToMapENU(lon, lat);
    const a = lonLatToMapENU(lon + step, lat), b = lonLatToMapENU(lon, lat + step);
    const ax = (a[0] - p[0]) / step, ay = (a[1] - p[1]) / step;
    const bx = (b[0] - p[0]) / step, by = (b[1] - p[1]) / step;
    const dx = east - p[0], dy = north - p[1], determinant = ax * by - ay * bx;
    lon += (dx * by - dy * bx) / determinant;
    lat += (ax * dy - ay * dx) / determinant;
  }
  return [lon, lat];
}
