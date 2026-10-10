// Public USGS / USDA native-resolution orthoimagery, not upscaled cached overview tiles.
const imageryService = "https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPPlus/ImageServer";

export const MAP_SURROUND = {
  cityId: "nyc-manhattan-midtown-official",
  role: "satellite-aerial-imagery",
  minZoom: 12,
  // A bounded client-side Mercator grid over the service's real imagery export API.
  // At Manhattan latitude, a 512 px level-19 image samples about 0.11 m per pixel.
  // Actual detail is still limited by the underlying aerial capture, not the grid.
  maxZoom: 19,
  maxVisibleTiles: 36,
  maxResidentTiles: 72,
  updateDelayMs: 100,
  maxConcurrentRequests: 4,
  maxRequestAttempts: 2,
  retryDelayMs: 1500,
  tileSize: 512,
  maxAnisotropy: 16,
  source: {
    url: `${imageryService}?f=json`,
    tileTemplate: `${imageryService}/exportImage`,
    attributionUrl: "https://www.usgs.gov/the-national-map-data-delivery",
    licenseUrl: "https://www.usgs.gov/tools/download-data-maps-national-map",
    copyright: "USDA · USGS The National Map",
    format: "512 px Web Mercator natural-color orthoimagery, JPEG quality 95",
    backgroundOnly: true,
    modifiesPlanningGeometry: false,
    extentPolicy: "Exact Web Mercator tile bounds projected into the physical city's ENU coordinates",
  },
};
