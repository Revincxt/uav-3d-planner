import * as THREE from "three";
import { afterEach, describe, expect, it, vi } from "vitest";
import { MAP_SURROUND } from "../src/map-surround-data";
import { lonLatToMapENU, mapENUToLonLat } from "../src/map-basemap";
import { addMapSurround, createTerrainTileGeometry, TerrainMapLayer, terrainTileBounds,
  terrainTileTemplate, terrainTileURL, updateMapSurround, visibleTerrainTiles } from "../src/map-surround";
import { addCityContext } from "../src/city-scene";
import { disposeRenderObject } from "../src/render-resources";
import { decodeImageryTile } from "../src/map-imagery";

vi.mock("../src/map-imagery", () => ({ decodeImageryTile: vi.fn(async () => ({} as HTMLImageElement)) }));

const bounds = { min: [-240.46513, -186.36859, 0], max: [2542.36633, 2496.13957, 480] };
const tile = { z: 15, x: 9648, y: 12315, key: "15/9648/12315" };
const template = MAP_SURROUND.source.tileTemplate;
const source = { capabilities: "Image,Metadata,Catalog", bandCount: 4,
  maxImageWidth: 4000, maxImageHeight: 4000, rasterFunctionInfos: [{ name: "NaturalColor" }], extent: {
  xmin: -Math.PI * 6378137, ymin: -Math.PI * 6378137, xmax: Math.PI * 6378137, ymax: Math.PI * 6378137,
  spatialReference: { wkid: 102100, latestWkid: 3857 },
} };
type TileMesh = THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>;

function camera(span = 4500, x = 1100): THREE.OrthographicCamera {
  const camera = new THREE.OrthographicCamera(-span / 2, span / 2, span / 3, -span / 3, 1, 100_000);
  camera.position.set(x, 8000, -1100); camera.up.set(0, 0, -1);
  camera.lookAt(x, 0, -1100); camera.updateProjectionMatrix(); camera.updateMatrixWorld(true);
  return camera;
}

async function settle() { for (let pass = 0; pass < 10; pass++) await Promise.resolve(); }

function mockTiles() {
  vi.useFakeTimers();
  vi.stubGlobal("document", {});
  const requests: { url: string; signal: AbortSignal; succeed: () => Promise<void>; fail: () => Promise<void> }[] = [];
  const fetcher = vi.fn((url: string, options: RequestInit) => {
    expect(options.credentials).toBe("omit");
    if (url === MAP_SURROUND.source.url) return Promise.resolve({ ok: true, json: async () => source });
    return new Promise((resolve, reject) => {
      requests.push({ url, signal: options.signal as AbortSignal,
        succeed: async () => { resolve({ ok: true, blob: async () => new Blob(["fixture"], { type: "image/jpeg" }) }); await settle(); },
        fail: async () => { reject(new Error("offline")); await settle(); } });
    });
  });
  vi.stubGlobal("fetch", fetcher);
  return { fetcher, requests };
}

afterEach(() => { vi.restoreAllMocks(); vi.unstubAllGlobals(); vi.useRealTimers(); });

describe("continuous satellite-style imagery map", () => {
  it("loads actual high-resolution public imagery without a key, with complete attribution", () => {
    expect(terrainTileTemplate(source)).toBe(template);
    const url = new URL(terrainTileURL(tile, template)), extent = terrainTileBounds(tile);
    expect(url.origin).toBe("https://imagery.nationalmap.gov");
    expect(url.pathname).toBe("/arcgis/rest/services/USGSNAIPPlus/ImageServer/exportImage");
    expect(url.searchParams.get("bbox")).toBe([...extent.min, ...extent.max].join(","));
    expect(url.searchParams.get("bboxSR")).toBe("3857"); expect(url.searchParams.get("imageSR")).toBe("3857");
    expect(url.searchParams.get("size")).toBe("512,512"); expect(url.searchParams.get("compressionQuality")).toBe("95");
    expect(url.searchParams.get("renderingRule")).toBe('{"rasterFunction":"NaturalColor"}');
    expect(url.searchParams.has("token")).toBe(false); expect(url.searchParams.has("key")).toBe(false);
    expect(MAP_SURROUND.role).toBe("satellite-aerial-imagery");
    expect(MAP_SURROUND.source.copyright).toMatch(/USDA.*USGS.*The National Map/);
    expect(MAP_SURROUND.source.licenseUrl).toBe("https://www.usgs.gov/tools/download-data-maps-national-map");
    expect(MAP_SURROUND.maxZoom).toBe(19);
    expect(MAP_SURROUND.source.backgroundOnly).toBe(true);
    expect(MAP_SURROUND.source.modifiesPlanningGeometry).toBe(false);
  });

  it.each([null, {}, { capabilities: "Image" }, { ...source, capabilities: "Catalog" },
    { ...source, maxImageWidth: 256 }, { ...source, maxImageHeight: 256 },
    { ...source, maxImageWidth: Infinity }, { ...source, maxImageHeight: NaN },
    { ...source, extent: { ...source.extent, spatialReference: { wkid: 4326 } } },
    { ...source, extent: { ...source.extent, xmin: NaN } },
    { ...source, extent: { ...source.extent, xmax: source.extent.xmin } },
    { ...source, rasterFunctionInfos: [] }, { ...source, bandCount: 1 },
  ])("rejects missing or misregistered natural-color imagery services: %j", payload => {
    expect(() => terrainTileTemplate(payload)).toThrow();
  });

  it("uses exact adjacent Mercator tile bounds without gaps or overlapping extents", () => {
    const here = terrainTileBounds(tile), east = terrainTileBounds({ ...tile, x: tile.x + 1 });
    const south = terrainTileBounds({ ...tile, y: tile.y + 1 });
    expect(here.max[0]).toBeCloseTo(east.min[0]!, 8);
    expect(here.min[1]).toBeCloseTo(south.max[1]!, 8);
    expect(here.max[0]! - here.min[0]!).toBeCloseTo(2 * Math.PI * 6378137 / 2 ** tile.z, 7);
  });

  it("registers north-up tile corners in the real city's ENU coordinate system", () => {
    const geometry = createTerrainTileGeometry(tile), extent = terrainTileBounds(tile);
    const positions = geometry.getAttribute("position"), uv = geometry.getAttribute("uv");
    expect(positions.count).toBe(81); expect(geometry.index!.count).toBe(384);
    for (const [index, mx, my, u, v] of [
      [0, extent.min[0], extent.max[1], 0, 1], [8, extent.max[0], extent.max[1], 1, 1],
      [72, extent.min[0], extent.min[1], 0, 0], [80, extent.max[0], extent.min[1], 1, 0],
    ] as const) {
      const latitude = (2 * Math.atan(Math.exp(my! / 6378137)) - Math.PI / 2) * 180 / Math.PI;
      const expected = lonLatToMapENU(mx! / 6378137 * 180 / Math.PI, latitude);
      expect(positions.getX(index)).toBeCloseTo(expected[0], 2);
      expect(-positions.getZ(index)).toBeCloseTo(expected[1], 2);
      expect(uv.getX(index)).toBe(u); expect(uv.getY(index)).toBe(v);
    }
    geometry.dispose();
  });

  it("uses flat opaque geometry with upward normals, without edge fade or invented elevation", () => {
    const geometry = createTerrainTileGeometry(tile), positions = geometry.getAttribute("position");
    expect(Object.keys(geometry.attributes).sort()).toEqual(["normal", "position", "uv"]);
    for (let index = 0; index < positions.count; index++) {
      expect(positions.getY(index)).toBe(0); expect(geometry.getAttribute("normal").getY(index)).toBe(1);
    }
    geometry.dispose();
  });

  it("requests a bounded current view rather than prefetching the surrounding city", () => {
    const original = JSON.stringify(bounds), view = camera();
    const matrix = view.matrixWorld.clone(), tiles = visibleTerrainTiles(view, 1200, 800, bounds);
    expect(tiles.length).toBeGreaterThan(0); expect(tiles.length).toBeLessThanOrEqual(36);
    expect(new Set(tiles.map(tile => tile.key)).size).toBe(tiles.length);
    expect(tiles.every(tile => tile.z >= MAP_SURROUND.minZoom && tile.z <= MAP_SURROUND.maxZoom)).toBe(true);
    expect(view.matrixWorld.equals(matrix)).toBe(true); expect(JSON.stringify(bounds)).toBe(original);
    const origin = mapENUToLonLat(1100, 1100);
    const zoom = tiles[0]!.z, count = 2 ** zoom;
    const x = Math.floor((origin[0] + 180) / 360 * count);
    const y = Math.floor((1 - Math.asinh(Math.tan(origin[1] * Math.PI / 180)) / Math.PI) / 2 * count);
    expect(tiles.some(tile => tile.x === x && tile.y === y)).toBe(true);
  });

  it("adapts resolution when zooming in while limiting very wide views", () => {
    const near = visibleTerrainTiles(camera(1000), 1200, 800, bounds);
    const far = visibleTerrainTiles(camera(24_000), 1200, 800, bounds);
    expect(near[0]!.z).toBeGreaterThan(far[0]!.z);
    const wide = visibleTerrainTiles(camera(100_000), 5000, 5000, bounds);
    expect(wide.length).toBeGreaterThan(0); expect(wide.length).toBeLessThanOrEqual(36);
    expect(wide.every(tile => tile.z <= MAP_SURROUND.maxZoom)).toBe(true);
  });

  it("can load real close-view detail beyond the old level-16 overview limit", () => {
    const tiles = visibleTerrainTiles(camera(75), 1200, 800, bounds);
    expect(tiles[0]!.z).toBe(19);
    expect(tiles.length).toBeLessThanOrEqual(MAP_SURROUND.maxVisibleTiles);
  });

  it("selects finer imagery for a Retina drawing buffer without increasing the request budget", () => {
    const view = camera(5000);
    const standard = visibleTerrainTiles(view, 500, 333, bounds);
    const retina = visibleTerrainTiles(view, 1000, 666, bounds);
    expect(retina[0]!.z).toBeGreaterThan(standard[0]!.z);
    expect(retina.length).toBeLessThanOrEqual(MAP_SURROUND.maxVisibleTiles);
  });

  it("does not spend the resolution budget on invisible corners of a rotated low-angle view", () => {
    const view = camera(6000); view.up.set(0, 1, 0);
    view.position.set(7100, 2300, 4900); view.lookAt(1100, 0, -1100); view.updateMatrixWorld(true);
    const tiles = visibleTerrainTiles(view, 2400, 1600, bounds);
    const rectangleCount = (Math.max(...tiles.map(tile => tile.x)) - Math.min(...tiles.map(tile => tile.x)) + 1)
      * (Math.max(...tiles.map(tile => tile.y)) - Math.min(...tiles.map(tile => tile.y)) + 1);
    expect(tiles.length).toBeLessThan(rectangleCount);
    expect(tiles.length).toBeLessThanOrEqual(MAP_SURROUND.maxVisibleTiles);
    const raycaster = new THREE.Raycaster(), point = new THREE.Vector3();
    for (const x of [-.9, 0, .9]) for (const y of [-.9, 0, .9]) {
      raycaster.setFromCamera(new THREE.Vector2(x, y), view);
      const distance = (-.001 - raycaster.ray.origin.y) / raycaster.ray.direction.y;
      raycaster.ray.at(distance, point);
      const [lon, lat] = mapENUToLonLat(point.x, -point.z), count = 2 ** tiles[0]!.z;
      const column = Math.floor((lon + 180) / 360 * count);
      const row = Math.floor((1 - Math.asinh(Math.tan(lat * Math.PI / 180)) / Math.PI) / 2 * count);
      expect(tiles.some(tile => tile.x === column && tile.y === row), `missing ground at ${x},${y}`).toBe(true);
    }
  });

  it("does not request invisible terrain for front, side, or underside views", () => {
    const front = camera(); front.up.set(0, 1, 0); front.position.set(1100, 200, 6000);
    front.lookAt(1100, 200, -1100); front.updateMatrixWorld(true);
    expect(visibleTerrainTiles(front, 1200, 800, bounds)).toEqual([]);
    const underneath = camera(); underneath.position.y = -8000; underneath.lookAt(1100, 0, -1100);
    expect(visibleTerrainTiles(underneath, 1200, 800, bounds)).toEqual([]);
  });
  it.each([[1200, 600], [390, 844], [844, 390]])("retains ground imagery in a perspective view with sky at %ix%i", (width, height) => {
    const view = new THREE.PerspectiveCamera(70, width / height, 0.15, 8000);
    view.position.set(1100, 90, -1100); view.rotation.set(-Math.PI / 12, -Math.PI / 2, 0, "YXZ");
    view.updateMatrixWorld(true);
    const top = new THREE.Raycaster(); top.setFromCamera(new THREE.Vector2(0, 1), view);
    expect(top.ray.direction.y).toBeGreaterThan(0); // Must not blank all ground because this ray sees sky.
    const tiles = visibleTerrainTiles(view, width, height, bounds);
    expect(tiles.length).toBeGreaterThan(0); expect(tiles.length).toBeLessThanOrEqual(MAP_SURROUND.maxVisibleTiles);
    expect(tiles.every(tile => Number.isFinite(tile.x) && Number.isFinite(tile.y))).toBe(true);
    expect(tiles.every(tile => tile.z <= MAP_SURROUND.maxZoom)).toBe(true);
    const previous = tiles.map(tile => tile.key);
    view.rotation.y += Math.PI;
    expect(visibleTerrainTiles(view, width, height, bounds).map(tile => tile.key)).not.toEqual(previous);
  });
  it("does not request ground outside a perspective camera's finite view", () => {
    const view = new THREE.PerspectiveCamera(50, 1, 0.15, 8000);
    view.position.set(1100, 90, -1100); view.lookAt(1100, 1000, -1100);
    expect(visibleTerrainTiles(view, 600, 600, bounds)).toEqual([]);
  });

  it("selects newly visible map areas when the camera pans", () => {
    const first = visibleTerrainTiles(camera(), 1200, 800, bounds).map(tile => tile.key);
    const second = visibleTerrainTiles(camera(4500, 9000), 1200, 800, bounds).map(tile => tile.key);
    expect(second).not.toEqual(first); expect(second.some(key => !first.includes(key))).toBe(true);
  });

  it("keeps physical bounds, offline roads and scenario data unchanged", () => {
    const host = new THREE.Group(), scenario = { id: "terrain-fixture", bounds,
      city: { id: MAP_SURROUND.cityId, name: "Midtown", sourceKind: "nyc-open-data" as const,
        collisionModel: "conservative-aabb" as const, sourceUrl: "https://data.cityofnewyork.us/",
        sourceSha256: "a".repeat(64) }, buildings: [] };
    const original = JSON.stringify(scenario), result = addCityContext(host, scenario);
    expect(result.min.toArray()).toEqual([bounds.min[0], 0, -bounds.max[1]!]);
    expect(result.max.toArray()).toEqual([bounds.max[0], 480, -bounds.min[1]!]);
    const layer = host.getObjectByName("city-map-surround") as TerrainMapLayer;
    expect(layer).toBeInstanceOf(TerrainMapLayer); expect(layer.userData.visualOnly).toBe(true);
    expect(layer.children).toHaveLength(0); // No hidden prefetch at scene construction.
    expect(host.getObjectByName("city-ground")!.position.y).toBe(-0.012);
    expect(host.getObjectByName("city-building")).toBeUndefined();
    expect(JSON.stringify(scenario)).toBe(original); disposeRenderObject(host);
  });

  it("does not attach this city-specific map to unrelated scenarios", () => {
    const host = new THREE.Group();
    expect(addMapSurround(host, "another-city", bounds)).toBeNull(); expect(host.children).toHaveLength(0);
    expect(() => updateMapSurround(host, camera(), 1200, 800)).not.toThrow();
  });

  it("renders crisp opaque tiles with normal scene depth and shadow reception", () => {
    const layer = new TerrainMapLayer(bounds); layer.update(camera(), 1200, 800);
    expect(layer.children.length).toBeGreaterThan(0);
    for (const child of layer.children as TileMesh[]) {
      expect(child.position.y).toBe(-0.001); expect(child.receiveShadow).toBe(true);
      expect(child.material.transparent).toBe(false); expect(child.material.opacity).toBe(1);
      expect(child.material.depthTest).toBe(true); expect(child.material.depthWrite).toBe(true);
      expect(child.material).toBeInstanceOf(THREE.MeshStandardMaterial);
      expect(child.material.map!.anisotropy).toBe(8); expect(child.material.map!.colorSpace).toBe(THREE.SRGBColorSpace);
      expect(child.material.map!.flipY).toBe(true); // Native north-up image, not ImageBitmap.
      expect(child.material.polygonOffset).toBe(true);
      expect(child.material.polygonOffsetFactor).toBeLessThan(-2); // Below buildings, above fallback roads.
      expect(child.material.polygonOffsetUnits).toBeLessThan(-2);
    }
    disposeRenderObject(layer);
  });

  it("loads only four tiles at once and does not duplicate requests on repeated renders", async () => {
    const { fetcher, requests } = mockTiles(), ready = vi.fn(), layer = new TerrainMapLayer(bounds, ready);
    try {
      const view = camera(); layer.update(view, 1200, 800);
      const total = layer.children.length;
      expect(fetcher).not.toHaveBeenCalled(); await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      expect(total).toBeGreaterThan(4); expect(requests).toHaveLength(4);
      layer.update(view, 1200, 800); expect(requests).toHaveLength(4);
      await requests[0]!.succeed(); expect(requests).toHaveLength(5); expect(ready).toHaveBeenCalledOnce();
      const mesh = layer.children[0]!;
      expect(mesh.visible).toBe(true); expect(layer.userData.basemapReady).toBe(true);
      for (let index = 1; index < total; index++) await requests[index]!.succeed();
      expect(requests).toHaveLength(total); expect(ready).toHaveBeenCalledTimes(total);
      expect(layer.children.every(child => child.visible)).toBe(true);
    } finally { disposeRenderObject(layer); }
  });

  it.each([1, 4, 16, 32, NaN])("respects the GPU's anisotropy limit for oblique imagery: %s", capability => {
    const layer = new TerrainMapLayer(bounds);
    layer.update(camera(), 1200, 800, capability);
    for (const mesh of layer.children as TileMesh[]) {
      const expected = Number.isFinite(capability) ? Math.min(capability, MAP_SURROUND.maxAnisotropy) : 1;
      expect(mesh.material.map!.anisotropy).toBe(expected);
    }
    disposeRenderObject(layer);
  });

  it("releases stale tiles and queued work when the visible area changes", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      layer.update(camera(), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      const old = layer.children.map(child => (child as TileMesh).material.map!);
      layer.update(camera(4500, 12_000), 1200, 800); await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      expect(old.every(texture => texture.userData.viewerDisposed)).toBe(true);
      expect(layer.children.length).toBeLessThanOrEqual(36);
      expect(requests.slice(0, 4).every(request => request.signal.aborted)).toBe(true);
      for (const request of requests.slice(0, 4)) await request.succeed();
      expect(layer.children.every(child => !child.visible)).toBe(true);
      expect(layer.userData.tileCount).toBe(layer.children.length);
    } finally { disposeRenderObject(layer); }
  });

  it("ignores late image completions and never starts queued work after scene disposal", async () => {
    const { requests } = mockTiles(), ready = vi.fn(), layer = new TerrainMapLayer(bounds, ready);
    layer.update(camera(), 1200, 800);
    await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
    const textures = layer.children.map(child => (child as TileMesh).material.map!);
    const disposals = textures.map(texture => vi.spyOn(texture, "dispose"));
    disposeRenderObject(layer); await Promise.resolve();
    for (const request of requests) { await request.succeed(); await request.fail(); }
    expect(requests).toHaveLength(4); expect(ready).not.toHaveBeenCalled();
    expect(requests.every(request => request.signal.aborted)).toBe(true);
    expect(layer.children.every(child => !child.visible)).toBe(true);
    disposeRenderObject(layer);
    for (const disposal of disposals) expect(disposal).toHaveBeenCalledOnce();
  });

  it("leaves unavailable tiles hidden and preserves the scene's offline fallback", async () => {
    const { requests } = mockTiles(), ready = vi.fn(), layer = new TerrainMapLayer(bounds, ready);
    try {
      layer.update(camera(), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      const total = layer.children.length;
      for (let index = 0; index < total; index++) await requests[index]!.fail();
      expect(layer.children.every(child => !child.visible)).toBe(true);
      expect(ready).toHaveBeenCalledTimes(total); expect(layer.userData.basemapReady).toBeUndefined();
      await requests[0]!.succeed(); expect(layer.children.every(child => !child.visible)).toBe(true);
    } finally { disposeRenderObject(layer); }
  });

  it("bounds stalled requests with timeouts and ignores later responses", async () => {
    const { requests } = mockTiles(), ready = vi.fn(), layer = new TerrainMapLayer(bounds, ready);
    try {
      layer.update(camera(), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      const total = layer.children.length;
      await vi.advanceTimersByTimeAsync(20_001 * Math.ceil(total / 4));
      expect(ready).toHaveBeenCalledTimes(total); expect(vi.getTimerCount()).toBe(0);
      expect(requests.every(request => request.signal.aborted)).toBe(true);
      for (const request of requests) await request.succeed();
      expect(layer.children.every(child => !child.visible)).toBe(true);
    } finally { disposeRenderObject(layer); }
  });

  it("reuses decoded tiles when rotating/panning back instead of requesting and painting them again", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      const originalView = camera(); layer.update(originalView, 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      for (let index = 0; index < requests.length; index++) await requests[index]!.succeed();
      const firstMeshes = [...layer.children] as TileMesh[];
      const firstTextures = firstMeshes.map(mesh => mesh.material.map!);
      layer.update(camera(4500, 9000), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      const networkCount = requests.length;
      expect(firstTextures.every(texture => !texture.userData.viewerDisposed)).toBe(true);
      const decodes = vi.mocked(decodeImageryTile).mock.calls.length;
      layer.update(originalView, 1200, 800);
      expect(firstMeshes.every(mesh => mesh.visible)).toBe(true);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      expect(requests).toHaveLength(networkCount);
      expect(vi.mocked(decodeImageryTile).mock.calls.length).toBe(decodes);
    } finally { disposeRenderObject(layer); }
  });

  it("keeps the previous LOD visible until the whole replacement is ready and swaps back from cache", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      const coarse = camera(24_000); layer.update(coarse, 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      for (let index = 0; index < requests.length; index++) await requests[index]!.succeed();
      const oldZoom = layer.userData.displayedZoom, before = requests.length;
      layer.update(camera(1000), 1200, 800);
      expect(layer.userData.wantedZoom).toBeGreaterThan(oldZoom);
      expect(layer.children.some(mesh => mesh.visible)).toBe(true);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      const replacementCount = layer.userData.tileCount;
      for (let index = before; index < before + replacementCount - 1; index++) {
        await requests[index]!.succeed();
        expect(layer.userData.displayedZoom).toBe(oldZoom);
        expect(layer.children.filter(mesh => mesh.visible).every(mesh => mesh.name.startsWith(`terrain-tile-${oldZoom}/`))).toBe(true);
      }
      await requests[before + replacementCount - 1]!.succeed();
      expect(layer.userData.displayedZoom).toBe(layer.userData.wantedZoom);
      expect(layer.children.filter(mesh => mesh.visible)).toHaveLength(replacementCount);
      const after = requests.length;
      layer.update(coarse, 1200, 800);
      expect(layer.userData.displayedZoom).toBe(oldZoom);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1); expect(requests).toHaveLength(after);
    } finally { disposeRenderObject(layer); }
  });

  it("bounds the resident cache and evicts only unused older textures", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      let cursor = 0;
      for (const x of [0, 7000, 14_000, 21_000, -7000, -14_000, -21_000, 0]) {
        layer.update(camera(6500, x), 1200, 800);
        await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
        while (cursor < requests.length) await requests[cursor++]!.succeed();
        expect(layer.children.length).toBeLessThanOrEqual(MAP_SURROUND.maxResidentTiles);
        expect(layer.userData.tileCount).toBeLessThanOrEqual(MAP_SURROUND.maxVisibleTiles);
        expect(layer.children.filter(mesh => mesh.visible)).toHaveLength(layer.userData.tileCount);
      }
    } finally { disposeRenderObject(layer); }
  });

  it("applies hysteresis near a resolution boundary instead of bouncing between LODs", () => {
    const view = camera(5000);
    expect(visibleTerrainTiles(view, 690, 460, bounds)[0]!.z).toBe(13);
    expect(visibleTerrainTiles(view, 720, 480, bounds)[0]!.z).toBe(14);
    expect(visibleTerrainTiles(view, 720, 480, bounds, 13)[0]!.z).toBe(13);
  });

  it("commits at most one decoded image per frame and cancels queued uploads on scene disposal", async () => {
    const { requests } = mockTiles(), ready = vi.fn(), layer = new TerrainMapLayer(bounds, ready);
    const frames = new Map<number, FrameRequestCallback>(); let sequence = 0;
    vi.stubGlobal("requestAnimationFrame", (callback: FrameRequestCallback) => {
      frames.set(++sequence, callback); return sequence;
    });
    vi.stubGlobal("cancelAnimationFrame", (id: number) => frames.delete(id));
    try {
      layer.update(camera(), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      for (const request of requests.slice(0, 4)) await request.succeed();
      expect(ready).not.toHaveBeenCalled(); expect(frames.size).toBe(1);
      const [id, callback] = [...frames.entries()][0]!; frames.delete(id); callback(0);
      expect(ready).toHaveBeenCalledOnce(); expect(frames.size).toBe(1);
      disposeRenderObject(layer); await settle();
      expect(frames.size).toBe(0);
      for (const request of requests) await request.succeed();
      expect(ready).toHaveBeenCalledOnce(); expect(frames.size).toBe(0);
    } finally { disposeRenderObject(layer); }
  });
});
