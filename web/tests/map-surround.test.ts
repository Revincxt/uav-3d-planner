import * as THREE from "three";
import { afterEach, describe, expect, it, vi } from "vitest";
import { MAP_SURROUND } from "../src/map-surround-data";
import { lonLatToMapENU, mapENUToLonLat } from "../src/map-basemap";
import { addMapSurround, createTerrainTileGeometry, TerrainMapLayer, terrainTileBounds,
  terrainPreviewURL, terrainTileURL, updateMapSurround, visibleTerrainTiles } from "../src/map-surround";
import { addCityContext } from "../src/city-scene";
import { disposeRenderObject } from "../src/render-resources";
import { decodeImageryTile } from "../src/map-imagery";
import { positionWesternOverview, mapFramingPadding } from '../src/map-navigation';
import { fitMapCamera } from '../src/viewer-geometry';

vi.mock("../src/map-imagery", () => ({ decodeImageryTile: vi.fn(async () => ({} as HTMLImageElement)) }));

const bounds = { min: [-240.46513, -186.36859, 0], max: [2542.36633, 2496.13957, 480] };
const tile = { z: 15, x: 9648, y: 12315, key: "15/9648/12315" };
const template = MAP_SURROUND.source.tileTemplate;
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
  const requests: { url: string; signal: AbortSignal; succeed: () => Promise<void>; fail: (status?: number) => Promise<void> }[] = [];
  const fetcher = vi.fn((url: string, options: RequestInit) => {
    expect(options.credentials).toBe("omit");
    if (url === MAP_SURROUND.source.url) return Promise.reject(new Error("Metadata is deliberately unavailable"));
    return new Promise((resolve, reject) => {
      const signal = options.signal as AbortSignal;
      if (signal.aborted) { reject(new DOMException("Cancelled", "AbortError")); return; }
      signal.addEventListener("abort", () => reject(new DOMException("Cancelled", "AbortError")), { once: true });
      requests.push({ url, signal: options.signal as AbortSignal,
        succeed: async () => { resolve({ ok: true, blob: async () => new Blob(["fixture"], { type: "image/jpeg" }) }); await settle(); },
        fail: async (status?: number) => {
          if (status === undefined) reject(new TypeError("offline"));
          else resolve({ ok: false, status });
          await settle();
        } });
    });
  });
  vi.stubGlobal("fetch", fetcher);
  return { fetcher, requests };
}

afterEach(() => { vi.restoreAllMocks(); vi.unstubAllGlobals(); vi.useRealTimers(); });

describe("continuous satellite-style imagery map", () => {
  it("retains a stationary camera's tile decision but invalidates pan, projection and Retina changes", () => {
    const view = camera(), layer = new TerrainMapLayer(bounds);
    const project = vi.spyOn(THREE.Vector3.prototype, "unproject");
    layer.update(view, 1200, 800);
    expect(project).toHaveBeenCalled(); project.mockClear();
    for (let frame = 0; frame < 120; frame++) layer.update(view, 1200, 800);
    expect(project).not.toHaveBeenCalled();
    view.position.x += 20; layer.update(view, 1200, 800);
    expect(project).toHaveBeenCalled(); project.mockClear();
    view.zoom = 2; view.updateProjectionMatrix(); layer.update(view, 1200, 800);
    expect(project).toHaveBeenCalled(); project.mockClear();
    layer.update(view, 2400, 1600); expect(project).not.toHaveBeenCalled(); project.mockClear();
    layer.update(view.clone(), 2400, 1600); expect(project).toHaveBeenCalled();
    disposeRenderObject(layer);
  });
  it("loads actual high-resolution public imagery without a key, with complete attribution", () => {
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

  it.each(["https://example.com/exportImage", `${template}?token=untrusted`,
    "http://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPPlus/ImageServer/exportImage",
  ])("never accepts arbitrary or authenticated tile providers: %s", url => {
    expect(() => terrainTileURL(tile, url)).toThrow(/Unsupported/);
  });

  it("registers the official cached overview and RGB high-resolution fallback without credentials", () => {
    expect(terrainPreviewURL(tile)).toBe("https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/tile/15/12315/9648");
    expect(() => terrainPreviewURL({ ...tile, z: 17 })).toThrow(/Uncached/);
    const fallback = new URL(terrainTileURL(tile, MAP_SURROUND.source.fallbackTileTemplate));
    expect(fallback.hostname).toBe("apps.geo.fpac.usda.gov");
    expect(fallback.searchParams.get("bandIds")).toBe("0,1,2");
    expect(fallback.searchParams.get("bboxSR")).toBe("3857");
    expect(fallback.searchParams.get("imageSR")).toBe("3857");
    expect(fallback.searchParams.has("renderingRule")).toBe(false);
    expect(fallback.searchParams.has("token")).toBe(false);
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

  it("crops a north-up parent image correctly into all four child ground footprints", () => {
    for (const dx of [0, 1]) for (const dy of [0, 1]) {
      const child = { z: tile.z + 1, x: tile.x * 2 + dx, y: tile.y * 2 + dy, key: `child-${dx}-${dy}` };
      const geometry = createTerrainTileGeometry(child, tile), own = createTerrainTileGeometry(child);
      expect(Array.from(geometry.getAttribute("position").array)).toEqual(Array.from(own.getAttribute("position").array));
      const uv = geometry.getAttribute("uv");
      expect(uv.getX(0)).toBe(dx / 2); expect(uv.getX(8)).toBe((dx + 1) / 2);
      expect(uv.getY(0)).toBe(1 - dy / 2); expect(uv.getY(72)).toBe((1 - dy) / 2);
      geometry.dispose(); own.dispose();
    }
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

  it('covers the expanded island panorama corners at the retained low western angle', () => {
    const island = {min:[-1214.11806,-4640.21941,0],max:[3294.2203,3056.08779,520]};
    const box = new THREE.Box3(new THREE.Vector3(island.min[0]!,0,-island.max[1]!),
      new THREE.Vector3(island.max[0]!,520,-island.min[1]!));
    const view=new THREE.OrthographicCamera();
    positionWesternOverview(view,box.getCenter(new THREE.Vector3()),7696.3072);
    view.lookAt(box.getCenter(new THREE.Vector3()));
    fitMapCamera(view,box,1920,1080,mapFramingPadding(true));
    const tiles=visibleTerrainTiles(view,1920,1080,island), count=2**tiles[0]!.z;
    expect(tiles.length).toBeLessThanOrEqual(MAP_SURROUND.maxVisibleTiles);
    const raycaster=new THREE.Raycaster();
    for(const x of [-.95,0,.95])for(const y of [-.95,0,.95]) {
      raycaster.setFromCamera(new THREE.Vector2(x,y),view);
      const point=raycaster.ray.at((-.001-raycaster.ray.origin.y)/raycaster.ray.direction.y,new THREE.Vector3());
      const depth=point.clone().project(view).z;
      expect(depth,`clipped island ground at ${x},${y}`).toBeGreaterThanOrEqual(-1);
      expect(depth,`clipped island ground at ${x},${y}`).toBeLessThanOrEqual(1);
      const [lon,lat]=mapENUToLonLat(point.x,-point.z);
      const column=Math.floor((lon+180)/360*count),row=Math.floor((1-Math.asinh(Math.tan(lat*Math.PI/180))/Math.PI)/2*count);
      expect(tiles.some(tile=>tile.x===column&&tile.y===row),`missing island ground at ${x},${y}`).toBe(true);
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

  it("reprioritizes overlapping queued tiles around the latest view center", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      layer.update(camera(), 1200, 800);
      const view = camera(4500, 2600), zoom = layer.userData.wantedZoom;
      const expected = visibleTerrainTiles(view, 1200, 800, bounds, zoom);
      layer.update(view, 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      expect(requests.map(request => request.url)).toEqual(expected.slice(0, 4).map(tile => terrainTileURL(tile, template)));
    } finally { disposeRenderObject(layer); }
  });

  it("starts imagery directly even when the nonessential metadata endpoint is unavailable", async () => {
    const { fetcher, requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      layer.update(camera(), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      expect(fetcher.mock.calls.some(([url]) => url === MAP_SURROUND.source.url)).toBe(false);
      expect(requests).toHaveLength(4);
    } finally { disposeRenderObject(layer); }
  });

  it("cannot starve requests by repeatedly changing the camera during follow", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      for (let index = 0; index < 20; index++) {
        layer.update(camera(1000, index * 200), 1200, 800);
        await vi.advanceTimersByTimeAsync(16);
      }
      expect(requests.length).toBeGreaterThan(0);
    } finally { disposeRenderObject(layer); }
  });

  it("cancels queued retries and reconnect listeners when its scene is disposed", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    const events = new EventTarget(); vi.stubGlobal("window", events);
    const remove = vi.spyOn(events, "removeEventListener");
    layer.update(camera(), 1200, 800);
    await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
    await requests[0]!.fail(503);
    const count = requests.length;
    disposeRenderObject(layer);
    events.dispatchEvent(new Event("online"));
    await vi.advanceTimersByTimeAsync(MAP_SURROUND.retryDelayMs + 1);
    expect(requests).toHaveLength(count); expect(vi.getTimerCount()).toBe(0);
    expect(remove).toHaveBeenCalledWith("online", expect.any(Function));
  });

  it("keeps the cached map visible while the native detail service is slow or unavailable", async () => {
    const { requests } = mockTiles(), host = new THREE.Group();
    addMapSurround(host, MAP_SURROUND.cityId, bounds);
    try {
      updateMapSurround(host, camera(), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      expect(requests.slice(0, 4).every(request => request.url.includes("USGSImageryOnly/MapServer/tile/"))).toBe(true);
      for (let index = 0; index < requests.length; index++) {
        if (requests[index]!.url.includes("USGSImageryOnly")) await requests[index]!.succeed();
      }
      const preview = host.getObjectByName("city-map-preview")!;
      expect(preview.children.some(child => child.visible)).toBe(true);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.requestTimeoutMs + 1);
      expect(preview.children.some(child => child.visible)).toBe(true);
      const pending = requests.filter(request => !request.signal.aborted && !request.url.includes("USGSImageryOnly"));
      expect(pending.length).toBeLessThanOrEqual(MAP_SURROUND.maxConcurrentRequests);
    } finally { disposeRenderObject(host); }
  });

  it("retries a transient visible-tile failure once, then completes the same view", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      layer.update(camera(), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      const total = layer.userData.tileCount;
      await requests[0]!.fail(503);
      for (let index = 1; index < total; index++) await requests[index]!.succeed();
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.retryDelayMs);
      expect(requests).toHaveLength(total + 1);
      expect(requests.at(-1)!.url).toBe(terrainTileURL(visibleTerrainTiles(camera(), 1200, 800, bounds)[0]!, MAP_SURROUND.source.fallbackTileTemplate));
      await requests.at(-1)!.succeed();
      expect(layer.children.every(child => child.visible)).toBe(true);
      expect(layer.userData.displayedZoom).toBe(layer.userData.wantedZoom);
    } finally { disposeRenderObject(layer); }
  });

  it("does not endlessly retry a tile that stays unavailable", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      layer.update(camera(), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      const total = layer.userData.tileCount;
      await requests[0]!.fail(503);
      for (let index = 1; index < total; index++) await requests[index]!.succeed();
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.retryDelayMs);
      await requests.at(-1)!.fail(503);
      await vi.advanceTimersByTimeAsync(5000);
      expect(requests).toHaveLength(total + 1); expect(vi.getTimerCount()).toBe(0);
    } finally { disposeRenderObject(layer); }
  });

  it("recovers failed visible imagery on reconnect without duplicating live requests", async () => {
    const { requests } = mockTiles(), events = new EventTarget(); vi.stubGlobal("window", events);
    const layer = new TerrainMapLayer(bounds);
    try {
      layer.update(camera(), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      const count = layer.userData.tileCount;
      for (let index = 0; index < count; index++) await requests[index]!.fail(404);
      expect(vi.getTimerCount()).toBe(0);
      events.dispatchEvent(new Event("online"));
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      expect(requests).toHaveLength(count + 4);
      events.dispatchEvent(new Event("online"));
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      expect(requests).toHaveLength(count + 4);
    } finally { disposeRenderObject(layer); }
  });

  it("shares ground projection between overview and detail even on Retina buffers", () => {
    const view = camera(), project = vi.spyOn(THREE.Vector3.prototype, "unproject");
    visibleTerrainTiles(view, 1200, 800, bounds);
    expect(project).toHaveBeenCalled(); project.mockClear();
    visibleTerrainTiles(view, 1200, 800, bounds, undefined, MAP_SURROUND.preview.maxZoom);
    visibleTerrainTiles(view, 2400, 1600, bounds);
    expect(project).not.toHaveBeenCalled();
    view.position.x += 10; view.updateMatrixWorld(true);
    visibleTerrainTiles(view, 1200, 800, bounds); expect(project).toHaveBeenCalled();
  });

  it("retains an overlapping in-flight parent when zooming instead of continually restarting it", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      layer.update(camera(24_000), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      layer.update(camera(1000), 1200, 800);
      const relevant = requests.filter(request => !request.signal.aborted);
      expect(relevant.length).toBeGreaterThan(0);
      for (const request of relevant) await request.succeed();
      expect(layer.children.some(mesh => mesh.visible)).toBe(true);
    } finally { disposeRenderObject(layer); }
  });

  it.each([403, 404])("does not retry permanent HTTP %s imagery failures", async status => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      layer.update(camera(), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      const total = layer.userData.tileCount;
      for (let index = 0; index < total; index++) await requests[index]!.fail(status);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.retryDelayMs + 1);
      expect(requests).toHaveLength(total); expect(vi.getTimerCount()).toBe(0);
      expect(layer.children.every(child => !child.visible)).toBe(true);
    } finally { disposeRenderObject(layer); }
  });

  it("cancels a failed tile's queued retry after panning away", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      layer.update(camera(), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      await requests[0]!.fail(503);
      const abandoned = requests[0]!.url;
      layer.update(camera(4500, 12_000), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.retryDelayMs + MAP_SURROUND.updateDelayMs + 1);
      expect(requests.filter(request => request.url === abandoned)).toHaveLength(1);
    } finally { disposeRenderObject(layer); }
  });

  it("does not refetch malformed image bodies", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    vi.mocked(decodeImageryTile).mockRejectedValueOnce(new Error("invalid image dimensions"));
    try {
      layer.update(camera(), 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      await requests[0]!.succeed();
      const failed = requests[0]!.url;
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.retryDelayMs + 1);
      expect(requests.filter(request => request.url === failed)).toHaveLength(1);
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
      await vi.advanceTimersByTimeAsync((MAP_SURROUND.requestTimeoutMs + MAP_SURROUND.fallbackTimeoutMs + MAP_SURROUND.retryDelayMs + 1) * total);
      expect(requests.length).toBeLessThanOrEqual(total * 2);
      expect(ready).toHaveBeenCalledTimes(requests.length); expect(vi.getTimerCount()).toBe(0);
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

  it("crops the previous imagery into child footprints and sharpens tiles independently without holes", async () => {
    const { requests } = mockTiles(), layer = new TerrainMapLayer(bounds);
    try {
      const coarse = camera(24_000); layer.update(coarse, 1200, 800);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      for (let index = 0; index < requests.length; index++) await requests[index]!.succeed();
      const oldZoom = layer.userData.displayedZoom, before = requests.length;
      const parentSources = (layer.children as TileMesh[]).map(mesh => mesh.material.map!.source);
      const parentImages = parentSources.map(source => source.data);
      layer.update(camera(1000), 1200, 800);
      expect(layer.userData.wantedZoom).toBeGreaterThan(oldZoom);
      expect(layer.children.some(mesh => mesh.visible)).toBe(true);
      await vi.advanceTimersByTimeAsync(MAP_SURROUND.updateDelayMs + 1);
      const replacementCount = layer.userData.tileCount;
      expect(layer.children.filter(mesh => mesh.visible).every(mesh => parentSources.includes((mesh as TileMesh).material.map!.source))).toBe(true);
      const expectedZoom = layer.userData.wantedZoom;
      expect(expectedZoom).toBeGreaterThan(oldZoom);
      for (let index = before; index < before + replacementCount - 1; index++) {
        await requests[index]!.succeed();
        expect(layer.userData.displayedZoom).toBe(expectedZoom);
        expect(layer.children.filter(mesh => mesh.visible).every(mesh => mesh.name.startsWith(`terrain-tile-${expectedZoom}/`))).toBe(true);
        expect(layer.children.filter(mesh => mesh.visible)).toHaveLength(replacementCount);
      }
      await requests[before + replacementCount - 1]!.succeed();
      expect(layer.userData.displayedZoom).toBe(layer.userData.wantedZoom);
      expect(layer.children.filter(mesh => mesh.visible)).toHaveLength(replacementCount);
      expect(parentSources.map(source => source.data)).toEqual(parentImages);
      expect(layer.children.filter(mesh => mesh.visible).every(mesh => !parentSources.includes((mesh as TileMesh).material.map!.source))).toBe(true);
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
