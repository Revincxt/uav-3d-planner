import * as THREE from "three";
import { lonLatToMapENU, mapENUToLonLat } from "./map-basemap";
import { MAP_SURROUND } from "./map-surround-data";
import { decodeImageryTile } from "./map-imagery";
import { disposeRenderObject } from "./render-resources";
import type { MapBounds } from "./map-background";

const RADIUS = 6378137, HALF_WORLD = Math.PI * RADIUS;
const MAX_CONTEXT_DISTANCE_M = 25_000;

export interface TerrainTile { z: number; x: number; y: number; key: string }

interface ImageryMetadata {
  extent: {
    xmin: number; ymin: number; xmax: number; ymax: number;
    spatialReference: { wkid: number; latestWkid?: number };
  };
  bandCount: number;
  capabilities: string;
  maxImageWidth: number; maxImageHeight: number;
  rasterFunctionInfos: { name: string }[];
}

export function terrainTileTemplate(payload: unknown): string {
  if (!payload || typeof payload !== "object") throw new Error("Invalid imagery metadata");
  const source = payload as Partial<ImageryMetadata>, extent = source.extent;
  const projection = extent?.spatialReference?.latestWkid ?? extent?.spatialReference?.wkid;
  if (!extent
    || ![3857, 102100, 102113].includes(projection!)
    || ![extent.xmin, extent.ymin, extent.xmax, extent.ymax].every(value => typeof value === "number" && Number.isFinite(value))
    || extent.xmin >= extent.xmax || extent.ymin >= extent.ymax
    || typeof source.capabilities !== "string" || !source.capabilities.split(",").includes("Image")
    || !Number.isInteger(source.bandCount) || source.bandCount! < 3
    || !Number.isInteger(source.maxImageWidth) || source.maxImageWidth! < MAP_SURROUND.tileSize
    || !Number.isInteger(source.maxImageHeight) || source.maxImageHeight! < MAP_SURROUND.tileSize
    || !Array.isArray(source.rasterFunctionInfos) || !source.rasterFunctionInfos.some(rule => rule?.name === "NaturalColor")) {
    throw new Error("Unsupported public natural-color imagery service");
  }
  // The response cannot redirect requests to an arbitrary host or API-key service.
  return MAP_SURROUND.source.tileTemplate;
}

export function terrainTileURL(tile: TerrainTile, template: string): string {
  const extent = terrainTileBounds(tile), url = new URL(template);
  // Request source pixels for the exact same ENU-registered ground footprint.
  // Calling /tile/17+ on the old cached service only returned errors, not finer imagery.
  for (const [key, value] of Object.entries({
    bbox: [...extent.min, ...extent.max].join(","), bboxSR: "3857", imageSR: "3857",
    size: `${MAP_SURROUND.tileSize},${MAP_SURROUND.tileSize}`, format: "jpg", compressionQuality: "95",
    interpolation: "RSP_BilinearInterpolation", renderingRule: '{"rasterFunction":"NaturalColor"}', f: "image",
  })) url.searchParams.set(key, value);
  return url.toString();
}

export function terrainTileBounds(tile: TerrainTile) {
  const size = 2 * HALF_WORLD / 2 ** tile.z;
  return { min: [-HALF_WORLD + tile.x * size, HALF_WORLD - (tile.y + 1) * size],
    max: [-HALF_WORLD + (tile.x + 1) * size, HALF_WORLD - tile.y * size] };
}

/** True Web Mercator tile geometry, not a stretched photograph or invented terrain height. */
export function createTerrainTileGeometry(tile: TerrainTile): THREE.BufferGeometry {
  const extent = terrainTileBounds(tile), segments = 8;
  const positions: number[] = [], uv: number[] = [], indices: number[] = [];
  for (let row = 0; row <= segments; row++) {
    const mercatorY = extent.max[1]! + (extent.min[1]! - extent.max[1]!) * row / segments;
    const latitude = (2 * Math.atan(Math.exp(mercatorY / RADIUS)) - Math.PI / 2) * 180 / Math.PI;
    for (let column = 0; column <= segments; column++) {
      const mercatorX = extent.min[0]! + (extent.max[0]! - extent.min[0]!) * column / segments;
      const [east, north] = lonLatToMapENU(mercatorX / RADIUS * 180 / Math.PI, latitude);
      positions.push(east, 0, -north);
      uv.push(column / segments, 1 - row / segments);
      if (row < segments && column < segments) {
        const a = row * (segments + 1) + column, b = a + 1, c = a + segments + 1, d = c + 1;
        indices.push(a, c, b, b, c, d);
      }
    }
  }
  const geometry = new THREE.BufferGeometry().setIndex(indices)
    .setAttribute("position", new THREE.Float32BufferAttribute(positions, 3))
    .setAttribute("uv", new THREE.Float32BufferAttribute(uv, 2));
  geometry.computeVertexNormals();
  return geometry;
}

function tilePosition(longitude: number, latitude: number, zoom: number): [number, number] {
  const latitudeRad = THREE.MathUtils.clamp(latitude, -85, 85) * Math.PI / 180;
  const count = 2 ** zoom;
  return [(longitude + 180) / 360 * count,
    (1 - Math.asinh(Math.tan(latitudeRad)) / Math.PI) / 2 * count];
}

/** Ground polygon of a finite perspective frustum, including views whose upper edge is sky. */
function perspectiveGroundHits(camera: THREE.PerspectiveCamera, floor: number): THREE.Vector3[] {
  const corners = [
    [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
    [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1],
  ].map(point => new THREE.Vector3(point[0], point[1], point[2]).unproject(camera));
  const edges = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]];
  const hits: THREE.Vector3[] = [];
  for (const [a, b] of edges) {
    const left = corners[a!]!, right = corners[b!]!;
    if ((left.y - floor) * (right.y - floor) > 0 || Math.abs(right.y - left.y) < 1e-10) continue;
    hits.push(left.clone().lerp(right, (floor - left.y) / (right.y - left.y)));
  }
  return hits;
}

function tileIntersectsView(tile: TerrainTile, frustum: THREE.Frustum, floor: number): boolean {
  const extent = terrainTileBounds(tile), box = new THREE.Box3();
  for (const x of [extent.min[0]!, extent.max[0]!]) for (const y of [extent.min[1]!, extent.max[1]!]) {
    const latitude = (2 * Math.atan(Math.exp(y / RADIUS)) - Math.PI / 2) * 180 / Math.PI;
    const [east, north] = lonLatToMapENU(x / RADIUS * 180 / Math.PI, latitude);
    box.expandByPoint(new THREE.Vector3(east, floor, -north));
  }
  // Conservative planar bounds retain seam tiles despite projection/float roundoff.
  const span = extent.max[0]! - extent.min[0]!;
  box.expandByScalar(Math.max(.02, span * span / RADIUS));
  return frustum.intersectsBox(box);
}

/** Only tiles intersecting the current ground view, with bounded resolution and request count. */
export function visibleTerrainTiles(
  camera: THREE.Camera, width: number, height: number, bounds: MapBounds, preferredZoom?: number,
): TerrainTile[] {
  camera.updateMatrixWorld(true);
  const floor = (bounds.min[2] ?? 0) - 0.001;
  const hits = camera instanceof THREE.PerspectiveCamera ? perspectiveGroundHits(camera, floor) : [];
  if (!(camera instanceof THREE.PerspectiveCamera)) {
    const raycaster = new THREE.Raycaster();
    for (const [x, y] of [[-1, -1], [-1, 1], [1, -1], [1, 1]]) {
      raycaster.setFromCamera(new THREE.Vector2(x!, y!), camera);
      const ray = raycaster.ray;
      if (ray.direction.y >= -1e-5) return [];
      const distance = (floor - ray.origin.y) / ray.direction.y;
      // Orthographic rays may start below the plane when zooming out at a low elevation.
      // Keep the rest of the visible ground instead of blanking the entire map.
      hits.push(ray.at(distance, new THREE.Vector3()));
    }
  }
  if (!hits.length) return [];
  const centerX = (bounds.min[0]! + bounds.max[0]!) / 2;
  const centerNorth = (bounds.min[1]! + bounds.max[1]!) / 2;
  const west = Math.max(centerX - MAX_CONTEXT_DISTANCE_M, Math.min(...hits.map(p => p.x)));
  const east = Math.min(centerX + MAX_CONTEXT_DISTANCE_M, Math.max(...hits.map(p => p.x)));
  const south = Math.max(centerNorth - MAX_CONTEXT_DISTANCE_M, Math.min(...hits.map(p => -p.z)));
  const north = Math.min(centerNorth + MAX_CONTEXT_DISTANCE_M, Math.max(...hits.map(p => -p.z)));
  if (east <= west || north <= south) return [];
  const geographic = [[west, south], [west, north], [east, south], [east, north]]
    .map(([x, y]) => mapENUToLonLat(x!, y!));
  const metersPerPixel = camera instanceof THREE.OrthographicCamera
    ? Math.max((camera.right - camera.left) / camera.zoom / Math.max(1, width),
      (camera.top - camera.bottom) / camera.zoom / Math.max(1, height))
    : Math.sqrt((east - west) * (north - south) / Math.max(1, width * height));
  const resolutionZoom = Math.log2(2 * HALF_WORLD * Math.cos(40.75 * Math.PI / 180)
    / (MAP_SURROUND.tileSize * metersPerPixel));
  // Round up rather than enlarging a coarser image when between resolution levels.
  let zoom = THREE.MathUtils.clamp(Math.ceil(resolutionZoom), MAP_SURROUND.minZoom, MAP_SURROUND.maxZoom);
  if (preferredZoom !== undefined && preferredZoom >= MAP_SURROUND.minZoom && preferredZoom <= MAP_SURROUND.maxZoom
    && Math.abs(resolutionZoom - preferredZoom) < 0.8) zoom = preferredZoom;
  const frustum = new THREE.Frustum().setFromProjectionMatrix(
    new THREE.Matrix4().multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse));
  let limits: number[] = [];
  let tiles: TerrainTile[] = [];
  do {
    const points = geographic.map(p => tilePosition(p[0], p[1], zoom));
    limits = [Math.floor(Math.min(...points.map(p => p[0]))), Math.floor(Math.min(...points.map(p => p[1]))),
      Math.floor(Math.max(...points.map(p => p[0]))), Math.floor(Math.max(...points.map(p => p[1])))];
    const count = (limits[2]! - limits[0]! + 1) * (limits[3]! - limits[1]! + 1);
    // Reserve headroom when upgrading: rotating around a tile boundary must not bounce
    // between a 36-tile fine view and a coarse one on successive frames.
    const budget = preferredZoom !== undefined && zoom > preferredZoom
      ? Math.floor(MAP_SURROUND.maxVisibleTiles * 0.75) : MAP_SURROUND.maxVisibleTiles;
    // A rotated low-angle ground view has a much larger geographic bounding box.
    // Count only intersecting tiles; otherwise invisible corners force the whole
    // map down to blurry overview imagery. Bound this work before enumerating.
    if (count <= MAP_SURROUND.maxVisibleTiles * 4) {
      tiles = [];
      for (let y = limits[1]!; y <= limits[3]!; y++) for (let x = limits[0]!; x <= limits[2]!; x++) {
        const tile = { z: zoom, x, y, key: `${zoom}/${x}/${y}` };
        if (tileIntersectsView(tile, frustum, floor)) tiles.push(tile);
      }
      if (tiles.length <= budget) break;
    }
    zoom--;
  } while (zoom >= 1);
  const middleX = (limits[0]! + limits[2]!) / 2, middleY = (limits[1]! + limits[3]!) / 2;
  return tiles.sort((a, b) => Math.hypot(a.x - middleX, a.y - middleY) - Math.hypot(b.x - middleX, b.y - middleY));
}

interface TileVisual {
  tile: TerrainTile;
  mesh: THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>;
  texture: THREE.Texture;
  state: "queued" | "loading" | "ready" | "failed";
  lastUsed: number;
}

function overlaps(a: TerrainTile, b: TerrainTile): boolean {
  const [parent, child] = a.z <= b.z ? [a, b] : [b, a];
  const scale = 2 ** (child.z - parent.z);
  return Math.floor(child.x / scale) === parent.x && Math.floor(child.y / scale) === parent.y;
}

export class TerrainMapLayer extends THREE.Group {
  private signature = "";
  private readonly tiles = new Map<string, TileVisual>();
  private queue: TileVisual[] = [];
  private pending = 0;
  private wanted = new Set<string>();
  private wantedZoom?: number;
  private displayedZoom?: number;
  private clock = 0;
  private decodeQueue: (() => void)[] = [];
  private decodeFrame?: number;
  private template?: string;
  private loadingSource = false;
  private sourceFailed = false;
  private sourceAbort?: AbortController;
  private launchTimer?: ReturnType<typeof setTimeout>;

  constructor(private readonly bounds: MapBounds, private readonly onReady?: (texture?: THREE.Texture) => void) {
    super();
    this.name = "city-map-surround";
    this.userData = { visualOnly: true, source: MAP_SURROUND.source, tileCount: 0 };
  }

  update(camera: THREE.Camera, width: number, height: number, maxAnisotropy = 8): void {
    const visible = visibleTerrainTiles(camera, width, height, this.bounds, this.wantedZoom);
    const signature = visible.map(tile => tile.key).sort().join(",");
    if (signature === this.signature) return;
    this.signature = signature;
    this.wanted = new Set(visible.map(tile => tile.key)); this.wantedZoom = visible[0]?.z;
    this.clock++;
    for (const [key, visual] of this.tiles) {
      if (this.wanted.has(key)) { visual.lastUsed = this.clock; continue; }
      // Ready textures stay in the bounded LRU cache. Cancel only stale unfinished work.
      if (visual.state === "ready") continue;
      this.remove(visual.mesh); disposeRenderObject(visual.mesh); this.tiles.delete(key);
    }
    this.queue = this.queue.filter(visual => !visual.texture.userData.viewerDisposed);
    for (const tile of visible) {
      if (this.tiles.has(tile.key)) continue;
      const texture = new THREE.Texture();
      texture.userData.viewerOwned = true;
      texture.addEventListener("dispose", () => {
        if ([...this.tiles.values()].every(item => item.texture.userData.viewerDisposed)) {
          clearTimeout(this.launchTimer); this.launchTimer = undefined; this.sourceAbort?.abort();
          if (this.decodeFrame !== undefined) cancelAnimationFrame(this.decodeFrame);
          this.decodeFrame = undefined; this.decodeQueue.length = 0;
        }
      });
      texture.colorSpace = THREE.SRGBColorSpace;
      texture.anisotropy = Number.isFinite(maxAnisotropy)
        ? Math.max(1, Math.min(MAP_SURROUND.maxAnisotropy, maxAnisotropy)) : 1;
      const mesh = new THREE.Mesh(createTerrainTileGeometry(tile), new THREE.MeshStandardMaterial({
        color: 0xffffff, map: texture, roughness: 1, metalness: 0,
        transparent: false, depthTest: true, depthWrite: true,
        // Offline ribbons already use -1 / -2 offsets. Keep the image in front of
        // those nearly coplanar layers without disabling depth or hiding fallback gaps.
        polygonOffset: true, polygonOffsetFactor: -3, polygonOffsetUnits: -3,
      }));
      mesh.name = `terrain-tile-${tile.key}`;
      mesh.position.y = (this.bounds.min[2] ?? 0) - 0.001;
      mesh.receiveShadow = true;
      mesh.visible = false;
      const visual: TileVisual = { tile, mesh, texture, state: "queued", lastUsed: this.clock };
      this.tiles.set(tile.key, visual); this.add(mesh); this.queue.push(visual);
    }
    this.refreshVisibility(); this.trimCache();
    // Camera fitting fires several synchronous renders. Request only the settled view,
    // not transient initialization frames or every intermediate pan position.
    clearTimeout(this.launchTimer);
    if (typeof document !== "undefined") {
      this.launchTimer = setTimeout(() => { this.launchTimer = undefined; this.pump(); }, MAP_SURROUND.updateDelayMs);
    }
  }

  private refreshVisibility(): void {
    const wanted = [...this.wanted].map(key => this.tiles.get(key)!);
    const ready = wanted.length > 0 && wanted.every(item => item.state === "ready");
    const hasPreviousMap = [...this.tiles.values()].some(item => item.state === "ready" && item.tile.z === this.displayedZoom);
    if (ready || !hasPreviousMap) this.displayedZoom = this.wantedZoom;
    const fallback = this.displayedZoom !== this.wantedZoom
      ? [...this.tiles.values()].filter(item => item.state === "ready" && item.tile.z === this.displayedZoom
        && wanted.some(next => overlaps(item.tile, next.tile)))
        .sort((a, b) => b.lastUsed - a.lastUsed).slice(0, MAP_SURROUND.maxVisibleTiles) : [];
    const keep = new Set(fallback.map(item => item.tile.key));
    for (const item of this.tiles.values()) {
      // Atomic LOD replacement: keep the previous map until every new visible tile is
      // decoded and uploaded. No fade, coplanar overlay, white holes or half-changed style.
      item.mesh.visible = item.state === "ready" && (keep.has(item.tile.key)
        || item.tile.z === this.displayedZoom && this.wanted.has(item.tile.key));
    }
    this.userData.tileCount = this.wanted.size;
    this.userData.displayedZoom = this.displayedZoom;
    this.userData.wantedZoom = this.wantedZoom;
    this.userData.residentTileCount = this.tiles.size;
  }

  private trimCache(): void {
    const unused = [...this.tiles.values()].filter(item => !this.wanted.has(item.tile.key) && !item.mesh.visible)
      .sort((a, b) => a.lastUsed - b.lastUsed);
    for (const item of unused) {
      if (this.tiles.size <= MAP_SURROUND.maxResidentTiles) break;
      this.tiles.delete(item.tile.key); this.remove(item.mesh); disposeRenderObject(item.mesh);
    }
    this.userData.residentTileCount = this.tiles.size;
  }

  private decodeNextFrame(action: () => void): void {
    if (typeof requestAnimationFrame === "undefined") { action(); return; }
    this.decodeQueue.push(action);
    const drain = () => {
      this.decodeFrame = undefined;
      this.decodeQueue.shift()?.();
      if (this.decodeQueue.length) this.decodeFrame = requestAnimationFrame(drain);
    };
    if (this.decodeFrame === undefined) this.decodeFrame = requestAnimationFrame(drain);
  }

  private pump(): void {
    if (typeof document === "undefined" || this.launchTimer !== undefined || this.sourceFailed
      || !this.queue.some(item => !item.texture.userData.viewerDisposed)) return;
    if (!this.template) {
      if (this.loadingSource) return;
      this.loadingSource = true;
      const abort = new AbortController(); this.sourceAbort = abort;
      let timedOut = false;
      const deadline = setTimeout(() => { timedOut = true; abort.abort(); }, 20_000);
      fetch(MAP_SURROUND.source.url, { signal: abort.signal, credentials: "omit" })
        .then(response => { if (!response.ok) throw new Error("Map source unavailable"); return response.json(); })
        .then(payload => { this.template = terrainTileTemplate(payload); })
        .catch(() => {
          // Aborted removed views may retry for newly visible tiles; real service errors
          // fail closed and leave the independent offline ground/replay available.
          if (!abort.signal.aborted || timedOut) this.sourceFailed = true;
        })
        .finally(() => {
          clearTimeout(deadline); this.loadingSource = false; this.sourceAbort = undefined;
          this.pump();
        });
      return;
    }
    while (this.pending < MAP_SURROUND.maxConcurrentRequests && this.queue.length) {
      const visual = this.queue.shift()!;
      if (visual.texture.userData.viewerDisposed) continue;
      visual.state = "loading";
      this.pending++;
      let finished = false;
      let deadline: ReturnType<typeof setTimeout>;
      const abort = new AbortController();
      const disposed = () => {
        abort.abort();
        finish(undefined, false);
        // Scene disposal marks all owned textures synchronously. Wait until that traversal
        // finishes so a removed scene cannot start requests for its formerly queued tiles.
        queueMicrotask(() => this.pump());
      };
      const finish = (image?: HTMLImageElement, resume = true) => {
        if (finished) return;
        finished = true; this.pending--;
        clearTimeout(deadline);
        visual.texture.removeEventListener("dispose", disposed);
        if (!visual.texture.userData.viewerDisposed) {
          if (image) {
            visual.texture.image = image; visual.texture.needsUpdate = true;
            visual.state = "ready"; this.userData.basemapReady = true;
          } else {
            visual.state = "failed";
          }
          this.refreshVisibility(); this.trimCache();
          this.onReady?.(image ? visual.texture : undefined);
        }
        if (resume) this.pump();
      };
      deadline = setTimeout(() => { abort.abort(); finish(); }, 20_000);
      visual.texture.addEventListener("dispose", disposed);
      // Normal browser CORS/caching. Disposing a tile also cancels its actual request.
      fetch(terrainTileURL(visual.tile, this.template), { signal: abort.signal, credentials: "omit" })
        .then(response => { if (!response.ok) throw new Error("Map tile unavailable"); return response.blob(); })
        .then(blob => finished ? undefined : decodeImageryTile(blob))
        // Decode asynchronously, then commit/upload at most one texture per frame.
        .then(image => { if (image && !finished) this.decodeNextFrame(() => finish(image)); })
        .catch(() => finish());
    }
  }
}

export function addMapSurround(host: THREE.Group, cityId: string, bounds: MapBounds, onReady?: (texture?: THREE.Texture) => void) {
  if (cityId !== MAP_SURROUND.cityId) return null;
  const layer = new TerrainMapLayer(bounds, onReady);
  host.add(layer);
  return layer;
}

export function updateMapSurround(host: THREE.Group, camera: THREE.Camera, width: number, height: number, maxAnisotropy = 8): void {
  const layer = host.getObjectByName("city-map-surround");
  if (layer instanceof TerrainMapLayer) layer.update(camera, width, height, maxAnisotropy);
}
