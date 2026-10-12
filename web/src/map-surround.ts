import * as THREE from "three";
import { lonLatToMapENU, mapENUToLonLat } from "./map-basemap";
import { MAP_SURROUND } from "./map-surround-data";
import { ImageryRequestError, loadImageryTile } from "./map-tile-cache";
import { disposeRenderObject } from "./render-resources";
import type { MapBounds } from "./map-background";

const RADIUS = 6378137, HALF_WORLD = Math.PI * RADIUS;
const MAX_CONTEXT_DISTANCE_M = 25_000;

function retryableRequest(error: unknown): boolean {
  return (error instanceof TypeError || error instanceof ImageryRequestError) && (!(error instanceof ImageryRequestError)
    || error.status === 408 || error.status === 429 || error.status >= 500);
}

export interface TerrainTile { z: number; x: number; y: number; key: string }

export function terrainTileURL(tile: TerrainTile, template = MAP_SURROUND.source.tileTemplate): string {
  if (![MAP_SURROUND.source.tileTemplate, MAP_SURROUND.source.fallbackTileTemplate].includes(template)) {
    throw new Error("Unsupported public imagery provider");
  }
  const extent = terrainTileBounds(tile), url = new URL(template);
  // Request source pixels for the exact same ENU-registered ground footprint.
  // Calling /tile/17+ on the old cached service only returned errors, not finer imagery.
  for (const [key, value] of Object.entries({
    bbox: [...extent.min, ...extent.max].join(","), bboxSR: "3857", imageSR: "3857",
    size: `${MAP_SURROUND.tileSize},${MAP_SURROUND.tileSize}`, format: "jpg", compressionQuality: "95",
    interpolation: "RSP_BilinearInterpolation", f: "image",
  })) url.searchParams.set(key, value);
  if (template === MAP_SURROUND.source.tileTemplate) url.searchParams.set("renderingRule", '{"rasterFunction":"NaturalColor"}');
  else url.searchParams.set("bandIds", "0,1,2");
  return url.toString();
}

export function terrainPreviewURL(tile: TerrainTile): string {
  if (tile.z > MAP_SURROUND.preview.maxZoom) throw new Error("Uncached imagery level");
  return MAP_SURROUND.preview.tileTemplate.replace("{z}", String(tile.z)).replace("{y}", String(tile.y)).replace("{x}", String(tile.x));
}

export function terrainTileBounds(tile: TerrainTile) {
  const size = 2 * HALF_WORLD / 2 ** tile.z;
  return { min: [-HALF_WORLD + tile.x * size, HALF_WORLD - (tile.y + 1) * size],
    max: [-HALF_WORLD + (tile.x + 1) * size, HALF_WORLD - tile.y * size] };
}

/** True Web Mercator tile geometry, not a stretched photograph or invented terrain height. */
export function createTerrainTileGeometry(tile: TerrainTile, imageTile = tile): THREE.BufferGeometry {
  const extent = terrainTileBounds(tile), segments = 8;
  const positions: number[] = [], uv: number[] = [], indices: number[] = [];
  for (let row = 0; row <= segments; row++) {
    const mercatorY = extent.max[1]! + (extent.min[1]! - extent.max[1]!) * row / segments;
    const latitude = (2 * Math.atan(Math.exp(mercatorY / RADIUS)) - Math.PI / 2) * 180 / Math.PI;
    for (let column = 0; column <= segments; column++) {
      const mercatorX = extent.min[0]! + (extent.max[0]! - extent.min[0]!) * column / segments;
      const [east, north] = lonLatToMapENU(mercatorX / RADIUS * 180 / Math.PI, latitude);
      positions.push(east, 0, -north);
      const scale = 2 ** (tile.z - imageTile.z);
      uv.push((column / segments + tile.x - imageTile.x * scale) / scale,
        (1 - row / segments + scale - 1 - tile.y + imageTile.y * scale) / scale);
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

const groundViews = new WeakMap<THREE.Camera, {
  world: THREE.Matrix4; projection: THREE.Matrix4; combined: THREE.Matrix4;
  floor: number; hits: THREE.Vector3[]; frustum: THREE.Frustum;
}>();
function groundView(camera: THREE.Camera, floor: number) {
  let view = groundViews.get(camera);
  if (view && view.floor === floor && view.world.equals(camera.matrixWorld)
    && view.projection.equals(camera.projectionMatrix)) return view;
  if (!view) {
    view = { world: new THREE.Matrix4(), projection: new THREE.Matrix4(), combined: new THREE.Matrix4(),
      floor, hits: [], frustum: new THREE.Frustum() };
    groundViews.set(camera, view);
  }
  view.world.copy(camera.matrixWorld); view.projection.copy(camera.projectionMatrix); view.floor = floor;
  view.hits = camera instanceof THREE.PerspectiveCamera ? perspectiveGroundHits(camera, floor) : [];
  if (!(camera instanceof THREE.PerspectiveCamera)) {
    const raycaster = new THREE.Raycaster();
    for (const [x, y] of [[-1, -1], [-1, 1], [1, -1], [1, 1]]) {
      raycaster.setFromCamera(new THREE.Vector2(x!, y!), camera);
      const ray = raycaster.ray;
      if (ray.direction.y >= -1e-5) { view.hits = []; break; }
      view.hits.push(ray.at((floor - ray.origin.y) / ray.direction.y, new THREE.Vector3()));
    }
  }
  view.frustum.setFromProjectionMatrix(view.combined.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse));
  return view;
}

/** Only tiles intersecting the current ground view, with bounded resolution and request count. */
export function visibleTerrainTiles(
  camera: THREE.Camera, width: number, height: number, bounds: MapBounds, preferredZoom?: number, maxZoom = MAP_SURROUND.maxZoom,
): TerrainTile[] {
  camera.updateMatrixWorld(true);
  const floor = (bounds.min[2] ?? 0) - 0.001;
  // Share projection between preview/detail. Pixel density changes only resolution,
  // not the ground footprint, so Retina resizes need no second ray/frustum rebuild.
  const { hits, frustum } = groundView(camera, floor);
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
  let zoom = THREE.MathUtils.clamp(Math.ceil(resolutionZoom), MAP_SURROUND.minZoom, maxZoom);
  if (preferredZoom !== undefined && preferredZoom >= MAP_SURROUND.minZoom && preferredZoom <= maxZoom
    && Math.abs(resolutionZoom - preferredZoom) < 0.8) zoom = preferredZoom;
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
  attempts: number;
  imageTile?: TerrainTile;
  failedAt?: number;
  retry?: ReturnType<typeof setTimeout>;
}

function overlaps(a: TerrainTile, b: TerrainTile): boolean {
  const [parent, child] = a.z <= b.z ? [a, b] : [b, a];
  const scale = 2 ** (child.z - parent.z);
  return Math.floor(child.x / scale) === parent.x && Math.floor(child.y / scale) === parent.y;
}

export class TerrainMapLayer extends THREE.Group {
  private signature = "";
  private viewCamera?: THREE.Camera;
  private readonly viewMatrix = new THREE.Matrix4();
  private readonly viewProjection = new THREE.Matrix4();
  private viewWidth = 0;
  private viewHeight = 0;
  private readonly tiles = new Map<string, TileVisual>();
  private queue: TileVisual[] = [];
  private pending = 0;
  private wanted = new Set<string>();
  private wantedZoom?: number;
  private clock = 0;
  private decodeQueue: (() => void)[] = [];
  private decodeFrame?: number;
  private template = MAP_SURROUND.source.tileTemplate;
  private launchTimer?: ReturnType<typeof setTimeout>;
  private listening = false;
  private readonly recover = () => {
    this.template = MAP_SURROUND.source.tileTemplate;
    for (const visual of this.tiles.values()) if (visual.state === "failed" && this.wanted.has(visual.tile.key)) {
      visual.state = "queued"; visual.attempts = 0;
    }
    this.prioritizeQueue(); this.schedule();
  };
  private readonly resume = () => { if (document.visibilityState === "visible") this.recover(); };

  constructor(private readonly bounds: MapBounds, private readonly onReady?: (texture?: THREE.Texture) => void,
    private readonly preview = false) {
    super();
    this.name = preview ? "city-map-preview" : "city-map-surround";
    this.userData = { visualOnly: true, source: MAP_SURROUND.source, tileCount: 0 };
  }

  update(camera: THREE.Camera, width: number, height: number, maxAnisotropy = 8): void {
    // Aircraft playback does not move the overview camera. In that common case
    // retain the exact tile decision, without repeating ground/frustum projection.
    // Camera identity matters when entering/leaving the perspective follow view.
    camera.updateMatrixWorld(true);
    if (camera === this.viewCamera && width === this.viewWidth && height === this.viewHeight
      && this.viewMatrix.equals(camera.matrixWorld) && this.viewProjection.equals(camera.projectionMatrix)) return;
    this.viewCamera = camera; this.viewWidth = width; this.viewHeight = height;
    this.viewMatrix.copy(camera.matrixWorld); this.viewProjection.copy(camera.projectionMatrix);
    let recovered = false;
    for (const visual of this.tiles.values()) if (visual.state === "failed" && this.wanted.has(visual.tile.key)
      && Date.now() - (visual.failedAt ?? 0) >= MAP_SURROUND.recoveryDelayMs) {
      visual.state = "queued"; visual.attempts = 0; recovered = true;
    }
    const visible = visibleTerrainTiles(camera, width, height, this.bounds, this.wantedZoom,
      this.preview ? MAP_SURROUND.preview.maxZoom : MAP_SURROUND.maxZoom);
    const signature = visible.map(tile => tile.key).sort().join(",");
    if (signature === this.signature) {
      if (recovered) { this.prioritizeQueue(); this.schedule(); }
      return;
    }
    this.signature = signature;
    this.wanted = new Set(visible.map(tile => tile.key)); this.wantedZoom = visible[0]?.z;
    this.clock++;
    for (const [key, visual] of this.tiles) {
      if (this.wanted.has(key)) { visual.lastUsed = this.clock; continue; }
      // Ready textures stay in the bounded LRU cache. Cancel only stale unfinished work.
      clearTimeout(visual.retry); visual.retry = undefined;
      if (visual.texture.image || visual.state === "loading" && visible.some(tile => overlaps(tile, visual.tile))) continue;
      this.remove(visual.mesh); disposeRenderObject(visual.mesh); this.tiles.delete(key);
    }
    for (const tile of visible) {
      if (this.tiles.has(tile.key)) continue;
      const texture = new THREE.Texture();
      texture.userData.viewerOwned = true;
      texture.addEventListener("dispose", () => {
        clearTimeout(visual.retry); visual.retry = undefined;
        if ([...this.tiles.values()].every(item => item.texture.userData.viewerDisposed)) {
          clearTimeout(this.launchTimer); this.launchTimer = undefined;
          if (typeof window !== "undefined") window.removeEventListener("online", this.recover);
          if (typeof document !== "undefined") document.removeEventListener?.("visibilitychange", this.resume);
          this.listening = false;
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
        polygonOffset: true, polygonOffsetFactor: this.preview ? -3 : -4, polygonOffsetUnits: this.preview ? -3 : -4,
      }));
      mesh.name = `terrain-tile-${tile.key}`;
      mesh.position.y = (this.bounds.min[2] ?? 0) - 0.001;
      mesh.receiveShadow = true;
      mesh.visible = false;
      const visual: TileVisual = { tile, mesh, texture, state: "queued", lastUsed: this.clock, attempts: 0 };
      this.tiles.set(tile.key, visual); this.add(mesh);
    }
    this.prioritizeQueue();
    if (visible.length && !this.listening && typeof window !== "undefined") {
      window.addEventListener("online", this.recover); this.listening = true;
      if (typeof document !== "undefined") document.addEventListener?.("visibilitychange", this.resume);
    }
    this.refreshVisibility(); this.trimCache();
    // Camera fitting fires several synchronous renders. Request only the settled view,
    // not transient initialization frames or every intermediate pan position.
    this.schedule();
  }

  private schedule(): void {
    // Throttle the latest view; resetting a debounce every frame starves follow/pan.
    if (typeof document !== "undefined" && this.launchTimer === undefined && this.queue.length) {
      this.launchTimer = setTimeout(() => { this.launchTimer = undefined; this.pump(); }, MAP_SURROUND.updateDelayMs);
    }
  }

  private prioritizeQueue(): void {
    // The wanted set follows the latest center-first tile order. Retained queued
    // edge tiles must not delay the center of a newly panned or zoomed view.
    this.queue = [...this.wanted].map(key => this.tiles.get(key)).filter((item): item is TileVisual =>
      item?.state === "queued" && !item.texture.userData.viewerDisposed);
  }

  private refreshVisibility(): void {
    const wanted = [...this.wanted].map(key => this.tiles.get(key)!);
    // Crop an already decoded parent into each child footprint, never stretch it
    // across a new extent. Children can sharpen independently without coplanar LODs.
    for (const visual of wanted) if (!visual.texture.image) {
      const parent = [...this.tiles.values()].filter(item => item.texture.image && item.tile.z < visual.tile.z && overlaps(item.tile, visual.tile))
        .sort((a, b) => b.tile.z - a.tile.z)[0];
      if (parent) {
        visual.imageTile = parent.imageTile ?? parent.tile;
        visual.mesh.geometry.dispose(); visual.mesh.geometry = createTerrainTileGeometry(visual.tile, visual.imageTile);
        // Share immutable image source/GPU storage while sharpening. A child with
        // its own native image gets a new source, so it never rewrites the parent.
        visual.texture.source = parent.texture.source; visual.texture.needsUpdate = true;
      }
    }
    for (const item of this.tiles.values()) {
      item.mesh.visible = Boolean(item.texture.image) && (this.wanted.has(item.tile.key)
        || wanted.some(next => !next.texture.image && overlaps(item.tile, next.tile))
          && !wanted.some(next => next.texture.image && overlaps(item.tile, next.tile)));
    }
    this.userData.tileCount = this.wanted.size;
    this.userData.displayedZoom = wanted.some(item => item.texture.image) ? this.wantedZoom : undefined;
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
    if (typeof document === "undefined" || this.launchTimer !== undefined
      || !this.queue.some(item => !item.texture.userData.viewerDisposed)) return;
    while (this.pending < MAP_SURROUND.maxConcurrentRequests && this.queue.length) {
      const visual = this.queue.shift()!;
      if (visual.texture.userData.viewerDisposed) continue;
      visual.state = "loading"; visual.attempts++;
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
      const finish = (image?: HTMLImageElement, resume = true, retryable = false) => {
        if (finished) return;
        finished = true; this.pending--;
        clearTimeout(deadline);
        visual.texture.removeEventListener("dispose", disposed);
        if (!visual.texture.userData.viewerDisposed) {
          if (image) {
            if (visual.imageTile && visual.imageTile.key !== visual.tile.key) {
              visual.mesh.geometry.dispose(); visual.mesh.geometry = createTerrainTileGeometry(visual.tile);
              visual.texture.source = new THREE.Source(image);
            }
            visual.imageTile = visual.tile;
            visual.texture.image = image; visual.texture.needsUpdate = true;
            visual.state = "ready"; this.userData.basemapReady = true;
          } else {
            visual.state = "failed";
            visual.failedAt = Date.now();
            if (retryable && this.wanted.has(visual.tile.key) && visual.attempts < MAP_SURROUND.maxRequestAttempts) {
              visual.retry = setTimeout(() => {
                visual.retry = undefined;
                if (visual.texture.userData.viewerDisposed || !this.wanted.has(visual.tile.key)) return;
                visual.state = "queued"; this.prioritizeQueue(); this.pump();
              }, MAP_SURROUND.retryDelayMs);
            }
          }
          this.refreshVisibility(); this.trimCache();
          this.onReady?.(image ? visual.texture : undefined);
        }
        if (resume) this.pump();
      };
      const failed = (retryable: boolean) => {
        if (retryable && !this.preview) this.template = MAP_SURROUND.source.fallbackTileTemplate;
        finish(undefined, true, retryable);
      };
      // Queue/cache waiting is not a failure of the remote provider. Start the
      // short network timeout only after acquiring a shared transport slot.
      deadline = setTimeout(() => { abort.abort(); finish(); }, 30_000);
      const template = this.template;
      const networkStarted = () => {
        clearTimeout(deadline);
        deadline = setTimeout(() => { abort.abort(); failed(!this.preview); },
          !this.preview && template === MAP_SURROUND.source.fallbackTileTemplate
            ? MAP_SURROUND.fallbackTimeoutMs : MAP_SURROUND.requestTimeoutMs);
      };
      visual.texture.addEventListener("dispose", disposed);
      // Normal browser CORS/caching. Disposing a tile also cancels its actual request.
      loadImageryTile(this.preview ? terrainPreviewURL(visual.tile) : terrainTileURL(visual.tile, template),
        abort.signal, this.preview ? MAP_SURROUND.preview.tileSize : MAP_SURROUND.tileSize, this.preview ? 0 : 1, networkStarted)
        // Decode asynchronously, then commit/upload at most one texture per frame.
        .then(image => {
          if (image && !finished) this.decodeNextFrame(() => finish(image));
          else finish();
        })
        .catch(error => { if (!finished) failed(!abort.signal.aborted && retryableRequest(error)); });
    }
  }
}

export function addMapSurround(host: THREE.Group, cityId: string, bounds: MapBounds, onReady?: (texture?: THREE.Texture) => void) {
  if (cityId !== MAP_SURROUND.cityId) return null;
  const layer = new TerrainMapLayer(bounds, onReady);
  const preview = new TerrainMapLayer(bounds, onReady, true);
  host.add(preview);
  host.add(layer);
  return layer;
}

export function updateMapSurround(host: THREE.Group, camera: THREE.Camera, width: number, height: number, maxAnisotropy = 8): void {
  const preview = host.getObjectByName("city-map-preview");
  if (preview instanceof TerrainMapLayer) preview.update(camera, width, height, maxAnisotropy);
  const layer = host.getObjectByName("city-map-surround");
  if (layer instanceof TerrainMapLayer) layer.update(camera, width, height, maxAnisotropy);
}
