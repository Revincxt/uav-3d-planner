import type { Camera, Vector3 } from "three";
import type { OrbitControls } from "three/addons/controls/OrbitControls.js";

export const MIN_MAP_ELEVATION_DEG = 15;
export const MIN_MAP_ELEVATION_RAD = MIN_MAP_ELEVATION_DEG * Math.PI / 180;
export const MAP_OVERVIEW_ELEVATION_DEG = MIN_MAP_ELEVATION_DEG;
export const MAP_OVERVIEW_SOUTH_OF_WEST_DEG = 12;

/** City panorama keeps both river banks in frame; Top retains the tight fit. */
export function mapFramingPadding(panorama: boolean): number {
  return panorama ? 1.5 : 1.12;
}

/** Low western panorama: Manhattan runs across the frame, not vertically into the distance. */
export function positionWesternOverview(camera: Camera, center: Vector3, span: number): void {
  const elevation = MAP_OVERVIEW_ELEVATION_DEG * Math.PI / 180;
  const azimuth = MAP_OVERVIEW_SOUTH_OF_WEST_DEG * Math.PI / 180;
  const distance = span * 2.2;
  const horizontal = distance * Math.cos(elevation);
  // ENU east is +X, north is -Z; stay west with a small southerly offset.
  camera.position.set(center.x - horizontal * Math.cos(azimuth),
    center.y + distance * Math.sin(elevation), center.z + horizontal * Math.sin(azimuth));
  camera.up.set(0, 1, 0);
}

/** Rotate the north-up map 90° counterclockwise: east is up, north is left.
 * A tiny western offset establishes the azimuth without changing world up or orbit limits. */
export function positionTopOverview(camera: Camera, center: Vector3, span: number, distanceScale = 2): void {
  camera.position.set(center.x - 0.001, center.y + span * distanceScale, center.z);
  camera.up.set(0, 1, 0);
}

/** The world up axis never changes, including Top and subsequent dragging. */
export function configureMapNavigation(controls: OrbitControls): void {
  controls.object.up.set(0, 1, 0);
  controls.minPolarAngle = 0;
  controls.maxPolarAngle = Math.PI / 2 - MIN_MAP_ELEVATION_RAD;
  // Pan on the horizontal world plane instead of dragging the target underground.
  controls.screenSpacePanning = false;
  controls.enableDamping = false;
  controls.minZoom = 0.55;
  controls.maxZoom = 5;
}

/** Coalesce pointer/map events to one render per frame; cancel before viewer disposal. */
export class FrameRenderer {
  private pending?: number;
  constructor(private readonly render: () => void) {}

  request(): void {
    if (this.pending !== undefined) return;
    if (typeof requestAnimationFrame === "undefined") { this.render(); return; }
    this.pending = requestAnimationFrame(() => { this.pending = undefined; this.render(); });
  }

  cancel(): void {
    if (this.pending !== undefined) cancelAnimationFrame(this.pending);
    this.pending = undefined;
  }
}
