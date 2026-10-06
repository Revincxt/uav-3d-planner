import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { afterEach, describe, expect, it, vi } from "vitest";
import { configureMapNavigation, FrameRenderer, mapFramingPadding, MAP_OVERVIEW_ELEVATION_DEG, MIN_MAP_ELEVATION_DEG } from "../src/map-navigation";
import { SceneViewer } from "../src/scene-viewer";
import { DynamicViewer } from "../src/dynamic-viewer";
import { PredictiveViewer } from "../src/predictive-viewer";

afterEach(() => { vi.unstubAllGlobals(); });

function elevation(camera: THREE.Camera, target: THREE.Vector3): number {
  const offset = camera.position.clone().sub(target);
  return Math.atan2(offset.y, Math.hypot(offset.x, offset.z)) * 180 / Math.PI;
}

function controlledCamera() {
  const camera = new THREE.OrthographicCamera();
  camera.position.set(0, 100, 300);
  const controls = new OrbitControls(camera, null);
  configureMapNavigation(controls);
  return { camera, controls };
}

describe("above-ground map navigation", () => {
  it("adds river context only to the city panorama, preserving focused mission and top-view framing", () => {
    expect(mapFramingPadding("city", true)).toBe(1.5);
    expect(mapFramingPadding("mission", true)).toBe(1.12);
    expect(mapFramingPadding("city", false)).toBe(1.12);
    expect(mapFramingPadding("mission", false)).toBe(1.12);
  });
  it.each([0, -1, -300, -30_000])("clamps a forbidden relative camera height of %s to 15 degrees", height => {
    const { camera, controls } = controlledCamera();
    camera.position.set(300, height, 300); controls.update();
    expect(elevation(camera, controls.target)).toBeCloseTo(MIN_MAP_ELEVATION_DEG, 8);
    expect(camera.position.y).toBeGreaterThan(0);
  });

  it("preserves unrestricted azimuth and the near-vertical top view without reversing world up", () => {
    const { camera, controls } = controlledCamera();
    for (let azimuth = -360; azimuth <= 360; azimuth += 45) {
      const radians = azimuth * Math.PI / 180;
      camera.position.set(300 * Math.sin(radians), 300, 300 * Math.cos(radians)); controls.update();
      expect(elevation(camera, controls.target)).toBeCloseTo(45, 8);
      expect(camera.up.toArray()).toEqual([0, 1, 0]);
    }
    camera.position.set(0, 300, 0.001); controls.update();
    expect(elevation(camera, controls.target)).toBeGreaterThan(89.99);
  });

  it("pans along the ground, keeping both the target and camera above it", () => {
    const { camera, controls } = controlledCamera();
    controls.target.set(0, 40, 0); controls.update();
    const internal = controls as unknown as { _panUp: (distance: number, matrix: THREE.Matrix4) => void };
    for (const distance of [-50_000, 50_000]) {
      internal._panUp(distance, camera.matrix); controls.update();
      expect(controls.target.y).toBe(40); expect(camera.position.y).toBeGreaterThan(40);
      expect(elevation(camera, controls.target)).toBeGreaterThanOrEqual(15 - 1e-8);
    }
  });

  it.each([["static", SceneViewer], ["dynamic", DynamicViewer], ["predictive", PredictiveViewer]] as const)(
    "opens and resets %s from the west, looking east without changing physical coordinates", (name, Viewer) => {
      const viewer = Object.create(Viewer.prototype);
      const { camera, controls } = controlledCamera();
      Object.assign(viewer, { camera, controls, scenario: {}, currentView: "isometric", resize: vi.fn(),
        sceneBounds: () => new THREE.Box3(new THREE.Vector3(-300, 0, -3800), new THREE.Vector3(3300, 480, 0)) });
      for (const preset of name === "predictive" ? ["isometric", "fit", "yz"] : ["isometric", "reset"]) {
        viewer.setView(preset);
        // ENU east is world +X: a western observer must sit on the target's -X side.
        expect(camera.position.x, preset).toBeLessThan(controls.target.x);
        const lookingEast = camera.getWorldDirection(new THREE.Vector3());
        expect(lookingEast.x, preset).toBeGreaterThan(0);
        expect(elevation(camera, controls.target), preset).toBeGreaterThanOrEqual(15 - 1e-8);
        expect(camera.up.toArray(), preset).toEqual([0, 1, 0]);
        if (preset !== "yz") {
          expect(elevation(camera, controls.target), preset).toBeCloseTo(MAP_OVERVIEW_ELEVATION_DEG, 8);
          // The real Manhattan street axis runs NNE; the reference places its northern end upper-left.
          const center = controls.target.clone().project(camera);
          const northward = controls.target.clone().add(new THREE.Vector3(500, 0, -866)).project(camera).sub(center);
          expect(northward.x, preset).toBeLessThan(0);
          expect(northward.y, preset).toBeGreaterThan(0);
        }
        camera.position.x = 100_000; controls.update();
      }
    });

  it.each([["static", SceneViewer], ["dynamic", DynamicViewer], ["predictive", PredictiveViewer]] as const)(
    "rotates %s Top counterclockwise by 90 degrees, preserving the world and orbit up axis", (name, Viewer) => {
      const viewer = Object.create(Viewer.prototype), { camera, controls } = controlledCamera();
      Object.assign(viewer, { camera, controls, scenario: {}, currentView: "isometric", resize: vi.fn(),
        sceneBounds: () => new THREE.Box3(new THREE.Vector3(-300, 0, -3800), new THREE.Vector3(3300, 480, 0)) });
      for (let pass = 0; pass < 2; pass++) {
        viewer.setView(name === "predictive" ? "xy" : "top");
        camera.updateMatrixWorld(true);
        const center = controls.target.clone().project(camera);
        const east = controls.target.clone().add(new THREE.Vector3(100, 0, 0)).project(camera).sub(center);
        const north = controls.target.clone().add(new THREE.Vector3(0, 0, -100)).project(camera).sub(center);
        expect(east.y).toBeGreaterThan(0); expect(Math.abs(east.x)).toBeLessThan(1e-8);
        expect(north.x).toBeLessThan(0); expect(Math.abs(north.y)).toBeLessThan(1e-8);
        expect(elevation(camera, controls.target)).toBeGreaterThan(89.99);
        expect(camera.up.toArray()).toEqual([0, 1, 0]);
        viewer.setView("isometric");
      }
    });

  it.each([["static", SceneViewer], ["dynamic", DynamicViewer], ["predictive", PredictiveViewer]] as const)(
    "keeps every %s view preset and Fit above the minimum, including after Top", (name, Viewer) => {
      const viewer = Object.create(Viewer.prototype);
      const { camera, controls } = controlledCamera();
      Object.assign(viewer, { camera, controls, scenario: {}, currentView: "isometric", resize: vi.fn(),
        sceneBounds: () => new THREE.Box3(new THREE.Vector3(0, 0, -2800), new THREE.Vector3(2800, 480, 0)) });
      const presets = name === "predictive" ? ["xy", "xz", "yz", "fit", "isometric", "xy"] : ["top", "isometric", "reset", "top"];
      for (const preset of presets) {
        viewer.setView(preset);
        expect(elevation(camera, controls.target), preset).toBeGreaterThanOrEqual(15 - 1e-8);
        expect(camera.position.y, preset).toBeGreaterThan(0);
        expect(camera.up.toArray(), preset).toEqual([0, 1, 0]);
        camera.position.y = -10_000; controls.update();
        expect(elevation(camera, controls.target), `${preset} → drag`).toBeCloseTo(15, 8);
      }
    });
});

describe("frame-coalesced rendering", () => {
  it("renders one frame for a burst of pointer and tile events", () => {
    let callback: FrameRequestCallback | undefined;
    const raf = vi.fn((next: FrameRequestCallback) => { callback = next; return 7; });
    vi.stubGlobal("requestAnimationFrame", raf);
    const draw = vi.fn(), frames = new FrameRenderer(draw);
    for (let index = 0; index < 100; index++) frames.request();
    expect(raf).toHaveBeenCalledOnce(); expect(draw).not.toHaveBeenCalled();
    callback!(0); expect(draw).toHaveBeenCalledOnce();
    frames.request(); expect(raf).toHaveBeenCalledTimes(2);
  });

  it("cancels pending GPU work before a viewer is disposed", () => {
    vi.stubGlobal("requestAnimationFrame", vi.fn(() => 9));
    const cancel = vi.fn(); vi.stubGlobal("cancelAnimationFrame", cancel);
    const draw = vi.fn(), frames = new FrameRenderer(draw);
    frames.request(); frames.cancel(); frames.cancel();
    expect(cancel).toHaveBeenCalledExactlyOnceWith(9); expect(draw).not.toHaveBeenCalled();
  });
});
