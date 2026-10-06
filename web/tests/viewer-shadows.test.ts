import * as THREE from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { describe, expect, it, vi } from "vitest";

import { DynamicViewer } from "../src/dynamic-viewer";
import { PredictiveViewer } from "../src/predictive-viewer";
import { SceneViewer } from "../src/scene-viewer";
import { disposeRenderObject } from "../src/render-resources";
import { sceneLineStyle } from "../src/scene-style";

const staticScenario = {
  id: "shadow-regression",
  label: "Shadow regression",
  description: "In-memory rendering fixture",
  scenarioSeed: 1,
  fingerprint: `sha256:${"a".repeat(64)}`,
  bounds: { min: [0, 0, 0], max: [100, 100, 80] },
  start: [10, 10, 20],
  goal: [90, 90, 20],
  constraints: { vehicleRadiusM: 1, safetyMarginM: 2, maxAltitudeM: 80 },
  buildings: [{ id: "building", min: [40, 40, 0], max: [60, 60, 30] }],
  noFlyZones: [],
  results: [],
};

const movingScenario = {
  ...staticScenario,
  staticNoFlyZones: [],
  temporaryNoFlyZones: [],
  movingSpheres: [{
    id: "traffic",
    radiusM: 2,
    keyframes: [
      { timeS: 0, position: [10, 50, 40] },
      { timeS: 10, position: [90, 50, 40] },
    ],
  }],
  runs: [],
};

const frame = {
  vehicle: [15, 15, 20],
  activeTemporaryZoneIds: [],
  movingSpheres: [{ id: "traffic", position: [30, 50, 40] }],
  path: [],
  executedPath: [],
};

const cityPresentation = {
  bounds: { min: [-240.46513, -186.36859, 0], max: [2542.36633, 2496.13957, 480] },
  city: {
    id: "nyc-manhattan-midtown-official",
    name: "Map lifecycle fixture",
    sourceKind: "nyc-open-data",
    collisionModel: "conservative-aabb",
    sourceUrl: "https://data.cityofnewyork.us/City-Government/BUILDING/5zhs-2jue",
    sourceSha256: "1".repeat(64),
  },
};

function effectivelyVisible(object: THREE.Object3D): boolean {
  for (let ancestor: THREE.Object3D | null = object; ancestor; ancestor = ancestor.parent) {
    if (!ancestor.visible) return false;
  }
  return true;
}

/** Exercise real viewer methods without constructing a browser/WebGL renderer. */
function viewerHarness(Viewer: typeof SceneViewer | typeof DynamicViewer | typeof PredictiveViewer) {
  const viewer = Object.create(Viewer.prototype);
  viewer.renderer = { shadowMap: { autoUpdate: false, needsUpdate: false } };
  viewer.scene = new THREE.Scene();
  viewer.content = new THREE.Group();
  viewer.scene.add(viewer.content);
  viewer.sun = new THREE.DirectionalLight();
  viewer.cityBounds = new THREE.Box3();
  viewer.lineMaterials = Viewer === SceneViewer ? [] : new Set();
  viewer.container = { clientWidth: 900, clientHeight: 600 };
  viewer.paths = new THREE.Group();
  viewer.temporaryZones = new Map();
  viewer.movingSpheres = new Map();
  viewer.mapScope = "city";
  viewer.layerVisibility = { buildings: true, zones: true, dynamic: true };
  for (const name of [
    "buildingsGroup", "zonesGroup", "dynamicGroup",
    "primaryPathGroup", "evidenceGroup",
  ]) {
    viewer[name] = new THREE.Group();
    if (Viewer === PredictiveViewer) viewer.content.add(viewer[name]);
  }
  // Only camera/GPU work is stubbed. Content construction and invalidation stay real.
  viewer.setView = vi.fn();
  viewer.render = vi.fn();
  return viewer;
}

function dynamicHarness() {
  const viewer = viewerHarness(DynamicViewer);
  viewer.setScenario(movingScenario);
  viewer.renderer.shadowMap.needsUpdate = false;
  viewer.replaceLine = vi.fn(() => null);
  return viewer;
}

function predictiveHarness() {
  const viewer = viewerHarness(PredictiveViewer);
  viewer.setScenario(movingScenario);
  const path = [
    { timeS: 0, position: [10, 10, 20] },
    { timeS: 10, position: [90, 90, 20] },
  ];
  viewer.replaceLine = vi.fn(() => null);
  viewer.setRun({
    rawTimedPath: path,
    geometryTimedPath: path,
    executionTimedPath: path,
    plannerMetrics: { minimumSeparationWitness: null },
    geometryMetrics: { minimumSeparationWitness: null },
    executionMetrics: { minimumSeparationWitness: null },
    executionFrames: [],
    executionWaitIntervals: [],
    smoothing: { execution: { status: "qualified", qualified: true, collisionCertified: true, qualification: { qualified: true } } },
  });
  viewer.renderer.shadowMap.needsUpdate = false;
  return viewer;
}

function depthHarness(Viewer: typeof SceneViewer | typeof DynamicViewer | typeof PredictiveViewer) {
  const viewer = viewerHarness(Viewer);
  const points = [[10, 10, 20], [50, 70, 20], [90, 90, 20]];
  const zone = { id: "zone", center: [70, 30], radiusM: 8, zMinM: 0, zMaxM: 50 };
  const scenario = {
    ...(Viewer === SceneViewer ? staticScenario : movingScenario),
    ...cityPresentation,
    noFlyZones: [zone],
    staticNoFlyZones: [zone],
    results: [{ plannerId: "astar-3d", paths: { raw: points, smoothed: points } }],
  };
  const before = JSON.stringify(scenario);
  if (Viewer === SceneViewer) {
    viewer.setScenario(scenario, new Set(["astar-3d"]), "smoothed");
  } else if (Viewer === DynamicViewer) {
    viewer.setScenario(scenario);
    viewer.setFrame({ ...frame, path: points, executedPath: points.slice(0, 2) });
  } else {
    viewer.setScenario(scenario);
    const path = points.map((position, index) => ({ timeS: index * 5, position }));
    const witness = {
      separationM: 2,
      timeS: 5,
      vehiclePosition: points[1],
      obstacleId: "traffic",
      obstacleKind: "moving-sphere",
      obstaclePosition: [52, 70, 20],
      declaredSafetyMarginM: 2,
      method: "exact-relative-linear-motion",
      exact: true,
    };
    viewer.setRun({
      rawTimedPath: path,
      geometryTimedPath: path,
      executionTimedPath: path,
      plannerMetrics: { minimumSeparationWitness: witness },
      geometryMetrics: { minimumSeparationWitness: witness },
      executionMetrics: { minimumSeparationWitness: witness },
      executionFrames: [],
      executionWaitIntervals: [],
      smoothing: { execution: { status: "qualified", qualified: true, collisionCertified: true, qualification: { qualified: true } } },
    });
    viewer.setTime(5);
  }
  expect(JSON.stringify(scenario)).toBe(before);
  return viewer;
}

describe("world-space scene occlusion", () => {
  it("keeps shared dynamic traffic on the world clock after the focused task arrives", () => {
    const viewer = dynamicHarness();
    const scenario = { ...movingScenario, mission: { origin: "A", destination: "B", purpose: "Cargo",
      sharedWorld: { id: "world", fingerprint: `sha256:${"a".repeat(64)}`, missionCount: 4,
        clockOriginS: 0, airframeSpanM: 18, missionDeconfliction: "not-jointly-optimized" } },
      movingSpheres: [{ ...movingScenario.movingSpheres[0], keyframes: [
        { timeS: 0, position: [10,50,40] }, { timeS: 40, position: [90,50,40] },
      ] }] };
    viewer.setScenario(scenario);
    viewer.overview = { setTime: vi.fn() };
    viewer.setFrame({ ...frame, timeS: 10 }, 20, 10);
    expect(viewer.movingSpheres.get("traffic").position.toArray()).toEqual([50,40,-50]);
    const traffic = viewer.movingSpheres.get("traffic");
    viewer.setScenario({ ...scenario, id: "another-task" });
    expect(viewer.movingSpheres.get("traffic")).toBe(traffic);
  });
  it("keeps shared predictive traffic on the world clock after the focused flight ends", () => {
    const viewer = predictiveHarness();
    viewer.scenario = { ...viewer.scenario, mission: { sharedWorld: { id: "world" } },
      movingSpheres: [{ ...movingScenario.movingSpheres[0], keyframes: [
        { timeS: 0, position: [10,50,40] }, { timeS: 40, position: [90,50,40] },
      ] }] };
    viewer.setTime(20);
    expect(viewer.currentTimeS).toBe(10);
    expect(viewer.playbackTimeS).toBe(20);
    expect(viewer.movingSpheres.get("traffic").position.toArray()).toEqual([50,40,-50]);
  });
  it.each([
    ["static", SceneViewer],
    ["dynamic", DynamicViewer],
    ["predictive", PredictiveViewer],
  ] as const)("depth-tests every %s scene object, including trajectories and evidence", (_name, Viewer) => {
    const viewer = depthHarness(Viewer);
    let tested = 0;
    viewer.content.traverse((object: THREE.Object3D) => {
      if (!(object instanceof THREE.Mesh || object instanceof THREE.Line)) return;
      for (const material of Array.isArray(object.material) ? object.material : [object.material]) {
        expect(material.depthTest, object.name).toBe(true);
        tested += 1;
      }
    });
    expect(tested).toBeGreaterThan(5);
    viewer.clearContent();
  });

  it.each([
    ["static", SceneViewer],
    ["dynamic", DynamicViewer],
    ["predictive", PredictiveViewer],
  ] as const)("makes %s endpoint symbols opaque, smaller, and normally depth-ordered", (_name, Viewer) => {
    const viewer = depthHarness(Viewer);
    for (const role of ["start", "goal"] as const) {
      const endpoint = viewer.content.getObjectByName(`endpoint-${role}`)!;
      endpoint.traverse((object: THREE.Object3D) => {
        if (!(object instanceof THREE.Mesh)) return;
        const material = object.material as THREE.MeshStandardMaterial;
        expect(material.transparent).toBe(false);
        expect(material.depthTest && material.depthWrite).toBe(true);
      });
      expect(endpoint.renderOrder).toBe(0);
      if (role === "start") expect(((endpoint as THREE.Mesh).geometry as THREE.SphereGeometry).parameters.radius).toBe(6);
      else {
        expect(endpoint.userData.kind).toBe("finish-marker");
        expect(endpoint.getObjectByName("finish-checkered-flag")).toBeDefined();
      }
      expect(endpoint.position.toArray()).toEqual(role === "start" ? [10, 20, -10] : [90, 20, -90]);
    }
    viewer.clearContent();
  });

  it.each([
    ["static", SceneViewer],
    ["dynamic", DynamicViewer],
    ["predictive", PredictiveViewer],
  ] as const)("keeps %s trajectory widths in world units without depth-buffer pollution", (_name, Viewer) => {
    const viewer = depthHarness(Viewer);
    let paths = 0;
    viewer.content.traverse((object: THREE.Object3D) => {
      if (!(object instanceof Line2)) return;
      expect(object.material.worldUnits).toBe(true);
      expect(object.material.depthWrite).toBe(false);
      expect(object.material.alphaToCoverage).toBe(true);
      expect(object.material.linewidth).toBeGreaterThan(0);
      expect(object.material.linewidth).toBeLessThanOrEqual(4.7);
      expect(object.renderOrder).toBeLessThanOrEqual(1);
      paths += 1;
    });
    expect(paths).toBeGreaterThan(0);
    viewer.clearContent();
  });

  it.each([
    ["dynamic", DynamicViewer],
    ["predictive", PredictiveViewer],
  ] as const)("depth-orders the %s vehicle and preserves physical traffic radii", (_name, Viewer) => {
    const viewer = depthHarness(Viewer);
    expect(viewer.vehicle.geometry.parameters.radius).toBe(4);
    expect(viewer.vehicle.geometry.parameters.height).toBe(10);
    expect(viewer.vehicle.material.transparent).toBe(false);
    expect(viewer.vehicle.material.depthWrite).toBe(true);
    expect(viewer.vehicle.renderOrder).toBe(0);
    const traffic = viewer.movingSpheres.get("traffic");
    if (traffic.userData.kind === "cargo-drone") {
      expect(traffic.userData.separationRadiusM).toBe(2);
      expect(traffic.userData.airframeSpanM).toBe(18);
      traffic.traverse((object: THREE.Object3D) => {
        if (object instanceof THREE.Mesh) {
          expect(object.material.transparent).toBe(false);
          expect(object.material.depthTest).toBe(true);
          expect(object.material.depthWrite).toBe(true);
        }
      });
      viewer.clearContent();
      return;
    }
    const mesh = traffic instanceof THREE.Mesh ? traffic : traffic.children[0];
    expect(mesh.geometry.parameters.radius).toBe(2);
    expect(mesh.material.transparent).toBe(false);
    expect(mesh.material.depthTest).toBe(true);
    expect(mesh.material.depthWrite).toBe(true);
    viewer.clearContent();
  });

  it("retains depth-tested world-space styling when predictive path buffers are reused", () => {
    const viewer = depthHarness(PredictiveViewer);
    const primary = viewer.primaryPath;
    const executed = viewer.executedPath;
    expect(viewer.rawPath).toBeUndefined();
    expect(viewer.rawPathGroup).toBeUndefined();
    for (const time of [7, 0, 5, 10]) {
      viewer.setTime(time);
      expect(viewer.primaryPath).toBe(primary);
      expect(viewer.executedPath).toBe(executed);
      for (const line of [primary, executed]) {
        expect(line.material.depthTest).toBe(true);
        expect(line.material.worldUnits).toBe(true);
        expect(line.material.depthWrite).toBe(false);
      }
    }
    viewer.clearContent();
  });

  it("does not restore an x-ray overlay at the minimum-separation instant", () => {
    const viewer = depthHarness(PredictiveViewer);
    expect(viewer.evidenceGroup.visible).toBe(true);
    viewer.evidenceGroup.traverse((object: THREE.Object3D) => {
      if (!(object instanceof THREE.Mesh)) return;
      const material = object.material as THREE.Material;
      expect(material.depthTest).toBe(true);
      expect(object.renderOrder).toBe(0);
      if (object.name === "declared-safety-margin-envelope") expect(material.depthWrite).toBe(false);
    });
    viewer.setTime(5.1);
    expect(viewer.evidenceGroup.visible).toBe(false);
    viewer.clearContent();
  });

  it("uses scene-scale dash lengths rather than tiny city-scale stippling", () => {
    expect(sceneLineStyle(true, 3, true)).toMatchObject({ linewidth: 3.75, dashSize: 18, gapSize: 10 });
    expect(sceneLineStyle(false, 3, true)).toMatchObject({ dashSize: 2.2, gapSize: 1.4 });
  });
});

describe("cached viewer shadows", () => {
  it.each([
    ["static", SceneViewer],
    ["dynamic", DynamicViewer],
    ["predictive", PredictiveViewer],
  ] as const)("invalidates %s shadows on the first scene and every rebuild", (_name, Viewer) => {
    const viewer = viewerHarness(Viewer);
    const scenario = Viewer === SceneViewer ? staticScenario : movingScenario;
    const rebuild = () => Viewer === SceneViewer
      ? viewer.setScenario(scenario, new Set(), "smoothed")
      : viewer.setScenario(scenario);

    rebuild();
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(true);
    viewer.renderer.shadowMap.needsUpdate = false;
    rebuild();
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(true);
  });

  it("reuses the dynamic city shadow map when animated objects do not cast shadows", () => {
    const viewer = dynamicHarness();
    expect(viewer.vehicle.castShadow).toBe(false);
    expect(viewer.movingSpheres.get("traffic").castShadow).toBe(false);
    viewer.setFrame(frame);
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(false);
  });

  it("invalidates dynamic shadows if the moving vehicle actually casts a shadow", () => {
    const viewer = dynamicHarness();
    viewer.vehicle.castShadow = true;
    viewer.setFrame(frame);
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(true);
  });

  it("invalidates dynamic shadows if a moving traffic mesh actually casts a shadow", () => {
    const viewer = dynamicHarness();
    viewer.movingSpheres.get("traffic").castShadow = true;
    viewer.setFrame(frame);
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(true);
  });

  it("refreshes the predictive vehicle shadow during playback", () => {
    const viewer = predictiveHarness();
    expect(viewer.vehicle.castShadow).toBe(true);
    viewer.setTime(2);
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(true);
  });

  it("detects a predictive caster inside its moving traffic group", () => {
    const viewer = predictiveHarness();
    viewer.vehicle.castShadow = false;
    const traffic = viewer.movingSpheres.get("traffic");
    expect(traffic.castShadow).toBe(false);
    expect(traffic.children[0].castShadow).toBe(true);
    viewer.setTime(2);
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(true);
  });

  it("reuses predictive shadows when no animated object casts one", () => {
    const viewer = predictiveHarness();
    viewer.vehicle.castShadow = false;
    viewer.dynamicGroup.traverse((child: THREE.Object3D) => { child.castShadow = false; });
    viewer.setTime(2);
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(false);
  });

  it("invalidates predictive shadows when buildings are hidden or restored", () => {
    const viewer = predictiveHarness();
    viewer.setLayerVisibility("buildings", false);
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(true);
    viewer.renderer.shadowMap.needsUpdate = false;
    viewer.setLayerVisibility("buildings", true);
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(true);
  });

  it("invalidates predictive shadows when dynamic casters are hidden or restored", () => {
    const viewer = predictiveHarness();
    viewer.setLayerVisibility("dynamic", false);
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(true);
    viewer.renderer.shadowMap.needsUpdate = false;
    viewer.setLayerVisibility("dynamic", true);
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(true);
  });

  it("does not refresh predictive shadows for hidden moving traffic", () => {
    const viewer = predictiveHarness();
    viewer.vehicle.castShadow = false;
    viewer.setLayerVisibility("dynamic", false);
    viewer.renderer.shadowMap.needsUpdate = false;
    viewer.setTime(2);
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(false);
  });

  it("does not invalidate shadows for non-casting zone and raw-path layer changes", () => {
    const viewer = predictiveHarness();
    viewer.setLayerVisibility("zones", false);
    viewer.setLayerVisibility("raw", true);
    expect(viewer.renderer.shadowMap.needsUpdate).toBe(false);
  });
});

describe("shared map background lifecycle", () => {
  it.each([
    ["static", SceneViewer],
    ["dynamic", DynamicViewer],
    ["predictive", PredictiveViewer],
  ] as const)("retains an owned %s map on focus and disposes it exactly once when source changes", (_name, Viewer) => {
    const viewer = viewerHarness(Viewer);
    const scenario = {
      ...(Viewer === SceneViewer ? staticScenario : movingScenario),
      ...cityPresentation,
    };
    const rebuild = () => Viewer === SceneViewer
      ? viewer.setScenario(scenario, new Set(), "smoothed")
      : viewer.setScenario(scenario);
    rebuild();
    for (let cycle = 0; cycle < 2; cycle += 1) {
      const oldContext = viewer.content.getObjectByName("city-context") as THREE.Group;
      const texture = new THREE.Texture();
      texture.userData.viewerOwned = true;
      const disposed = vi.spyOn(texture, "dispose");
      const disposedMarkers: boolean[] = [];
      texture.addEventListener("dispose", () => {
        disposedMarkers.push(texture.userData.viewerDisposed === true);
      });
      // Multiple materials may share one locally owned aerial image. Disposal belongs to the
      // scene, not to each material individually, and must mark stale asynchronous callbacks.
      oldContext.add(new THREE.Mesh(new THREE.PlaneGeometry(1, 1), [
        new THREE.MeshBasicMaterial({ map: texture }),
        new THREE.MeshBasicMaterial({ map: texture }),
      ]));
      expect(texture.userData.viewerDisposed).toBeUndefined();
      rebuild();
      expect(viewer.content.getObjectByName("city-context") === oldContext).toBe(true);
      expect(disposed).not.toHaveBeenCalled();
      scenario.city = { ...scenario.city, sourceSha256: String(cycle + 2).repeat(64) };
      rebuild();
      expect(disposed).toHaveBeenCalledOnce();
      expect(disposedMarkers).toEqual([true]);
      expect(texture.userData.viewerDisposed).toBe(true);
    }
    viewer.clearContent();
    viewer.cityLayer.dispose();
  });

  it.each([
    ["static", SceneViewer],
    ["dynamic", DynamicViewer],
    ["predictive", PredictiveViewer],
  ] as const)("retains exactly one %s background and its GPU resources after mission focus", (_name, Viewer) => {
    const viewer = viewerHarness(Viewer);
    const scenario = {
      ...(Viewer === SceneViewer ? staticScenario : movingScenario),
      ...cityPresentation,
    };
    const originalScenario = JSON.stringify(scenario);
    const rebuild = () => Viewer === SceneViewer
      ? viewer.setScenario(scenario, new Set(), "smoothed")
      : viewer.setScenario(scenario);
    rebuild();
    const oldContext = viewer.content.getObjectByName("city-context") as THREE.Group;
    expect(oldContext).toBeDefined();
    expect(oldContext.getObjectByName("city-ground")).toBeDefined();
    const geometries = new Set<THREE.BufferGeometry>();
    const materials = new Set<THREE.Material>();
    oldContext.traverse((object) => {
      if (object instanceof THREE.Mesh || object instanceof THREE.Line) {
        geometries.add(object.geometry);
        for (const material of Array.isArray(object.material) ? object.material : [object.material]) {
          materials.add(material);
        }
      }
    });
    expect(geometries.size).toBeGreaterThan(0);
    const geometryDisposals = [...geometries].map((geometry) => vi.spyOn(geometry, "dispose"));
    const materialDisposals = [...materials].map((material) => vi.spyOn(material, "dispose"));

    rebuild();
    for (const disposal of [...geometryDisposals, ...materialDisposals]) {
      expect(disposal).not.toHaveBeenCalled();
    }
    const contexts: THREE.Object3D[] = [];
    viewer.content.traverse((object: THREE.Object3D) => {
      if (object.name === "city-context") contexts.push(object);
    });
    expect(contexts).toHaveLength(1);
    expect(contexts[0] === oldContext).toBe(true);
    expect(JSON.stringify(scenario)).toBe(originalScenario);
    viewer.clearContent();
    viewer.cityLayer.dispose();
    for (const disposal of [...geometryDisposals, ...materialDisposals]) expect(disposal).toHaveBeenCalledOnce();
  });

  it.each([
    ["static", SceneViewer],
    ["dynamic", DynamicViewer],
    ["predictive", PredictiveViewer],
  ] as const)("does not let %s decorative background geometry alter camera or mission bounds", (_name, Viewer) => {
    const viewer = viewerHarness(Viewer);
    const scenario = {
      ...(Viewer === SceneViewer ? staticScenario : movingScenario),
      ...cityPresentation,
    };
    viewer.camera = new THREE.OrthographicCamera();
    viewer.controls = {
      target: new THREE.Vector3(),
      update: vi.fn(() => viewer.camera.lookAt(viewer.controls.target)),
    };
    viewer.renderer.setSize = vi.fn();
    // Exercise the real framing math; only the GPU renderer stays stubbed.
    viewer.setView = Viewer.prototype.setView;
    if (Viewer === SceneViewer) viewer.setScenario(scenario, new Set(), "smoothed");
    else viewer.setScenario(scenario);
    const context = viewer.content.getObjectByName("city-context") as THREE.Group;
    const backgroundProbe = new THREE.Mesh(
      new THREE.PlaneGeometry(80_000, 80_000), new THREE.MeshBasicMaterial(),
    );
    backgroundProbe.position.set(1_000_000, -1_000_000, 1_000_000);
    const snapshot = () => ({
      position: viewer.camera.position.toArray(),
      target: viewer.controls.target.toArray(),
      projection: [...viewer.camera.projectionMatrix.elements],
      bounds: { min: viewer.sceneBounds().min.toArray(), max: viewer.sceneBounds().max.toArray() },
      city: { min: viewer.cityBounds.min.toArray(), max: viewer.cityBounds.max.toArray() },
    });
    for (const scope of ["city", "mission"] as const) {
      for (const preset of ["isometric", "top"] as const) {
        viewer.setMapScope(scope);
        viewer.setView(preset);
        const before = snapshot();
        context.add(backgroundProbe);
        viewer.setView(preset);
        expect(snapshot()).toEqual(before);
        context.remove(backgroundProbe);
      }
    }
    backgroundProbe.geometry.dispose();
    backgroundProbe.material.dispose();
    viewer.clearContent();
  });

  it("keeps predictive city ground visible while the buildings layer is hidden or restored", () => {
    const viewer = viewerHarness(PredictiveViewer);
    const scenario = { ...movingScenario, ...cityPresentation };
    viewer.setScenario(scenario);
    const context = viewer.content.getObjectByName("city-context") as THREE.Group;
    const ground = context.getObjectByName("city-ground")!;
    expect(context.parent === viewer.cityLayer.group).toBe(true);
    expect(viewer.buildingsGroup.getObjectByName("city-context")).toBeUndefined();
    const cityBounds = viewer.cityBounds.clone();
    const missionBounds = viewer.sceneBounds().clone();
    for (const visible of [false, true, false]) {
      viewer.setLayerVisibility("buildings", visible);
      expect(viewer.buildingsGroup.visible).toBe(visible);
      expect(effectivelyVisible(context)).toBe(true);
      expect(effectivelyVisible(ground)).toBe(true);
      expect(viewer.cityBounds.equals(cityBounds)).toBe(true);
      expect(viewer.sceneBounds().equals(missionBounds)).toBe(true);
    }
    viewer.clearContent();
  });
});

describe("viewer-owned texture disposal", () => {
  it("deduplicates shared owned maps across materials and repeated subtree disposal", () => {
    const root = new THREE.Group();
    const texture = new THREE.Texture();
    texture.userData.viewerOwned = true;
    const disposed = vi.spyOn(texture, "dispose");
    root.add(
      new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({ map: texture })),
      new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshStandardMaterial({ map: texture })),
    );
    disposeRenderObject(root);
    disposeRenderObject(root);
    expect(disposed).toHaveBeenCalledOnce();
    expect(texture.userData.viewerDisposed).toBe(true);
  });

  it("does not dispose borrowed map or environment textures", () => {
    const borrowedMap = new THREE.Texture();
    const environment = new THREE.Texture();
    environment.userData.viewerOwned = true;
    const borrowedDisposal = vi.spyOn(borrowedMap, "dispose");
    const environmentDisposal = vi.spyOn(environment, "dispose");
    const mesh = new THREE.Mesh(
      new THREE.PlaneGeometry(1, 1),
      new THREE.MeshStandardMaterial({ map: borrowedMap, envMap: environment }),
    );
    const geometryDisposal = vi.spyOn(mesh.geometry, "dispose");
    const materialDisposal = vi.spyOn(mesh.material, "dispose");
    disposeRenderObject(mesh);
    expect(borrowedDisposal).not.toHaveBeenCalled();
    expect(environmentDisposal).not.toHaveBeenCalled();
    expect(borrowedMap.userData.viewerDisposed).toBeUndefined();
    expect(environment.userData.viewerDisposed).toBeUndefined();
    expect(geometryDisposal).toHaveBeenCalledOnce();
    expect(materialDisposal).toHaveBeenCalledOnce();
  });

  it("keeps untextured line geometry and material cleanup unchanged", () => {
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]),
      new THREE.LineBasicMaterial(),
    );
    const geometryDisposal = vi.spyOn(line.geometry, "dispose");
    const materialDisposal = vi.spyOn(line.material, "dispose");
    disposeRenderObject(line);
    expect(geometryDisposal).toHaveBeenCalledOnce();
    expect(materialDisposal).toHaveBeenCalledOnce();
  });
});
