import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";

import { enuToThree } from "./coordinates";
import { addZoneVisual, fitMapCamera } from "./viewer-geometry";
import { cityMissionBounds } from "./city-scene";
import { RetainedCity } from "./retained-city";
import { RouteOverview, routeColor, type OverviewRoute } from "./route-overview";
import { DroneFollow } from "./drone-follow";
import { positionEncounterCamera } from "./encounter-view";
import { sameSharedWorld } from "./shared-world";
import { disposeRenderObject } from "./render-resources";
import { createEndpointMarker, sceneLineStyle } from "./scene-style";
import { addMissionTaskMarkers, avoidDroneMarkerOverlap, orientMissionTaskGates, sizeMissionTaskMarkers } from "./mission-tasks";
import { updateMapSurround } from "./map-surround";
import { configureMapNavigation, FrameRenderer, mapFramingPadding, positionTopOverview, positionWesternOverview } from "./map-navigation";
import { MapCameraMotion } from "./map-camera-motion";
import type { RoutePick } from "./route-picking";
import type { DemoScenario, PlannerId, Vec3 } from "./schema";

type PathMode = "raw" | "smoothed";
type ViewPreset = "isometric" | "top" | "reset";

const PATH_STYLES: Record<
  PlannerId,
  { color: number; dashed: boolean; dashScale: number }
> = {
  "astar-3d": { color: 0x2aa84c, dashed: false, dashScale: 1 },
  "lazy-theta-star": { color: 0x0d7062, dashed: true, dashScale: 1.2 },
  "rrt-star": { color: 0x83a923, dashed: true, dashScale: 0.45 },
};

function disposeObject(object: THREE.Object3D): void {
  disposeRenderObject(object);
}

export class SceneViewer {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.OrthographicCamera();
  private readonly controls: OrbitControls;
  private motion?: MapCameraMotion;
  private readonly frames = new FrameRenderer(() => this.drawFrame());
  private readonly content = new THREE.Group();
  private readonly paths = new THREE.Group();
  private readonly sun = new THREE.DirectionalLight(0xffffff, 2);
  private readonly lineMaterials: LineMaterial[] = [];
  private readonly resizeObserver: ResizeObserver;
  private scenario: DemoScenario | null = null;
  private mode: PathMode = "smoothed";
  private visiblePlanners = new Set<PlannerId>();
  private cityBounds = new THREE.Box3();
  private mapScope: "city" | "mission" = "city";
  private viewPreset: ViewPreset = "isometric";
  private cityLayer?: RetainedCity;
  private overview?: RouteOverview;
  private follow?: DroneFollow;
  private playbackTimeS = 0;
  private observationBounds?: THREE.Box3;
  onFollowChange?: (enabled: boolean) => void;

  constructor(private readonly container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.shadowMap.autoUpdate = false;
    this.container.append(this.renderer.domElement);
    this.scene.background = new THREE.Color(0xffffff);
    this.scene.add(this.content);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xa3a3a3, 1.2));
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    this.sun.shadow.bias = -0.00015;
    this.sun.shadow.normalBias = 0.15;
    this.scene.add(this.sun, this.sun.target);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    configureMapNavigation(this.controls);
    this.motion = new MapCameraMotion(this.camera, this.controls, () => this.frames.request());
    this.controls.addEventListener("start", () => this.motion?.cancel());
    this.controls.addEventListener("change", () => this.frames.request());
    this.resizeObserver = new ResizeObserver(() => { this.motion?.cancel(); this.resize(); });
    this.resizeObserver.observe(container);
  }

  setScenario(scenario: DemoScenario, visible: Set<PlannerId>, mode: PathMode): void {
    const first = !this.scenario;
    if (sameSharedWorld(this.scenario?.mission, scenario.mission)) {
      this.scenario = scenario;
      this.visiblePlanners = new Set(visible); this.mode = mode;
      const tasks = this.content.getObjectByName("mission-task-points");
      if (tasks) { tasks.removeFromParent(); disposeObject(tasks); }
      addMissionTaskMarkers(this.content, scenario.mission);
      this.addPaths();
      if (this.observationBounds && !this.follow?.routeId) this.observeEncounter(scenario.mission?.challenge?.focusPosition ?? scenario.start);
      this.render();
      return;
    }
    this.clearContent();
    this.scenario = scenario;
    this.visiblePlanners = new Set(visible);
    this.mode = mode;
    this.content.add(this.paths);
    this.cityLayer ??= new RetainedCity();
    this.cityBounds = this.cityLayer.mount(this.content, scenario, texture => {
      if (texture) this.renderer.initTexture(texture);
      this.frames.request();
    });
    scenario.noFlyZones.forEach((zone) => this.addNoFlyZone(zone));
    this.addEndpoint(scenario.start, "start");
    this.addEndpoint(scenario.goal, "goal");
    addMissionTaskMarkers(this.content, scenario.mission);
    this.addPaths();
    this.configureSun();
    if (first) this.setView("isometric");
    else if (this.observationBounds && !this.follow?.routeId) this.observeEncounter(scenario.mission?.challenge?.focusPosition ?? scenario.start);
    this.render();
  }

  setRoutes(routes: OverviewRoute[], activeId: string): void {
    this.overview ??= new RouteOverview();
    this.scene.add(this.overview.group);
    this.overview.setRoutes(routes, activeId, this.container.clientWidth, this.container.clientHeight);
    orientMissionTaskGates(this.content, routes.find(route => route.id === activeId)?.points ?? [], routeColor(routes.findIndex(route => route.id === activeId)));
    this.paths.visible = false;
    this.content.traverse(object => { if (object.name.startsWith("endpoint-")) object.visible = false; });
    this.render();
  }

  focusRoute(id: string): void { this.overview?.setFocus(id); this.render(); }
  setRouteSelection(id: string | null): void { this.overview?.setSelection(id); this.render(); }
  pickRouteAt(clientX: number, clientY: number, radius = 7): RoutePick | null {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const camera = this.follow?.routeId ? this.follow.camera : this.camera;
    return this.overview?.pick(camera, clientX - rect.left, clientY - rect.top, rect.width, rect.height,
      point => this.cityLayer?.blocksSight(camera, point) ?? false, radius) ?? null;
  }
  setPlaying(playing: boolean): void { this.overview?.setPlaying(playing); this.render(); }
  get followedRouteId(): string | null { return this.follow?.routeId ?? null; }

  setTime(timeS: number): void {
    this.playbackTimeS = timeS;
    this.overview?.setTime(timeS);
    this.render();
  }

  setFollowRoute(id: string | null): void {
    this.motion?.cancel();
    if (id === null) { this.follow?.stop(); this.resize(); return; }
    const drone = this.overview?.vehicle(id);
    if (!drone) return;
    this.follow ??= new DroneFollow(this.controls);
    this.follow.setBuildings(this.scenario?.buildings ?? []);
    this.follow.onChange = enabled => this.onFollowChange?.(enabled);
    this.follow.start(id, drone.position, this.overview!.heading(id, this.playbackTimeS), this.container.clientWidth, this.container.clientHeight);
    this.render();
  }

  setPathMode(mode: PathMode): void {
    if (!this.scenario || this.mode === mode) return;
    this.mode = mode;
    this.addPaths();
    this.render();
  }

  setPlannerVisibility(planners: Set<PlannerId>): void {
    if (!this.scenario) return;
    this.visiblePlanners = new Set(planners);
    this.addPaths();
    this.render();
  }

  setMapScope(scope: "city" | "mission"): void {
    this.mapScope = scope;
    this.setView(this.viewPreset);
  }

  observeEncounter(position: Vec3): void {
    this.follow?.stop();
    const move = (): void => { this.observationBounds = positionEncounterCamera(this.camera, this.controls, position); this.resize(); };
    if (this.motion) this.motion.transition(move); else move();
  }

  setView(preset: ViewPreset): void {
    if (!this.scenario) return;
    this.observationBounds = undefined;
    this.follow?.stop();
    this.viewPreset = preset === "reset" ? "isometric" : preset;
    const move = (): void => {
    const bounds = this.sceneBounds();
    const center = bounds.getCenter(new THREE.Vector3());
    const size = bounds.getSize(new THREE.Vector3());
    const span = Math.max(size.x, size.y, size.z);
    if (preset === "top") {
      positionTopOverview(this.camera, center, span);
    } else {
      positionWesternOverview(this.camera, center, span);
    }
    this.camera.zoom = 1;
    this.controls.target.copy(center);
    this.controls.update();
    this.resize();
    };
    if (this.motion) this.motion.transition(move); else move();
  }

  dispose(): void {
    this.motion?.cancel();
    this.follow?.stop();
    this.frames.cancel();
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.clearContent();
    this.cityLayer?.dispose();
    this.overview?.dispose();
    this.sun.shadow.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  private clearContent(): void {
    this.lineMaterials.length = 0;
    for (const child of [...this.content.children]) {
      if (child === this.cityLayer?.group) continue;
      this.content.remove(child);
      disposeObject(child);
    }
    this.paths.clear();
    this.cityBounds.makeEmpty();
  }

  private configureSun(): void {
    const bounds = this.cityBounds.clone().union(this.missionBounds());
    const center = bounds.getCenter(new THREE.Vector3());
    const size = bounds.getSize(new THREE.Vector3());
    const span = Math.max(size.x, size.y, size.z, 1);
    this.sun.position.set(center.x - span * 0.6, center.y + span * 1.2, center.z + span * 0.7);
    this.sun.target.position.copy(center);
    Object.assign(this.sun.shadow.camera, {
      left: -span,
      right: span,
      top: span,
      bottom: -span,
      near: 0.1,
      far: span * 4,
    });
    this.sun.shadow.camera.updateProjectionMatrix();
    this.renderer.shadowMap.needsUpdate = true;
  }

  private addNoFlyZone(zone: DemoScenario["noFlyZones"][number]): void {
    addZoneVisual(this.content, zone);
  }

  private addEndpoint(point: Vec3, kind: "start" | "goal"): void {
    if (this.overview) return;
    const marker = createEndpointMarker(kind, Boolean(this.scenario?.city), kind === "start" ? 0x28a745 : 0x009b87);
    marker.position.fromArray(enuToThree(point));
    marker.name = `endpoint-${kind}`;
    marker.receiveShadow = true;
    this.content.add(marker);
  }

  private addPaths(): void {
    if (!this.scenario) return;
    if (this.overview) return;
    for (const child of [...this.paths.children]) {
      this.paths.remove(child);
      disposeObject(child);
    }
    this.lineMaterials.length = 0;
    for (const result of this.scenario.results) {
      if (!result.paths || !this.visiblePlanners.has(result.plannerId)) continue;
      const path = result.paths[this.mode];
      const positions = path.flatMap((point) => enuToThree(point));
      const style = PATH_STYLES[result.plannerId];
      const geometry = new LineGeometry();
      geometry.setPositions(positions);
      const material = new LineMaterial({
        ...sceneLineStyle(Boolean(this.scenario.city), 3.2, style.dashed),
        color: style.color,
        dashSize: (this.scenario.city ? 18 : 7) * style.dashScale,
        gapSize: this.scenario.city ? 10 : 4,
      });
      material.resolution.set(Math.max(1, this.container.clientWidth), Math.max(1, this.container.clientHeight));
      this.lineMaterials.push(material);
      const line = new Line2(geometry, material);
      line.name = `trajectory-${result.plannerId}`;
      line.computeLineDistances();
      this.paths.add(line);
    }
  }

  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height, false);
    if (!this.follow?.resize(width, height))
      fitMapCamera(this.camera, this.sceneBounds(), width, height,
        this.observationBounds ? 1.12 : mapFramingPadding(this.mapScope, this.viewPreset === "isometric"));
    this.lineMaterials.forEach((material) => material.resolution.set(width, height));
    this.overview?.resize(width, height);
    this.render();
  }

  private sceneBounds(): THREE.Box3 {
    if (this.observationBounds) return this.observationBounds.clone();
    if (!this.scenario) return new THREE.Box3(new THREE.Vector3(-50, 0, -50), new THREE.Vector3(50, 50, 50));
    const bounds = this.missionBounds();
    if (this.mapScope === "city") bounds.union(this.cityBounds);
    if (this.mapScope === "city" && this.overview) bounds.union(this.overview.bounds);
    return bounds;
  }

  private missionBounds(): THREE.Box3 {
    if (!this.scenario) return new THREE.Box3();
    if (this.scenario.city) {
      return cityMissionBounds(this.scenario, this.scenario.results.flatMap((result) => result.paths
        ? [...result.paths.raw, ...result.paths.smoothed] : []));
    }
    const { min, max } = this.scenario.bounds;
    const bounds = new THREE.Box3(
      new THREE.Vector3(min[0], min[2], -max[1]),
      new THREE.Vector3(max[0], max[2], -min[1]),
    );
    for (const building of this.scenario.buildings) {
      bounds.expandByPoint(new THREE.Vector3(building.min[0], building.min[2], -building.max[1]));
      bounds.expandByPoint(new THREE.Vector3(building.max[0], building.max[2], -building.min[1]));
    }
    for (const zone of this.scenario.noFlyZones) {
      bounds.expandByPoint(new THREE.Vector3(zone.center[0] - zone.radiusM, zone.zMinM, -zone.center[1] - zone.radiusM));
      bounds.expandByPoint(new THREE.Vector3(zone.center[0] + zone.radiusM, zone.zMaxM, -zone.center[1] + zone.radiusM));
    }
    bounds.expandByPoint(new THREE.Vector3(...enuToThree(this.scenario.start)));
    bounds.expandByPoint(new THREE.Vector3(...enuToThree(this.scenario.goal)));
    for (const result of this.scenario.results) {
      for (const path of result.paths ? [result.paths.raw, result.paths.smoothed] : []) {
        for (const point of path) bounds.expandByPoint(new THREE.Vector3(...enuToThree(point)));
      }
    }
    return bounds;
  }

  private render(): void { this.frames.request(); }

  private drawFrame(): void {
    if (this.motion?.step()) this.frames.request();
    const drone = this.follow?.routeId ? this.overview?.vehicle(this.follow.routeId) : undefined;
    if (drone && this.follow?.update(drone.position, this.overview!.heading(this.follow.routeId!, this.playbackTimeS), this.playbackTimeS)) this.frames.request();
    const camera = drone ? this.follow!.camera : this.camera;
    this.overview?.updateSymbols(camera, this.container.clientHeight, this.follow?.routeId);
    sizeMissionTaskMarkers(this.content, camera, this.container.clientHeight, Boolean(drone) || Boolean(this.observationBounds));
    avoidDroneMarkerOverlap(this.content, this.overview?.vehicle(this.scenario?.id ?? ""));
    const pixelRatio = this.renderer.getPixelRatio();
    updateMapSurround(this.content, camera, this.container.clientWidth * pixelRatio,
      this.container.clientHeight * pixelRatio, this.renderer.capabilities.getMaxAnisotropy());
    const background = this.scene.background;
    if (drone) this.scene.background = this.follow!.sky;
    this.renderer.render(this.scene, camera);
    this.scene.background = background;
  }
}
