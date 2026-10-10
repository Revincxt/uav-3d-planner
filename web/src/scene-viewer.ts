import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

import { enuToThree } from "./coordinates";
import { addZoneVisual, fitMapCamera } from "./viewer-geometry";
import { cityMissionBounds } from "./city-scene";
import { RetainedCity } from "./retained-city";
import { RouteOverview, routeColor, type OverviewRoute } from "./route-overview";
import { DroneFollow } from "./drone-follow";
import { positionEncounterCamera } from "./encounter-view";
import { sameSharedWorld } from "./shared-world";
import { disposeRenderObject } from "./render-resources";
import { addMissionTaskMarkers, avoidDroneMarkerOverlap, orientMissionTaskGates, sizeMissionTaskMarkers } from "./mission-tasks";
import { updateMapSurround } from "./map-surround";
import { configureMapNavigation, FrameRenderer, mapFramingPadding, positionTopOverview, positionWesternOverview } from "./map-navigation";
import { MapCameraMotion } from "./map-camera-motion";
import type { RoutePick } from "./route-picking";
import type { DemoScenario, Vec3 } from "./schema";

type ViewPreset = "isometric" | "top" | "reset";


export class SceneViewer {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.OrthographicCamera();
  private readonly controls: OrbitControls;
  private motion?: MapCameraMotion;
  private readonly frames = new FrameRenderer(() => this.drawFrame());
  private readonly content = new THREE.Group();
  private readonly sun = new THREE.DirectionalLight(0xffffff, 2);
  private readonly resizeObserver: ResizeObserver;
  private scenario: DemoScenario | null = null;
  private cityBounds = new THREE.Box3();
  private readonly routeBounds = new WeakMap<DemoScenario, THREE.Box3>();
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

  setScenario(scenario: DemoScenario): void {
    const first = !this.scenario;
    if (sameSharedWorld(this.scenario?.mission, scenario.mission)) {
      this.scenario = scenario;
      const tasks = this.content.getObjectByName("mission-task-points");
      if (tasks) { tasks.removeFromParent(); disposeRenderObject(tasks); }
      addMissionTaskMarkers(this.content, scenario.mission);
      if (this.observationBounds && !this.follow?.routeId) this.observeEncounter(scenario.mission?.challenge?.focusPosition ?? scenario.start);
      this.render();
      return;
    }
    this.clearContent();
    this.scenario = scenario;
    this.cityLayer ??= new RetainedCity();
    this.cityBounds = this.cityLayer.mount(this.content, scenario, texture => {
      if (texture) this.renderer.initTexture(texture);
      this.frames.request();
    });
    scenario.noFlyZones.forEach((zone) => addZoneVisual(this.content, zone));
    addMissionTaskMarkers(this.content, scenario.mission);
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
    for (const child of [...this.content.children]) {
      if (child === this.cityLayer?.group) continue;
      this.content.remove(child);
      disposeRenderObject(child);
    }
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



  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height, false);
    if (!this.follow?.resize(width, height))
      fitMapCamera(this.camera, this.sceneBounds(), width, height,
        this.observationBounds ? 1.12 : mapFramingPadding(this.viewPreset === "isometric"));
    this.overview?.resize(width, height);
    this.render();
  }

  private sceneBounds(): THREE.Box3 {
    if (this.observationBounds) return this.observationBounds.clone();
    if (!this.scenario) return new THREE.Box3(new THREE.Vector3(-50, 0, -50), new THREE.Vector3(50, 50, 50));
    const bounds = this.missionBounds();
    bounds.union(this.cityBounds);
    if (this.overview) bounds.union(this.overview.bounds);
    return bounds;
  }

  private missionBounds(): THREE.Box3 {
    if (!this.scenario) return new THREE.Box3();
    if (this.scenario.city) {
      let bounds = this.routeBounds.get(this.scenario);
      if (!bounds) {
        const scenario = this.scenario;
        function* points() {
          for (const result of scenario.results) if (result.paths) {
            yield* result.paths.raw; yield* result.paths.smoothed;
          }
        }
        bounds = cityMissionBounds(scenario, points()); this.routeBounds.set(scenario, bounds);
      }
      return bounds.clone();
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
