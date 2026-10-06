import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { addZoneVisual, fitMapCamera, type ZoneVisual } from "./viewer-geometry";
import { cityMissionBounds } from "./city-scene";
import { enuToThree } from "./coordinates";
import { RetainedCity } from "./retained-city";
import { RouteOverview, routeColor, type OverviewRoute } from "./route-overview";
import { DroneFollow } from "./drone-follow";
import { positionEncounterCamera } from "./encounter-view";
import { createTrafficAircraft, sizeTrafficAircraft, updateTrafficAircraft } from "./traffic-aircraft";
import { sameSharedWorld } from "./shared-world";
import { disposeRenderObject } from "./render-resources";
import { createEndpointMarker, sceneLineStyle, vehicleGeometry } from "./scene-style";
import { addMissionTaskMarkers, avoidDroneMarkerOverlap, orientMissionTaskGates, sizeMissionTaskMarkers } from "./mission-tasks";
import { updateMapSurround } from "./map-surround";
import { configureMapNavigation, FrameRenderer, mapFramingPadding, positionTopOverview, positionWesternOverview } from "./map-navigation";

import type {
  DynamicFrame,
  DynamicScenario,
  StaticNoFlyZone,
  Vec3,
} from "./dynamic-schema";

type ViewPreset = "isometric" | "top" | "reset";


interface ThemePalette {
  background: number;
  foreground: number;
  movingSphere: number;
  plannedPath: number;
  executedPath: number;
  goal: number;
}

const LIGHT_PALETTE: ThemePalette = {
  background: 0xffffff,
  foreground: 0x285334,
  movingSphere: 0x98535b,
  plannedPath: 0x28a745,
  executedPath: 0x12722e,
  goal: 0x174f27,
};

function disposeObject(object: THREE.Object3D): void {
  disposeRenderObject(object);
}

export class DynamicViewer {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.OrthographicCamera();
  private readonly controls: OrbitControls;
  private readonly frames = new FrameRenderer(() => this.drawFrame());
  private readonly content = new THREE.Group();
  private readonly sun = new THREE.DirectionalLight(0xffffff, 2);
  private readonly resizeObserver: ResizeObserver;
  private readonly lineMaterials = new Set<LineMaterial>();
  private scenario: DynamicScenario | null = null;
  private temporaryZones = new Map<string, ZoneVisual>();
  private movingSpheres = new Map<string, THREE.Object3D>();
  private vehicle: THREE.Mesh | null = null;
  private plannedPath: Line2 | null = null;
  private executedPath: Line2 | null = null;
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
    this.scene.add(this.content);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xa3a3a3, 1.2));
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    this.sun.shadow.bias = -0.00015;
    this.sun.shadow.normalBias = 0.15;
    this.scene.add(this.sun, this.sun.target);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    configureMapNavigation(this.controls);
    this.controls.addEventListener("change", () => this.frames.request());
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
  }

  setScenario(scenario: DynamicScenario): void {
    const first = !this.scenario;
    const shared = sameSharedWorld(this.scenario?.mission, scenario.mission);
    this.scenario = scenario;
    if (shared) {
      const tasks = this.content.getObjectByName("mission-task-points");
      if (tasks) { tasks.removeFromParent(); disposeObject(tasks); }
      addMissionTaskMarkers(this.content, scenario.mission);
    } else this.buildScene();
    if (first) this.setView("isometric");
    else if (this.observationBounds && !this.follow?.routeId) this.observeEncounter(scenario.mission?.challenge?.focusPosition ?? scenario.start);
  }

  setRoutes(routes: OverviewRoute[], activeId: string): void {
    this.overview ??= new RouteOverview();
    this.scene.add(this.overview.group);
    this.overview.setRoutes(routes, activeId, this.container.clientWidth, this.container.clientHeight);
    orientMissionTaskGates(this.content, routes.find(route => route.id === activeId)?.points ?? [], routeColor(routes.findIndex(route => route.id === activeId)));
    if (this.vehicle) this.vehicle.visible = false;
    if (this.plannedPath) this.plannedPath.visible = false;
    if (this.executedPath) this.executedPath.visible = false;
    this.content.traverse(object => { if (object.name.startsWith("endpoint-")) object.visible = false; });
    this.render();
  }

  focusRoute(id: string): void { this.overview?.setFocus(id); this.render(); }
  setPlaying(playing: boolean): void { this.overview?.setPlaying(playing); this.render(); }
  get followedRouteId(): string | null { return this.follow?.routeId ?? null; }
  setTime(timeS: number): void { this.playbackTimeS = timeS; this.overview?.setTime(timeS); this.render(); }

  setFollowRoute(id: string | null): void {
    if (id === null) { this.follow?.stop(); this.resize(); return; }
    const drone = this.overview?.vehicle(id);
    if (!drone) return;
    this.follow ??= new DroneFollow(this.controls);
    this.follow.setBuildings(this.scenario?.buildings ?? []);
    this.follow.onChange = active => this.onFollowChange?.(active);
    this.follow.start(id, drone.position, this.overview!.heading(id, this.playbackTimeS), this.container.clientWidth, this.container.clientHeight);
    this.render();
  }

  setFrame(frame: DynamicFrame, timeS = frame.timeS, completedAtS = timeS): void {
    if (!this.scenario) return;
    this.playbackTimeS = timeS;
    this.overview?.setTime(timeS);
    const hazardTime = this.scenario.mission?.sharedWorld ? timeS : Math.min(timeS, completedAtS);
    let animatedShadows = this.vehicle?.castShadow ?? false;
    this.vehicle?.position.fromArray(enuToThree(frame.vehicle));

    const activeZones = new Set(this.overview
      ? this.scenario.temporaryNoFlyZones.filter(z => z.activeFromS <= hazardTime && hazardTime < z.activeUntilS).map(z => z.id)
      : frame.activeTemporaryZoneIds);
    for (const [id, visual] of this.temporaryZones) {
      animatedShadows ||= visual.fill.castShadow;
      const active = activeZones.has(id);
      visual.fill.visible = visual.outline.visible = active;
      visual.fill.material.opacity = active ? 0.18 : 0.025;
      visual.outline.traverse((child) => {
        if (child instanceof THREE.Line && child.material instanceof THREE.LineBasicMaterial) {
          child.material.opacity = active ? 0.6 : 0.18;
        }
      });
    }

    const states = new Map(frame.movingSpheres.map((state) => [state.id, state]));
    for (const [id, mesh] of this.movingSpheres) {
      animatedShadows ||= mesh.castShadow;
      const state = states.get(id);
      mesh.visible = state !== undefined;
      if (state) mesh.position.fromArray(enuToThree(state.position));
      if (this.overview) {
        const definition = this.scenario.movingSpheres.find(s => s.id === id);
        if (definition) { mesh.visible = true; updateTrafficAircraft(mesh, definition, hazardTime); }
      }
    }

    if (!this.overview) {
      this.replaceLine("planned", frame.path);
      this.replaceLine("executed", frame.executedPath);
    }
    if (animatedShadows) this.renderer.shadowMap.needsUpdate = true;
    this.render();
  }

  setMapScope(scope: "city" | "mission"): void {
    this.mapScope = scope;
    this.setView(this.viewPreset);
  }

  observeEncounter(position: Vec3): void {
    this.follow?.stop();
    this.observationBounds = positionEncounterCamera(this.camera, this.controls, position);
    this.resize();
  }

  setView(preset: ViewPreset): void {
    if (!this.scenario) return;
    this.observationBounds = undefined;
    this.follow?.stop();
    this.viewPreset = preset === "reset" ? "isometric" : preset;
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
  }

  dispose(): void {
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

  private get palette(): ThemePalette {
    return LIGHT_PALETTE;
  }

  private buildScene(): void {
    if (!this.scenario) return;
    this.clearContent();
    const palette = this.palette;
    this.scene.background = new THREE.Color(palette.background);
    this.cityLayer ??= new RetainedCity();
    this.cityBounds = this.cityLayer.mount(this.content, this.scenario, texture => {
      if (texture) this.renderer.initTexture(texture);
      this.frames.request();
    });

    for (const zone of this.scenario.staticNoFlyZones) {
      this.addZone(zone, "static");
    }
    for (const zone of this.scenario.temporaryNoFlyZones) {
      this.temporaryZones.set(zone.id, this.addZone(zone, "temporary"));
    }
    for (const definition of this.scenario.movingSpheres) {
      const mesh = this.scenario.city ? createTrafficAircraft(definition) : new THREE.Mesh(
        new THREE.SphereGeometry(definition.radiusM, 22, 14),
        new THREE.MeshStandardMaterial({
          color: palette.movingSphere,
          roughness: 0.75,
        }),
      );
      mesh.visible = false;
      mesh.receiveShadow = true;
      mesh.position.fromArray(enuToThree(definition.keyframes[0]!.position));
      this.movingSpheres.set(definition.id, mesh);
      this.content.add(mesh);
    }

    this.addEndpoint(this.scenario.start, "start");
    this.addEndpoint(this.scenario.goal, "goal");
    addMissionTaskMarkers(this.content, this.scenario.mission);
    this.vehicle = new THREE.Mesh(
      vehicleGeometry(Boolean(this.scenario.city)),
      new THREE.MeshStandardMaterial({
        color: palette.foreground,
        emissive: palette.background,
        emissiveIntensity: 0.16,
        depthTest: true,
        depthWrite: true,
      }),
    );
    this.vehicle.rotation.z = -Math.PI / 2;
    this.vehicle.position.fromArray(enuToThree(this.scenario.start));
    this.vehicle.name = "vehicle";
    this.vehicle.visible = !this.overview;
    this.vehicle.receiveShadow = true;
    this.content.add(this.vehicle);
    this.configureSun();
  }

  private clearContent(): void {
    this.temporaryZones.clear();
    this.movingSpheres.clear();
    this.vehicle = null;
    this.plannedPath = null;
    this.executedPath = null;
    this.lineMaterials.clear();
    for (const child of [...this.content.children]) {
      if (child === this.cityLayer?.group) continue;
      this.content.remove(child);
      disposeObject(child);
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

  private addZone(zone: StaticNoFlyZone, role: "static" | "temporary"): ZoneVisual {
    return addZoneVisual(this.content, zone, role === "temporary");
  }

  private addEndpoint(point: Vec3, role: "start" | "goal"): void {
    if (this.overview) return;
    const palette = this.palette;
    const marker = createEndpointMarker(role, Boolean(this.scenario?.city),
      role === "start" ? palette.executedPath : palette.goal,
      role === "start" ? palette.executedPath : palette.foreground, role === "start" ? 0.08 : 0.04);
    marker.position.fromArray(enuToThree(point));
    marker.name = `endpoint-${role}`;
    marker.receiveShadow = true;
    this.content.add(marker);
  }

  private replaceLine(role: "planned" | "executed", points: Vec3[]): void {
    const previous = role === "planned" ? this.plannedPath : this.executedPath;
    if (previous) {
      this.content.remove(previous);
      this.lineMaterials.delete(previous.material);
      disposeObject(previous);
    }
    if (points.length < 2) {
      if (role === "planned") this.plannedPath = null;
      else this.executedPath = null;
      return;
    }
    const palette = this.palette;
    const geometry = new LineGeometry();
    geometry.setPositions(points.flatMap((point) => enuToThree(point)));
    const material = new LineMaterial({
      ...sceneLineStyle(Boolean(this.scenario?.city), role === "planned" ? 2.8 : 3.6,
        role === "planned"),
      color: role === "planned" ? palette.plannedPath : palette.executedPath,
    });
    material.resolution.set(Math.max(1, this.container.clientWidth), Math.max(1, this.container.clientHeight));
    this.lineMaterials.add(material);
    const line = new Line2(geometry, material);
    line.name = `trajectory-${role}`;
    // Draw the executed prefix last only where it coincides with the planned path.
    // Both still test against the same opaque scene depth.
    line.renderOrder = role === "planned" ? 0 : 1;
    if (role === "planned") line.computeLineDistances();
    this.content.add(line);
    if (role === "planned") this.plannedPath = line;
    else this.executedPath = line;
  }

  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height, false);
    if (!this.follow?.resize(width, height))
      fitMapCamera(this.camera, this.sceneBounds(), width, height,
        this.observationBounds ? 1.12 : mapFramingPadding(this.mapScope, this.viewPreset === "isometric"));
    for (const material of this.lineMaterials) material.resolution.set(width, height);
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
      return cityMissionBounds(this.scenario, this.scenario.runs.flatMap((run) => run.frames.flatMap((frame) =>
        [frame.vehicle, ...frame.path, ...frame.executedPath])));
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
    for (const zone of [...this.scenario.staticNoFlyZones, ...this.scenario.temporaryNoFlyZones]) {
      bounds.expandByPoint(new THREE.Vector3(zone.center[0] - zone.radiusM, zone.zMinM, -zone.center[1] - zone.radiusM));
      bounds.expandByPoint(new THREE.Vector3(zone.center[0] + zone.radiusM, zone.zMaxM, -zone.center[1] + zone.radiusM));
    }
    bounds.expandByPoint(new THREE.Vector3(...enuToThree(this.scenario.start)));
    bounds.expandByPoint(new THREE.Vector3(...enuToThree(this.scenario.goal)));
    for (const sphere of this.scenario.movingSpheres) {
      for (const keyframe of sphere.keyframes) {
        const position = new THREE.Vector3(...enuToThree(keyframe.position));
        bounds.expandByPoint(position.clone().addScalar(sphere.radiusM));
        bounds.expandByPoint(position.clone().addScalar(-sphere.radiusM));
      }
    }
    for (const run of this.scenario.runs) {
      for (const frame of run.frames) {
        bounds.expandByPoint(new THREE.Vector3(...enuToThree(frame.vehicle)));
        for (const point of frame.path) bounds.expandByPoint(new THREE.Vector3(...enuToThree(point)));
        for (const point of frame.executedPath) bounds.expandByPoint(new THREE.Vector3(...enuToThree(point)));
      }
    }
    return bounds;
  }

  private render(): void { this.frames.request(); }

  private drawFrame(): void {
    const drone = this.follow?.routeId ? this.overview?.vehicle(this.follow.routeId) : undefined;
    if (drone && this.follow?.update(drone.position, this.overview!.heading(this.follow.routeId!, this.playbackTimeS), this.playbackTimeS)) this.frames.request();
    const camera = drone ? this.follow!.camera : this.camera;
    for (const aircraft of this.movingSpheres.values()) sizeTrafficAircraft(aircraft, camera, this.container.clientHeight);
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
