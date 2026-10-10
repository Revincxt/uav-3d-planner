import * as THREE from "three";
import { MapCameraMotion } from "./map-camera-motion";
import type { RoutePick } from "./route-picking";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { addZoneVisual, fitMapCamera, ringPoints, type ZoneVisual } from "./viewer-geometry";
import { cityMissionBounds } from "./city-scene";
import { enuToThree } from "./coordinates";
import { RetainedCity } from "./retained-city";
import { RouteOverview, pathPrefix, routeColor, timedPosition, type OverviewRoute } from "./route-overview";
import { DroneFollow } from "./drone-follow";
import { positionEncounterCamera } from "./encounter-view";
import { createTrafficAircraft, sizeTrafficAircraft, updateTrafficAircraft } from "./traffic-aircraft";
import { revealReplayWindow } from "./replay-line";
import { sameSharedWorld } from "./shared-world";
import { finalFlight, type FinalFlight } from "./final-flight";
import { disposeRenderObject } from "./render-resources";
import { createEndpointMarker, sceneLineStyle, vehicleGeometry } from "./scene-style";
import { addMissionTaskMarkers, avoidDroneMarkerOverlap, orientMissionTaskGates, sizeMissionTaskMarkers } from "./mission-tasks";
import { updateMapSurround } from "./map-surround";
import { configureMapNavigation, FrameRenderer, mapFramingPadding, MIN_MAP_ELEVATION_RAD, positionTopOverview, positionWesternOverview } from "./map-navigation";

import type {
  MovingSphereDefinition,
  PredictiveMinimumSeparationWitness,
  PredictivePathMode,
  PredictiveRun,
  PredictiveScenario,
  StaticNoFlyZone,
  TimedWaypoint,
  Vec3,
} from "./predictive-schema";

export type ViewPreset = "isometric" | "xy" | "xz" | "yz" | "fit";
export type ViewerLayer = "buildings" | "zones" | "dynamic";

const WITNESS_TIME_TOLERANCE_S = 1e-7;

export interface MinimumSeparationEvidence {
  witness: PredictiveMinimumSeparationWitness;
  safetyEnvelopeRadiusM: number;
  connectorDashed: boolean;
}

export function minimumSeparationEvidence(
  scenario: Pick<PredictiveScenario, "constraints">,
  run: Pick<PredictiveRun, "plannerMetrics" | "geometryMetrics" | "executionMetrics">,
  mode: PredictivePathMode,
): MinimumSeparationEvidence | null {
  const metrics =
    mode === "raw"
      ? run.plannerMetrics
      : mode === "geometry"
        ? run.geometryMetrics
        : run.executionMetrics;
  const witness = metrics?.minimumSeparationWitness ?? null;
  if (witness === null) return null;
  return {
    witness,
    safetyEnvelopeRadiusM:
      scenario.constraints.vehicleRadiusM + witness.declaredSafetyMarginM,
    connectorDashed: !witness.exact,
  };
}

export function isMinimumSeparationEvidenceTime(
  evidence: MinimumSeparationEvidence | null,
  timeS: number,
): boolean {
  return (
    evidence !== null &&
    Math.abs(timeS - evidence.witness.timeS) <= WITNESS_TIME_TOLERANCE_S
  );
}


interface ThemePalette {
  background: number;
  foreground: number;
  movingSphere: number;
  witnessVehicle: number;
  witnessSurface: number;
  witnessConnector: number;
  safetyEnvelope: number;
  plannedPath: number;
  executedPath: number;
  goal: number;
}

const LIGHT_PALETTE: ThemePalette = {
  background: 0xffffff,
  foreground: 0x313b35,
  movingSphere: 0x98535b,
  witnessVehicle: 0x12722e,
  witnessSurface: 0xcc4545,
  witnessConnector: 0xb67d3f,
  safetyEnvelope: 0xd79445,
  plannedPath: 0x28a745,
  executedPath: 0x12722e,
  goal: 0x3f4540,
};

function disposeObject(object: THREE.Object3D): void {
  disposeRenderObject(object);
}

function movingPosition(definition: MovingSphereDefinition, timeS: number): Vec3 {
  return timedPosition(definition.keyframes, timeS);
}

function flattenPoints(points: Vec3[]): number[] {
  return points.flatMap((point) => enuToThree(point));
}


export class PredictiveViewer {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.OrthographicCamera();
  private readonly controls: OrbitControls;
  private motion?: MapCameraMotion;
  private readonly frames = new FrameRenderer(() => this.drawFrame());
  private readonly content = new THREE.Group();
  private readonly buildingsGroup = new THREE.Group();
  private readonly zonesGroup = new THREE.Group();
  private readonly dynamicGroup = new THREE.Group();
  private readonly primaryPathGroup = new THREE.Group();
  private readonly evidenceGroup = new THREE.Group();
  private readonly resizeObserver: ResizeObserver;
  private readonly layerVisibility: Record<ViewerLayer, boolean> = {
    buildings: true,
    zones: true,
    dynamic: true,
  };
  private scenario: PredictiveScenario | null = null;
  private run: PredictiveRun | null = null;
  private flight: FinalFlight | null = null;
  private currentTimeS = 0;
  private currentView: ViewPreset = "isometric";
  private mapScope: "city" | "mission" = "city";
  private cityBounds = new THREE.Box3();
  private temporaryZones = new Map<string, ZoneVisual>();
  private movingSpheres = new Map<string, THREE.Object3D>();
  private trafficForecasts = new Map<string, Line2>();
  private forecastWindows = new Map<string, ReturnType<typeof revealReplayWindow>>();
  private observationBounds?: THREE.Box3;
  private lineMaterials = new Set<LineMaterial>();
  private vehicle: THREE.Mesh | null = null;
  private primaryPath: Line2 | null = null;
  private executedPath: Line2 | null = null;
  private cityLayer?: RetainedCity;
  private overview?: RouteOverview;
  private follow?: DroneFollow;
  private playbackTimeS = 0;
  onFollowChange?: (enabled: boolean) => void;
  private sun?: THREE.DirectionalLight;

  constructor(private readonly container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.shadowMap.autoUpdate = false;
    this.container.append(this.renderer.domElement);
    this.content.add(
      this.buildingsGroup,
      this.zonesGroup,
      this.dynamicGroup,
      this.primaryPathGroup,
      this.evidenceGroup,
    );
    this.scene.add(this.content);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    configureMapNavigation(this.controls);
    this.motion = new MapCameraMotion(this.camera, this.controls, () => this.frames.request());
    this.controls.addEventListener("start", () => this.motion?.cancel());
    this.controls.addEventListener("change", () => this.frames.request());
    this.resizeObserver = new ResizeObserver(() => { this.motion?.cancel(); this.resize(); });
    this.resizeObserver.observe(container);
  }

  setScenario(scenario: PredictiveScenario): void {
    const first = !this.scenario;
    const shared = sameSharedWorld(this.scenario?.mission, scenario.mission);
    this.scenario = scenario;
    this.run = null;
    this.flight = null;
    this.currentTimeS = 0;
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
    if (this.primaryPath) this.primaryPath.visible = false;
    if (this.executedPath) this.executedPath.visible = false;
    if (this.vehicle) this.vehicle.visible = false;
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

  setFollowRoute(id: string | null): void {
    this.motion?.cancel();
    if (id === null) { this.follow?.stop(); this.resize(); return; }
    const drone = this.overview?.vehicle(id);
    if (!drone) return;
    this.follow ??= new DroneFollow(this.controls);
    this.follow.setBuildings(this.scenario?.buildings ?? []);
    this.follow.onChange = active => this.onFollowChange?.(active);
    this.follow.start(id, drone.position, this.overview!.heading(id, this.playbackTimeS), this.container.clientWidth, this.container.clientHeight);
    this.render();
  }

  setRun(run: PredictiveRun): void {
    const flight = finalFlight(run);
    this.run = run;
    this.flight = flight;
    this.replacePathLines();
    this.replaceMinimumSeparationEvidence();
    this.setTime(0);
  }

  setLayerVisibility(layer: ViewerLayer, visible: boolean): void {
    this.layerVisibility[layer] = visible;
    this.updateLayerVisibility();
    this.render();
  }

  setTime(timeS: number): void {
    if (!this.scenario || !this.run) return;
    this.playbackTimeS = timeS;
    this.overview?.setTime(timeS);
    let animatedShadows = Boolean(
      this.vehicle?.castShadow && this.vehicle.visible && this.primaryPathGroup.visible,
    );
    const path = this.activePath;
    const duration = path.at(-1)!.timeS;
    this.currentTimeS = Math.max(0, Math.min(timeS, duration));
    const hazardTime = this.scenario.mission?.sharedWorld ? Math.max(0, timeS) : this.currentTimeS;
    const vehiclePosition = timedPosition(path, this.currentTimeS);
    this.vehicle?.position.fromArray(enuToThree(vehiclePosition));
    this.orientVehicle(path, this.currentTimeS);

    for (const zone of this.scenario.temporaryNoFlyZones) {
      const visual = this.temporaryZones.get(zone.id);
      if (!visual) continue;
      animatedShadows ||= this.zonesGroup.visible && visual.fill.castShadow;
      const active = zone.activeFromS <= hazardTime && hazardTime < zone.activeUntilS;
      visual.fill.visible = visual.outline.visible = active;
      visual.fill.material.opacity = active ? 0.18 : 0.025;
      visual.outline.traverse((child) => {
        if (child instanceof THREE.Line && child.material instanceof THREE.LineBasicMaterial) {
          child.material.opacity = active ? 0.72 : 0.2;
        }
      });
    }

    for (const definition of this.scenario.movingSpheres) {
      const forecast = this.trafficForecasts?.get(definition.id);
      if (forecast) {
        const window = this.forecastWindows?.get(definition.id);
        const end = Math.min(hazardTime + 20, definition.keyframes.at(-1)!.timeS);
        forecast.visible = this.run.predictive && (Boolean(this.scenario.mission?.sharedWorld) || this.currentTimeS < duration) && end > hazardTime;
        if (window) { window.start.value = hazardTime; window.end.value = end; }
      }
      const mesh = this.movingSpheres.get(definition.id);
      if (mesh) {
        mesh.position.fromArray(enuToThree(movingPosition(definition, hazardTime)));
        if (this.scenario.city) updateTrafficAircraft(mesh, definition, hazardTime);
        if (this.dynamicGroup.visible) {
          mesh.traverseVisible((child) => { animatedShadows ||= child.castShadow; });
        }
      }
    }

    if (!this.overview) this.executedPath = this.replaceLine(
      this.primaryPathGroup,
      this.executedPath,
      pathPrefix(path, this.currentTimeS),
      this.palette.executedPath,
      3.7,
      false,
      1,
      1,
    );
    this.evidenceGroup.visible = isMinimumSeparationEvidenceTime(
      minimumSeparationEvidence(this.scenario, this.run, "execution"),
      this.currentTimeS,
    );
    if (animatedShadows) this.renderer.shadowMap.needsUpdate = true;
    this.render();
  }

  setView(preset: ViewPreset): void {
    if (!this.scenario) return;
    this.observationBounds = undefined;
    this.follow?.stop();
    if (preset !== "fit") this.currentView = preset;
    const move = (): void => {
    const bounds = this.sceneBounds();
    const center = bounds.getCenter(new THREE.Vector3());
    const extent = bounds.getSize(new THREE.Vector3());
    const span = Math.max(extent.x, extent.y, extent.z);
    if (this.currentView === "xy") {
      positionTopOverview(this.camera, center, span, 2.2);
    } else if (this.currentView === "xz") {
      this.camera.position.set(center.x, center.y + Math.tan(MIN_MAP_ELEVATION_RAD) * span * 2.2, center.z + span * 2.2);
      this.camera.up.set(0, 1, 0);
    } else if (this.currentView === "yz") {
      this.camera.position.set(center.x - span * 2.2, center.y + Math.tan(MIN_MAP_ELEVATION_RAD) * span * 2.2, center.z);
      this.camera.up.set(0, 1, 0);
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

  setMapScope(scope: "city" | "mission"): void {
    if (scope === this.mapScope && !this.follow?.routeId) return;
    this.mapScope = scope;
    this.setView("fit");
  }

  observeEncounter(position: Vec3): void {
    this.follow?.stop();
    const move = (): void => { this.observationBounds = positionEncounterCamera(this.camera, this.controls, position); this.resize(); };
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
    this.sun?.shadow.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  private get palette(): ThemePalette {
    return LIGHT_PALETTE;
  }

  private get activePath(): TimedWaypoint[] {
    return this.flight?.path ?? [];
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
    if (!this.sun) {
      this.scene.add(new THREE.HemisphereLight(0xffffff, 0xa3a3a3, 1.2));
      this.sun = new THREE.DirectionalLight(0xffffff, 2);
      this.scene.add(this.sun);
    }
    const sun = this.sun;
    const lightTarget = this.cityBounds.getCenter(new THREE.Vector3());
    const cityExtent = this.cityBounds.getSize(new THREE.Vector3());
    const sceneSpan = Math.max(cityExtent.x, cityExtent.z);
    sun.position.set(
      lightTarget.x - sceneSpan * 0.6,
      lightTarget.y + sceneSpan * 1.2,
      lightTarget.z + sceneSpan * 0.7,
    );
    sun.target.position.copy(lightTarget);
    this.content.add(sun.target);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.bias = -0.0004;
    sun.shadow.camera.left = -sceneSpan;
    sun.shadow.camera.right = sceneSpan;
    sun.shadow.camera.top = sceneSpan;
    sun.shadow.camera.bottom = -sceneSpan;
    sun.shadow.camera.far = sceneSpan * 5;
    this.scene.add(sun);


    for (const zone of this.scenario.staticNoFlyZones) this.addZone(zone, "static");
    for (const zone of this.scenario.temporaryNoFlyZones) {
      this.temporaryZones.set(zone.id, this.addZone(zone, "temporary"));
    }
    for (const definition of this.scenario.movingSpheres) this.addMovingSphere(definition);

    this.addEndpoint(this.scenario.start, "start");
    this.addEndpoint(this.scenario.goal, "goal");
    addMissionTaskMarkers(this.content, this.scenario.mission);
    this.vehicle = new THREE.Mesh(
      vehicleGeometry(Boolean(this.scenario.city)),
      new THREE.MeshStandardMaterial({
        color: palette.foreground,
        emissive: palette.background,
        emissiveIntensity: 0.12,
        depthTest: true,
        depthWrite: true,
      }),
    );
    this.vehicle.position.fromArray(enuToThree(this.scenario.start));
    this.vehicle.name = "vehicle";
    this.vehicle.visible = !this.overview;
    this.vehicle.receiveShadow = true;
    this.vehicle.castShadow = true;
    this.primaryPathGroup.add(this.vehicle);
    this.updateLayerVisibility();
    this.renderer.shadowMap.needsUpdate = true;
  }

  private clearContent(): void {
    this.trafficForecasts?.clear();
    this.forecastWindows?.clear();
    this.temporaryZones.clear();
    this.movingSpheres.clear();
    this.lineMaterials.clear();
    this.vehicle = null;
    this.primaryPath = null;
    this.executedPath = null;
    for (const group of [
      this.buildingsGroup,
      this.zonesGroup,
      this.dynamicGroup,
      this.primaryPathGroup,
      this.evidenceGroup,
    ]) {
      for (const child of [...group.children]) {
        group.remove(child);
        disposeObject(child);
      }
    }
    for (const child of [...this.content.children]) {
      if (child === this.cityLayer?.group) continue;
      if (![
        this.buildingsGroup,
        this.zonesGroup,
        this.dynamicGroup,
        this.primaryPathGroup,
        this.evidenceGroup,
      ].includes(child as THREE.Group)) {
        this.content.remove(child);
        disposeObject(child);
      }
    }
  }

  private addZone(zone: StaticNoFlyZone, role: "static" | "temporary"): ZoneVisual {
    return addZoneVisual(this.zonesGroup, zone, role === "temporary");
  }

  private addMovingSphere(definition: MovingSphereDefinition): void {
    if (this.scenario?.city) {
      const aircraft = createTrafficAircraft(definition);
      updateTrafficAircraft(aircraft, definition, 0);
      this.movingSpheres.set(definition.id, aircraft);
      this.dynamicGroup.add(aircraft);
      const forecast = this.replaceLine(this.dynamicGroup, null,
        definition.keyframes.map(point => point.position),
        0xb56d32, 2.2, true, 0.55)!;
      forecast.name = `traffic-forecast:${definition.id}`;
      forecast.visible = false;
      (this.forecastWindows ??= new Map()).set(definition.id, revealReplayWindow(forecast.geometry,
        definition.keyframes.map(point => point.timeS), forecast.material));
      (this.trafficForecasts ??= new Map()).set(definition.id, forecast);
      return;
    }
    const palette = this.palette;
    const visual = new THREE.Group();
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(definition.radiusM, 24, 16),
      new THREE.MeshStandardMaterial({
        color: palette.movingSphere,
        roughness: 0.72,
      }),
    );
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    visual.add(mesh);
    const haloMaterial = new THREE.LineBasicMaterial({
      color: palette.movingSphere,
      transparent: true,
      opacity: 0.5,
      depthTest: true,
      depthWrite: false,
    });
    const horizontalHalo = new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints(ringPoints(definition.radiusM * 1.45, 0, 48)),
      haloMaterial,
    );
    visual.add(horizontalHalo);
    visual.position.fromArray(enuToThree(definition.keyframes[0]!.position));
    this.movingSpheres.set(definition.id, visual);
    this.dynamicGroup.add(visual);
    this.replaceLine(
      this.dynamicGroup,
      null,
      definition.keyframes.map((keyframe) => keyframe.position),
      palette.movingSphere,
      1.15,
      true,
      0.22,
    );
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
    this.primaryPathGroup.add(marker);
  }

  private clearMinimumSeparationEvidence(): void {
    this.evidenceGroup.traverse((child) => {
      if (child instanceof Line2 && child.material instanceof LineMaterial) {
        this.lineMaterials.delete(child.material);
      }
    });
    for (const child of [...this.evidenceGroup.children]) {
      this.evidenceGroup.remove(child);
      disposeObject(child);
    }
  }

  private replaceMinimumSeparationEvidence(): void {
    this.clearMinimumSeparationEvidence();
    if (!this.scenario || !this.run) return;
    const evidence = minimumSeparationEvidence(this.scenario, this.run, "execution");
    if (evidence === null) return;

    const { witness } = evidence;
    const palette = this.palette;
    const markerRadius = Math.max(
      0.22,
      Math.min(0.72, evidence.safetyEnvelopeRadiusM * 0.28),
    );
    const pointMaterial = (color: number): THREE.MeshStandardMaterial =>
      new THREE.MeshStandardMaterial({ color, roughness: 0.7, depthTest: true, depthWrite: true });

    const vehiclePoint = new THREE.Mesh(
      new THREE.SphereGeometry(markerRadius, 18, 12),
      pointMaterial(palette.witnessVehicle),
    );
    vehiclePoint.name = "minimum-separation-vehicle-point";
    vehiclePoint.position.fromArray(enuToThree(witness.vehiclePosition));

    const obstacleSurfacePoint = new THREE.Mesh(
      new THREE.OctahedronGeometry(markerRadius * 1.12),
      pointMaterial(palette.witnessSurface),
    );
    obstacleSurfacePoint.name = "minimum-separation-obstacle-surface-point";
    obstacleSurfacePoint.position.fromArray(enuToThree(witness.obstaclePosition));

    const envelope = new THREE.Mesh(
      new THREE.SphereGeometry(evidence.safetyEnvelopeRadiusM, 18, 12),
      new THREE.MeshBasicMaterial({
        color: palette.safetyEnvelope,
        wireframe: true,
        transparent: true,
        opacity: 0.58,
        depthTest: true,
        depthWrite: false,
      }),
    );
    envelope.name = "declared-safety-margin-envelope";
    envelope.position.copy(vehiclePoint.position);

    const connector = this.replaceLine(
      this.evidenceGroup,
      null,
      [witness.vehiclePosition, witness.obstaclePosition],
      palette.witnessConnector,
      2.8,
      evidence.connectorDashed,
      0.94,
    );
    if (connector) {
      connector.name = evidence.connectorDashed
        ? "minimum-separation-connector-approximate"
        : "minimum-separation-connector-exact";
    }

    this.evidenceGroup.add(envelope, vehiclePoint, obstacleSurfacePoint);
    this.evidenceGroup.visible = false;
  }

  private replacePathLines(): void {
    if (!this.run) return;
    const path = this.activePath;
    if (!this.overview) this.primaryPath = this.replaceLine(
      this.primaryPathGroup,
      this.primaryPath,
      path.map((waypoint) => waypoint.position),
      this.palette.plannedPath,
      3.1,
      false,
      0.92,
    );
    this.updateLayerVisibility();
  }

  private replaceLine(
    group: THREE.Group,
    previous: Line2 | null,
    points: Vec3[],
    color: number,
    weight: number,
    dashed: boolean,
    opacity: number,
    coincidentOrder = 0,
  ): Line2 | null {
    if (previous) {
      if (points.length < 2) {
        previous.visible = false;
        return previous;
      }
      const geometry = previous.geometry as LineGeometry;
      const material = previous.material as LineMaterial;
      geometry.setPositions(flattenPoints(points));
      material.color.setHex(color);
      Object.assign(material, sceneLineStyle(Boolean(this.scenario?.city), weight, dashed, opacity));
      material.needsUpdate = true;
      previous.visible = true;
      previous.renderOrder = coincidentOrder;
      if (dashed) previous.computeLineDistances();
      return previous;
    }
    if (points.length < 2) return null;
    const geometry = new LineGeometry();
    geometry.setPositions(flattenPoints(points));
    const material = new LineMaterial({
      ...sceneLineStyle(Boolean(this.scenario?.city), weight, dashed, opacity),
      color,
    });
    material.resolution.set(
      Math.max(1, this.container.clientWidth),
      Math.max(1, this.container.clientHeight),
    );
    this.lineMaterials.add(material);
    const line = new Line2(geometry, material);
    if (dashed) line.computeLineDistances();
    line.renderOrder = coincidentOrder;
    group.add(line);
    return line;
  }

  private orientVehicle(path: TimedWaypoint[], timeS: number): void {
    if (!this.vehicle || path.length < 2) return;
    const duration = path.at(-1)!.timeS;
    const before = timedPosition(path, Math.max(0, timeS - 0.03));
    const after = timedPosition(path, Math.min(duration, timeS + 0.03));
    const direction = new THREE.Vector3(...enuToThree(after)).sub(
      new THREE.Vector3(...enuToThree(before)),
    );
    if (direction.lengthSq() <= 1e-10) return;
    direction.normalize();
    this.vehicle.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction);
  }

  private updateLayerVisibility(): void {
    if (this.buildingsGroup.visible !== this.layerVisibility.buildings
      || this.dynamicGroup.visible !== this.layerVisibility.dynamic) {
      this.renderer.shadowMap.needsUpdate = true;
    }
    this.buildingsGroup.visible = this.layerVisibility.buildings;
    if (this.cityLayer) this.cityLayer.buildings.visible = this.layerVisibility.buildings;
    this.zonesGroup.visible = this.layerVisibility.zones;
    this.dynamicGroup.visible = this.layerVisibility.dynamic;
  }

  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height, false);
    for (const material of this.lineMaterials) material.resolution.set(width, height);
    this.overview?.resize(width, height);
    if (!this.follow?.resize(width, height))
      fitMapCamera(this.camera, this.sceneBounds(), width, height,
        this.observationBounds ? 1.12 : mapFramingPadding(this.mapScope, this.currentView === "isometric"));
    this.render();
  }

  private sceneBounds(): THREE.Box3 {
    if (this.observationBounds) return this.observationBounds.clone();
    if (!this.scenario) return new THREE.Box3(new THREE.Vector3(-50, 0, -50), new THREE.Vector3(50, 50, 50));
    if (this.scenario.city) {
      const bounds = cityMissionBounds(this.scenario, this.scenario.runs.flatMap((run) =>
        (run.executionTimedPath ?? []).map((waypoint) => waypoint.position)));
      if (this.mapScope === "city") bounds.union(this.cityBounds);
      if (this.mapScope === "city" && this.overview) bounds.union(this.overview.bounds);
      return bounds;
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
    for (const sphere of this.scenario.movingSpheres) {
      for (const keyframe of sphere.keyframes) {
        const position = new THREE.Vector3(...enuToThree(keyframe.position));
        bounds.expandByPoint(position.clone().addScalar(sphere.radiusM * 1.45));
        bounds.expandByPoint(position.clone().addScalar(-sphere.radiusM * 1.45));
      }
    }
    for (const run of this.scenario.runs) {
      for (const waypoint of run.executionTimedPath ?? []) bounds.expandByPoint(new THREE.Vector3(...enuToThree(waypoint.position)));
    }
    if (this.mapScope === "city") bounds.union(this.cityBounds);
    return bounds;
  }

  private render(): void { this.frames.request(); }

  private drawFrame(): void {
    if (this.motion?.step()) this.frames.request();
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
