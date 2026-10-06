import * as THREE from "three";
import { afterEach, describe, expect, it, vi } from "vitest";
import { auditMissionTaskVisits, parseCityMission, type CityMission } from "../src/city-schema";
import { addMissionTaskMarkers, sizeMissionTaskMarkers } from "../src/mission-tasks";
import { disposeRenderObject } from "../src/render-resources";

const bounds = { min: [0, 0, 0], max: [100, 100, 100] };
const mission: CityMission = {
  origin: "Dispatch", destination: "Receiving", purpose: "Simulation",
  taskPoints: Array.from({ length: 8 }, (_, index) => ({
    id: `task-${index + 1}`, order: index + 1, label: `Roof ${index + 1}`,
    action: "Deliver parcel", position: [10 + index * 10, 20, 30], serviceDurationS: 6,
    buildingId: `building-${index + 1}`,
  })),
};

afterEach(() => vi.unstubAllGlobals());

describe("required mission stops", () => {
  it("accepts 6–8 ordered stops and keeps single-leg legacy missions", () => {
    expect(parseCityMission(mission, bounds)?.taskPoints).toHaveLength(8);
    expect(parseCityMission({ ...mission, taskPoints: mission.taskPoints!.slice(0, 6) }, bounds)?.taskPoints).toHaveLength(6);
    expect(parseCityMission({ ...mission, taskPoints: undefined }, bounds)?.taskPoints).toBeUndefined();
  });

  it.each(["count", "order", "duplicate", "position", "duration", "source"])("rejects invalid %s", defect => {
    const invalid = structuredClone(mission);
    const tasks = invalid.taskPoints!;
    if (defect === "count") tasks.splice(0, 3);
    if (defect === "order") tasks[0]!.order = 2;
    if (defect === "duplicate") tasks[1]!.id = tasks[0]!.id;
    if (defect === "position") tasks[0]!.position[0] = 1000;
    if (defect === "duration") tasks[0]!.serviceDurationS = 0;
    if (defect === "source") tasks[0]!.buildingId = "";
    expect(() => parseCityMission(invalid, bounds)).toThrow();
  });

  it("audits ordered exact visits and dwell time independently of markers", () => {
    const positions = mission.taskPoints!.flatMap(task => [task.position, task.position]);
    const times = positions.map((_, index) => Math.floor(index / 2) * 10 + (index % 2) * 6);
    expect(() => auditMissionTaskVisits(positions, mission, times)).not.toThrow();
    expect(() => auditMissionTaskVisits(positions.slice(2), mission)).toThrow(/omitted/);
    expect(() => auditMissionTaskVisits([...positions].reverse(), mission)).toThrow(/order/);
    const shortened = [...times]; shortened[1] = 5;
    expect(() => auditMissionTaskVisits(positions, mission, shortened)).toThrow(/duration/);
    const nearMiss = positions.map(point => [...point]); nearMiss[0]![0]! += 0.02; nearMiss[1]![0]! += 0.02;
    expect(() => auditMissionTaskVisits(nearMiss, mission)).toThrow(/omitted/);
  });

  it("uses ENU positions, real depth testing, world-size labels and owned textures", () => {
    const context = { fillRect: vi.fn(), fillText: vi.fn() };
    vi.stubGlobal("document", { createElement: () => ({ width: 96, height: 96, getContext: () => context }) });
    const host = new THREE.Group();
    addMissionTaskMarkers(host, mission);
    const tasks = host.getObjectByName("mission-task-points")!;
    expect(tasks.children).toHaveLength(8);
    tasks.children.forEach((task, index) => {
      const point = mission.taskPoints![index]!.position;
      expect(task.position.toArray()).toEqual([point[0], point[2], -point[1]]);
      const number = task.getObjectByName("task-number") as THREE.Sprite;
      expect(number.scale.x).toBe(7);
      const gate = task.getObjectByName("mission-gate") as THREE.Mesh;
      expect(gate.geometry.type).toBe("ExtrudeGeometry");
      expect((gate.material as THREE.Material).depthTest).toBe(true);
      expect(number.material.depthTest).toBe(true);
      expect(number.material.depthWrite).toBe(false);
      expect(number.material.sizeAttenuation).toBe(true);
      expect(number.material.map!.userData.viewerOwned).toBe(true);
    });
    const sprite = tasks.children[0]!.getObjectByName("task-number") as THREE.Sprite;
    const disposed = vi.spyOn(sprite.material.map!, "dispose");
    disposeRenderObject(host);
    expect(disposed).toHaveBeenCalledOnce();
  });
  it("caps close-follow annotations and restores world sizes without moving task positions", () => {
    const host = new THREE.Group(), tasks = new THREE.Group(), task = new THREE.Group();
    tasks.name = "mission-task-points";
    const dot = new THREE.Mesh(new THREE.SphereGeometry(5), new THREE.MeshStandardMaterial());
    const label = new THREE.Sprite(new THREE.SpriteMaterial());
    task.position.set(100, 90, -200); task.add(dot, label); tasks.add(task); host.add(tasks);
    const camera = new THREE.OrthographicCamera(-160, 160, 80, -80);
    sizeMissionTaskMarkers(host, camera, 600, true);
    expect(label.scale.x * 600 / 160).toBeCloseTo(20);
    expect(dot.scale.x).toBe(1);
    expect(label.position.y).toBeLessThan(14);
    camera.zoom = 2; sizeMissionTaskMarkers(host, camera, 600, true);
    expect(label.scale.x * 600 * 2 / 160).toBeCloseTo(20);
    expect(dot.scale.x).toBe(1);
    sizeMissionTaskMarkers(host, camera, 600, false);
    expect(label.scale.toArray()).toEqual([7, 7, 1]);
    expect(label.position.y).toBe(10); expect(dot.scale.x).toBe(1);
    expect(task.position.toArray()).toEqual([100, 90, -200]);
    expect(dot.material.depthTest).toBe(true); expect(label.material.depthTest).toBe(true);
    disposeRenderObject(host);
  });
});
