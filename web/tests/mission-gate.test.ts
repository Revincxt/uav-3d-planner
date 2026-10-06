import * as THREE from "three";
import { describe, expect, it } from "vitest";
import { GATE_SIZE_M, GATE_BEVEL_M, missionGateGeometry, missionGateMaterial, missionGateMatrix, missionGateYaw } from "../src/mission-gate";
import { auditMissionTaskVisits, parseCityMission } from "../src/city-schema";

describe("Open square flight gates", () => {
  it("has four rounded rails and a genuinely empty center", () => {
    const geometry = missionGateGeometry(), gate = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial());
    geometry.computeBoundingBox();
    expect(geometry.boundingBox!.getSize(new THREE.Vector3()).x).toBeCloseTo(GATE_SIZE_M + 2 * GATE_BEVEL_M);
    gate.updateMatrixWorld();
    const ray = new THREE.Raycaster(new THREE.Vector3(0,0,30), new THREE.Vector3(0,0,-1));
    expect(ray.intersectObject(gate)).toHaveLength(0);
    ray.ray.origin.x = GATE_SIZE_M / 2 - 0.2; expect(ray.intersectObject(gate).length).toBeGreaterThan(0);
    geometry.dispose(); (gate.material as THREE.Material).dispose();
  });
  it("paints both standalone and instanced frames with checkerboard corners without disabling depth", () => {
    const material = missionGateMaterial(0x2563eb);
    const shader = { vertexShader: "#include <begin_vertex>", fragmentShader: "#include <color_fragment>" } as Parameters<typeof material.onBeforeCompile>[0];
    material.onBeforeCompile(shader, null as unknown as THREE.WebGLRenderer);
    expect(shader.vertexShader).toContain("vRaceGateXY = position.xy");
    expect(shader.fragmentShader).toContain("cornerPatch");
    expect(shader.fragmentShader).toContain("innerRim");
    expect(shader.fragmentShader).toContain("fwidth");
    expect(material.color.getHex()).toBe(0x2563eb);
    expect(material.depthTest && material.depthWrite).toBe(true);
    expect(material.userData.style).toBe("racing-gate");
    material.dispose();
  });
  it("is perpendicular to the actual horizontal flight tangent and centered at the knot", () => {
    const position: [number,number,number] = [50,0,100], points = [[0,0,100],position,[100,0,100]];
    expect(missionGateYaw(points, position)).toBeCloseTo(Math.PI / 2);
    expect(new THREE.Vector3().setFromMatrixPosition(missionGateMatrix(points, position)).toArray()).toEqual([50,100,-0]);
    expect(missionGateYaw([[50,-50,100],position,[50,50,100]], position)).toBeCloseTo(Math.PI);
  });
  it("accepts zero dwell only when explicitly declared fly-through and rejects hidden waits", () => {
    const tasks = Array.from({ length: 6 }, (_, i) => ({ id: String(i), order: i + 1, label: "Point", action: "Fly through",
      position: [i * 10,0,30], buildingId: String(i), serviceDurationS: 0, visitMode: "fly-through" }));
    const mission = parseCityMission({ origin: "A", destination: "B", purpose: "Fly through", taskPoints: tasks }, { min: [0,0,0], max: [100,100,100] })!;
    expect(mission.taskPoints![0]!.visitMode).toBe("fly-through");
    expect(() => auditMissionTaskVisits(tasks.map(t => t.position), mission, tasks.map((_,i) => i * 10))).not.toThrow();
    expect(() => auditMissionTaskVisits([tasks[0]!.position,...tasks.map(t => t.position)], mission, [0,...tasks.map((_,i) => 1 + i * 10)])).toThrow("must not contain a dwell");
  });
});
