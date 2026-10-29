import * as THREE from "three";
import { describe, expect, it } from "vitest";
import { createDrone } from "../src/drone-model";
import { setDroneRotorTime } from "../src/drone-rotors";
import { createTrafficAircraft, updateTrafficAircraft } from "../src/traffic-aircraft";
import { disposeRenderObject } from "../src/render-resources";

function rotorUniforms(drone: THREE.Object3D) {
  const rotor = drone.getObjectByName("drone-rotors") as THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>;
  const shader = { uniforms: {}, vertexShader: THREE.ShaderLib.standard.vertexShader, fragmentShader: THREE.ShaderLib.standard.fragmentShader };
  rotor.material.onBeforeCompile(shader as Parameters<THREE.Material["onBeforeCompile"]>[0], {} as THREE.WebGLRenderer);
  return { rotor, uniforms: shader.uniforms as Record<string, THREE.IUniform<number>> };
}
const tau = Math.PI * 2;

describe("recorded-clock rotor animation", () => {
  it("pairs opposite-handed propellers on four independent axes without moving the airframe", () => {
    const drone = createDrone(0x009b87), { rotor } = rotorUniforms(drone);
    const pivots = rotor.geometry.getAttribute("rotorPivot"), disks = rotor.geometry.getAttribute("rotorDisk");
    const motors = new Map<string, number>();
    for (let i = 0; i < pivots.count; i++) {
      const x = pivots.getX(i), z = pivots.getY(i), sign = pivots.getZ(i);
      expect(sign).toBe(-Math.sign(x * z)); motors.set(`${Math.sign(x)},${Math.sign(z)}`, sign);
    }
    expect([...motors.values()].sort()).toEqual([-1, -1, 1, 1]);
    expect(disks.array.some(v => v === 1)).toBe(true);
    expect(rotor.material.forceSinglePass).toBe(true);
    const positions = rotor.geometry.getAttribute("position").array.slice();
    setDroneRotorTime(drone, .1); setDroneRotorTime(drone, .2);
    expect(rotor.geometry.getAttribute("position").array).toEqual(positions);
    expect(drone.position.toArray()).toEqual([0, 0, 0]); expect(drone.rotation.y).toBe(0);
    disposeRenderObject(drone);
  });

  it("advances in the correct direction at all replay rates, caps hitches, freezes pauses and resets rewinds", () => {
    for (const hz of [15, 30, 60, 120]) for (const rate of [2, 4, 8]) {
      const drone = createDrone(0x009b87), { uniforms } = rotorUniforms(drone);
      const phase = uniforms.rotorPhase!, blur = uniforms.rotorMotion!;
      setDroneRotorTime(drone, 0); expect(blur.value).toBe(0);
      let previous = phase.value;
      for (let frame = 1; frame <= hz; frame++) {
        setDroneRotorTime(drone, frame * rate / hz);
        const advance = (phase.value - previous + tau) % tau;
        expect(advance).toBeGreaterThan(0); expect(advance).toBeLessThanOrEqual(Math.PI / 3 + 1e-9);
        expect(advance).toBeCloseTo(Math.min(Math.PI / 3, rate / hz * .8 * tau));
        previous = phase.value;
      }
      setDroneRotorTime(drone, rate); expect(phase.value).toBe(previous); expect(blur.value).toBe(1);
      setDroneRotorTime(drone, rate + 1); expect((phase.value - previous + tau) % tau).toBeCloseTo(Math.PI / 3);
      previous = phase.value;
      setDroneRotorTime(drone, rate + 5); expect((phase.value - previous + tau) % tau).toBeCloseTo(Math.PI / 3);
      setDroneRotorTime(drone, .125); expect(phase.value).toBeCloseTo(.125 * .8 * tau);
      setDroneRotorTime(drone, 0); expect(phase.value).toBe(0); expect(blur.value).toBe(0);
      disposeRenderObject(drone);
    }
  });

  it("animates cargo traffic while hovering and keeps clocks independent between aircraft", () => {
    const definition = { id: "traffic", radiusM: 18, keyframes: [
      { timeS: 0, position: [0, 0, 30] as [number, number, number] },
      { timeS: 2, position: [0, 0, 30] as [number, number, number] },
    ] };
    const aircraft = createTrafficAircraft(definition), other = createDrone(0x2563eb);
    const { uniforms } = rotorUniforms(aircraft), otherClock = rotorUniforms(other).uniforms;
    updateTrafficAircraft(aircraft, definition, .1); const phase = uniforms.rotorPhase!.value;
    updateTrafficAircraft(aircraft, definition, .2);
    expect(uniforms.rotorPhase!.value).not.toBe(phase);
    expect(otherClock.rotorPhase!.value).toBe(0);
    expect(aircraft.position.toArray()).toEqual([0, 30, -0]);
    disposeRenderObject(aircraft); disposeRenderObject(other);
  });
});
