import * as THREE from "three";

interface RotorClock {
  phase: THREE.IUniform<number>;
  blur: THREE.IUniform<number>;
  timeS?: number;
}
const clocks = new WeakMap<THREE.Object3D, RotorClock>();
const TURNS_PER_TRACE_SECOND = .8;

/** Animate all four motor axes in one draw, including normals and the swept disks. */
export function configureDroneRotors(drone: THREE.Object3D, rotors: THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>): void {
  const clock: RotorClock = { phase: { value: 0 }, blur: { value: 0 } };
  clocks.set(drone, clock);
  const material = rotors.material;
  material.transparent = true; material.depthWrite = false;
  material.side = THREE.DoubleSide; material.forceSinglePass = true;
  material.customProgramCacheKey = () => "uav-rotors-1";
  material.onBeforeCompile = shader => {
    shader.uniforms.rotorPhase = clock.phase; shader.uniforms.rotorMotion = clock.blur;
    shader.vertexShader = `uniform float rotorPhase;
attribute vec3 rotorPivot;
attribute float rotorDisk;
varying vec2 vRotorUV;
varying float vRotorDisk;
${shader.vertexShader}`.replace("#include <beginnormal_vertex>", `#include <beginnormal_vertex>
float angle = rotorPivot.z * rotorPhase;
mat2 spin = mat2(cos(angle), -sin(angle), sin(angle), cos(angle));
objectNormal.xz = spin * objectNormal.xz;`).replace("#include <begin_vertex>", `#include <begin_vertex>
transformed.xz = spin * (position.xz - rotorPivot.xy) + rotorPivot.xy;
vRotorUV = (position.xz - rotorPivot.xy) / .35;
vRotorDisk = rotorDisk;`);
    shader.fragmentShader = `uniform float rotorMotion;
varying vec2 vRotorUV;
varying float vRotorDisk;
${shader.fragmentShader}`.replace("#include <color_fragment>", `#include <color_fragment>
if (vRotorDisk > .5) {
  if (rotorMotion < .001) discard;
  float radius = length(vRotorUV);
  float sweep = smoothstep(.16, .32, radius) * (1. - smoothstep(.78, 1., radius));
  float streak = pow(abs(vRotorUV.x) / max(radius, .001), 8.);
  diffuseColor.a *= rotorMotion * sweep * (.10 + .07 * streak);
} else {
  diffuseColor.a *= mix(1., .22, rotorMotion);
}`);
  };
}

/** A sampled visual phase, not motor telemetry: blur conveys high shaft speed.
 * Bound the visible two-blade advance below 90 degrees to prevent reverse aliasing.
 * Only the replay clock advances it; pauses, camera motion and extra renders do not. */
export function setDroneRotorTime(drone: THREE.Object3D, timeS: number): void {
  const clock = clocks.get(drone);
  if (!clock || !Number.isFinite(timeS)) return;
  const time = Math.max(0, timeS), delta = clock.timeS === undefined ? 0 : time - clock.timeS;
  const tau = Math.PI * 2;
  clock.phase.value = clock.timeS === undefined || delta < 0
    ? (time * TURNS_PER_TRACE_SECOND % 1) * tau
    : (clock.phase.value + Math.min(Math.PI / 3, delta * TURNS_PER_TRACE_SECOND * tau)) % tau;
  clock.blur.value = Math.min(1, time / .6);
  clock.timeS = time;
}
