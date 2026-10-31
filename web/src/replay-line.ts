import * as THREE from "three";
import type { LineGeometry } from "three/addons/lines/LineGeometry.js";
import type { LineMaterial } from "three/addons/lines/LineMaterial.js";

function writeReplayTimes(geometry: LineGeometry, times: ArrayLike<number>): void {
  const count = times.length - 1;
  if (count !== geometry.getAttribute("instanceStart").count) throw new Error("Replay clocks must match segment capacity");
  for (const [name, offset] of [["instanceTimeStart", 0], ["instanceTimeEnd", 1]] as const) {
    let attribute = geometry.getAttribute(name) as THREE.InstancedBufferAttribute | undefined;
    if (!attribute) {
      attribute = new THREE.InstancedBufferAttribute(new Float32Array(count), 1).setUsage(THREE.DynamicDrawUsage);
      geometry.setAttribute(name, attribute);
    }
    for (let i = 0; i < count; i++) attribute.setX(i, times[i + offset]!);
    attribute.needsUpdate = true;
  }
}

/** Retained known traffic, clipped at both ends in world space with two uniforms. */
export function revealReplayWindow(geometry: LineGeometry, times: ArrayLike<number>, material: LineMaterial): { start: { value: number }; end: { value: number } } {
  writeReplayTimes(geometry, times);
  // A reactive plan replaces its positions/times, not the compiled shader program.
  if (material.uniforms.windowStart && material.uniforms.windowEnd)
    return { start: material.uniforms.windowStart, end: material.uniforms.windowEnd };
  const start = { value: 0 }, end = { value: 0 };
  material.uniforms.windowStart = start; material.uniforms.windowEnd = end;
  material.vertexShader = `uniform float windowStart;
uniform float windowEnd;
attribute float instanceTimeStart;
attribute float instanceTimeEnd;
varying float windowVisible;\n` + material.vertexShader;
  material.vertexShader = material.vertexShader.replace("void main() {", `void main() {
    float span = max(instanceTimeEnd - instanceTimeStart, 0.000001);
    float from = clamp((windowStart - instanceTimeStart) / span, 0.0, 1.0);
    float to = clamp((windowEnd - instanceTimeStart) / span, 0.0, 1.0);
    windowVisible = (to > from && distance(instanceStart, instanceEnd) > 0.000001) ? 1.0 : 0.0;
    vec3 clippedStart = mix(instanceStart, instanceEnd, from);
    vec3 clippedEnd = mix(instanceStart, instanceEnd, to > from ? to : 1.0);`);
  material.vertexShader = material.vertexShader.replace("vec4( instanceStart, 1.0 )", "vec4( clippedStart, 1.0 )");
  material.vertexShader = material.vertexShader.replace("vec4( instanceEnd, 1.0 )", "vec4( clippedEnd, 1.0 )");
  material.fragmentShader = "varying float windowVisible;\n" + material.fragmentShader;
  material.fragmentShader = material.fragmentShader.replace("void main() {", "void main() { if (windowVisible < 0.5) discard;");
  material.needsUpdate = true;
  return { start, end };
}

/** Trim the current segment in world space. Playback updates one uniform, never route buffers. */
export function revealReplayLine(
  geometry: LineGeometry, times: ArrayLike<number>, materials: LineMaterial[],
): { value: number } {
  writeReplayTimes(geometry, times);
  const clock = { value: 0 };
  for (const material of materials) {
    material.uniforms.replayTime = clock;
    material.vertexShader = `uniform float replayTime;
attribute float instanceTimeStart;
attribute float instanceTimeEnd;
varying float replayVisible;\n` + material.vertexShader;
    material.vertexShader = material.vertexShader.replace("void main() {", `void main() {
      float elapsed = replayTime - instanceTimeStart;
      float fraction = clamp(elapsed / max(instanceTimeEnd - instanceTimeStart, 0.000001), 0.0, 1.0);
      replayVisible = (elapsed > 0.0 && distance(instanceStart, instanceEnd) > 0.000001) ? 1.0 : 0.0;
      vec3 replayEnd = mix(instanceStart, instanceEnd, fraction > 0.0 ? fraction : 1.0);`);
    material.vertexShader = material.vertexShader.replace("vec4( instanceEnd, 1.0 )", "vec4( replayEnd, 1.0 )");
    material.fragmentShader = "varying float replayVisible;\n" + material.fragmentShader;
    material.fragmentShader = material.fragmentShader.replace("void main() {", "void main() { if (replayVisible < 0.5) discard;");
    material.needsUpdate = true;
  }
  return clock;
}
