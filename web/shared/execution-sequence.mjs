const same = (a, b) => Math.hypot(...a.map((v, i) => v - b[i])) <= 1e-9;
const close = (a, b) => Math.abs(a - b) <= 1e-6 * Math.max(1, Math.abs(a), Math.abs(b));

/** Execution keeps every original knot and duration; declared scheduling may add only holds. */
export function validateExecutionSequence(geometry, execution, allowAdditionalHolds = false) {
  if (!geometry.length || !execution.length || !same(geometry[0].position, execution[0].position) ||
      !close(geometry[0].timeS, execution[0].timeS)) throw new Error("Execution changed departure");
  let cursor = 0;
  for (let index = 1; index < geometry.length; index++) {
    const previous = geometry[index - 1], next = geometry[index];
    while (cursor + 1 < execution.length && !same(execution[cursor + 1].position, next.position)) {
      if (!allowAdditionalHolds || !same(execution[cursor + 1].position, previous.position))
        throw new Error("Execution changed the geometry waypoint sequence");
      cursor++;
    }
    const a = execution[cursor], b = execution[++cursor];
    if (!b || !same(b.position, next.position)) throw new Error("Execution omitted a geometry knot");
    const original = next.timeS - previous.timeS, actual = b.timeS - a.timeS;
    if (actual + 1e-6 < original) throw new Error("Execution shortened a geometry segment");
    if (same(previous.position, next.position) && !close(actual, original))
      throw new Error("Execution changed an original hold duration");
  }
  if (cursor !== execution.length - 1) throw new Error("Execution added an undeclared suffix");
}
