/** Reuse identical shared inputs only during one synchronous validation.
 * No result survives a call: edited or previously rejected data is rechecked.
 */
export class ValidationCache {
  private entries: WeakMap<object, Map<string, unknown>> | null = null;
  run<T>(validate: () => T, enabled = true): T {
    const previous = this.entries;
    this.entries = enabled ? new WeakMap() : null;
    try { return validate(); } finally { this.entries = previous; }
  }
  memo<T>(input: unknown, key: string, parse: () => T): T {
    if (!this.entries || !input || typeof input !== "object") return parse();
    let values = this.entries.get(input);
    if (values?.has(key)) return values.get(key) as T;
    const result = parse();
    if (!values) { values = new Map(); this.entries.set(input, values); }
    values.set(key, result);
    return result;
  }
}
