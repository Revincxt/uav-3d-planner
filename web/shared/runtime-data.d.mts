export interface RuntimeData { runtimeVersion: 1; pool: unknown[]; bundle: Record<string, unknown> }
export function packRuntimeData(bundle: { scenarios: unknown[]; [key: string]: unknown }): RuntimeData;
export function unpackRuntimeData(value: unknown): unknown;
export function isRuntimeData(value: unknown): boolean;
