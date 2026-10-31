export interface RuntimeData { pool: unknown[]; bundle: Record<string, unknown> }
export function packRuntimeData(bundle: { scenarios: unknown[]; [key: string]: unknown }): RuntimeData;
export function splitRuntimeCity(value: RuntimeData): { pool: unknown[]; bundle: Record<string, unknown>; city: Record<string, unknown> };
export function unpackRuntimeData(value: unknown, city: unknown): unknown;
export function isRuntimeData(value: unknown): boolean;
