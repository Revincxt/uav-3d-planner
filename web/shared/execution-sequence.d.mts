interface Waypoint { timeS: number; position: readonly number[] }
export function validateExecutionSequence(geometry: readonly Waypoint[], execution: readonly Waypoint[], allowAdditionalHolds?: boolean): void;
