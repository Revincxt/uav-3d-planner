import type { CityMission } from "./city-schema";
import type { RoutePoint, RouteWaypoint } from "./route-overview";

export const STATIC_PLAYBACK_SPEED_MPS = 15;

/** Display clock only: follows the certified spatial nodes, without claiming flight feasibility. */
export function staticPlaybackPath(
  points: readonly RoutePoint[], mission?: CityMission, speedMps = STATIC_PLAYBACK_SPEED_MPS, maxClimbRateMps?: number,
): RouteWaypoint[] {
  if (!points.length || !Number.isFinite(speedMps) || speedMps <= 0) throw new Error("Invalid static playback path or speed");
  if (maxClimbRateMps !== undefined && (!Number.isFinite(maxClimbRateMps) || maxClimbRateMps <= 0)) throw new Error("Invalid climb rate");
  const path: RouteWaypoint[] = [{ timeS: 0, position: points[0]! }];
  const tasks = mission?.taskPoints ?? [];
  let stop = 0;
  for (let index = 1; index < points.length; index++) {
    const previous = path.at(-1)!, position = points[index]!;
    const length = Math.hypot(...position.map((v, axis) => v - previous.position[axis]!));
    if (length > 0) path.push({ timeS: previous.timeS + Math.max(length / speedMps,
      maxClimbRateMps ? Math.abs(position[2] - previous.position[2]) / maxClimbRateMps : 0), position });
    const task = tasks[stop];
    if (task && Math.hypot(...position.map((v, axis) => v - task.position[axis]!)) <= 1e-6) {
      if (task.serviceDurationS > 0) path.push({ timeS: path.at(-1)!.timeS + task.serviceDurationS, position });
      stop++;
    }
  }
  if (stop !== tasks.length) throw new Error("Static playback cannot skip a required task point");
  return path;
}
