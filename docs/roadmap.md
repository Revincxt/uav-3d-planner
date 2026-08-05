# Roadmap

## v0.1 — static baseline

- static axis-aligned buildings and finite cylindrical no-fly zones;
- 3D A*, Lazy Theta*, and seeded RRT*;
- shared obstacle model and exact segment checks;
- collision-certified common smoothing;
- four curated and seeded random city scenes;
- CLI benchmark records and static Three.js comparison page.

## v0.2 — stronger experiments (released)

- explicit wall-clock and algorithmic budget contracts;
- voxel-resolution sweeps;
- repeated timing harness in isolated processes;
- random-scene dataset manifests and rejection logs;
- per-scene RRT* quality–budget curves;
- median, interquartile range, and scene-clustered bootstrap intervals;
- CSV and publication-ready plot export.

The committed Pages study is explicitly descriptive and uses the four curated scenes as a
small-n diagnostic cohort. The CLI supports larger seeded random cohorts without filtering on any
planner outcome.

## v0.3 — dynamic replanning

- time-indexed temporary no-fly volumes and moving obstacles;
- receding-horizon replanning baselines;
- D* Lite or an equivalent incremental graph-search baseline;
- latency and path-disruption metrics.

## Later, deliberately separate

- kinodynamic planning and vehicle attitude constraints;
- minimum-snap trajectory generation;
- wind and energy models;
- perception uncertainty;
- PX4/MAVLink export and simulator integration.

These features should not be added to the static comparison retroactively because they change the
research question and planner interface.
