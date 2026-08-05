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

## v0.3 — dynamic replanning (released)

- time-indexed temporary cylindrical no-fly volumes and piecewise-linear moving spheres;
- an exact continuous space-time safety audit for executed motion;
- deterministic receding-horizon simulation with scheduled replanning and safe holds;
- repeated 3D A*, repeated Lazy Theta*, and state-reusing 3D D* Lite;
- queue-pop, changed-edge, completion, route-length, hold, and safety-gate metrics;
- four curated dynamic episodes with a synchronized Three.js timeline;
- committed run records, scenario manifest, provenance digests, and replay audit.

The v0.3 comparison is deliberately a deterministic, small-n diagnostic study. Planner wall-clock
timing is excluded from the replay records because it varies by machine; the records compare
algorithm-specific work and mission outcomes under one event stream.

## Later, deliberately separate

- kinodynamic planning and vehicle attitude constraints;
- minimum-snap trajectory generation;
- wind and energy models;
- perception uncertainty;
- predictive space-time planning and uncertain obstacle forecasts;
- PX4/MAVLink export and simulator integration.

These features should not be added to the static comparison retroactively because they change the
research question and planner interface.
