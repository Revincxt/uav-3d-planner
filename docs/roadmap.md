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

## v0.4 — predictive space-time planning (released)

- explicit timestamped paths with fixed-speed movement and wait actions;
- finite-horizon Space-Time A* over `(voxel, time-step)` states;
- exact continuous space-time checks for every planned move, wait, and endpoint connector;
- reset and state-reusing D* Lite conditions for a controlled incremental-state ablation;
- six outcome-independent predictive scenarios across calibration, demo, and curated diagnostic
  cohorts;
- a fixed four-condition protocol with stable run IDs, CSV/manifest provenance, Python replay
  audits, and independent Web structural validation;
- a restrained predictive study page with synchronized 3D and time-height views.

The v0.4 public bundle is a 24-run non-confirmatory diagnostic. It assumes a complete deterministic
forecast and reports Space-Time A* as earliest-arrival only on the declared spatial/time lattice.
Total stationary time includes separately labelled endpoint time-lattice alignment waits.

## v0.5 — complex-city trajectory study (released)

- eight outcome-independent predictive scenarios across calibration, demo, and diagnostic cohorts;
- seven complex maps with 14–16 unequal-height buildings, static no-fly volumes, temporary
  restrictions, and moving hazards;
- raw planner paths retained beside a common wait-preserving circular-fillet post-processor;
- deterministic radius fallback plus continuous space-time and speed certification for every
  published execution segment;
- a schema-v2, 32-run public bundle with compact semantic event frames;
- a redesigned academic figure with raw/certified comparison, continuous playback, orthographic
  views, semantic layers, outcome tables, and provenance downloads.

The v0.5 public bundle remains non-confirmatory and is not pooled with v0.4. Rounded output is a
dense piecewise-linear geometric path, not a kinodynamic or minimum-snap trajectory.

## v0.6 — extended-city diagnostics and study interface (released)

- ten outcome-independent predictive scenarios and a fixed 40-run four-condition matrix;
- `braided-skyway` and `harbor-switchback`, each with 20 unequal-height buildings, one static
  no-fly volume, two temporary restrictions, and two moving hazards;
- goal-aware multi-anchor D* Lite endpoint handling to reduce avoidable connector reversals;
- dynamic closest-approach witnesses, with exact relative-motion results for moving spheres and an
  explicitly approximate deterministic search for temporary cylinders;
- raw/output discrete kinematic diagnostics for reversal count, velocity change, acceleration proxy,
  and climb rate, without claiming continuous-dynamics feasibility;
- a reorganized predictive study interface that keeps scenario selection, recorded evidence,
  trajectory inspection, diagnostics, and provenance distinct.

The v0.6 cohort remains curated, non-preregistered, and non-confirmatory. It includes the v0.5 cases
as a historical subset, so v0.5 and v0.6 rows are not independent observations. Collision
certification continues to cover the dense piecewise-linear space-time path only.

## v0.7 — discrete execution-envelope audit (current)

- the v0.6 ten-scenario by four-condition matrix remains frozen, with no outcome-based additions;
- planner output, a collision-audited geometry candidate, and an optional retimed execution
  candidate are retained as three separate evidence layers;
- a deterministic time reparameterizer may only extend local movement durations, preserves wait
  durations, and re-audits the changed timestamps against the dynamic schedule;
- a declared benchmark envelope limits segment-average speed, climb/descent rate,
  adjacent-segment acceleration proxy, reversals, and mission time;
- planner-domain metrics are computed only from the raw path, so downstream geometry and timing
  changes cannot be attributed to a planner;
- execution qualification has explicit failure states and is never inferred from collision
  certification alone;
- the predictive figure renders and can jump directly to the closest-approach witness.

The v0.7 envelope is a deliberately limited discrete diagnostic, not a kinodynamic model or flight
certificate. The qualified count is reported as an outcome and is not used to tune thresholds or
accept the release.

## Later, deliberately separate

- kinodynamic planning and vehicle attitude constraints;
- minimum-snap trajectory generation;
- wind and energy models;
- perception uncertainty;
- uncertain obstacle forecasts and forecast calibration;
- larger generated predictive cohorts and temporal-resolution sensitivity;
- PX4/MAVLink export and simulator integration.

These features should not be added to the static comparison retroactively because they change the
research question and planner interface.
