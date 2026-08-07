# Changelog

All notable changes are documented here. The project follows Semantic Versioning.

## [0.6.0] - 2026-08-07

### Added

- Two outcome-independent extended-city diagnostic scenarios: `braided-skyway` and
  `harbor-switchback`. Each contains 20 unequal-height buildings, one static no-fly volume, two
  temporary restrictions, and two moving hazards.
- Dynamic closest-approach witnesses for every predictive run. Moving-sphere separation is exact
  under the declared piecewise-linear model; temporary-cylinder separation is labelled as a
  deterministic approximate search.
- Raw and output discrete kinematic diagnostics covering reversals, velocity change, acceleration
  proxy, climb rate, and maximum segment-average speed.
- An explicit v0.6 model/UI plan and a reorganized predictive study interface.

### Changed

- The predictive matrix now contains ten scenarios by four planner conditions, for 40 deterministic
  non-confirmatory missions.
- D* Lite endpoint handling now evaluates multiple visible start and goal anchors with a
  goal-aware, non-backtracking preference while retaining reset/reuse comparability.
- Predictive records distinguish dense-piecewise-linear collision certification from descriptive
  kinematic diagnostics and carry the closest-approach method and exactness flag.

### Interpretation boundary

- The added velocity-change and acceleration-proxy values do not certify continuous acceleration,
  attitude, curvature, jerk, or vehicle dynamics.
- The v0.6 cohort contains the v0.5 cases as a historical subset; results from the two releases must
  not be pooled as independent scenarios.

## [0.5.0] - 2026-08-05

### Added

- Eight deterministic predictive cases spanning dense street grids, urban canyons, rooftop
  transfers, static no-fly volumes, temporary restrictions, and coordinated air traffic.
- A wait-preserving trajectory post-processor that rounds feasible turns, retimes samples by
  path progress, and continuously re-audits every space-time segment before accepting it.
- Explicit raw-versus-certified trajectory records with smoothing provenance, turn diagnostics,
  and fail-closed fallback to the planner output.
- A rebuilt predictive-study figure with continuous-time playback, raw/certified comparison,
  orthogonal camera views, layer controls, richer urban context, and motion-profile plots.

### Changed

- The predictive public bundle now uses its v2 schema and compact semantic event frames instead of
  repeating path prefixes, suffixes, or every dense smoothing sample.
- The predictive protocol is now a distinct v0.5 diagnostic cohort; results are not pooled with or
  treated as directly comparable to the v0.4 cohort.
- The default GitHub Pages predictive view opens on a complex demonstration case rather than the
  calibration corridor.

## [0.4.0] - 2026-08-05

### Added

- A deterministic `TimedPath` contract with explicit movement and wait actions.
- Six predictive scenarios covering opening and closing windows, periodic traffic, chained
  restrictions, multiple moving obstacles, and a vertical time window.
- Finite-horizon 4D Space-Time A* over `(voxel, time-step)` states with deterministic tie breaking,
  fixed-speed moves, explicit waits, exact endpoints, and continuous space-time edge checks.
- A controlled D* Lite reset-versus-state-reuse ablation that preserves the v0.3 default behavior.
- Predictive CLI commands and a fixed 6-scenario by 4-planner export protocol.
- A restrained academic predictive-study page with synchronized 3D replay, event annotations,
  time-height traces, outcome tables, and provenance downloads.
- Predictive CSV and scenario-manifest artifacts with stable run IDs, byte sizes, SHA-256 digests,
  Python trajectory replay audits, and independent JavaScript structural validation.

### Changed

- Package scope and documentation now distinguish static planning, reactive snapshot replanning,
  and deterministic schedule-aware planning.
- Total stationary time is separated from wait-interval reasons so short time-lattice alignment
  waits are not misreported as policy decisions.
- GitHub Pages now builds four academic study views.

## [0.3.0] - 2026-08-05

### Added

- Deterministic dynamic-scenario contracts for half-open temporary no-fly intervals and
  piecewise-linear moving spheres.
- Continuous space-time collision checks using exact interval overlap and relative-motion tests.
- Conservative static snapshots plus an execution-time dynamic safety gate and safe-hold behavior.
- Repeated 3D A*, repeated Lazy Theta*, and state-reusing 3D D* Lite replanning baselines.
- Four curated dynamic scenarios covering a pop-up restriction, crossing traffic, a closing gate,
  and a vertical escape.
- Dynamic simulation and export CLI commands with complete frame and mission records.
- A synchronized Three.js dynamic-replanning page with playback, stepping, scrubbing, camera views,
  recorded work, and outcome tables.
- Dynamic CSV/manifest downloads with source revision, SHA-256, byte-size, and deterministic replay
  audits in CI.

### Changed

- Package scope now distinguishes the v0.2 static-planning baseline from the v0.3 dynamic
  replanning experiment.
- GitHub Pages now builds three restrained academic views: static paths, static benchmark results,
  and dynamic replanning.
- The committed dynamic study excludes machine-dependent wall-clock timings and reports
  algorithm-specific planning work without treating different work units as equivalent.

## [0.2.0] - 2026-08-05

### Added

- Explicit algorithmic and optional wall-clock budget contracts with termination provenance.
- Separate setup, search, smoothing, and validation timings in v2 experiment records.
- RRT* anytime quality traces at fixed sample-attempt checkpoints.
- Voxel-resolution and RRT* budget sweep commands.
- Random-scene dataset manifests with complete acceptance and rejection records.
- Randomized isolated-process timing harness with environment metadata.
- Scene-weighted median, IQR, and fixed-seed scene-clustered bootstrap summaries.
- Run-level CSV, summary CSV/JSON, checksums, and dependency-free SVG report export.
- Independent academic benchmark-results page with sensitivity figures and an exact table.
- Python re-audit of committed trajectories, problem fingerprints, manifests, and CSV run IDs.
- SHA-256 and byte-size provenance for every downloadable benchmark artifact.

### Changed

- Scene fingerprints now hash only planning semantics, normalize numeric spelling, and ignore labels and metadata.
- Deterministic planners run once per scene in path-quality benchmarks; repeated executions are reserved for timing.
- Planner success is based on a certified raw path, while post-processing outcome is recorded separately.
- GitHub Pages now builds both the recorded trajectory view and the descriptive benchmark view.
- Stable configuration IDs canonicalize equivalent numeric parameter spellings, and statistical
  clusters use semantic problem fingerprints rather than display IDs.
- The results table and summary CSV distinguish path-quality runs from isolated timing runs.

## [0.1.0] - 2026-08-05

### Added

- Shared 3D scene contracts for buildings, finite cylindrical no-fly zones, vehicle radius, and safety margin.
- Continuous segment collision tests and independent path auditing.
- Collision-aware 26-connected 3D A* with exact virtual endpoints.
- Lazy Theta* with delayed line-of-sight validation and repair.
- Seeded finite-budget RRT* with parent selection, rewiring, cycle protection, and descendant cost updates.
- Shared shortcut and collision-certified cubic B-spline smoothing pipeline.
- Four curated urban maps and deterministic random-city generation.
- CLI commands for scene generation, individual planning, benchmarking, and demo-data export.
- Academic Three.js comparison page with recorded raw and smoothed trajectories.
- Python and web tests, package build verification, GitHub Actions CI, and Pages deployment.

[0.1.0]: https://github.com/Revincxt/uav-3d-planner-lab/releases/tag/v0.1.0
[0.2.0]: https://github.com/Revincxt/uav-3d-planner-lab/releases/tag/v0.2.0
[0.3.0]: https://github.com/Revincxt/uav-3d-planner-lab/releases/tag/v0.3.0
[0.4.0]: https://github.com/Revincxt/uav-3d-planner-lab/releases/tag/v0.4.0
[0.5.0]: https://github.com/Revincxt/uav-3d-planner-lab/releases/tag/v0.5.0
