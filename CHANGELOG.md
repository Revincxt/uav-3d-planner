# Changelog

All notable changes are documented here. The project follows Semantic Versioning.

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
