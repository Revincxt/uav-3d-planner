# Changelog

All notable changes are documented here. The project follows Semantic Versioning.

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
