# Data schemas

## Scene JSON

Committed curated scenes live in `scenarios/` and conform to `scenarios/schema.json`.

Required physical fields:

- `bounds.minimum`, `bounds.maximum`: 3D ENU bounds in metres;
- `start`, `goal`: exact vehicle-center endpoints;
- `drone_radius`, `safety_margin`: non-negative metres;
- `buildings`: axis-aligned boxes with unique IDs;
- `no_fly_zones`: finite vertical cylinders with unique IDs.

`metadata` is descriptive and must not change planner semantics. The canonical SHA-256 scene fingerprint sorts obstacles by ID before serialization so harmless input ordering does not change experiment identity.

## Experiment record

`uav3d plan` and `uav3d benchmark` include:

- scene ID and fingerprint;
- planner ID, complete parameter snapshot, and seed;
- status and explicit failure reason;
- timing and search-effort counters;
- raw and smoothed certified polylines;
- independent audits for both paths;
- smoothing method, sample count, and optional blend factor.

Failed runs contain no fabricated trajectory. `sample-budget-exhausted` for RRT* and graph exhaustion for a voxel planner have different meanings.

## Demo bundle

`web/public/demo-data.json` is a presentation bundle rather than a benchmark database. It declares:

- schema version and generation time;
- `DEMO_NON_CONFIRMATORY` verification status;
- coordinate frame and units;
- planner labels;
- four complete scenes;
- one recorded result per planner and scene;
- scenario fingerprints and RRT* seeds.

The build-time validator recomputes every raw and smoothed polyline length, checks exact endpoints, requires all four scenes and three planners, and rejects any successful record that was not collision-certified by Python.

