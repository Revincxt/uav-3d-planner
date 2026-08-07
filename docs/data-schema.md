# Data schemas

## Scene JSON

Committed curated scenes live in `scenarios/` and conform to `scenarios/schema.json`.

Required physical fields:

- `bounds.minimum`, `bounds.maximum`: 3D ENU bounds in metres;
- `start`, `goal`: exact vehicle-center endpoints;
- `drone_radius`, `safety_margin`: non-negative metres;
- `buildings`: axis-aligned boxes with unique IDs;
- `no_fly_zones`: finite vertical cylinders with unique IDs.

`metadata` is descriptive and must not change planner semantics. The v2 semantic problem
fingerprint excludes IDs, names, obstacle labels, and metadata; normalizes every number with
`float.hex()`; and sorts obstacles by geometry. Integer and floating-point JSON spellings therefore
produce the same fingerprint for the same physical problem.
Start and goal must be distinct because path excess is normalized by their Euclidean separation.

## Experiment record

`uav3d plan`, `uav3d benchmark`, and both sweep commands emit schema `2.0` records with:

- stable run ID, run purpose, scene ID, and semantic problem fingerprint;
- planner ID, configuration ID, complete parameter snapshot, and nullable planner seed;
- primary status, explicit failure reason, raw-path validity, and separate post-processing status;
- algorithmic budget, optional wall-clock limit, observed usage, and termination reason;
- planning, setup, search, smoothing, and validation timings;
- search-effort counters and optional RRT* quality checkpoints;
- raw and smoothed certified polylines;
- independent audits for both paths;
- smoothing method, sample count, and optional blend factor.

Failed runs contain no fabricated trajectory. `budget-exhausted`, `timeout`, `no-path`, and `invalid`
remain distinct top-level outcomes. Algorithm-specific reasons such as
`sample-budget-exhausted`, `expansion-budget-exhausted`, and `graph-exhausted` are preserved.

## Dataset manifest

`uav3d dataset` writes `manifest.json` plus accepted scene files. The manifest stores the generator
name and version, complete generation parameters, requested/accepted/rejected counts, and one record
per seed. Accepted records include the semantic fingerprint and relative scene path. Rejected records
include the exception type and reason. No planner is invoked during dataset acceptance.

## Timing bundle

`uav3d timing` writes schema `2.0` data containing the randomized order seed, fixed planner seed,
repetition count, process timeout, Python/package/platform context, and one record per isolated child
process. Process status and wall time are distinct from the nested planner record and planner time.

## Reports

Passing `--report-dir` to a nominal benchmark or sweep creates:

- `records.csv`: one metric/provenance row per run, including parameters, validity flags, both
  budget dimensions, observed usage, and quality traces while excluding bulky paths;
- `summary.csv`: tidy metric summaries with conditioning and sample counts;
- `summary.json`: the scene-clustered statistical contract and planner summaries;
- `planner-summary.svg`: an accessible dependency-free interval figure;
- `checksums.json`: SHA-256 and byte size for every exported report file.

## Demo bundle

`web/public/demo-data.json` is a presentation bundle rather than a benchmark database. It declares:

- schema version and generation time;
- `DEMO_NON_CONFIRMATORY` verification status;
- coordinate frame and units;
- planner labels;
- four complete scenes;
- one recorded result per planner and scene;
- scenario fingerprints and RRT* seeds.

The JavaScript build-time validator recomputes every raw and smoothed polyline length, checks exact
endpoints, and requires all four scenes and three planners. The separate Python release audit reloads
the physical scenes and independently collision-certifies every committed trajectory.

## Benchmark web bundle

`web/public/benchmark-data.json` is schema version `2` and carries the
`DESCRIPTIVE_BENCHMARK` evidence label. It declares:

- full source revision, protocol ID, bootstrap seed/resamples, path-quality seeds, and timing
  repetitions;
- diagnostic dataset counts and the SHA-256 of `dataset-manifest.json`;
- explicit nominal and sensitivity budgets;
- exactly one dataset-level summary per planner;
- graph-planner voxel-resolution sensitivity;
- RRT* sample-budget sensitivity.
- paths, byte sizes, and SHA-256 digests for the records CSV, summary CSV, dataset manifest, and
  timing manifest.

Metric summaries state their estimator and conditioning. Undefined successful-run metrics are JSON
`null`; `nDefinedScenes` makes their actual problem support explicit, and NaN/zero substitution is
forbidden. Browser validation checks interval ordering,
counts, budget references, artifact digests, and schema invariants. The nominal table and summary CSV
report path-quality and timing sample counts separately. `scripts/validate_committed_data.py`
independently re-audits trajectories, resolves the source revision, recomputes configuration and run
IDs, verifies every download digest, and cross-checks CSV/timing sample counts against the JSON bundle.

## Dynamic scenario and run

The Python `dynamic-scenario-v1` contract contains one complete static scene plus:

- `temporary_cylinders`: finite cylinders with an inclusive `active_from` and exclusive
  `active_until` time in seconds;
- `moving_spheres`: positive radii and at least two strictly increasing keyframes, each carrying a
  simulation time and ENU position;
- a semantic dynamic fingerprint that excludes labels and IDs while retaining all geometry and
  schedule values.

`dynamic-run-v1` records the scenario fingerprint, replanning algorithm, complete simulation
parameters, deterministic frames, and mission metrics. Each frame stores its simulation time,
vehicle position, current planned polyline, replanning status/reason, algorithm-specific work, and
changed-edge count. Planner wall-clock timing is intentionally absent from this deterministic
record. Public floating-point records are rounded to 12 decimal places at the serialization boundary
so all supported Python versions emit the same evidence bytes.

## Dynamic Web bundle

`web/public/dynamic-data.json` is schema version `1` with the evidence label
`DYNAMIC_DEMO_NON_CONFIRMATORY`. It contains exactly four curated dynamic scenarios and one run for
each of the three replanning baselines. Presentation frames additionally materialize:

- the complete executed prefix ending at the current vehicle position;
- active temporary-zone IDs under half-open interval semantics;
- every moving-sphere position at the frame time;
- a structured event annotation and nullable planner timing field;
- the current path, per-frame work, and D* Lite edge-change count.

The bundle points to `dynamic-records.csv` and `dynamic-scenario-manifest.json` with byte counts and
SHA-256 digests. JavaScript validates schema, endpoint, time-order, run-count, event, path, and
artifact contracts. The Python release audit resolves the recorded source revision, reconstructs all
12 deterministic episodes, compares every non-timing field, rechecks executed space-time segments,
and cross-checks the CSV and manifest.

## Predictive path and Web bundle

`predictive-run-v2` contains a semantic dynamic-scenario fingerprint, one of four planner-condition
IDs, numeric protocol parameters, a raw timestamped planner path, a certified timestamped execution
path, smoothing diagnostics, and mission metrics. `TimedPath` timestamps must increase strictly.
Its first action is `start`; each later segment is explicitly classified as `move` or `wait`, and
that classification must agree with its endpoint positions.

`web/public/predictive-data.json` is schema version `2`, uses protocol identifier
`predictive-space-time-v3`, and carries the verification status
`PREDICTIVE_DEMO_NON_CONFIRMATORY`. Version 0.6 contains ten scenarios and one run for each
condition, producing 40 deterministic mission records. The added `braided-skyway` and
`harbor-switchback` cases are extended-city diagnostics with 20 unequal-height buildings, one
static no-fly volume, two temporary restrictions, and two moving hazards each. The four conditions
remain:

- repeated 3D A*;
- D* Lite reset at every epoch;
- D* Lite with state reuse;
- 4D Space-Time A* with the complete deterministic schedule.

Every run exports `rawTimedPath` and `timedPath`. The former is the direct planner/simulator output;
the latter is either a common sampled circular-fillet polyline that passed the continuous
space-time audit or an explicit certified raw fallback. The smoothing object records the method,
`collisionCertified`, its dense-piecewise-linear certification scope, input/output waypoint counts,
rounded-corner count, requested/applied radius, sample spacing, and maximum sampled turn angle before
and after processing. Wait blocks are hard boundaries: their positions and absolute timestamps are
preserved.

`kinematicDiagnostics` contains `raw` and `output` measurements derived from segment-average
velocities: segment and movement counts, reversal count and threshold, maximum speed, maximum
adjacent velocity change, a finite-difference acceleration proxy, and maximum absolute climb rate.
Both the wrapper and each path record declare `continuousDynamicsCertified = false` (or the
snake-case equivalent). These are waypoint diagnostics, not bounds on continuous acceleration,
attitude, curvature, jerk, or actuator dynamics.

Explicit wait intervals retain their reasons. `waitTimeS` is total stationary time, including short
time-lattice alignment waits. Work units are carried in each metric record and are not normalized
across algorithms. Compact presentation frames retain only semantic event anchors: timestamp,
vehicle position, active temporary-zone IDs, moving-sphere states, and event annotation. Path arrays
and uneventful dense smoothing samples are not duplicated inside frames; the Web client derives
continuous positions, prefixes, and suffixes from `timedPath`.

Predictive metrics include `minimumSeparationM` and `minimumSeparationWitness`. Separation is the
physical surface-to-surface distance after subtracting the vehicle radius; the declared safety
margin is stored separately in the witness. The witness contains its time, vehicle position,
nearest obstacle-surface position, obstacle ID/kind, method, `exact` flag, and declared margin.
Moving-sphere witnesses are exact under piecewise-linear relative motion. Temporary-cylinder
witnesses use a deterministic convex distance search and explicitly carry `exact: false`. Neither
field changes the independent continuous collision verdict.

`predictive-records.csv` projects the same closest-approach distance, witness time, obstacle
identity/kind, and exactness flag. It also carries raw/output reversal counts and the output
velocity-change, acceleration-proxy, and climb-rate diagnostics so downloaded records can be
audited without parsing the dense trajectory arrays.

The bundle references `predictive-records.csv` and `predictive-scenario-manifest.json` by byte size
and SHA-256. Python reruns all 40 deterministic missions, independently collision-audits both raw
and certified timestamped paths, and cross-checks the JSON/CSV/manifest identities. JavaScript
independently validates structural geometry bounds, temporal order, endpoints, waits, smoothing,
frames, metrics, digests, and run-count invariants; it does not repeat the continuous collision
calculation. Predictive public floats are rounded to eleven decimal places at the serialization
boundary so platform-level `libm` drift cannot change deterministic evidence bytes.
