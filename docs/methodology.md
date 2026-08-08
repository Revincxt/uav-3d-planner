# Methodology

## Scope

This repository establishes a reproducible static-planning baseline. Each experiment has one known bounded city, axis-aligned building volumes, optional finite vertical cylindrical no-fly zones, one start, one goal, and one spherical vehicle model. All coordinates are metres in an east–north–up frame.

The benchmark is intended for algorithm study. It is not a certified motion planner, flight controller, or operational risk assessment.

Version 0.3 preserves this static baseline and adds a separate dynamic-replanning experiment.
Version 0.7 extends the predictive track with a frozen discrete execution envelope while retaining
the planner, geometry, and collision models used by the v0.6 cohort. Neither extension introduces a
flight controller, continuous vehicle dynamics, wind, or sensing uncertainty.

## Shared collision world

Let the vehicle radius be `r` and the requested safety margin be `m`. The center of the vehicle must remain at least

```text
c = r + m
```

away from forbidden physical volume.

The implementation applies this rule consistently:

- every building AABB expands by `c` along all three axes;
- every cylindrical zone expands radially by `c`, with its lower and upper altitude expanded by `c`;
- every face of the world boundary moves inward by `c`;
- touching an inflated obstacle counts as collision;
- touching the contracted world boundary is allowed within numerical tolerance.

The box and cylinder inflation is a conservative representation at corners for a spherical vehicle. It is deliberately shared by all planners.

Segment/AABB queries use the slab intersection algorithm. Segment/cylinder queries intersect the segment's valid `z` parameter interval with the interval in which its projected `xy` line lies inside the expanded disk. Planning never relies on fixed-step collision sampling.

## Exact endpoints and voxel graph

3D A* and Lazy Theta* use the same lattice and 26-connected local graph. Edge cost is three-dimensional Euclidean distance, and every local edge undergoes the same continuous segment check.

Start and goal are not silently rounded to one voxel. Each is a virtual node connected to every collision-free lattice vertex in the local `3 x 3 x 3` stencil around its nearest index. If that stencil has no valid anchor, the implementation searches for the nearest visible free vertex. A direct start-to-goal edge is used when visible. Every reported path retains the exact supplied endpoints, and the connector lengths are included in metrics. Scene contracts reject coincident endpoints because the study's path-excess denominator must be strictly positive.

## Planner configurations

### 3D A*

- 26-connected neighbors;
- Euclidean edge costs and Euclidean admissible heuristic;
- stable heap tie breaking;
- default resolution `4 m`;
- default expansion limit `120,000`.

Exhausting the graph means no path was found at the chosen discretization; it does not prove the continuous scene is infeasible.

### Lazy Theta*

Lazy Theta* shares the voxel graph but allows any-angle parent links. Successor updates optimistically inherit the current node's parent without an immediate line-of-sight query. When a node is removed from the open queue, `SetVertex` checks that parent link and repairs it using a visible closed neighbor if needed.

This delayed check distinguishes the implementation from ordinary Theta*. It does not imply that every Lazy Theta* path must be shorter than every A* path.

### RRT*

- continuous uniform sampling inside the contracted world boundary;
- deterministic `random.Random(seed)` stream;
- default attempt budget `3,000`;
- goal bias `0.12`;
- maximum steering distance `7 m`;
- virtual goal connection within `9 m`;
- rewiring radius `min(r_max, gamma (log n / n)^(1/3))`;
- lowest-cost safe parent selection and descendant cost propagation;
- ancestor checks before rewiring to prevent parent cycles;
- final rescan of every goal-connectable node after rewiring.

RRT* continues until the fixed sample-attempt budget even after finding a first solution. A finite-budget result is the best path in that run, not a claim of optimality or continuous infeasibility.

## Budget and timing contract

Each planner exposes the same two-part finite-budget contract:

- an algorithmic work unit and positive work limit;
- an optional positive wall-clock protection limit.

The graph planners consume expanded nodes. RRT* consumes sample attempts, including rejected
samples. The two work units describe different algorithms and are not comparable measures of equal
effort. When both limits exist, planning stops at the first limit reached. A wall-clock stop may vary
across machines and therefore is not used as the primary path-quality budget.

The wall-clock deadline begins at planner entry. Planner timing is divided into:

- **setup**: endpoint checks, direct line of sight, grid anchoring, or tree initialization;
- **search**: the search loop, goal scan, path reconstruction, and requested trace observations.

Smoothing and independent audits are timed separately. Isolated-process timing repetitions use a
fixed algorithmic budget, one fresh Python process per case, randomized execution order, and a
planner seed distinct from the repetition index. Process startup time is recorded by the harness but
is not included in planner time.

RRT* can record the best goal-connectable path at fixed sample-attempt checkpoints. A single maximum
budget run supplies the full curve, preserving one common random prefix. Once an incumbent exists,
its recorded length must be non-increasing within that run.

## Shared smoothing pipeline

Raw planner paths are the primary algorithm output. Successful paths enter the same post-processing sequence:

1. remove duplicate consecutive points;
2. take a deterministic farthest-visible shortcut polyline;
3. sample a clamped cubic B-spline candidate;
4. if necessary, blend that candidate toward an arc-length resampling of the certified shortcut;
5. connect the samples as a polyline and check every segment continuously;
6. return the first certified candidate, otherwise return the shortcut fallback;
7. audit the final path again outside the smoothing module.

The exported “smoothed path” is the certified sampled polyline, not an unverified analytic spline. A browser or downstream consumer must not interpolate it again without another collision check.

## Dynamic-replanning protocol

A dynamic scenario combines one immutable static `Scene` with two schedule types:

- a finite vertical cylinder active on the half-open interval `[active_from, active_until)`;
- a moving sphere with strictly increasing timestamped keyframes and piecewise-linear motion.

Scenario identity hashes the static semantic problem, every time interval, sphere radius, keyframe
time, and keyframe position. Labels, IDs, metadata, and record order do not affect identity. The four
committed scenarios are curated diagnostic episodes; they are not selected by planner outcome.

The simulator advances on a deterministic clock. At each planning epoch the planner receives the
same static snapshot, exact current vehicle position, goal, resolution, and work limit. The vehicle
then follows the returned polyline at constant cruise speed for at most one simulation step. A
continuous space-time safety gate checks every proposed executed segment before motion. If the
segment is unsafe, the planner receives one immediate replanning opportunity; if no safe movement is
available, the vehicle may hold only when remaining stationary is safe for the whole step.

Dynamic collision checks retain the static continuous predicates. Temporary-cylinder checks
intersect the geometric contact interval with the half-open activation interval. Moving-sphere
checks subtract the sphere's piecewise-linear motion from the vehicle segment and minimize distance
to the resulting relative segment. Keyframe boundaries split the query, so contact between samples
cannot be skipped. Touching an inflated dynamic obstacle counts as collision.

### Replanning baselines

- **Repeated 3D A\*** discards search state and solves the current 26-connected voxel graph again.
- **Repeated Lazy Theta\*** likewise starts from scratch but permits any-angle parent links.
- **3D D\* Lite** retains `g`, `rhs`, and its priority queue across compatible snapshots, moves the
  start as the vehicle advances, and updates vertices incident to changed cached edges.

All three planners use the same conservative snapshot representation. A moving sphere becomes a
finite vertical cylinder at its instantaneous position for planning, while executed motion is still
certified against the exact moving sphere. The snapshot is therefore conservative and the execution
audit remains the safety authority.

Endpoint handling is recorded as part of each planner contract. All three return the same exact
start-to-goal segment when it is directly visible. In obstructed cases, the current D* Lite baseline
uses its nearest visible voxel anchor, whereas the repeated graph planners can consider multiple
visible endpoint anchors. Consequently, route-quality differences remain diagnostic and may reflect
both replanning strategy and endpoint anchoring; this release does not claim a controlled
search-state-only ablation.

The deterministic study records simulation time rather than planner wall-clock time. Per-replan work
is expanded nodes for the repeated planners and queue pops for D* Lite; these units characterize each
algorithm and are not treated as equal computational effort. Machine-dependent timing belongs in a
separate isolated-process experiment.

### Dynamic outcomes

- mission success and explicit failure reason;
- collision count for independently audited executed segments;
- completion time on the deterministic simulation clock;
- executed distance and excess over the direct start-goal distance;
- scheduled and safety-triggered replans, failed replans, and safe holds;
- total algorithm-specific planning work and D* Lite changed-edge count.

Every committed Web episode stores all frames needed to reconstruct the vehicle position, current
plan, executed prefix, active temporary zones, moving-sphere positions, replanning reason, and work
usage. Re-running the same source revision and protocol must reproduce the non-timing records exactly.

## Predictive space-time protocol

Version 0.7 freezes the v0.6 predictive cohort and planner configurations, then separates planner
output, geometric post-processing, and an optional retimed execution candidate. A `TimedPath`
carries a strictly increasing timestamp at every exact endpoint, grid waypoint, and stationary
interval. Equal-position segments must be explicit `wait` actions.

The predictive baseline searches states `(v, k)`, where `v` is a free spatial voxel and `k` is a
discrete time layer. The frozen public configuration uses a `4 m` spatial lattice, `0.5 s` time
layers, `8 m/s` cruise speed, a `90 s` horizon, six axis-aligned movement actions, and a one-layer
wait action. Every candidate transition calls the same continuous space-time predicate as the
release audit. Exact start and goal points remain virtual endpoints connected at fixed speed; if a
connector arrives between layers, the vehicle must safely wait until the next layer before entering
the voxel-time graph.

Space-Time A* uses deterministic tie breaking and an admissible Euclidean travel-time heuristic.
Its result is earliest-arrival on the declared finite lattice. Temporal and spatial discretization
mean it is not a claim of globally earliest arrival in continuous state-time space. A finite horizon
failure likewise does not prove that no continuous trajectory exists.

### Predictive comparison conditions

- **Repeated 3D A\*** sees only the current conservative snapshot and replans from scratch.
- **D\* Lite reset** sees the same snapshot and constructs a fresh D* Lite planner at every epoch.
- **D\* Lite reuse** keeps `g`, `rhs`, and queue state across compatible snapshots.
- **4D Space-Time A\*** sees the complete deterministic schedule at mission start.

Reset and reuse D* Lite share one graph, endpoint, heuristic, clock, work cap, and execution
contract, so they form a controlled search-state ablation. The broader Space-Time A* contrast also
changes the information model, 6- versus 26-connected graph, 0.5 versus 1 s temporal discretization,
one-shot versus periodic planning, safety-gate behavior, and budget scope. It therefore describes
two complete information-and-control protocols rather than a pure causal estimate of forecast
value.

### Frozen study matrix and analysis unit

Version 0.7 uses the same ten v0.6 scenarios and four planner conditions, for 40 deterministic
planner-condition records. There are no outcome-dependent additions or removals. The scenario is
the independent unit (`n = 10`); the four conditions are paired within scenario. Planner-condition
rows, playback frames, waits, and replanning epochs are repeated or nested observations, not
additional independent samples.

The ten scenarios are accepted by construction and metadata validation before any planner is
invoked. One calibration, three demonstration, and six curated diagnostic labels are identified by
semantic fingerprints. Seven non-calibration scenes contain 14–16 unequal-height buildings;
`braided-skyway` and `harbor-switchback` each contain 20 buildings, one static no-fly volume, two
altitude-selective temporary restrictions, and two moving hazards. The cases were not preregistered
in an earlier immutable revision, so the 40 records remain non-confirmatory diagnostic evidence.
Version 0.6 is a paired historical baseline and must not be pooled with v0.7 as an independent
cohort; v0.5 and v0.4 rows likewise do not add independent scenarios.

The frozen public identifiers are schema version `3`, protocol
`predictive-space-time-v4`, and dataset `predictive-execution-envelope-v0.7`. All execution-envelope
thresholds are declared before regeneration. The number of qualified execution candidates is an
observed result, not a release gate or a parameter-tuning target.

Expanded spatial nodes, D* Lite queue pops, and expanded space-time states remain
algorithm-specific work indicators rather than interchangeable measures of equal effort. The
240,000 cap applies to each reactive replanning call but to the complete Space-Time A* mission
search, so cumulative reactive work may exceed that number.

### Three evidence layers

Each successful planner record retains three explicitly named domains:

1. `rawTimedPath` is the direct planner or online-policy result. `plannerMetrics` is computed only
   from this path and contains the planner success/failure outcome, raw arrival and wait time, raw
   length and path excess, replans, algorithm-specific work, safety audit, and raw
   closest-approach witness.
2. `geometryTimedPath` is the common downstream geometric candidate under the original timing.
   `geometryMetrics` is recomputed from this candidate. Geometry changes are never attributed to
   the planner.
3. `executionTimedPath` is an optional duration-only retiming of the geometry candidate.
   `executionMetrics` exists only for a published execution candidate and is never substituted for
   planner-domain results.

The geometry stage divides a path at each wait, proposes sampled three-dimensional circular fillets
for eligible movement corners, and deterministically tries radius scales of `1.0`, `0.75`, `0.5`,
and `0.25` from a requested `6 m`. Samples are spaced no farther than `0.5 m`; intermediate
timestamps are assigned by cumulative chord length while each movement block keeps its original
departure and arrival time. A candidate must pass the continuous space-time collision predicate;
otherwise the stage records an explicit certified raw-geometry fallback. The output is a dense
piecewise-linear path, not an analytic spline or curvature certificate.

### Discrete execution envelope and retiming

The optional execution candidate is evaluated against this frozen waypoint-level envelope:

| Quantity | Limit |
| --- | ---: |
| Segment-average speed | `8 m/s` |
| Absolute segment-average climb or descent rate | `3 m/s` |
| Boundary-aware discrete acceleration proxy | `4 m/s²` |
| Reversal threshold | `150°` |
| Allowed non-zero-speed reversals | `0` |
| Maximum candidate arrival time | `90 s` |

The acceleration proxy uses changes between segment-average velocity vectors. Consecutive movement
segments use the time between their segment midpoints; every movement block begins and ends at a
zero-speed boundary. Waits also delimit movement blocks, and their duration is not counted as extra
braking time. This boundary-aware definition exposes abrupt start, stop, and move-to-wait changes
that an interior-only finite difference could omit or dilute.

Retiming is deterministic and keeps the geometry and action sequence fixed. It may only increase
movement-segment durations; it preserves each explicit wait duration, although later absolute
timestamps may move. Speed and climb constraints are applied first, followed by finite local
duration-growth sweeps for the acceleration proxy. A reversal at or above `150°` is not repaired by
time dilation because slowing cannot change direction geometry. Failure records an explicit status
and leaves `executionTimedPath` null; it never silently promotes a geometry path to an execution-
qualified result.

Every non-null retimed candidate is subjected to the complete continuous space-time collision audit
again. This second audit is required because delayed timestamps change the positions and activation
states of dynamic obstacles. Qualification therefore requires both the discrete envelope and the
post-retiming collision verdict within the `90 s` limit.

Passing this envelope is not continuous-dynamics feasibility or flight certification. The model does
not bound continuous acceleration, curvature, attitude, thrust, jerk, actuator response, minimum
snap, wind response, or tracking error, and it does not establish regulatory or operational safety.

In the committed v0.7 bundle, 5 of 40 candidates qualify. Thirty-four fail the frozen 90 s limit
and one fails the repeated dynamic collision audit after its timestamps are delayed. These counts
are reported descriptively within the ten paired scenarios; they were not used to alter the
envelope, select scenarios, or rank planner families.

### Dynamic minimum-separation diagnostic

Each metric domain may carry its own `minimumSeparationM` and witness. Separation is physical
surface-to-surface distance after subtracting the spherical vehicle radius; the declared safety
margin remains a separate threshold. The associated witness records time, vehicle position,
nearest obstacle-surface position, obstacle identity and kind, method, declared margin, and whether
the closest-approach calculation is exact. A geometry or execution witness must not be presented as
the planner-path witness.

For a moving sphere, closest approach is exact under the declared piecewise-linear vehicle and
obstacle motion model. For an active temporary cylinder, the implementation uses a deterministic
one-dimensional convex distance search and records `exact = false`. This diagnostic is independent
of collision certification: it describes the closest recorded approach but does not certify
continuous vehicle dynamics, forecast validity, or operational risk.

Python independently reruns all 40 public missions and collision-audits raw, geometry, and every
non-null execution path against the declared dynamic schedule. It also checks JSON, CSV,
scenario-manifest identities, run IDs, source ancestry, required generator files, byte sizes, and
SHA-256 digests. The Web build independently validates structural, numeric, temporal, evidence-layer,
and digest invariants; it does not reimplement collision geometry. Public floats are rounded to 11
decimal places only at the serialization boundary.

The complete frozen design and release criteria are recorded in the
[v0.7 execution-envelope experiment plan](v0.7-execution-envelope-plan.md).

## Experiment identity

`problem_fingerprint` is a versioned SHA-256 digest of planning semantics: bounds, exact endpoints,
vehicle radius, safety margin, building geometry, and finite-cylinder geometry. It excludes scene
ID, name, obstacle labels, and descriptive metadata. Numeric values are normalized through their
floating-point representation, so `1` and `1.0` identify the same problem. Obstacles are sorted by
geometry before hashing.

Every v2 run also receives a stable `run_id` derived from the problem fingerprint, algorithm
configuration, budget, planner seed, run purpose, and optional timing repetition. A full source-file
digest is provenance for the document, not for physical problem identity.
Configuration parameters use the same canonical floating-point representation, so equivalent Python
spellings such as `4` and `4.0` produce the same configuration and run identities.

## Metrics

- **Success**: the planner returned an exact-endpoint raw path and the independent raw audit passed.
- **Post-processing status**: whether the separate smoothing pipeline returned a certified path. A
  smoothing failure does not erase a valid planner solution.
- **Planning time**: setup plus search time inside the planner. It excludes smoothing, audit, process
  startup, and serialization.
- **Raw length**: Euclidean length of the planner's exact-endpoint polyline.
- **Smoothed length**: Euclidean length after the shared pipeline.
- **Minimum clearance**: minimum sampled distance from the vehicle surface to physical obstacles or boundary. Collision status itself remains continuous and exact.
- **Expanded nodes / samples**: planner-specific work indicators. They are not interchangeable units of computational budget.

Clearance sampling defaults to `0.5 m`. It is a descriptive approximation; collision certification does not use that sampling.

## Comparison protocol

For a formal evaluation:

1. freeze map-generation seeds separately from RRT* planner seeds;
2. store the semantic problem fingerprint and stable run ID with every run;
3. calibrate parameters on maps excluded from final evaluation;
4. report success rate before conditional path-quality metrics;
5. report deterministic planners once per scene for path quality and repeat only timing measurements;
6. run RRT* under multiple seeds but treat those runs as repeated measurements within a scene;
7. randomize execution order and record hardware/software context;
8. perform a voxel-resolution sweep for the graph planners;
9. keep raw-path findings primary and smoothing findings secondary;
10. retain failed scenes instead of filtering them from the dataset.

Dataset generation records every requested seed as accepted or rejected. Acceptance depends only on
scene construction, never on the outcome of a planner under comparison. The identity

```text
requested scenes = accepted scenes + rejected scenes
```

must hold in every manifest.

## Descriptive statistics

RRT* seeds are repeated measurements nested within a scene. Deterministic planners contribute one
path-quality run per scene; their repeated processes contribute only to timing. Analysis proceeds in
two stages:

1. compute a problem-level success proportion or median for each semantic problem fingerprint and
   planner condition;
2. aggregate those problem-level values while giving every physical problem equal weight.

Success is the mean of scene success proportions. Continuous metrics are the median of scene
medians. Q1 and Q3 use linear Type-7 quantiles. The descriptive 95% interval is the 2.5th and 97.5th
percentile of 10,000 scene-clustered bootstrap resamples generated with seed `20260805`. Path length,
path excess, and clearance condition on successful raw paths; success rate retains every attempt.
Missing values remain null and are never replaced by zero or infinity.

The committed benchmark page uses only four curated diagnostic scenes and is labeled
`DESCRIPTIVE_BENCHMARK` and exploratory small-n. It illustrates the protocol; it is not a powered or
confirmatory comparison. Larger evaluations should generate an independently seeded random cohort
and declare calibration and held-out splits before running planners.

The committed dynamic page follows the same restraint: four curated episodes × three planners are
recorded as `DYNAMIC_DEMO_NON_CONFIRMATORY`. Frames within an episode are repeated observations, not
independent samples, and the page does not rank planners or make confirmatory claims.
