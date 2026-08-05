# Methodology

## Scope

This repository establishes a reproducible static-planning baseline. Each experiment has one known bounded city, axis-aligned building volumes, optional finite vertical cylindrical no-fly zones, one start, one goal, and one spherical vehicle model. All coordinates are metres in an east–north–up frame.

The benchmark is intended for algorithm study. It is not a certified motion planner, flight controller, or operational risk assessment.

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
