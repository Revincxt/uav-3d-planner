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

Start and goal are not silently rounded to one voxel. Each is a virtual node connected to every collision-free lattice vertex in the local `3 x 3 x 3` stencil around its nearest index. If that stencil has no valid anchor, the implementation searches for the nearest visible free vertex. A direct start-to-goal edge is used when visible. Every reported path retains the exact supplied endpoints, and the connector lengths are included in metrics.

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

## Metrics

- **Success**: the planner returned an exact-endpoint path and both raw and post-processed audits passed.
- **Planning time**: wall time inside the planner, including its data-structure setup. It excludes smoothing and JSON export.
- **Raw length**: Euclidean length of the planner's exact-endpoint polyline.
- **Smoothed length**: Euclidean length after the shared pipeline.
- **Minimum clearance**: minimum sampled distance from the vehicle surface to physical obstacles or boundary. Collision status itself remains continuous and exact.
- **Expanded nodes / samples**: planner-specific work indicators. They are not interchangeable units of computational budget.

Clearance sampling defaults to `0.5 m`. It is a descriptive approximation; collision certification does not use that sampling.

## Comparison protocol

For a formal evaluation:

1. freeze map-generation seeds separately from RRT* planner seeds;
2. store the scene fingerprint with every run;
3. calibrate parameters on maps excluded from final evaluation;
4. report success rate before conditional path-quality metrics;
5. report deterministic planners once per scene for path quality and repeat only timing measurements;
6. run RRT* under multiple seeds but treat those runs as repeated measurements within a scene;
7. randomize execution order and record hardware/software context;
8. perform a voxel-resolution sweep for the graph planners;
9. keep raw-path findings primary and smoothing findings secondary;
10. retain failed scenes instead of filtering them from the dataset.

The recorded web demo deliberately does not aggregate a single seed into a general conclusion.

