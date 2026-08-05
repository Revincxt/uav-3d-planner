# UAV 3D Planner Lab

[![CI](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/ci.yml/badge.svg)](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/ci.yml)
[![Pages](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/pages.yml/badge.svg)](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/pages.yml)
[![Python 3.11+](https://img.shields.io/badge/python-3.11%2B-3776ab.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/license-MIT-555.svg)](LICENSE)

A dependency-free Python benchmark for collision-aware UAV planning in three-dimensional cities.
The static study compares **3D A\***, **Lazy Theta\***, and **RRT\***; the separate dynamic study
compares repeated A*, repeated Lazy Theta*, and state-reusing **3D D* Lite** under deterministic
temporary restrictions and moving obstacles. The v0.4 predictive study adds **4D Space-Time A\***
with explicit waits and compares complete schedule access against reactive snapshots. Every study
uses explicit contracts, continuous collision checks, reproducible records, and independent release
audits.

**Web study:** [recorded paths](https://revincxt.github.io/uav-3d-planner-lab/) ·
[benchmark results](https://revincxt.github.io/uav-3d-planner-lab/results.html) ·
[dynamic replanning](https://revincxt.github.io/uav-3d-planner-lab/dynamic.html) ·
[predictive planning](https://revincxt.github.io/uav-3d-planner-lab/predictive.html)

> The trajectory, dynamic, and predictive pages contain non-confirmatory recorded runs. The static
> results page is a descriptive, small-n diagnostic study with scene-clustered intervals. None of
> these pages is confirmatory evidence or a flight-safety system.

## Research question

How do grid-constrained, any-angle, and sampling-based planners differ in path quality and search behavior when the vehicle, buildings, fixed no-fly zones, and safety margin are held constant?

The v0.3 extension asks a separate question: when the same deterministic obstacle schedule is
presented to every planner, how do cold-start and state-reusing geometric replanners differ in
mission outcome, route stability, and algorithm-specific work?

The v0.4 extension asks whether complete deterministic look-ahead can avoid late holds, detours, or
no-safe-action outcomes produced by snapshot-only planning, while separately controlling D* Lite
state reuse with reset and reuse conditions.

| Planner | Search space | Characteristic | Reproducibility |
| --- | --- | --- | --- |
| 3D A* | 26-connected voxel graph | Resolution-complete graph search | Deterministic |
| Lazy Theta* | Same voxel graph with delayed line-of-sight repair | Any-angle parent links | Deterministic |
| RRT* | Continuous bounded 3D space | Seeded sampling and rewiring | Deterministic for a fixed seed and sample budget |

Dynamic baselines intentionally share a conservative snapshot model and exact execution gate:

| Replanner | Reused state | Characteristic |
| --- | --- | --- |
| Repeated 3D A* | None | Solves the current voxel graph from scratch |
| Repeated Lazy Theta* | None | Cold-start any-angle snapshot replanning |
| 3D D* Lite | `g`, `rhs`, priority queue | Updates affected cached edges as the vehicle and scene change |

Predictive conditions keep the v0.3 execution audit but change the information contract:

| Condition | Information | Characteristic |
| --- | --- | --- |
| Repeated 3D A* | Current snapshot | Cold-start reactive baseline |
| 3D D* Lite (reset) | Current snapshot | Fresh backward search at every epoch |
| 3D D* Lite (reuse) | Current snapshot | Controlled incremental-state ablation |
| 4D Space-Time A* | Complete schedule | Six-connected `(voxel, time-step)` search with waits |

For the static benchmark, the primary outputs are planning success, planning time, raw path length,
and minimum clearance. The dynamic and predictive recorded studies omit machine-dependent planning
time and report mission outcomes plus algorithm-specific work units instead. Smoothed length is
secondary because a common post-processing step can hide differences between planners.

## Included scenes

- `open-blocks` — irregular city blocks with several broad route choices.
- `urban-canyon` — alternating narrow gates between tall building slabs.
- `restricted-core` — a full-height cylindrical no-fly zone at the city center.
- `vertical-gate` — a low restricted gate where gaining altitude may help.
- `random-city-<seed>` — deterministic random buildings with a reserved diagonal corridor.

Dynamic episodes are kept separate from the static cohort:

- `pop-up-nfz` — a cylindrical exclusion volume activates across the direct route.
- `crossing-traffic` — a piecewise-linear moving sphere crosses the flight corridor.
- `closing-gate` — a scheduled restriction closes an urban passage.
- `vertical-escape` — a temporary low-altitude block rewards a vertical response.

The v0.4 predictive cohort contains six curated deterministic episodes: `wait-then-straight`,
`closing-window`, `periodic-traffic`, `chained-restrictions`, `multi-obstacle`, and
`vertical-time-window`. Scenario acceptance is based on construction contracts, never planner
outcome. These are diagnostic cases in this source revision, not preregistered held-out data.

Scene distances use metres in an ENU frame. A spherical UAV is represented conservatively by inflating every obstacle by `vehicle radius + safety margin` and shrinking the flight boundary by the same amount.

## Quick start

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .

uav3d scene list
uav3d plan --scene restricted-core --algorithm lazy-theta-star --output result.json
uav3d benchmark \
  --scenes open-blocks,urban-canyon,restricted-core,vertical-gate \
  --algorithms astar-3d,lazy-theta-star,rrt-star \
  --seeds 0,1,2 \
  --output benchmark.json \
  --report-dir artifacts/nominal
```

The seed list applies only to stochastic RRT*. Deterministic planners run once per scene for path
quality; timing repetition is a separate experiment.

Generate a reusable random city:

```bash
uav3d dataset --seeds 40,41,42 --buildings 18 --output-dir artifacts/random-cohort
uav3d plan --scene artifacts/random-cohort/scenes/random-city-42.json \
  --algorithm rrt-star --seed 7
```

Run the v0.2 sensitivity and timing protocols:

```bash
uav3d sweep resolution \
  --scenes open-blocks,urban-canyon,restricted-core,vertical-gate \
  --resolutions 3,4,6,8 \
  --output artifacts/resolution.json

uav3d sweep rrt-budget \
  --scenes open-blocks,urban-canyon,restricted-core,vertical-gate \
  --budgets 250,500,1000,2000,3000 \
  --seeds 11,23,37,47,59 \
  --output artifacts/rrt-budget.json

uav3d timing --repetitions 5 --output artifacts/timing.json
```

RRT* budget curves come from one maximum-budget run per scene and seed; checkpoints do not rerun
shorter budgets. The timing harness randomizes case order and launches every repetition in a fresh
Python process.

Inspect and run the v0.3 dynamic protocol:

```bash
uav3d dynamic list
uav3d dynamic simulate \
  --scenario pop-up-nfz \
  --algorithm dstar-lite-3d \
  --output artifacts/dynamic-run.json

uav3d export-dynamic \
  --output-dir artifacts/dynamic-study \
  --source-commit "$(git rev-parse HEAD)"
```

The simulator uses a deterministic clock, constant cruise speed, scheduled replanning, and a
continuous space-time gate before executing every segment. Its committed records contain no
machine-dependent planner timing; use algorithm-specific work counters for replayable diagnostics.

Inspect and run the v0.4 predictive protocol:

```bash
uav3d predictive list
uav3d predictive plan \
  --scenario wait-then-straight \
  --algorithm space-time-astar-4d \
  --output artifacts/predictive-run.json

uav3d export-predictive \
  --output-dir artifacts/predictive-study \
  --source-commit "$(git rev-parse HEAD)"
```

Space-Time A* is earliest-arrival on its declared `4 m × 0.5 s` lattice, not in continuous
state-time space. Public `waitTimeS` is total stationary time and includes separately labelled
time-lattice alignment waits.

## Web studies

The Vite application has four restrained academic views. The static trajectory view uses Three.js
to inspect recorded paths. The results view uses accessible inline SVG to show voxel-resolution and
RRT* budget sensitivity. The dynamic view synchronizes a Three.js scene with play, pause, stepping,
scrubbing, event annotations, current plans, executed prefixes, and exact outcome tables. The
predictive view adds a time-height trace, explicit wait intervals, paired planner outcomes, and
provenance downloads. No planner runs in the browser.

```bash
cd web
pnpm install --frozen-lockfile
pnpm test
pnpm build
pnpm dev
```

The build validates all committed study datasets. Python separately re-audits every trajectory, resolves
the recorded source commit, recomputes all downloadable-artifact digests, and checks configuration,
run, sample-count, timing, and summary identities.

## Experiment contracts

- Graph planners spend `expanded-nodes`; RRT* spends `sample-attempts`. These units are never treated
  as interchangeable.
- An optional wall-clock limit is a protective termination condition, not an equivalent algorithmic
  budget across machines.
- Every record stores the work limit and usage, termination reason, setup/search timing, semantic
  problem fingerprint, stable run ID, raw-path audit, and separate post-processing outcome.
- Descriptive summaries first aggregate by semantic problem fingerprint, then weight physical
  problems equally even if display IDs differ. Continuous metrics use the median of problem-level
  medians; success uses the mean of problem-level success proportions.
- IQR uses linear Type-7 quantiles. The 95% percentile interval uses 10,000 fixed-seed scene-clustered
  bootstrap resamples.
- Dynamic planners receive identical event schedules, simulation clocks, snapshot geometry,
  resolutions, and work limits. Frames within one mission are not independent observations.
- Dynamic and predictive replay records omit wall-clock planner time. Expanded spatial nodes,
  D* Lite queue pops, and expanded space-time states are algorithm-specific work indicators and are
  never presented as equivalent units.
- Predictive contrasts use the scenario fingerprint as the independent unit. Frames, waits, and
  replanning epochs are not counted as samples.
- The reactive conditions use 26-connected snapshot graphs, a 1 s execution clock, periodic
  replanning, and a per-replan work cap. Space-Time A* uses a six-connected 0.5 s lattice, plans once
  from the complete schedule, and has a per-mission expansion cap. Their comparison describes the
  full information-and-control protocols; it does not isolate a causal forecast effect or impose an
  equal computational budget.

## Safety and validation model

- AABB intersection uses a continuous slab test.
- Finite vertical cylinders use an exact parameter-interval test in `xy` and `z`.
- Every 26-neighbor grid edge is checked continuously, so diagonal corner cutting is rejected.
- Start and goal remain exact virtual endpoints connected to all visible vertices in a local `3 x 3 x 3` stencil.
- Scene contracts require distinct start and goal points so path-excess metrics have a nonzero lower bound.
- Fixed-work RRT* uses `random.Random(seed)` and records incumbent quality at requested checkpoints.
- The shared post-processor first takes deterministic farthest-visible shortcuts, proposes a sampled cubic B-spline blend, validates every resulting segment, and falls back to the certified shortcut if needed.
- An independent audit rechecks endpoints, collision status, path length, waypoint count, and sampled clearance before export.
- Temporary restrictions use half-open active intervals. Moving-sphere collision uses exact relative
  motion on every keyframe segment; touching an inflated dynamic obstacle is collision.
- The dynamic simulator audits each executed segment in space and time. An unsafe proposal may
  trigger one immediate replan and can become a hold only when remaining stationary is safe.
- Space-Time A* audits every movement, explicit wait, and exact endpoint connector against the same
  continuous space-time predicates used by the independent release validator.

See [Methodology](docs/methodology.md) for definitions, assumptions, statistical estimands, and
comparison limits.

## Reproducing checks

```bash
python -m pip install -e '.[dev]'
ruff check .
ruff format --check .
mypy src
pytest --cov=uav3d --cov-report=term-missing
python -m build

python scripts/export_scenarios.py --check
python scripts/validate_committed_data.py
cd web && pnpm validate:data && pnpm test && pnpm typecheck && pnpm build
```

The Python package has no runtime dependencies. CI tests supported Python versions, builds and
installs a wheel, smoke-tests the CLI, re-audits committed study data, and builds all four Pages
views.

## Repository layout

```text
src/uav3d/             static/dynamic/predictive planning, simulation, reports, and CLI
scenarios/             four committed curated scene JSON files and schema
tests/                 contracts, collision, planning, smoothing, CLI
web/                   four recorded academic study views and their validators
docs/                  methodology, schemas, and roadmap
.github/workflows/     CI and GitHub Pages deployment
```

## Current limits

Version `0.4.0` supports deterministic complete-schedule prediction on a finite 4D lattice while
retaining a spherical vehicle and constant-speed geometric motion. Its six-scenario, 24-run bundle
is non-confirmatory. It does not model attitude, acceleration, wind, sensing or forecast
uncertainty, energy, minimum-snap trajectories, PX4/MAVLink export, or regulatory compliance.

Planned extensions are listed in the [roadmap](docs/roadmap.md). Kinodynamic trajectory generation,
uncertain prediction, and flight-stack integration remain separate experiments so they do not blur
the deterministic predictive baseline.

## References

- Hart, Nilsson, and Raphael. “A Formal Basis for the Heuristic Determination of Minimum Cost Paths.” *IEEE Transactions on Systems Science and Cybernetics*, 1968.
- Nash, Koenig, and Tovey. “Lazy Theta*: Any-Angle Path Planning and Path Length Analysis in 3D.” *AAAI*, 2010.
- Karaman and Frazzoli. “Sampling-based Algorithms for Optimal Motion Planning.” *International Journal of Robotics Research*, 2011.
- Koenig and Likhachev. “D* Lite.” *AAAI*, 2002.

## Citation and license

Citation metadata is provided in [CITATION.cff](CITATION.cff). The source is released under the [MIT License](LICENSE).
