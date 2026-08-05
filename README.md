# UAV 3D Planner Lab

[![CI](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/ci.yml/badge.svg)](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/ci.yml)
[![Pages](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/pages.yml/badge.svg)](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/pages.yml)
[![Python 3.11+](https://img.shields.io/badge/python-3.11%2B-3776ab.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/license-MIT-555.svg)](LICENSE)

A dependency-free Python benchmark for collision-aware UAV path planning in static
three-dimensional cities. It compares **3D A\***, **Lazy Theta\***, and **RRT\*** under one
geometry model, with explicit budgets, reproducible parameter sweeps, scene-aware statistics, and
the same collision-certified smoothing pipeline.

**Web study:** [recorded paths](https://revincxt.github.io/uav-3d-planner-lab/) ·
[benchmark results](https://revincxt.github.io/uav-3d-planner-lab/results.html)

> The trajectory page contains non-confirmatory single runs. The results page is a descriptive,
> small-n diagnostic study with scene-clustered intervals. Neither page is confirmatory evidence or
> a flight-safety system.

## Research question

How do grid-constrained, any-angle, and sampling-based planners differ in path quality and search behavior when the vehicle, buildings, fixed no-fly zones, and safety margin are held constant?

| Planner | Search space | Characteristic | Reproducibility |
| --- | --- | --- | --- |
| 3D A* | 26-connected voxel graph | Resolution-complete graph search | Deterministic |
| Lazy Theta* | Same voxel graph with delayed line-of-sight repair | Any-angle parent links | Deterministic |
| RRT* | Continuous bounded 3D space | Seeded sampling and rewiring | Deterministic for a fixed seed and sample budget |

The primary outputs are planning success, planning time, raw path length, and minimum clearance. Smoothed length is secondary because a common post-processing step can hide differences between planners.

## Included scenes

- `open-blocks` — irregular city blocks with several broad route choices.
- `urban-canyon` — alternating narrow gates between tall building slabs.
- `restricted-core` — a full-height cylindrical no-fly zone at the city center.
- `vertical-gate` — a low restricted gate where gaining altitude may help.
- `random-city-<seed>` — deterministic random buildings with a reserved diagonal corridor.

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

## Static web study

The Vite application has two restrained academic views. The trajectory view uses Three.js to inspect
recorded paths. The results view uses accessible inline SVG to show voxel-resolution and RRT* budget
sensitivity plus an exact results table. No planner runs in the browser.

```bash
cd web
pnpm install --frozen-lockfile
pnpm test
pnpm build
pnpm dev
```

The build validates both committed datasets. Python separately re-audits every trajectory, resolves
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

## Safety and validation model

- AABB intersection uses a continuous slab test.
- Finite vertical cylinders use an exact parameter-interval test in `xy` and `z`.
- Every 26-neighbor grid edge is checked continuously, so diagonal corner cutting is rejected.
- Start and goal remain exact virtual endpoints connected to all visible vertices in a local `3 x 3 x 3` stencil.
- Scene contracts require distinct start and goal points so path-excess metrics have a nonzero lower bound.
- Fixed-work RRT* uses `random.Random(seed)` and records incumbent quality at requested checkpoints.
- The shared post-processor first takes deterministic farthest-visible shortcuts, proposes a sampled cubic B-spline blend, validates every resulting segment, and falls back to the certified shortcut if needed.
- An independent audit rechecks endpoints, collision status, path length, waypoint count, and sampled clearance before export.

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
installs a wheel, smoke-tests the CLI, re-audits committed study data, and builds both Pages views.

## Repository layout

```text
src/uav3d/             geometry, planners, experiments, statistics, reports, CLI
scenarios/             four committed curated scene JSON files and schema
tests/                 contracts, collision, planning, smoothing, CLI
web/                   recorded Three.js paths and academic benchmark results
docs/                  methodology, schemas, and roadmap
.github/workflows/     CI and GitHub Pages deployment
```

## Current limits

Version `0.2.0` assumes a known static environment, a spherical vehicle, fixed no-fly zones, and
unconstrained point-to-point flight. It does not model vehicle dynamics, wind, sensing uncertainty,
moving obstacles, energy, minimum-snap trajectories, PX4/MAVLink export, or regulatory compliance.

Planned extensions are listed in the [roadmap](docs/roadmap.md). Dynamic replanning and kinodynamic trajectory generation will be separate experiments so they do not blur the static-planning baseline.

## References

- Hart, Nilsson, and Raphael. “A Formal Basis for the Heuristic Determination of Minimum Cost Paths.” *IEEE Transactions on Systems Science and Cybernetics*, 1968.
- Nash, Koenig, and Tovey. “Lazy Theta*: Any-Angle Path Planning and Path Length Analysis in 3D.” *AAAI*, 2010.
- Karaman and Frazzoli. “Sampling-based Algorithms for Optimal Motion Planning.” *International Journal of Robotics Research*, 2011.

## Citation and license

Citation metadata is provided in [CITATION.cff](CITATION.cff). The source is released under the [MIT License](LICENSE).
