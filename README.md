# UAV 3D Planner Lab

[![CI](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/ci.yml/badge.svg)](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/ci.yml)
[![Pages](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/pages.yml/badge.svg)](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/pages.yml)
[![Python 3.11+](https://img.shields.io/badge/python-3.11%2B-3776ab.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/license-MIT-555.svg)](LICENSE)

A dependency-free Python benchmark for collision-aware UAV path planning in static three-dimensional cities. It compares **3D A\***, **Lazy Theta\***, and **RRT\*** under one geometry model, then applies the same collision-certified smoothing pipeline to every successful path.

**Recorded demo:** [revincxt.github.io/uav-3d-planner-lab](https://revincxt.github.io/uav-3d-planner-lab/)

> The web page contains recorded, non-confirmatory single runs. It is useful for inspecting geometry and algorithm behavior, but it is not statistical evidence and not a flight-safety system.

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
  --output benchmark.json
```

Generate a reusable random city:

```bash
uav3d scene generate --seed 42 --buildings 18 --output scenarios/random-city-42.json
uav3d plan --scene scenarios/random-city-42.json --algorithm rrt-star --seed 7
```

## Static 3D demo

The page is a small Vite + TypeScript + Three.js application. It reads committed trajectories from `web/public/demo-data.json`; no backend or browser-side planner is involved.

```bash
cd web
pnpm install --frozen-lockfile
pnpm test
pnpm build
pnpm dev
```

The figure supports scene selection, raw/smoothed trajectory switching, independent planner visibility, and orthographic isometric/top views. Its table remains a complete non-visual representation of the recorded results.

## Safety and validation model

- AABB intersection uses a continuous slab test.
- Finite vertical cylinders use an exact parameter-interval test in `xy` and `z`.
- Every 26-neighbor grid edge is checked continuously, so diagonal corner cutting is rejected.
- Start and goal remain exact virtual endpoints connected to all visible vertices in a local `3 x 3 x 3` stencil.
- RRT* uses a fixed attempt budget and `random.Random(seed)`; no wall-clock termination changes the sampled sequence.
- The shared post-processor first takes deterministic farthest-visible shortcuts, proposes a sampled cubic B-spline blend, validates every resulting segment, and falls back to the certified shortcut if needed.
- An independent audit rechecks endpoints, collision status, path length, waypoint count, and sampled clearance before export.

See [Methodology](docs/methodology.md) for definitions, assumptions, and comparison limits.

## Reproducing checks

```bash
python -m pip install -e '.[dev]'
ruff check .
ruff format --check .
mypy src
pytest --cov=uav3d --cov-report=term-missing
python -m build

python scripts/export_scenarios.py --check
cd web && pnpm validate:data && pnpm test && pnpm typecheck && pnpm build
```

The Python package has no runtime dependencies. CI tests Python 3.11 and 3.12, builds a wheel, performs an installed-CLI smoke test, validates the recorded demo data, and builds the static Pages artifact.

## Repository layout

```text
src/uav3d/             geometry, planners, smoothing, benchmark, CLI
scenarios/             four committed curated scene JSON files and schema
tests/                 contracts, collision, planning, smoothing, CLI
web/                   static academic Three.js figure
docs/                  methodology, schemas, and roadmap
.github/workflows/     CI and GitHub Pages deployment
```

## Current limits

Version `0.1.0` assumes a known static environment, a spherical vehicle, fixed no-fly zones, and unconstrained point-to-point flight. It does not model vehicle dynamics, wind, sensing uncertainty, moving obstacles, energy, minimum-snap trajectories, PX4/MAVLink export, or regulatory compliance.

Planned extensions are listed in the [roadmap](docs/roadmap.md). Dynamic replanning and kinodynamic trajectory generation will be separate experiments so they do not blur the static-planning baseline.

## References

- Hart, Nilsson, and Raphael. “A Formal Basis for the Heuristic Determination of Minimum Cost Paths.” *IEEE Transactions on Systems Science and Cybernetics*, 1968.
- Nash, Koenig, and Tovey. “Lazy Theta*: Any-Angle Path Planning and Path Length Analysis in 3D.” *AAAI*, 2010.
- Karaman and Frazzoli. “Sampling-based Algorithms for Optimal Motion Planning.” *International Journal of Robotics Research*, 2011.

## Citation and license

Citation metadata is provided in [CITATION.cff](CITATION.cff). The source is released under the [MIT License](LICENSE).

