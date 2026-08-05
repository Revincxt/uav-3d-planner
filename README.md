<div align="center">

# UAV 3D Planner Lab

**Reproducible 3D planning and space–time replanning in structured urban airspace**

[![CI](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/ci.yml/badge.svg)](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/ci.yml)
[![Pages](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/pages.yml/badge.svg)](https://github.com/Revincxt/uav-3d-planner-lab/actions/workflows/pages.yml)
[![Release](https://img.shields.io/github/v/release/Revincxt/uav-3d-planner-lab?display_name=tag&sort=semver)](https://github.com/Revincxt/uav-3d-planner-lab/releases/tag/v0.5.0)
[![Python 3.11+](https://img.shields.io/badge/python-3.11%2B-3776ab.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/license-MIT-555.svg)](LICENSE)

[**Open the live study**](https://revincxt.github.io/uav-3d-planner-lab/predictive.html) ·
[Reproduce a run](#quick-start) ·
[Read the methodology](docs/methodology.md) ·
[Download v0.5.0](https://github.com/Revincxt/uav-3d-planner-lab/releases/tag/v0.5.0)

</div>

<a href="https://revincxt.github.io/uav-3d-planner-lab/predictive.html?scenario=urban-canyon-merge&amp;planner=space-time-astar-4d&amp;path=certified&amp;time=0.00">
  <img src="docs/assets/readme/predictive-urban-canyon-focused.jpg" alt="Predictive planning study showing a dense urban canyon, no-fly volumes, moving hazards, a certified trajectory, and the recorded-state panel">
</a>

<p align="center"><sub><strong>Recorded v0.5 evidence.</strong> Urban canyon merge · 15 buildings · 4 hazards · 4D Space-Time A* · certified trajectory.</sub></p>

UAV 3D Planner Lab is an experiment-first Python benchmark for studying how grid, any-angle,
sampling-based, incremental, and space–time planners behave in the same declared urban geometry.
Planning output, downstream trajectory rounding, continuous collision auditing, and browser
presentation remain separate so that each result can be inspected and reproduced.

The Python package has no runtime dependencies. The web application does not run a planner or
smoother: it replays committed records that Python regenerates and independently validates.

## What the lab studies

| Study track | Research question | Planner conditions | Explore |
| --- | --- | --- | --- |
| **Static 3D** | How do graph, any-angle, and sampling-based methods differ when geometry and safety margin are fixed? | 3D A* · Lazy Theta* · RRT* | [Paths](https://revincxt.github.io/uav-3d-planner-lab/) · [Results](https://revincxt.github.io/uav-3d-planner-lab/results.html) |
| **Reactive dynamic** | How do cold-start and state-reusing replanners respond to the same deterministic obstacle schedule? | Repeated 3D A* · Repeated Lazy Theta* · 3D D* Lite | [Replay](https://revincxt.github.io/uav-3d-planner-lab/dynamic.html) |
| **Predictive 4D** | What changes when snapshot-only conditions are compared with access to a complete deterministic schedule? | Repeated 3D A* · D* Lite reset/reuse · 4D Space-Time A* | [Study](https://revincxt.github.io/uav-3d-planner-lab/predictive.html) |

<img src="docs/assets/readme/study-overview.svg" alt="Study-design schematic from urban airspace through planning protocols to collision-certified recorded evidence">

<p align="center"><sub>Study design. Planner families are compared within explicit information, discretization, execution, and work-budget contracts.</sub></p>

## Recorded evidence in v0.5

| Scope | Recorded result |
| --- | ---: |
| Curated deterministic missions | **8** — 1 calibration case + 7 complex city cases |
| Planner conditions per mission | **4** |
| Completed recorded runs | **32 / 32** |
| Safety violations in the declared continuous space–time audit | **0** |
| Runs accepting sampled circular-fillet rounding | **26 / 32** |
| Runs retaining a certified execution polyline | **32 / 32** |

Each non-calibration map contains 14–16 unequal-height buildings, at least one static no-fly zone,
and at least two scheduled or moving hazards. The raw timed path is retained for every run. Rounding
is a common downstream operation—not a property attributed to any planner—and falls back to the
certified raw polyline if a candidate fails the audit.

> These are curated, non-preregistered diagnostic cases. “0 violations” describes the fixed v0.5
> records under the declared geometry and deterministic schedule; it is not a general safety claim
> or flight certification. The independent unit is the scenario (n = 8), not the 32 planner runs,
> playback frames, waits, or replanning epochs.

## Algorithm map

| Track | Search / information model | Methods |
| --- | --- | --- |
| Static | 26-connected voxel graph or bounded continuous 3D space | 3D A* · Lazy Theta* · seeded RRT* |
| Dynamic | Current conservative snapshot with deterministic execution gating | Repeated 3D A* · Repeated Lazy Theta* · state-reusing 3D D* Lite |
| Predictive | Three snapshot conditions plus one complete-schedule condition | Repeated 3D A* · 3D D* Lite (reset) · 3D D* Lite (state reuse) · 4D Space-Time A* |

Space-Time A* is earliest-arrival on a declared `4 m × 0.5 s` finite lattice with explicit waits;
it is not a claim of continuous state–time optimality. Expanded spatial nodes, D* Lite queue pops,
and expanded space–time states remain algorithm-specific work indicators and are not normalized
into a single compute metric.

## Quick start

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
uav3d scene list
uav3d predictive plan --scenario urban-canyon-merge --algorithm space-time-astar-4d
```

Export the complete recorded predictive study:

```bash
uav3d export-predictive \
  --output-dir artifacts/predictive-study \
  --source-commit "$(git rev-parse HEAD)"
```

Run the academic web views locally:

```bash
cd web
pnpm install --frozen-lockfile
pnpm test && pnpm build && pnpm dev
```

Static benchmark, dynamic simulation, sensitivity, timing, and dataset-generation protocols are
documented in the CLI help and [methodology](docs/methodology.md).

## Reproducibility and validation

Every exported record carries its semantic problem fingerprint, source commit, work limit and
usage, termination reason, raw-path audit, post-processing outcome, and stable run ID. Release
validation resolves the recorded source revision, recomputes downloadable-artifact digests, and
checks configuration, timing, sample-count, summary, and geometry identities.

```bash
python -m pip install -e '.[dev]'
ruff check . && ruff format --check .
mypy src
pytest --cov=uav3d --cov-report=term-missing
python scripts/export_scenarios.py --check
python scripts/validate_committed_data.py
cd web && pnpm validate:data && pnpm test && pnpm typecheck && pnpm build
```

The geometry audit uses continuous segment tests for axis-aligned boxes and finite vertical
cylinders. Dynamic validation adds exact relative-motion checks for moving spheres, scheduled
restrictions, endpoint connectors, explicit waits, and every dense linear space–time segment.

## Project guide

| Resource | Contents |
| --- | --- |
| [Methodology](docs/methodology.md) | Assumptions, experiment contracts, estimands, collision model, and comparison limits |
| [Data schema](docs/data-schema.md) | Static, dynamic, and predictive record formats |
| [v0.5 study plan](docs/v0.5-demo-plan.md) | Scenario contracts, smoothing policy, and release acceptance criteria |
| [Roadmap](docs/roadmap.md) | Planned research extensions and explicit non-goals |
| [Changelog](CHANGELOG.md) | Version-by-version changes |
| [Citation metadata](CITATION.cff) | Repository citation information |

Core code lives in `src/uav3d/`; committed scenes and schemas live in `scenarios/`; validation and
behavioral tests live in `tests/`; the four recorded study views live in `web/`.

## Current limits

- The vehicle is spherical and follows a speed-limited dense piecewise-linear geometric path.
- The model does not certify attitude, acceleration, turn rate, continuous curvature, jerk,
  minimum-snap dynamics, wind, energy use, sensing uncertainty, or forecast uncertainty.
- Predictive experiments use a known deterministic schedule; the Pages application is recorded
  continuous-time playback, not a real-time planner.
- The v0.5 cohort is diagnostic and non-confirmatory. Its protocol differs from v0.4, so results
  should not be pooled across those versions.
- The project is a planning research benchmark, not a flight controller, regulatory-compliance
  tool, or operational safety system.

## References

- Hart, Nilsson, and Raphael. “A Formal Basis for the Heuristic Determination of Minimum Cost Paths.” *IEEE Transactions on Systems Science and Cybernetics*, 1968.
- Nash, Koenig, and Tovey. “Lazy Theta*: Any-Angle Path Planning and Path Length Analysis in 3D.” *AAAI*, 2010.
- Karaman and Frazzoli. “Sampling-based Algorithms for Optimal Motion Planning.” *International Journal of Robotics Research*, 2011.
- Koenig and Likhachev. “D* Lite.” *AAAI*, 2002.

## Citation and license

Citation metadata is provided in [CITATION.cff](CITATION.cff). Source code is released under the
[MIT License](LICENSE).
