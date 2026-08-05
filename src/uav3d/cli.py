"""Command-line interface for planning, experiment sweeps, and report export."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any, cast

from uav3d.analysis import (
    BOOTSTRAP_RESAMPLES,
    BOOTSTRAP_SEED,
    RecordLike,
    planner_summaries,
    resolution_sensitivity,
    rrt_budget_sensitivity,
)
from uav3d.benchmark import (
    ExperimentRecord,
    list_planners,
    run_benchmark,
    run_experiment,
    run_resolution_sweep,
    run_rrt_budget_curve,
    summarize_records,
)
from uav3d.dataset import generate_dataset_manifest
from uav3d.demo import build_demo_bundle
from uav3d.reporting import write_report_bundle
from uav3d.scene import (
    generate_random_city,
    list_builtin_scenes,
    load_builtin_scene,
    load_scene,
    save_scene,
)
from uav3d.study import export_web_benchmark
from uav3d.timing import run_timing_harness
from uav3d.version import __version__


def _write_json(path: Path, data: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(data, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _resolve_scene(reference: str) -> Any:
    if reference in list_builtin_scenes():
        return load_builtin_scene(reference)
    path = Path(reference)
    if path.is_file():
        return load_scene(path)
    raise ValueError(f"scene {reference!r} is neither a built-in ID nor a readable JSON file")


def _csv_values(value: str) -> list[str]:
    values = [item.strip() for item in value.split(",") if item.strip()]
    if not values:
        raise argparse.ArgumentTypeError("provide at least one comma-separated value")
    return values


def _csv_ints(value: str) -> list[int]:
    try:
        return [int(item) for item in _csv_values(value)]
    except ValueError as error:
        raise argparse.ArgumentTypeError("values must be comma-separated integers") from error


def _csv_floats(value: str) -> list[float]:
    try:
        return [float(item) for item in _csv_values(value)]
    except ValueError as error:
        raise argparse.ArgumentTypeError("values must be comma-separated numbers") from error


def _add_budget_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--work-limit", type=int)
    parser.add_argument("--wall-time-limit-ms", type=float)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="uav3d",
        description="Reproducible static-city benchmark for 3D UAV path planning.",
    )
    parser.add_argument("--version", action="version", version=f"uav3d {__version__}")
    commands = parser.add_subparsers(dest="command", required=True)

    scene_parser = commands.add_parser("scene", help="Inspect or generate individual scenes.")
    scene_commands = scene_parser.add_subparsers(dest="scene_command", required=True)
    scene_commands.add_parser("list", help="List built-in curated scenes.")
    show_parser = scene_commands.add_parser("show", help="Print a scene as JSON.")
    show_parser.add_argument("scene", help="Built-in scene ID or JSON path.")
    generate_parser = scene_commands.add_parser("generate", help="Generate a seeded random city.")
    generate_parser.add_argument("--seed", type=int, required=True)
    generate_parser.add_argument("--buildings", type=int, default=18)
    generate_parser.add_argument("--output", type=Path, required=True)

    dataset_parser = commands.add_parser(
        "dataset", help="Generate a random-scene cohort and rejection manifest."
    )
    dataset_parser.add_argument("--seeds", type=_csv_ints, required=True)
    dataset_parser.add_argument("--buildings", type=int, default=18)
    dataset_parser.add_argument("--output-dir", type=Path, required=True)

    plan_parser = commands.add_parser("plan", help="Run one planner and validate its output.")
    plan_parser.add_argument("--scene", required=True, help="Built-in scene ID or JSON path.")
    plan_parser.add_argument("--algorithm", choices=list_planners(), required=True)
    plan_parser.add_argument("--seed", type=int, default=0)
    plan_parser.add_argument("--resolution", type=float)
    _add_budget_arguments(plan_parser)
    plan_parser.add_argument("--quality-checkpoints", type=_csv_ints, default=[])
    plan_parser.add_argument("--purpose", default="path-quality")
    plan_parser.add_argument("--timing-repetition", type=int)
    plan_parser.add_argument("--output", type=Path)

    benchmark_parser = commands.add_parser(
        "benchmark", help="Run nominal quality cases without deterministic pseudo-replication."
    )
    benchmark_parser.add_argument("--scenes", type=_csv_values, default=list(list_builtin_scenes()))
    benchmark_parser.add_argument("--algorithms", type=_csv_values, default=list(list_planners()))
    benchmark_parser.add_argument("--seeds", type=_csv_ints, default=[0])
    benchmark_parser.add_argument("--wall-time-limit-ms", type=float)
    benchmark_parser.add_argument("--output", type=Path, required=True)
    benchmark_parser.add_argument("--report-dir", type=Path)

    sweep_parser = commands.add_parser("sweep", help="Run parameter-sensitivity experiments.")
    sweep_commands = sweep_parser.add_subparsers(dest="sweep_command", required=True)
    resolution_parser = sweep_commands.add_parser(
        "resolution", help="Sweep graph-planner voxel resolutions."
    )
    resolution_parser.add_argument(
        "--scenes", type=_csv_values, default=list(list_builtin_scenes())
    )
    resolution_parser.add_argument("--resolutions", type=_csv_floats, required=True)
    _add_budget_arguments(resolution_parser)
    resolution_parser.add_argument("--output", type=Path, required=True)
    resolution_parser.add_argument("--report-dir", type=Path)

    rrt_parser = sweep_commands.add_parser(
        "rrt-budget", help="Record RRT* incumbent quality at fixed sample checkpoints."
    )
    rrt_parser.add_argument("--scenes", type=_csv_values, default=list(list_builtin_scenes()))
    rrt_parser.add_argument("--budgets", type=_csv_ints, required=True)
    rrt_parser.add_argument("--seeds", type=_csv_ints, required=True)
    rrt_parser.add_argument("--wall-time-limit-ms", type=float)
    rrt_parser.add_argument("--output", type=Path, required=True)
    rrt_parser.add_argument("--report-dir", type=Path)

    timing_parser = commands.add_parser(
        "timing", help="Repeat planner timing in randomized isolated processes."
    )
    timing_parser.add_argument("--scenes", type=_csv_values, default=list(list_builtin_scenes()))
    timing_parser.add_argument("--algorithms", type=_csv_values, default=list(list_planners()))
    timing_parser.add_argument("--repetitions", type=int, default=5)
    timing_parser.add_argument("--planner-seed", type=int, default=17)
    timing_parser.add_argument("--order-seed", type=int, default=BOOTSTRAP_SEED)
    timing_parser.add_argument("--process-timeout-s", type=float, default=120.0)
    timing_parser.add_argument("--resolution", type=float)
    _add_budget_arguments(timing_parser)
    timing_parser.add_argument("--output", type=Path, required=True)

    export_parser = commands.add_parser(
        "export-demo", help="Record all curated scenes for the static trajectory page."
    )
    export_parser.add_argument("--output", type=Path, required=True)
    export_parser.add_argument("--seed", type=int, default=17)

    benchmark_export_parser = commands.add_parser(
        "export-benchmark", help="Run and export the fixed descriptive web protocol."
    )
    benchmark_export_parser.add_argument("--output-dir", type=Path, required=True)
    benchmark_export_parser.add_argument("--source-commit", required=True)
    benchmark_export_parser.add_argument("--timing-repetitions", type=int, default=3)
    return parser


def _handle_scene(args: argparse.Namespace) -> int:
    if args.scene_command == "list":
        for scene_id in list_builtin_scenes():
            scene = load_builtin_scene(scene_id)
            print(f"{scene.scene_id}\t{scene.name}")
        return 0
    if args.scene_command == "show":
        print(json.dumps(_resolve_scene(args.scene).to_dict(), indent=2))
        return 0
    if args.scene_command == "generate":
        scene = generate_random_city(args.seed, args.buildings)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        save_scene(scene, args.output)
        print(f"saved {scene.scene_id} to {args.output}")
        return 0
    raise AssertionError("unreachable scene command")


def _handle_dataset(args: argparse.Namespace) -> int:
    manifest = generate_dataset_manifest(args.seeds, args.buildings, args.output_dir)
    print(
        f"saved dataset manifest with {manifest['accepted']} accepted and "
        f"{manifest['rejected']} rejected scenes to {args.output_dir}"
    )
    return 0


def _handle_plan(args: argparse.Namespace) -> int:
    scene = _resolve_scene(args.scene)
    record = run_experiment(
        scene,
        args.algorithm,
        args.seed,
        resolution=args.resolution,
        work_limit=args.work_limit,
        wall_time_limit_ms=args.wall_time_limit_ms,
        quality_checkpoints=tuple(args.quality_checkpoints),
        run_purpose=args.purpose,
        timing_repetition=args.timing_repetition,
    )
    payload = record.to_dict()
    if args.output:
        _write_json(args.output, payload)
        print(f"saved {record.status} result to {args.output}")
    else:
        print(json.dumps(payload, indent=2, allow_nan=False))
    return 0 if record.status == "success" else 1


def _validate_algorithms(algorithms: list[str]) -> None:
    if len(algorithms) != len(set(algorithms)):
        raise ValueError("algorithms must be unique")
    invalid_algorithms = sorted(set(algorithms) - set(list_planners()))
    if invalid_algorithms:
        raise ValueError("unknown algorithms: " + ", ".join(invalid_algorithms))


def _experiment_payload(records: list[ExperimentRecord], experiment_type: str) -> dict[str, object]:
    typed_records = cast(Sequence[RecordLike], records)
    return {
        "schema_version": "2.0",
        "experiment_type": experiment_type,
        "bootstrap": {
            "method": "scene-clustered-percentile",
            "resamples": BOOTSTRAP_RESAMPLES,
            "seed": BOOTSTRAP_SEED,
            "quantile_method": "linear-type-7",
        },
        "records": [record.to_dict() for record in records],
        "cell_summaries": summarize_records(records),
        "planner_summaries": planner_summaries(typed_records),
    }


def _write_experiment(
    records: list[ExperimentRecord],
    output: Path,
    report_dir: Path | None,
    experiment_type: str,
    sensitivity: list[dict[str, object]] | None = None,
) -> None:
    payload = _experiment_payload(records, experiment_type)
    if sensitivity is not None:
        payload["sensitivity"] = sensitivity
    _write_json(output, payload)
    if report_dir is not None:
        write_report_bundle(records, report_dir)


def _handle_benchmark(args: argparse.Namespace) -> int:
    _validate_algorithms(args.algorithms)
    scenes = [_resolve_scene(reference) for reference in args.scenes]
    records = run_benchmark(
        scenes,
        args.algorithms,
        args.seeds,
        wall_time_limit_ms=args.wall_time_limit_ms,
    )
    _write_experiment(records, args.output, args.report_dir, "nominal")
    success_count = sum(record.status == "success" for record in records)
    print(f"saved {len(records)} runs ({success_count} successful) to {args.output}")
    return 0


def _handle_sweep(args: argparse.Namespace) -> int:
    scenes = [_resolve_scene(reference) for reference in args.scenes]
    if args.sweep_command == "resolution":
        records = run_resolution_sweep(
            scenes,
            args.resolutions,
            work_limit=args.work_limit,
            wall_time_limit_ms=args.wall_time_limit_ms,
        )
        sensitivity = resolution_sensitivity(cast(Sequence[RecordLike], records))
        _write_experiment(records, args.output, args.report_dir, "resolution-sweep", sensitivity)
        print(f"saved {len(records)} resolution cases to {args.output}")
        return 0
    if args.sweep_command == "rrt-budget":
        records = run_rrt_budget_curve(
            scenes,
            args.budgets,
            args.seeds,
            wall_time_limit_ms=args.wall_time_limit_ms,
        )
        sensitivity = rrt_budget_sensitivity(cast(Sequence[RecordLike], records))
        _write_experiment(records, args.output, args.report_dir, "rrt-budget-curve", sensitivity)
        print(f"saved {len(records)} RRT* quality traces to {args.output}")
        return 0
    raise AssertionError("unreachable sweep command")


def _handle_timing(args: argparse.Namespace) -> int:
    _validate_algorithms(args.algorithms)
    payload = run_timing_harness(
        args.scenes,
        args.algorithms,
        args.repetitions,
        planner_seed=args.planner_seed,
        order_seed=args.order_seed,
        process_timeout_s=args.process_timeout_s,
        resolution=args.resolution,
        work_limit=args.work_limit,
        wall_time_limit_ms=args.wall_time_limit_ms,
        working_directory=Path.cwd(),
    )
    _write_json(args.output, payload)
    completed = sum(record["process_status"] == "completed" for record in payload["records"])
    print(f"saved {completed}/{len(payload['records'])} isolated timing runs to {args.output}")
    return 0 if completed == len(payload["records"]) else 1


def _handle_export_demo(args: argparse.Namespace) -> int:
    scenes = [load_builtin_scene(scene_id) for scene_id in list_builtin_scenes()]
    bundle = build_demo_bundle(scenes, args.seed)
    _write_json(args.output, bundle)
    print(f"saved {len(scenes)} recorded scenes to {args.output}")
    return 0


def _handle_export_benchmark(args: argparse.Namespace) -> int:
    scenes = [load_builtin_scene(scene_id) for scene_id in list_builtin_scenes()]
    bundle = export_web_benchmark(
        scenes,
        args.output_dir,
        source_commit=args.source_commit,
        timing_repetitions=args.timing_repetitions,
        working_directory=Path.cwd(),
    )
    summaries = cast(list[object], bundle["summaries"])
    print(f"saved {len(summaries)} planner summaries and sensitivity data to {args.output_dir}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "scene":
            return _handle_scene(args)
        if args.command == "dataset":
            return _handle_dataset(args)
        if args.command == "plan":
            return _handle_plan(args)
        if args.command == "benchmark":
            return _handle_benchmark(args)
        if args.command == "sweep":
            return _handle_sweep(args)
        if args.command == "timing":
            return _handle_timing(args)
        if args.command == "export-demo":
            return _handle_export_demo(args)
        if args.command == "export-benchmark":
            return _handle_export_benchmark(args)
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    raise AssertionError("unreachable command")


if __name__ == "__main__":
    raise SystemExit(main())
