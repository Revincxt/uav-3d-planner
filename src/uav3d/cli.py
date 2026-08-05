"""Command-line interface for planning, benchmarking, and demo export."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from uav3d.benchmark import (
    list_planners,
    run_benchmark,
    run_experiment,
    summarize_records,
)
from uav3d.demo import build_demo_bundle
from uav3d.scene import (
    generate_random_city,
    list_builtin_scenes,
    load_builtin_scene,
    load_scene,
    save_scene,
)
from uav3d.version import __version__


def _write_json(path: Path, data: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


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
        raise argparse.ArgumentTypeError("seeds must be comma-separated integers") from error


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="uav3d",
        description="Reproducible static-city benchmark for 3D UAV path planning.",
    )
    parser.add_argument("--version", action="version", version=f"uav3d {__version__}")
    commands = parser.add_subparsers(dest="command", required=True)

    scene_parser = commands.add_parser("scene", help="Inspect or generate scenes.")
    scene_commands = scene_parser.add_subparsers(dest="scene_command", required=True)
    scene_commands.add_parser("list", help="List built-in curated scenes.")
    show_parser = scene_commands.add_parser("show", help="Print a scene as JSON.")
    show_parser.add_argument("scene", help="Built-in scene ID or JSON path.")
    generate_parser = scene_commands.add_parser("generate", help="Generate a seeded random city.")
    generate_parser.add_argument("--seed", type=int, required=True)
    generate_parser.add_argument("--buildings", type=int, default=18)
    generate_parser.add_argument("--output", type=Path, required=True)

    plan_parser = commands.add_parser("plan", help="Run one planner and validate its output.")
    plan_parser.add_argument("--scene", required=True, help="Built-in scene ID or JSON path.")
    plan_parser.add_argument("--algorithm", choices=list_planners(), required=True)
    plan_parser.add_argument("--seed", type=int, default=0)
    plan_parser.add_argument("--output", type=Path)

    benchmark_parser = commands.add_parser("benchmark", help="Run a reproducible experiment grid.")
    benchmark_parser.add_argument("--scenes", type=_csv_values, default=list(list_builtin_scenes()))
    benchmark_parser.add_argument("--algorithms", type=_csv_values, default=list(list_planners()))
    benchmark_parser.add_argument("--seeds", type=_csv_ints, default=[0])
    benchmark_parser.add_argument("--output", type=Path, required=True)

    export_parser = commands.add_parser(
        "export-demo", help="Record all curated scenes for the static web demo."
    )
    export_parser.add_argument("--output", type=Path, required=True)
    export_parser.add_argument("--seed", type=int, default=17)
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


def _handle_plan(args: argparse.Namespace) -> int:
    scene = _resolve_scene(args.scene)
    record = run_experiment(scene, args.algorithm, args.seed)
    payload = record.to_dict()
    if args.output:
        _write_json(args.output, payload)
        print(f"saved {record.status} result to {args.output}")
    else:
        print(json.dumps(payload, indent=2))
    return 0 if record.status == "success" else 1


def _handle_benchmark(args: argparse.Namespace) -> int:
    invalid_algorithms = sorted(set(args.algorithms) - set(list_planners()))
    if invalid_algorithms:
        raise ValueError("unknown algorithms: " + ", ".join(invalid_algorithms))
    scenes = [_resolve_scene(reference) for reference in args.scenes]
    records = run_benchmark(scenes, args.algorithms, args.seeds)
    payload = {
        "schema_version": "1.0",
        "records": [record.to_dict() for record in records],
        "summary": summarize_records(records),
    }
    _write_json(args.output, payload)
    success_count = sum(record.status == "success" for record in records)
    print(f"saved {len(records)} runs ({success_count} successful) to {args.output}")
    return 0


def _handle_export_demo(args: argparse.Namespace) -> int:
    scenes = [load_builtin_scene(scene_id) for scene_id in list_builtin_scenes()]
    bundle = build_demo_bundle(scenes, args.seed)
    _write_json(args.output, bundle)
    print(f"saved {len(scenes)} recorded scenes to {args.output}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "scene":
            return _handle_scene(args)
        if args.command == "plan":
            return _handle_plan(args)
        if args.command == "benchmark":
            return _handle_benchmark(args)
        if args.command == "export-demo":
            return _handle_export_demo(args)
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    raise AssertionError("unreachable command")


if __name__ == "__main__":
    raise SystemExit(main())
