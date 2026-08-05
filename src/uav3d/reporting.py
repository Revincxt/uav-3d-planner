"""Dependency-free CSV, JSON, and publication-style SVG export."""

from __future__ import annotations

import csv
import hashlib
import html
import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, cast

from uav3d.analysis import (
    BOOTSTRAP_RESAMPLES,
    BOOTSTRAP_SEED,
    RecordLike,
    planner_summaries,
)
from uav3d.benchmark import ExperimentRecord

PLANNER_LABELS = {
    "astar-3d": "3D A*",
    "lazy-theta-star": "Lazy Theta*",
    "rrt-star": "RRT*",
}
PLANNER_COLORS = {
    "astar-3d": "#3269a8",
    "lazy-theta-star": "#287d65",
    "rrt-star": "#b4553d",
}


def _metric(record: ExperimentRecord, name: str) -> float | None:
    if name == "raw_length_m":
        return record.raw_audit.length_m if record.raw_path_valid and record.raw_audit else None
    if name == "raw_excess_pct":
        return (
            (record.raw_audit.length_m / record.straight_line_distance_m - 1) * 100
            if record.raw_path_valid and record.raw_audit
            else None
        )
    if name == "smoothed_length_m":
        return (
            record.smoothed_audit.length_m
            if record.smoothed_path_valid and record.smoothed_audit
            else None
        )
    if name == "minimum_clearance_m":
        return (
            record.smoothed_audit.minimum_clearance_m
            if record.smoothed_path_valid and record.smoothed_audit
            else None
        )
    raise ValueError(f"unknown record metric: {name}")


def write_records_csv(
    records: Sequence[ExperimentRecord],
    path: Path,
    *,
    metadata: Mapping[str, str] | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    metadata_values = dict(metadata or {})
    record_fields = [
        "run_id",
        "run_purpose",
        "timing_repetition",
        "scene_id",
        "problem_fingerprint",
        "algorithm",
        "configuration_id",
        "planner_seed",
        "status",
        "failure_reason",
        "planner_solution_found",
        "raw_path_valid",
        "postprocessing_status",
        "smoothed_path_valid",
        "straight_line_distance_m",
        "parameters_json",
        "planning_time_ms",
        "setup_time_ms",
        "search_time_ms",
        "smoothing_time_ms",
        "validation_time_ms",
        "work_unit",
        "work_limit",
        "wall_time_limit_ms",
        "work_used",
        "wall_time_used_ms",
        "termination",
        "quality_trace_json",
        "raw_length_m",
        "raw_excess_pct",
        "smoothed_length_m",
        "minimum_clearance_m",
        "expanded_nodes",
        "generated_nodes",
        "iterations",
    ]
    if set(metadata_values) & set(record_fields):
        raise ValueError("CSV metadata fields must not shadow record fields")
    fieldnames = [*metadata_values, *record_fields]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for record in records:
            writer.writerow(
                metadata_values
                | {
                    "run_id": record.run_id,
                    "run_purpose": record.run_purpose,
                    "timing_repetition": record.timing_repetition,
                    "scene_id": record.scene_id,
                    "problem_fingerprint": record.scene_fingerprint,
                    "algorithm": record.algorithm,
                    "configuration_id": record.configuration_id,
                    "planner_seed": record.seed,
                    "status": record.status,
                    "failure_reason": record.failure_reason,
                    "planner_solution_found": record.planner_solution_found,
                    "raw_path_valid": record.raw_path_valid,
                    "postprocessing_status": record.postprocessing_status,
                    "smoothed_path_valid": record.smoothed_path_valid,
                    "straight_line_distance_m": record.straight_line_distance_m,
                    "parameters_json": json.dumps(
                        record.parameters, sort_keys=True, separators=(",", ":")
                    ),
                    "planning_time_ms": record.planning_time_ms,
                    "setup_time_ms": record.setup_time_ms,
                    "search_time_ms": record.search_time_ms,
                    "smoothing_time_ms": record.smoothing_time_ms,
                    "validation_time_ms": record.validation_time_ms,
                    "work_unit": record.budget.work_unit if record.budget else None,
                    "work_limit": record.budget.work_limit if record.budget else None,
                    "wall_time_limit_ms": (
                        record.budget.wall_time_limit_ms if record.budget else None
                    ),
                    "work_used": record.budget_usage.work_used if record.budget_usage else None,
                    "wall_time_used_ms": (
                        record.budget_usage.wall_time_used_ms if record.budget_usage else None
                    ),
                    "termination": (
                        record.budget_usage.termination if record.budget_usage else None
                    ),
                    "quality_trace_json": json.dumps(
                        [point.to_dict() for point in record.quality_trace],
                        sort_keys=True,
                        separators=(",", ":"),
                    ),
                    "raw_length_m": _metric(record, "raw_length_m"),
                    "raw_excess_pct": _metric(record, "raw_excess_pct"),
                    "smoothed_length_m": _metric(record, "smoothed_length_m"),
                    "minimum_clearance_m": _metric(record, "minimum_clearance_m"),
                    "expanded_nodes": record.expanded_nodes,
                    "generated_nodes": record.generated_nodes,
                    "iterations": record.iterations,
                }
            )


def write_summary_csv(summaries: Sequence[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "planner_id",
        "budget_id",
        "runs",
        "successes",
        "metric",
        "conditioning",
        "estimator",
        "value",
        "q1",
        "q3",
        "ci95_low",
        "ci95_high",
        "n_scenes",
        "n_runs",
        "n_successes",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for summary in summaries:
            metrics = summary["metrics"]
            if not isinstance(metrics, dict):
                raise TypeError("summary metrics must be a mapping")
            for metric_name, raw_metric in metrics.items():
                if not isinstance(raw_metric, dict):
                    raise TypeError("metric summary must be a mapping")
                writer.writerow(
                    {
                        "planner_id": summary["plannerId"],
                        "budget_id": summary["budgetId"],
                        "runs": summary["runs"],
                        "successes": summary["successes"],
                        "metric": metric_name,
                        "conditioning": raw_metric["conditioning"],
                        "estimator": raw_metric["estimator"],
                        "value": raw_metric["value"],
                        "q1": raw_metric["q1"],
                        "q3": raw_metric["q3"],
                        "ci95_low": raw_metric["ci95Low"],
                        "ci95_high": raw_metric["ci95High"],
                        "n_scenes": raw_metric["nScenes"],
                        "n_runs": raw_metric["nRuns"],
                        "n_successes": raw_metric["nSuccesses"],
                    }
                )


def _metric_bounds(
    summaries: Sequence[dict[str, object]], metric_name: str, *, log_scale: bool
) -> tuple[float, float]:
    values: list[float] = []
    for summary in summaries:
        metrics = summary["metrics"]
        if not isinstance(metrics, dict):
            continue
        metric = metrics.get(metric_name)
        if not isinstance(metric, dict):
            continue
        for key in ("ci95Low", "ci95High", "q1", "q3", "value"):
            value = metric.get(key)
            if isinstance(value, (int, float)) and (not log_scale or value > 0):
                values.append(float(value))
    if not values:
        return (0.1, 1.0) if log_scale else (0.0, 1.0)
    lower = min(values)
    upper = max(values)
    if math.isclose(lower, upper):
        padding = max(abs(lower) * 0.1, 0.1)
        return max(1e-9, lower - padding) if log_scale else lower - padding, upper + padding
    padding = (upper - lower) * 0.08
    return max(1e-9, lower - padding) if log_scale else min(0.0, lower - padding), upper + padding


def _scale(value: float, lower: float, upper: float, width: float, log_scale: bool) -> float:
    if log_scale:
        value, lower, upper = math.log10(value), math.log10(lower), math.log10(upper)
    return (value - lower) / (upper - lower) * width


def write_summary_svg(summaries: Sequence[dict[str, object]], path: Path) -> None:
    """Write a four-panel, journal-friendly interval plot without plotting dependencies."""

    panels = [
        ("successRate", "Success rate", False, 100.0, "%"),
        ("planningTimeMs", "Planning time", True, 1.0, "ms"),
        ("rawPathExcessPct", "Raw path excess", False, 1.0, "%"),
        ("minimumClearanceM", "Minimum clearance", False, 1.0, "m"),
    ]
    width = 1200
    height = 180 + 54 * max(1, len(summaries))
    left = 164
    gap = 28
    panel_width = (width - left - 44 - gap * 3) / 4
    rows_top = 92
    fragments = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" role="img">',
        "<title>Planner benchmark summary</title>",
        "<desc>Point estimates with interquartile ranges and "
        "scene-clustered 95 percent bootstrap intervals.</desc>",
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        "<style>text{font-family:Arial,sans-serif;fill:#222} .muted{fill:#666} "
        ".axis{stroke:#aaa;stroke-width:1} .ci{stroke-width:1.2} "
        ".iqr{stroke-width:5} .point{stroke:#fff;stroke-width:1.2}</style>",
        '<text x="24" y="30" font-size="18" font-weight="600">Static 3D planning benchmark</text>',
        '<text x="24" y="52" font-size="12" class="muted">Scene-weighted summaries; '
        "path metrics condition on successful raw paths.</text>",
    ]
    bounds = {
        name: (
            (0.0, 1.0) if name == "successRate" else _metric_bounds(summaries, name, log_scale=log)
        )
        for name, _label, log, _factor, _unit in panels
    }
    for panel_index, (name, label, _log_scale, factor, unit) in enumerate(panels):
        x = left + panel_index * (panel_width + gap)
        lower, upper = bounds[name]
        fragments.append(f'<text x="{x:.1f}" y="76" font-size="13">{html.escape(label)}</text>')
        fragments.append(
            f'<line class="axis" x1="{x:.1f}" y1="84" x2="{x + panel_width:.1f}" y2="84"/>'
        )
        fragments.append(
            f'<text x="{x:.1f}" y="{height - 18}" font-size="10" '
            f'class="muted">{lower * factor:.2g}</text>'
        )
        fragments.append(
            f'<text x="{x + panel_width:.1f}" y="{height - 18}" text-anchor="end" '
            f'font-size="10" class="muted">{upper * factor:.2g} {unit}</text>'
        )
    for row_index, summary in enumerate(summaries):
        y = rows_top + row_index * 54
        algorithm = str(summary["plannerId"])
        color = PLANNER_COLORS.get(algorithm, "#444444")
        label = PLANNER_LABELS.get(algorithm, algorithm)
        fragments.append(f'<text x="24" y="{y + 4}" font-size="12">{html.escape(label)}</text>')
        metrics = summary["metrics"]
        if not isinstance(metrics, dict):
            continue
        for panel_index, (name, _label, log_scale, _factor, _unit) in enumerate(panels):
            metric = metrics.get(name)
            if not isinstance(metric, dict) or not isinstance(metric.get("value"), (int, float)):
                continue
            lower, upper = bounds[name]
            x = left + panel_index * (panel_width + gap)
            scaled: dict[str, float] = {}
            for key in ("ci95Low", "ci95High", "q1", "q3", "value"):
                value = metric.get(key)
                if isinstance(value, (int, float)) and (not log_scale or value > 0):
                    scaled[key] = x + _scale(float(value), lower, upper, panel_width, log_scale)
            if {"ci95Low", "ci95High"} <= scaled.keys():
                fragments.append(
                    f'<line class="ci" stroke="{color}" x1="{scaled["ci95Low"]:.1f}" '
                    f'y1="{y}" x2="{scaled["ci95High"]:.1f}" y2="{y}"/>'
                )
            if {"q1", "q3"} <= scaled.keys():
                fragments.append(
                    f'<line class="iqr" stroke="{color}" x1="{scaled["q1"]:.1f}" '
                    f'y1="{y}" x2="{scaled["q3"]:.1f}" y2="{y}"/>'
                )
            if "value" in scaled:
                fragments.append(
                    f'<circle class="point" fill="{color}" cx="{scaled["value"]:.1f}" '
                    f'cy="{y}" r="4.5"/>'
                )
    fragments.append("</svg>\n")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(fragments), encoding="utf-8")


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_report_bundle(
    records: Sequence[ExperimentRecord],
    output_dir: Path,
    *,
    bootstrap_resamples: int = BOOTSTRAP_RESAMPLES,
    bootstrap_seed: int = BOOTSTRAP_SEED,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    summaries = planner_summaries(
        cast(Sequence[RecordLike], records),
        resamples=bootstrap_resamples,
        seed=bootstrap_seed,
    )
    records_path = output_dir / "records.csv"
    summary_csv_path = output_dir / "summary.csv"
    summary_json_path = output_dir / "summary.json"
    svg_path = output_dir / "planner-summary.svg"
    write_records_csv(records, records_path)
    write_summary_csv(summaries, summary_csv_path)
    summary_json_path.write_text(
        json.dumps(
            {
                "schema_version": "2.0",
                "bootstrap": {
                    "method": "scene-clustered-percentile",
                    "resamples": bootstrap_resamples,
                    "seed": bootstrap_seed,
                    "confidence_level": 0.95,
                    "quantile_method": "linear-type-7",
                },
                "planner_summaries": summaries,
            },
            indent=2,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    write_summary_svg(summaries, svg_path)
    files = [records_path, summary_csv_path, summary_json_path, svg_path]
    manifest = {
        "schema_version": "2.0",
        "files": [
            {"path": path.name, "sha256": _digest(path), "bytes": path.stat().st_size}
            for path in files
        ],
    }
    (output_dir / "checksums.json").write_text(
        json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return manifest
