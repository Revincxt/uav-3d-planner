"""Isolated-process timing harness for planner runtime measurements."""

from __future__ import annotations

import hashlib
import json
import os
import platform
import random
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from uav3d.version import __version__


@dataclass(frozen=True, slots=True)
class TimingCase:
    scene_reference: str
    algorithm: str
    repetition: int
    planner_seed: int

    @property
    def case_id(self) -> str:
        payload = (
            f"{self.scene_reference}|{self.algorithm}|{self.repetition}|{self.planner_seed}"
        ).encode()
        return "timing-" + hashlib.sha256(payload).hexdigest()[:16]


def _environment() -> dict[str, object]:
    return {
        "python": platform.python_version(),
        "implementation": platform.python_implementation(),
        "package_version": __version__,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "processor": platform.processor() or None,
        "cpu_count": os.cpu_count(),
    }


def _command(
    case: TimingCase,
    *,
    resolution: float | None,
    work_limit: int | None,
    wall_time_limit_ms: float | None,
) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "uav3d",
        "plan",
        "--scene",
        case.scene_reference,
        "--algorithm",
        case.algorithm,
        "--seed",
        str(case.planner_seed),
        "--purpose",
        "timing",
        "--timing-repetition",
        str(case.repetition),
    ]
    if resolution is not None and case.algorithm != "rrt-star":
        command.extend(("--resolution", str(resolution)))
    if work_limit is not None:
        command.extend(("--work-limit", str(work_limit)))
    if wall_time_limit_ms is not None:
        command.extend(("--wall-time-limit-ms", str(wall_time_limit_ms)))
    return command


def run_timing_harness(
    scene_references: list[str],
    algorithms: list[str],
    repetitions: int,
    *,
    planner_seed: int = 17,
    order_seed: int = 20_260_805,
    process_timeout_s: float = 120.0,
    resolution: float | None = None,
    work_limit: int | None = None,
    wall_time_limit_ms: float | None = None,
    working_directory: Path | None = None,
) -> dict[str, Any]:
    if repetitions <= 0:
        raise ValueError("repetitions must be positive")
    if process_timeout_s <= 0:
        raise ValueError("process_timeout_s must be positive")
    if not scene_references or not algorithms:
        raise ValueError("timing harness requires scenes and algorithms")
    cases = [
        TimingCase(scene, algorithm, repetition, planner_seed)
        for scene in scene_references
        for algorithm in algorithms
        for repetition in range(repetitions)
    ]
    random.Random(order_seed).shuffle(cases)
    records: list[dict[str, object]] = []
    for order_index, case in enumerate(cases):
        command = _command(
            case,
            resolution=resolution,
            work_limit=work_limit,
            wall_time_limit_ms=wall_time_limit_ms,
        )
        started = time.perf_counter()
        process = subprocess.Popen(
            command,
            cwd=working_directory,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            stdout, stderr = process.communicate(timeout=process_timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            stdout, stderr = process.communicate()
            records.append(
                {
                    "case_id": case.case_id,
                    "order_index": order_index,
                    "scene_reference": case.scene_reference,
                    "algorithm": case.algorithm,
                    "timing_repetition": case.repetition,
                    "planner_seed": planner_seed if case.algorithm == "rrt-star" else None,
                    "pid": process.pid,
                    "process_status": "hard-timeout",
                    "process_returncode": process.returncode,
                    "process_wall_time_ms": (time.perf_counter() - started) * 1000,
                    "stderr": stderr[-2000:],
                    "planner_record": None,
                }
            )
            continue
        process_wall_time_ms = (time.perf_counter() - started) * 1000
        planner_record: object | None = None
        parse_error: str | None = None
        try:
            planner_record = json.loads(stdout)
        except json.JSONDecodeError as error:
            parse_error = str(error)
        process_status = (
            "completed"
            if planner_record is not None and process.returncode in {0, 1}
            else "crashed"
        )
        records.append(
            {
                "case_id": case.case_id,
                "order_index": order_index,
                "scene_reference": case.scene_reference,
                "algorithm": case.algorithm,
                "timing_repetition": case.repetition,
                "planner_seed": planner_seed if case.algorithm == "rrt-star" else None,
                "pid": process.pid,
                "process_status": process_status,
                "process_returncode": process.returncode,
                "process_wall_time_ms": process_wall_time_ms,
                "stderr": stderr[-2000:] or None,
                "parse_error": parse_error,
                "planner_record": planner_record,
            }
        )
    return {
        "schema_version": "2.0",
        "harness": "isolated-process-v1",
        "order_seed": order_seed,
        "planner_seed": planner_seed,
        "repetitions_per_cell": repetitions,
        "process_timeout_s": process_timeout_s,
        "environment": _environment(),
        "records": records,
    }
