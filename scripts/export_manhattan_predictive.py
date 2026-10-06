"""Compute real-geometry Manhattan demo records without changing the frozen study."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main() -> int:
    from uav3d.manhattan_predictive import export_manhattan_predictive

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "web" / "public")
    parser.add_argument("--cache-dir", type=Path)
    parser.add_argument(
        "--city-file", type=Path, help="Compute against a staged physical city before publishing."
    )
    parser.add_argument(
        "--probe",
        action="store_true",
        help="Compute and audit the first mission before exporting the full city cohort",
    )
    args = parser.parse_args()
    bundle = export_manhattan_predictive(
        args.output_dir, probe=args.probe, cache_dir=args.cache_dir, city_path=args.city_file
    )
    if bundle is not None:
        run_count = sum(len(s["runs"]) for s in bundle["scenarios"])
        print(
            f"Exported {len(bundle['scenarios'])} Manhattan missions and {run_count} computed "
            f"planner runs to {args.output_dir}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
