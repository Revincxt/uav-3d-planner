"""Write or verify the curated scene JSON fixtures."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from uav3d.scene import list_builtin_scenes, load_builtin_scene

ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIRECTORY = ROOT / "scenarios"


def rendered_scenes() -> dict[Path, str]:
    return {
        SCENARIO_DIRECTORY / f"{scene_id}.json": json.dumps(
            load_builtin_scene(scene_id).to_dict(), indent=2
        )
        + "\n"
        for scene_id in list_builtin_scenes()
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    mismatches: list[Path] = []
    for path, expected in rendered_scenes().items():
        if arguments.check:
            if not path.is_file() or path.read_text(encoding="utf-8") != expected:
                mismatches.append(path)
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(expected, encoding="utf-8")
    if mismatches:
        names = ", ".join(path.name for path in mismatches)
        raise SystemExit(f"curated scenario files are stale: {names}")
    action = "verified" if arguments.check else "wrote"
    print(f"{action} {len(rendered_scenes())} curated scenarios")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
