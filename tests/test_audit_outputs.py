"""Generated diagnostics belong to artifacts, not the versioned source datasets."""

from __future__ import annotations

import argparse
import runpy
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("script", ["audit_route_diversity", "audit_shared_airspace"])
@pytest.mark.parametrize("custom", [False, True])
def test_diagnostic_output_defaults_to_artifacts_and_allows_override(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, script: str, custom: bool
) -> None:
    class ArgumentsChecked(Exception):
        pass

    def check_arguments(parser: argparse.ArgumentParser) -> argparse.Namespace:
        override = tmp_path / "diagnostic.json"
        arguments, unknown = parser.parse_known_args(["--output", str(override)] if custom else [])
        assert not unknown
        name = script.removeprefix("audit_").replace("_", "-")
        assert arguments.output == (
            override if custom else ROOT / "artifacts" / f"{name}-audit.json"
        )
        raise ArgumentsChecked

    monkeypatch.syspath_prepend(str(ROOT / "scripts"))
    monkeypatch.setattr(argparse.ArgumentParser, "parse_args", check_arguments)
    with pytest.raises(ArgumentsChecked):
        runpy.run_path(str(ROOT / "scripts" / f"{script}.py"), run_name="__main__")
