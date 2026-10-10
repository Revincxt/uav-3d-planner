"""Fetch the official NYC OTI footprint extract and reproducibly preprocess it.

Run from the repository: PYTHONPATH=src python3 data/manhattan/fetch_source.py
The query selects intersecting source polygons without clipping or simplifying them.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import UTC, datetime
from pathlib import Path
from urllib.parse import urlencode

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from uav3d.manhattan_city import (  # noqa: E402
    METADATA_URL,
    ROI_WGS84,
    SERVICE_URL,
    preprocess_city,
)

DESTINATION = Path(__file__).resolve().parent


def request(parameters: dict[str, object]) -> dict[str, object]:
    url = f"{SERVICE_URL}/query?{urlencode(parameters)}"
    response = subprocess.run(
        ["curl", "--fail", "--silent", "--show-error", "--location", "--max-time", "90", url],
        check=True,
        capture_output=True,
    )
    result = json.loads(response.stdout)
    if "error" in result:
        raise RuntimeError(f"Official source query failed: {result['error']}")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DESTINATION,
        help="Stage a new source extract without replacing the currently published city.",
    )
    destination = parser.parse_args().output_dir
    selection: dict[str, object] = {
        # Borough 1 tax lots only: the wider southern envelope crosses the East
        # River, but Brooklyn buildings must not become Manhattan obstacles.
        "where": "BASE_BBL LIKE '1%'",
        "geometry": ",".join(str(value) for value in ROI_WGS84),
        "geometryType": "esriGeometryEnvelope",
        "inSR": 4326,
        "spatialRel": "esriSpatialRelIntersects",
    }
    count = int(request({**selection, "returnCountOnly": "true", "f": "json"})["count"])
    print(f"Official NYC source contains {count} intersecting features", flush=True)
    features: list[object] = []
    for offset in range(0, count, 2000):
        page = request(
            {
                **selection,
                "outSR": 4326,
                "outFields": (
                    "OBJECTID,DOITT_ID,BIN,BASE_BBL,HEIGHT_ROOF,GROUND_ELEVATION,"
                    "NAME,FEATURE_CODE,GEOM_SOURCE,LAST_EDITED_DATE"
                ),
                "returnGeometry": "true",
                "orderByFields": "OBJECTID ASC",
                "resultOffset": offset,
                "resultRecordCount": 2000,
                "f": "geojson",
            }
        )
        batch = page.get("features", [])
        if not isinstance(batch, list):
            raise RuntimeError("Official source returned an invalid GeoJSON page")
        features.extend(batch)
        print(f"Fetched {len(features)}/{count} source features", flush=True)
    if len(features) != count:
        raise RuntimeError(f"Incomplete source extract: {len(features)} of {count} records")
    destination.mkdir(parents=True, exist_ok=True)
    source = {"type": "FeatureCollection", "features": features}
    raw = json.dumps(source, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    source_path = destination / "building-footprints.geojson"
    source_path.write_bytes(raw + b"\n")
    source_sha = hashlib.sha256(raw + b"\n").hexdigest()
    provenance = {
        "sourceUrl": SERVICE_URL,
        "metadataUrl": METADATA_URL,
        "sourceKind": "nyc-open-data",
        "publisher": "New York City Office of Technology and Innovation",
        "serviceItemId": "870bf69e8a8044aea4488e564c0b4010",
        "sourcePath": "data/manhattan/building-footprints.geojson",
        "sourceSha256": source_sha,
        "fetchedAt": datetime.now(UTC).replace(microsecond=0).isoformat(),
        "requestedBoundsWgs84": list(ROI_WGS84),
        "sourceFeatureCount": count,
        "selection": (
            "All Manhattan source footprints intersecting the requested bounding box; "
            "complete polygons retained, no simplification or clipping."
        ),
        "heightSourceField": "HEIGHT_ROOF",
        "heightSourceUnit": "ft",
        "heightMeaning": "Roof height above local ground, not elevation above sea level.",
        "heightUnitEvidence": (
            "NYC source Empire State Building BIN 1015862 has HEIGHT_ROOF 1238.79032716: "
            "converting feet to metres gives 377.58329 m, consistent with the building owner's "
            "published roof height of approximately 1250 feet / 380 m. "
            "Source value is retained unchanged."
        ),
        "heightUnitReferenceUrl": "https://www.esbnyc.com/about/facts-figures",
    }
    processed = preprocess_city(source, provenance)
    (destination / "city.json").write_text(
        json.dumps(processed, separators=(",", ":")) + "\n", encoding="utf-8"
    )
    (destination / "provenance.json").write_text(
        json.dumps(processed["metadata"], indent=2) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "sourceSha256": source_sha,
                "buildingCount": len(processed["buildings"]),
                "bounds": processed["bounds"],
                "heightAssumptions": processed["metadata"]["heightAssumptions"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
