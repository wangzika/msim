#!/usr/bin/env python3
"""Export FAST_GLIO antenna truth for rtklib-plot-python at RINEX GPST epochs."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from apps.fastglio.trajectory import antenna_state, load

WEEK_NS = 604800_000_000_000


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def ecef_to_lla(x, y, z):
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - e2))
    for _ in range(10):
        sin_lat = math.sin(lat)
        radius = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
        height = p / math.cos(lat) - radius
        lat = math.atan2(z, p * (1.0 - e2 * radius / (radius + height)))
    radius = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    height = p / math.cos(lat) - radius
    return math.degrees(lat), math.degrees(lon), height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rinex-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.rinex_dir / "manifest.json").read_text())
    reference = manifest.get("rover_truth_trajectory")
    if not reference:
        parser.error("RINEX manifest has no rover trajectory; export with --trajectory")
    path = Path(reference["path"])
    if sha256(path) != reference["sha256"]:
        parser.error("trajectory hash differs from RINEX manifest")
    document = load(path)
    stamps = [sample["sim_time_ns"] for sample in document["samples"]]
    count = manifest["receivers"]["rover"]["full"]["epoch_count"]
    interval_ns = round(manifest["interval_s"] * 1_000_000_000)
    if count < 1 or interval_ns <= 0:
        parser.error("invalid RINEX epoch schedule")
    first_week = manifest["first_gpst"]["week"]
    first_tow = manifest["first_gpst"]["tow_ns"]
    origin_ns = reference["origin_sim_time_ns"]
    fields = ["GPS TOW (s)", "GPS Week", "Latitude (deg)", "Longitude (deg)",
              "Ellipsoid Height (m)", "ECEF X (m)", "ECEF Y (m)", "ECEF Z (m)"]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        for index in range(count):
            stamp = origin_ns + index * interval_ns
            position, _ = antenna_state(document, stamp, stamps)
            lat, lon, height = ecef_to_lla(*position)
            gps_ns = first_week * WEEK_NS + first_tow + index * interval_ns
            week, tow_ns = divmod(gps_ns, WEEK_NS)
            writer.writerow([f"{tow_ns / 1e9:.9f}", week, f"{lat:.10f}",
                             f"{lon:.10f}", f"{height:.5f}",
                             *(f"{value:.6f}" for value in position)])
    print(f"{args.output}: {count} antenna truth epochs")


if __name__ == "__main__":
    main()
