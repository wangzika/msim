#!/usr/bin/env python3
"""Generate FAST_GLIO rover/base RINEX with the pinned SignalSim core."""

import argparse
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from apps.fastglio.trajectory import antenna_state, load

GPS_EPOCH = dt.datetime(1980, 1, 6, tzinfo=dt.timezone.utc)
WEEK_NS = 604800_000_000_000
NTP_OFFSET = 2208988800
PIN = "baba9deb68ed193efb9fd9a362b431ae8c491fa4"
NAV_SHA = "5d8576cde4032406b4a82d681e0aed49142dccd355491606816e568dd31ce167"


def sha(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def rinex_counts(path):
    """Count complete 5 Hz epochs and their satellite rows before RTKLIB reads them."""
    epochs = records = 0
    pending = None
    with path.open() as stream:
        for line in stream:
            if line.startswith(">"):
                if pending not in (None, 0):
                    raise ValueError(f"{path}: incomplete RINEX epoch")
                fields = line.split()
                pending = int(fields[8])
                epochs += 1
            elif pending is not None:
                if not re.match(r"^[GREC]\d\d", line):
                    raise ValueError(f"{path}: malformed satellite row")
                pending -= 1
                records += 1
                if pending < 0:
                    raise ValueError(f"{path}: too many satellite rows")
    if pending != 0:
        raise ValueError(f"{path}: truncated final epoch")
    return epochs, records


def validate_events(path, receiver, expected_epochs, expected_records, start_gpst_ns):
    records = 0
    index = -1
    with path.open() as stream:
        for index, line in enumerate(stream):
            if index >= expected_epochs:
                raise ValueError(f"{path}: extra epoch")
            epoch = json.loads(line)
            week, tow_ns = divmod(start_gpst_ns + index * 200_000_000, WEEK_NS)
            if (epoch["receiver"] != receiver or epoch["sim_time_ns"] != index * 200_000_000 or
                    epoch["gps_week"] != week or epoch["tow_ns"] != tow_ns):
                raise ValueError(f"{path}: epoch time or receiver differs from RINEX schedule")
            observations = epoch["observations"]
            if len(observations) % 2:
                raise ValueError(f"{path}: unmatched dual-frequency signal")
            for obs in observations:
                if not all(math.isfinite(obs[key]) for key in
                           ("pseudorange_m", "carrier_phase_cycles", "doppler_hz", "cn0_dbhz")):
                    raise ValueError(f"{path}: non-finite observable")
            records += len(observations) // 2
    if index + 1 != expected_epochs or records != expected_records:
        raise ValueError(f"{path}: event count differs from RINEX")


def gpst_start(utc_text, leap_path):
    utc = dt.datetime.fromisoformat(utc_text.replace("Z", "+00:00"))
    if utc.tzinfo is None:
        raise ValueError("UTC start must include timezone")
    entries = []
    for line in leap_path.read_text().splitlines():
        fields = line.split("#", 1)[0].split()
        if len(fields) == 2 and all(value.isdigit() for value in fields):
            entries.append((int(fields[0]), int(fields[1])))
    offset = next((tai - 19 for timestamp, tai in reversed(entries)
                   if timestamp <= utc.timestamp() + NTP_OFFSET), None)
    if offset is None:
        raise ValueError("UTC start is not covered by leap-second table")
    elapsed = utc - GPS_EPOCH
    return round(elapsed.total_seconds() * 1e9) + offset * 1_000_000_000


def ecef_to_lla(x, y, z):
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    longitude = math.atan2(y, x)
    horizontal = math.hypot(x, y)
    latitude = math.atan2(z, horizontal * (1.0 - e2))
    for _ in range(10):
        radius = a / math.sqrt(1.0 - e2 * math.sin(latitude) ** 2)
        altitude = horizontal / math.cos(latitude) - radius
        latitude = math.atan2(z, horizontal * (1.0 - e2 * radius / (radius + altitude)))
    radius = a / math.sqrt(1.0 - e2 * math.sin(latitude) ** 2)
    altitude = horizontal / math.cos(latitude) - radius
    return math.degrees(latitude), math.degrees(longitude), altitude


def ecef_from_lla(lat_deg, lon_deg, height):
    latitude, longitude = math.radians(lat_deg), math.radians(lon_deg)
    e2 = (1 / 298.257223563) * (2 - 1 / 298.257223563)
    radius = 6378137.0 / math.sqrt(1 - e2 * math.sin(latitude) ** 2)
    return ((radius + height) * math.cos(latitude) * math.cos(longitude),
            (radius + height) * math.cos(latitude) * math.sin(longitude),
            (radius * (1 - e2) + height) * math.sin(latitude))


def enu_to_ecef(east, north, up, lat_deg, lon_deg):
    latitude, longitude = math.radians(lat_deg), math.radians(lon_deg)
    return (-math.sin(longitude) * east - math.sin(latitude) * math.cos(longitude) * north +
            math.cos(latitude) * math.cos(longitude) * up,
            math.cos(longitude) * east - math.sin(latitude) * math.sin(longitude) * north +
            math.cos(latitude) * math.sin(longitude) * up,
            math.cos(latitude) * north + math.sin(latitude) * up)


def scenario_document(start, nav, position, name):
    # SignalSim's stock trajectory DSL cannot reproduce sampled Gazebo truth.
    # The C++ adapter replaces its position/velocity at every GNSS epoch.
    lat, lon, height = position
    return {
        "version": 1.0,
        "initTime": {"type": "UTC", "year": start.year, "month": start.month,
                     "day": start.day, "hour": start.hour, "minute": start.minute,
                     "second": start.second},
        "trajectory": {"name": name,
                       "initPosition": {"type": "LLA", "format": "d",
                                        "longitude": lon, "latitude": lat, "altitude": height},
                       "initVelocity": {"type": "SCU", "speed": 0, "course": 0},
                       "trajectoryList": [{"type": "Const", "duration": 301}]},
        "ephemeris": {"type": "RINEX", "name": str(nav)},
        "output": {"type": "observation", "format": "RINEX", "name": name + ".obs",
                   "interval": 0.2, "config": {"elevationMask": 10},
                   "systemSelect": [
                       {"system": "GPS", "signal": "L1CA", "enable": True},
                       {"system": "GPS", "signal": "L2C", "enable": True},
                       {"system": "BDS", "signal": "B1I", "enable": True},
                       {"system": "BDS", "signal": "B2I", "enable": True},
                       {"system": "Galileo", "signal": "E1", "enable": True},
                       {"system": "Galileo", "signal": "E5b", "enable": True},
                       {"system": "GLONASS", "signal": "G1", "enable": True},
                       {"system": "GLONASS", "signal": "G2", "enable": True}]},
        "power": {"noiseFloor": -172,
                  "initPower": {"unit": "dBHz", "value": 45},
                  "elevationAdjust": True},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", required=True, type=Path)
    ap.add_argument("--scenario", type=Path,
                    help="frozen SignalSim scenario; defaults to RUN/scenario.json")
    ap.add_argument("--output-dir", required=True, type=Path)
    ap.add_argument("--signalsim-source", type=Path,
                    default=ROOT / "build/deps/SignalSim")
    ap.add_argument("--binary", type=Path,
                    default=ROOT / "build/signalsim/signalsim_observables")
    ap.add_argument("--retain-nlos", action="store_true",
                    help="include one-bounce-only NLOS signals for degraded urban stress tests")
    args = ap.parse_args()
    run, output, source = args.run.resolve(), args.output_dir.resolve(), args.signalsim_source.resolve()
    if output.exists():
        ap.error(f"output already exists: {output}")
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if revision != PIN:
        ap.error(f"SignalSim revision {revision} differs from pin {PIN}")
    if not args.binary.is_file():
        ap.error(f"SignalSim adapter binary missing: {args.binary}")
    config_path = (args.scenario or run / "scenario.json").resolve()
    config = json.loads(config_path.read_text())
    capture = json.loads((run / "capture-report.json").read_text())
    trajectory_path = run / "trajectory.json"
    trajectory = load(trajectory_path)
    stamps = [sample["sim_time_ns"] for sample in trajectory["samples"]]
    if capture["origin_lla_deg_m"] != config["geodesy"]["origin_lla_deg_m"]:
        ap.error("trajectory and scenario geodetic origins differ")
    duration = capture["duration_s"]
    if duration != config["time"]["duration_s"] or config["time"]["gnss_rate_hz"] != 5:
        ap.error("capture and SignalSim scenario schedule differ")
    start = dt.datetime.fromisoformat(config["time"]["utc_start"].replace("Z", "+00:00"))
    leaps = Path(config["gnss"]["leap_seconds_source"])
    gpst_ns = gpst_start(config["time"]["utc_start"], leaps)
    lat, lon, height = config["geodesy"]["origin_lla_deg_m"]
    origin = ecef_from_lla(lat, lon, height)
    base_enu = config["receivers"].get("base_enu_m", [0.0, 0.0, 0.0])
    delta = enu_to_ecef(*base_enu, lat, lon)
    base = [origin[i] + delta[i] for i in range(3)]
    nav_source = source / "EphData/BRDC00IGS_R_20211700000_01D_MN.rnx"
    if not nav_source.is_file():
        ap.error(f"SignalSim mixed navigation file missing: {nav_source}")
    if sha(nav_source) != NAV_SHA:
        ap.error("SignalSim mixed navigation file differs from pinned source")
    if (config["gnss"].get("backend") == "signalsim_observables" and
            Path(config["gnss"]["ephemeris"]).resolve() != nav_source):
        ap.error("SignalSim scenario ephemeris differs from pinned mixed navigation file")
    output.mkdir(parents=True)
    nav = output / "navigation.nav"
    shutil.copyfile(nav_source, nav)
    boxes = output / "buildings.txt"
    reflection = config["gnss"].get("urban_reflection_coefficient", 0.0)
    with boxes.open("w") as stream:
        stream.write(f"{lat} {lon} {height} {reflection} {int(args.retain_nlos)}\n")
        for box in config["gnss"].get("urban_boxes_enu_m", []):
            if len(box) != 6 or not all(math.isfinite(value) for value in box):
                raise ValueError("invalid urban building box")
            stream.write(" ".join(str(value) for value in box) + "\n")
    signal_counts = {}
    receiver_counts = {}
    receiver_truth = {}
    for receiver in ("rover", "base"):
        first_position = None
        epochs_file = output / f"{receiver}-epochs.txt"
        with epochs_file.open("w") as stream:
            for index in range(duration * 5):
                stamp = capture["gnss_origin_sim_time_ns"] + index * 200_000_000
                position, velocity = (antenna_state(trajectory, stamp, stamps)
                                      if receiver == "rover" else (base, [0.0] * 3))
                if first_position is None:
                    first_position = position
                week, tow_ns = divmod(gpst_ns + index * 200_000_000, WEEK_NS)
                if tow_ns % 1_000_000:
                    raise ValueError("SignalSim 1 ms GNSS time cannot represent epoch")
                stream.write(f"{week} {tow_ns // 1_000_000} " +
                             " ".join(f"{value:.9f}" for value in position + velocity) + "\n")
        # The upstream scenario uses the initial point only for initialization;
        # every observation is then driven by the recorded antenna ECEF state.
        x, y, z = first_position
        receiver_truth[receiver] = list(first_position)
        first_lla = ecef_to_lla(x, y, z)
        scenario = scenario_document(start, nav, first_lla, receiver)
        scenario_file = output / f"{receiver}-signalsim.json"
        scenario_file.write_text(json.dumps(scenario, indent=2) + "\n")
        process = subprocess.run([str(args.binary.resolve()), str(scenario_file),
                                  str(epochs_file), str(boxes),
                                  str(output / f"{receiver}.obs"),
                                  str(output / f"{receiver}-observations.jsonl"), receiver],
                                 text=True, capture_output=True)
        (output / f"{receiver}-signalsim.log").write_text(process.stdout + process.stderr)
        if process.returncode != 0:
            raise RuntimeError(f"SignalSim {receiver} failed: {process.stderr.strip()}")
        signal_counts[receiver] = process.stdout.strip()
        observed_epochs, records = rinex_counts(output / f"{receiver}.obs")
        if observed_epochs != duration * 5:
            raise ValueError(f"SignalSim {receiver} epoch count differs from capture")
        receiver_counts[receiver] = {"full": {"satellite_record_count": records,
                                               "epoch_count": observed_epochs}}
        validate_events(output / f"{receiver}-observations.jsonl", receiver,
                        observed_epochs, records, gpst_ns)
    combined_events = output / "epochs.jsonl"
    with combined_events.open("w") as destination:
        for receiver in ("base", "rover"):
            with (output / f"{receiver}-observations.jsonl").open() as source_stream:
                shutil.copyfileobj(source_stream, destination)
    manifest = {"backend": "signalsim_observables", "source_commit": revision,
                "adapter_sha256": sha(args.binary.resolve()),
                "source_nav_sha256": sha(nav_source), "trajectory_sha256": sha(trajectory_path),
                "scenario_sha256": sha(config_path),
                "origin_sim_time_ns": capture["gnss_origin_sim_time_ns"],
                "first_gpst": {"week": divmod(gpst_ns, WEEK_NS)[0],
                               "tow_ns": divmod(gpst_ns, WEEK_NS)[1]},
                "interval_s": 0.2, "expected_epochs": duration * 5,
                "receiver_truth_ecef_m": receiver_truth,
                "rover_velocity_ecef_mps": [0.0, 0.0, 0.0],
                "rover_truth_trajectory": {"path": str(trajectory_path),
                                           "sha256": sha(trajectory_path),
                                           "origin_sim_time_ns": capture["gnss_origin_sim_time_ns"]},
                "receivers": receiver_counts,
                "rtklib_bds_prn_limit": 46,
                "retain_nlos": args.retain_nlos,
                "generator_output": signal_counts,
                "files_sha256": {path.name: sha(path) for path in output.iterdir() if path.is_file()}}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
