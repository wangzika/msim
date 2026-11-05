#!/usr/bin/env python3
"""Run the locked RTKLIB catkin processor and validate its .pos output."""
import argparse
import datetime as dt
import hashlib
import json
import math
import os
import pathlib
import re
import signal
import socket
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def parse_pos(path):
    rows = []
    headers = []
    previous = None
    for line_number, line in enumerate(path.read_text(encoding="utf-8", errors="strict").splitlines(), 1):
        if not line.strip() or line.startswith("%"):
            if line.startswith("%"):
                headers.append(line)
            continue
        fields = line.split()
        if len(fields) < 15 or not re.fullmatch(r"\d{4}/\d{2}/\d{2}", fields[0]):
            raise ValueError(f"{path}:{line_number}: malformed RTKLIB solution row")
        try:
            when = dt.datetime.strptime(fields[0] + " " + fields[1], "%Y/%m/%d %H:%M:%S.%f")
            latitude, longitude, height = float(fields[2]), float(fields[3]), float(fields[4])
            status, satellites, ratio = int(fields[5]), int(fields[6]), float(fields[14])
            if not all(math.isfinite(value) for value in (latitude, longitude, height, ratio)):
                raise ValueError("non-finite solution value")
            if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
                raise ValueError("invalid latitude/longitude")
            if status not in range(1, 8) or not 0 <= satellites <= 255:
                raise ValueError("invalid solution status or satellite count")
            if previous is not None and when <= previous:
                raise ValueError("solution times are not strictly increasing")
            previous = when
            # RTKLIB is configured with out-timesys=gpst; do not label these values UTC.
            rows.append({"gpst": when.isoformat(), "latitude_deg": latitude,
                         "longitude_deg": longitude, "height_m": height,
                         "status_code": status, "satellites": satellites, "ratio": ratio,
                         "fields": fields})
        except (ValueError, IndexError) as exc:
            raise ValueError(f"{path}:{line_number}: {exc}") from exc
    if not headers or not any("program" in row.lower() for row in headers):
        raise ValueError(f"{path}: missing RTKLIB solution header")
    if not any("GPST" in row for row in headers):
        raise ValueError(f"{path}: missing explicit GPST header")
    return rows


def config_text(mode, ar, snr_error, with_stat=False, navsys=1):
    status = {"spp": "single", "relative_float": "kinematic", "relative_fixed": "kinematic"}[mode]
    ar_value = {"spp": "off", "relative_float": "off", "relative_fixed": ar}[mode]
    return ("pos1-posmode=" + status + "\n" +
            "pos1-frequency=l1+l2\npos1-elmask=10\npos1-ionoopt=brdc\n"
            "pos1-tropopt=saas\npos1-sateph=brdc\npos1-navsys=" + str(navsys) + "\n"
            "ant1-postype=rinexhead\nant2-postype=rinexhead\n"
            "pos1-dynamics=off\npos2-armode=" + ar_value + "\npos2-arthres=3\n"
            "stats-snrmax=52\nstats-errsnr=" + str(snr_error) + "\n"
            "out-solformat=llh\nout-outhead=on\nout-outopt=on\nout-outvel=on\n"
            "out-timesys=gpst\nout-timeform=hms\nout-timendec=3\nout-degform=deg\nout-outsingle=on\n" +
            ("out-outstat=residual\n" if with_stat else ""))


def ecef_from_pos(lat_deg, lon_deg, height):
    import math
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    a, e2 = 6378137.0, 6.69437999014e-3
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    return ((n + height) * math.cos(lat) * math.cos(lon),
            (n + height) * math.cos(lat) * math.sin(lon),
            (n * (1 - e2) + height) * math.sin(lat))


def expected_position(truth, receiver, elapsed):
    if receiver == "rover" and "_trajectory" in truth:
        from apps.fastglio.trajectory import antenna_state
        reference = truth["rover_truth_trajectory"]
        stamp = reference["origin_sim_time_ns"] + round(elapsed * 1e9)
        return antenna_state(truth["_trajectory"], stamp, truth["_trajectory_stamps"])[0]
    expected = list(truth["receiver_truth_ecef_m"][receiver])
    if receiver == "rover":
        expected = [expected[i] + truth["rover_velocity_ecef_mps"][i] * elapsed
                    for i in range(3)]
    return expected


def error_summary(rows, receiver, truth):
    tow_ns = truth["first_gpst"]["tow_ns"]
    first = dt.datetime(1980, 1, 6) + dt.timedelta(weeks=truth["first_gpst"]["week"],
                                                    seconds=tow_ns // 1_000_000_000,
                                                    microseconds=tow_ns % 1_000_000_000 // 1000)
    errors = []
    for row in rows:
        when = dt.datetime.fromisoformat(row["gpst"])
        elapsed = (when - first).total_seconds()
        expected = expected_position(truth, receiver, elapsed)
        measured = ecef_from_pos(row["latitude_deg"], row["longitude_deg"], row["height_m"])
        errors.append(math.sqrt(sum((measured[i] - expected[i]) ** 2 for i in range(3))))
    ordered = sorted(errors)
    return {"samples": len(errors), "mean_3d_error_m": sum(errors) / len(errors),
            "rmse_3d_m": math.sqrt(sum(e * e for e in errors) / len(errors)),
            "p95_3d_error_m": ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)],
            "max_3d_error_m": max(errors)} if errors else None


def baseline_summary(rows, truth):
    """Compare rover solution to the known base truth and moving rover truth."""
    import math
    tow_ns = truth["first_gpst"]["tow_ns"]
    first = dt.datetime(1980, 1, 6) + dt.timedelta(weeks=truth["first_gpst"]["week"],
                                                    seconds=tow_ns // 1_000_000_000,
                                                    microseconds=tow_ns % 1_000_000_000 // 1000)
    base = truth["receiver_truth_ecef_m"]["base"]
    length_errors, vector_errors = [], []
    for row in rows:
        when = dt.datetime.fromisoformat(row["gpst"])
        elapsed = (when - first).total_seconds()
        expected_rover = expected_position(truth, "rover", elapsed)
        measured_rover = ecef_from_pos(row["latitude_deg"], row["longitude_deg"], row["height_m"])
        expected_baseline = [expected_rover[i] - base[i] for i in range(3)]
        measured_baseline = [measured_rover[i] - base[i] for i in range(3)]
        length_errors.append(abs(math.sqrt(sum(x*x for x in measured_baseline)) -
                                 math.sqrt(sum(x*x for x in expected_baseline))))
        vector_errors.append(math.sqrt(sum((measured_baseline[i] - expected_baseline[i]) ** 2
                                           for i in range(3))))
    ordered = sorted(length_errors)
    ordered_vectors = sorted(vector_errors)
    if not rows:
        return None
    return {"samples": len(rows),
            "baseline_length_abs_error_mean_m": sum(length_errors) / len(rows),
            "baseline_length_abs_error_rmse_m": math.sqrt(sum(e * e for e in length_errors) / len(rows)),
            "baseline_length_abs_error_p95_m": ordered[min(len(ordered)-1, math.ceil(.95*len(ordered))-1)],
            "baseline_length_abs_error_max_m": max(length_errors),
            "baseline_vector_error_rmse_m": math.sqrt(sum(e * e for e in vector_errors) / len(rows)),
            "baseline_vector_error_p95_m": ordered_vectors[min(len(ordered_vectors)-1,
                                                                  math.ceil(.95*len(ordered_vectors))-1)],
            "baseline_vector_error_max_m": max(vector_errors)}


def write_launch(path, binary, config, rover, base, nav, pos, name):
    root = ET.Element("launch")
    node = ET.SubElement(root, "node", {"pkg": "rtklib", "type": pathlib.Path(binary).name,
                                         "name": name, "output": "screen", "required": "true"})
    # roslaunch searches devel/lib/rtklib by package+type; package path is in the built catkin overlay.
    for key, value in (("rover_obs", rover), ("base_obs", base), ("nav_file", nav),
                       ("output_pos", pos), ("config_file", config), ("startup_delay_s", "0.0")):
        ET.SubElement(node, "param", {"name": key, "value": str(value)})
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def run_process_group(command, cwd, env, log, timeout):
    """Run one command in an owned process group and reap it on every exit."""
    def terminate(process):
        if process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)

    with log.open("wb") as out:
        process = subprocess.Popen(command, cwd=cwd, env=env, stdout=out,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return_code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            terminate(process)
            raise RuntimeError(f"process group timed out after {timeout}s; see {log}") from exc
        except KeyboardInterrupt:
            terminate(process)
            raise
    if return_code != 0:
        raise RuntimeError(f"process group exited {return_code}; see {log}")
    return return_code


def run_one(source_root, setup, binary, config, rover, base, nav, pos, log, timeout):
    launch = pos.with_suffix(".launch")
    write_launch(launch, binary, config, rover, base, nav, pos, "msim_rtklib")
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    env = os.environ.copy()
    env["ROS_HOME"] = str(pos.parent / "ros-home")
    env["ROS_MASTER_URI"] = f"http://127.0.0.1:{port}"
    env["ROS_HOSTNAME"] = "127.0.0.1"
    command = ["pixi", "run", "--no-install", "--frozen", "-e", "default", "bash", "-c",
               'source "$1" && exec roslaunch "$2"', "_", str(setup), str(launch)]
    try:
        run_process_group(command, source_root, env, log, timeout)
    finally:
        launch.unlink(missing_ok=True)
    if not pos.is_file():
        raise RuntimeError(f"RTKLIB returned success without .pos output; see {log}")
    return parse_pos(pos)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rinex-dir", type=pathlib.Path, required=True)
    ap.add_argument("--output-dir", type=pathlib.Path, required=True)
    ap.add_argument("--source-root", type=pathlib.Path, default=pathlib.Path("/home/wzb/FAST_GLIO"))
    ap.add_argument("--build-manifest", type=pathlib.Path, required=True)
    ap.add_argument("--mode", choices=("spp", "relative_float", "relative_fixed"), default="spp")
    ap.add_argument("--timeout", type=int, default=180)
    ap.add_argument("--snr-error", type=float, default=0.0)
    ap.add_argument("--with-stat", action="store_true",
                    help="ask RTKLIB to write .pos.stat satellite residual and status records")
    ap.add_argument("--navsys", type=int, default=1,
                    help="RTKLIB system bitmask; 45 enables GPS, GLONASS, Galileo, BeiDou")
    args = ap.parse_args()
    rinex = args.rinex_dir.resolve()
    output = args.output_dir.resolve()
    root = pathlib.Path(__file__).resolve().parents[2]
    source_root = args.source_root.resolve()
    if output.exists():
        raise SystemExit(f"output already exists: {output}")
    manifest_path = args.build_manifest.resolve()
    manifest = json.loads(manifest_path.read_text())
    rinex_manifest = json.loads((rinex / "manifest.json").read_text())
    trajectory_reference = rinex_manifest.get("rover_truth_trajectory")
    if trajectory_reference:
        from apps.fastglio.trajectory import load as load_trajectory
        trajectory_path = pathlib.Path(trajectory_reference["path"])
        if sha(trajectory_path) != trajectory_reference["sha256"]:
            raise SystemExit("RINEX truth trajectory changed")
        rinex_manifest["_trajectory"] = load_trajectory(trajectory_path)
        rinex_manifest["_trajectory_stamps"] = [sample["sim_time_ns"] for sample in
                                                 rinex_manifest["_trajectory"]["samples"]]
    binary = pathlib.Path(manifest["executable"])
    probe = pathlib.Path(manifest["rinex_probe"])
    setup = binary.parents[2] / "setup.bash"
    required = [rinex / "base.obs", rinex / "rover.obs", rinex / "navigation.nav"]
    if not binary.is_file() or not probe.is_file() or not setup.is_file() or not all(p.is_file() for p in required):
        raise SystemExit("missing RTKLIB build or RINEX input")
    if sha(binary) != manifest["executable_sha256"] or sha(probe) != manifest["rinex_probe_sha256"]:
        raise SystemExit("RTKLIB processor or probe does not match build manifest")
    for item in required:
        if sha(item) != rinex_manifest["files_sha256"].get(item.name):
            raise SystemExit(f"RINEX input does not match its manifest: {item.name}")
    if not math.isfinite(args.snr_error) or args.snr_error < 0:
        raise SystemExit("--snr-error must be finite and non-negative")
    if args.navsys < 1 or args.navsys > 255:
        raise SystemExit("--navsys must be a nonzero RTKLIB system bitmask")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir()
    stage = output
    (stage / "run-manifest.json").write_text(json.dumps({"schema_version": "1.0", "status": "running",
         "mode": args.mode, "build_manifest": str(manifest_path), "snr_error": args.snr_error},
         indent=2, allow_nan=False) + "\n")
    parsed = {}
    try:
        for receiver_file, receiver_id in ((required[0], "base"), (required[1], "rover")):
            checked = subprocess.run(["pixi", "run", "--no-install", "--frozen", "-e", "default", str(probe),
                                      str(receiver_file), str(required[2])], cwd=source_root,
                                    check=True, capture_output=True, text=True, timeout=min(args.timeout, 60))
            match = re.search(r"obs_count=(\d+).*gps_ephemerides=(\d+)", checked.stdout, re.S)
            if not match:
                raise RuntimeError("RTKLIB RINEX probe returned unrecognized output")
            parsed[receiver_file.name] = {"observation_records": int(match.group(1)),
                                          "gps_ephemerides": int(match.group(2))}
            expected = rinex_manifest["receivers"][receiver_id]["full"]["satellite_record_count"]
            nav_expected = rinex_manifest.get("nav_records")
            if (parsed[receiver_file.name]["observation_records"] != expected or
                    (nav_expected is not None and
                     parsed[receiver_file.name]["gps_ephemerides"] != nav_expected) or
                    parsed[receiver_file.name]["gps_ephemerides"] <= 0):
                raise RuntimeError(f"RTKLIB parsed incomplete RINEX input for {receiver_id}")
        config = stage / "rtklib.conf"
        config.write_text(config_text(args.mode, "continuous", args.snr_error,
                                      args.with_stat, args.navsys))
        results = {}
        if args.mode == "spp":
            # RTKLIB's node requires both paths; single mode consumes the first observation file.
            results["rover"] = run_one(source_root, setup, binary, config, required[1], required[0], required[2],
                                       stage / "rover_spp.pos", stage / "rover_spp.log", args.timeout)
            results["base"] = run_one(source_root, setup, binary, config, required[0], required[1], required[2],
                                      stage / "base_spp.pos", stage / "base_spp.log", args.timeout)
        else:
            ar = "continuous" if args.mode == "relative_fixed" else "off"
            config.write_text(config_text(args.mode, ar, args.snr_error,
                                          args.with_stat, args.navsys))
            results["relative"] = run_one(source_root, setup, binary, config, required[1], required[0], required[2],
                                          stage / f"{args.mode}.pos", stage / f"{args.mode}.log", args.timeout)
        if any(not rows for rows in results.values()):
            raise RuntimeError("RTKLIB returned no positioning epochs; inspect the captured .log/.pos files")
        if args.with_stat:
            outputs = ([stage / "rover_spp.pos", stage / "base_spp.pos"] if args.mode == "spp"
                       else [stage / f"{args.mode}.pos"])
            for pos in outputs:
                stat = pathlib.Path(str(pos) + ".stat")
                if not stat.is_file() or "$SAT," not in stat.read_text(errors="replace"):
                    raise RuntimeError(f"RTKLIB did not write satellite records: {stat}")
        if args.mode == "relative_float" and not any(r["status_code"] == 2 for r in results["relative"]):
            raise RuntimeError("relative processing produced no Float epochs; check base antenna position and common satellites")
        error_stats = ({key: error_summary(rows, key, rinex_manifest) for key, rows in results.items()}
                       if args.mode == "spp" else
                       {"rover": error_summary(results["relative"], "rover", rinex_manifest)})
        status_error = {}
        baseline_error = None
        if args.mode != "spp":
            rows = results["relative"]
            status_error = {str(code): error_summary([r for r in rows if r["status_code"] == code],
                                                     "rover", rinex_manifest)
                            for code in sorted({r["status_code"] for r in rows})}
            baseline_error = baseline_summary(rows, rinex_manifest)
        summary = {"mode": args.mode, "rtklib_version": manifest["version"],
                   "snr_error_m": args.snr_error,
                   "rtklib_binary_sha256": manifest["executable_sha256"],
                   "rinex_inputs_sha256": {p.name: sha(p) for p in required},
                   "rtklib_parsed_inputs": parsed,
                   "solution_count": {key: len(rows) for key, rows in results.items()},
                   "truth_error": error_stats,
                   "truth_error_by_status": status_error,
                   "baseline_error": baseline_error,
                   "status_counts": {key: {str(code): sum(r["status_code"] == code for r in rows)
                                           for code in sorted({r["status_code"] for r in rows})}
                                     for key, rows in results.items()},
                   "status_names": {"1": "FIXED", "2": "FLOAT", "5": "SINGLE"},
                   "ambiguity_ratio_by_status": {str(code): {
                       "samples": sum(r["status_code"] == code for r in results.get("relative", [])),
                       "minimum": min((r["ratio"] for r in results.get("relative", []) if r["status_code"] == code), default=None),
                       "median": sorted(r["ratio"] for r in results.get("relative", []) if r["status_code"] == code)[
                           len([r for r in results.get("relative", []) if r["status_code"] == code]) // 2]
                           if any(r["status_code"] == code for r in results.get("relative", [])) else None}
                       for code in sorted({r["status_code"] for r in results.get("relative", [])})},
                   "outputs_sha256": {p.name: sha(p) for p in stage.iterdir()
                                      if p.is_file() and p.name not in ("run-manifest.json", "summary.json")}}
        if any(sha(item) != rinex_manifest["files_sha256"][item.name] for item in required):
            raise RuntimeError("RINEX input changed during RTKLIB run")
        if sha(binary) != manifest["executable_sha256"] or sha(probe) != manifest["rinex_probe_sha256"]:
            raise RuntimeError("RTKLIB executable changed during run")
        (stage / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        (stage / "run-manifest.json").write_text(json.dumps({"schema_version": "1.0", "status": "completed",
             "mode": args.mode, "build_manifest": str(manifest_path), "snr_error": args.snr_error},
             indent=2, allow_nan=False) + "\n")
    except Exception as exc:
        (stage / "run-manifest.json").write_text(json.dumps({"schema_version": "1.0", "status": "failed",
             "mode": args.mode, "error_type": type(exc).__name__, "error": str(exc)},
             indent=2, allow_nan=False) + "\n")
        raise
    except KeyboardInterrupt:
        (stage / "run-manifest.json").write_text(json.dumps({"schema_version": "1.0", "status": "cancelled",
             "mode": args.mode, "error_type": "KeyboardInterrupt", "error": "user cancelled run"},
             indent=2, allow_nan=False) + "\n")
        raise
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
