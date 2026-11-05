#!/usr/bin/env python3
"""Build the RTKLIB source pinned in third_party/manifest.lock with its Pixi catkin env."""
import argparse
import hashlib
import io
import json
import pathlib
import shutil
import subprocess
import tarfile


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--workspace", type=pathlib.Path, default=pathlib.Path(__file__).resolve().parents[2])
    ap.add_argument("--source-root", type=pathlib.Path, required=True, help="FAST_GLIO repository root containing pixi.toml")
    ap.add_argument("--output-root", type=pathlib.Path, required=True)
    ap.add_argument("--manifest", type=pathlib.Path, required=True)
    ap.add_argument("--jobs", type=int, default=2)
    args = ap.parse_args()
    root, source_root, output = args.workspace.resolve(), args.source_root.resolve(), args.output_root.resolve()
    lock = json.loads((root / "third_party/manifest.lock").read_text())["components"]["rtklib"]
    source = pathlib.Path(lock["path"]).resolve()
    if source != (source_root / "rtklib").resolve():
        raise SystemExit(f"source root mismatch: lock expects {source}, got {source_root}")
    commit = subprocess.check_output(["git", "-C", str(source_root), "rev-parse", "HEAD"], text=True).strip()
    tree = subprocess.check_output(["git", "-C", str(source_root), "rev-parse", "HEAD:rtklib"], text=True).strip()
    if commit != lock["repository_commit"] or tree != lock["subdirectory_tree"]:
        raise SystemExit("RTKLIB source does not match third_party/manifest.lock; refusing to build unpinned code")
    dirty = subprocess.check_output(["git", "-C", str(source_root), "status", "--porcelain=v1",
                                     "--untracked-files=all", "--", "rtklib"], text=True).strip()
    if dirty:
        raise SystemExit("RTKLIB source worktree has tracked or untracked changes; refusing ambiguous build")
    for name in ("pixi.toml", "pixi.lock"):
        actual = sha(source_root / name)
        if actual != lock[f"{name.replace('.', '_')}_sha256"]:
            raise SystemExit(f"{name} does not match third_party/manifest.lock")
    if args.jobs < 1 or args.jobs > 32:
        raise SystemExit("jobs must be in [1, 32]")
    src_space = output / "src"
    if src_space.exists():
        raise SystemExit(f"source snapshot already exists; choose a new output root: {src_space}")
    src_space.mkdir(parents=True)
    package_link = src_space / "rtklib"
    package_link.mkdir()
    archive = subprocess.check_output(["git", "-C", str(source_root), "archive", "--format=tar", "HEAD:rtklib"])
    with tarfile.open(fileobj=io.BytesIO(archive), mode="r:") as bundle:
        for member in bundle.getmembers():
            if member.name.startswith("/") or ".." in pathlib.PurePosixPath(member.name).parts:
                raise SystemExit("unsafe path in locked RTKLIB git archive")
        bundle.extractall(package_link)
    probe_link = src_space / "msim_rtklib_probe"
    shutil.copytree(root / "tools/rtklib/rtklib_probe", probe_link)
    cmd = ["pixi", "run", "-e", "default", "catkin_make", "-C", str(output),
           "--source", str(src_space), "--pkg", "rtklib", "msim_rtklib_probe", "-DCATKIN_ENABLE_TESTING=OFF",
           "-DCMAKE_POLICY_VERSION_MINIMUM=3.5", "-DCMAKE_BUILD_TYPE=Release", f"-j{args.jobs}"]
    subprocess.run(cmd, cwd=source_root, check=True)
    binary = output / "devel/lib/rtklib/rtklib_rinex_processor"
    probe = output / "devel/lib/msim_rtklib_probe/msim_rtklib_probe"
    library = output / "devel/lib/librtklib.a"
    if not binary.is_file() or not library.is_file() or not probe.is_file():
        raise SystemExit("catkin returned success but expected binary/library is missing")
    compiler = subprocess.check_output(["pixi", "run", "-e", "default", "c++", "--version"], cwd=source_root, text=True).splitlines()[0]
    fingerprint = {"component": "rtklib", "version": lock["reported_version"],
                   "repository_commit": commit, "subdirectory_tree": tree,
                   "source_path": str(source), "build_environment": "FAST_GLIO pixi default; ROS Noetic; catkin",
                   "compiler": compiler, "build_type": "Release", "catkin_testing": False,
                   "source_snapshot": str(package_link),
                   "executable": str(binary), "executable_sha256": sha(binary),
                   "rinex_probe": str(probe), "rinex_probe_sha256": sha(probe),
                   "static_library": str(library), "static_library_sha256": sha(library),
                   "pixi_toml_sha256": sha(source_root / "pixi.toml"),
                   "pixi_lock_sha256": sha(source_root / "pixi.lock"),
                   "package_cmake_sha256": sha(source / "CMakeLists.txt"),
                   "probe_source_sha256": sha(probe_link / "main.cpp"),
                   "build_command": cmd}
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(fingerprint, indent=2) + "\n")
    print(json.dumps(fingerprint, indent=2))


if __name__ == "__main__":
    main()
