"""Backend selection. The compatibility backend delegates without changing flags."""
import os
from pathlib import Path
import shutil
import subprocess
import sys


def gazebo_command(arguments, root=None):
    root = Path(root or Path(__file__).resolve().parents[1]).resolve()
    script = root / 'tools/px4-uav-capture.sh'
    if not script.is_file():
        raise ValueError('Gazebo compatibility backend requires the msim source checkout (--root).')
    if sys.platform != 'linux':
        raise ValueError('The existing Gazebo/PX4/ROS backend requires Linux; use a Linux host for the complete stack.')
    bash = shutil.which('bash')
    if not bash:
        raise ValueError('bash is required by the original simulation backend')
    return [bash, str(script), *arguments], root


def run_gazebo(arguments, root=None, dry_run=False):
    command, root = gazebo_command(arguments, root)
    if dry_run:
        return dict(backend='gazebo', command=command, cwd=str(root),
                    compatibility='Original PX4, sensor plugins, FAST-LIO and web entry; no parameter translation')
    # Inherit the original script's environment and process group. Ctrl+C reaches
    # its existing recorder/finalization handlers; do not launch a hidden orphan.
    process = subprocess.Popen(command, cwd=root, env=os.environ.copy())
    try:
        return_code = process.wait()
    except KeyboardInterrupt:
        try:
            return_code = process.wait(timeout=45)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        raise
    return dict(backend='gazebo', return_code=return_code)
