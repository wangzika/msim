"""Bounded startup-log checks; no ROS dependency."""
from pathlib import Path
import re
import shutil


def require_recording_space(path,minimum_bytes=2*1024**3):
    path=Path(path)
    while not path.exists():path=path.parent
    free=shutil.disk_usage(path).free
    if free<minimum_bytes:
        raise RuntimeError(f'录包磁盘空间不足：{path} 可用 {free/1024**3:.2f} GiB，启动至少需要 {minimum_bytes/1024**3:.0f} GiB；请迁移数据或指定 --run。')


def raise_recorder_exit(path,returncode):
    path=Path(path)
    tail=path.read_bytes()[-65536:].decode('utf8',errors='replace') if path.is_file() else ''
    if 'Less than 1G of space free' in tail or 'No space left on device' in tail:
        raise RuntimeError(f'录包进程因磁盘空间不足停止（rosbag 最低剩余空间约 1 GB）；仿真退出是后续结果。请检查 {path}')
    raise RuntimeError(f'录包进程退出，代码 {returncode}；请检查 {path}')


def raise_if_simulation_crashed(path):
    path=Path(path)
    try:
        with path.open('rb') as stream:
            stream.seek(max(0,path.stat().st_size-65536))
            tail=stream.read().decode('utf8',errors='replace')
    except FileNotFoundError:return
    if re.search(r'Segmentation fault|\[(?:gazebo|sitl)[^\]]*\].*process has died|Aborted \(core dumped\)',tail):
        raise RuntimeError(f'Gazebo/PX4 simulation crashed; GNSS cannot publish after the crash. See {path}')
