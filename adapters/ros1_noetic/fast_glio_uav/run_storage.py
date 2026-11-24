"""External recording storage with a POSIX runtime for PX4 on exFAT/NTFS."""
import argparse
import errno
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import uuid


def recording_root(project):
    project=Path(project)
    override=os.environ.get('MSIM_RUN_ROOT')
    if override:return Path(override).expanduser().resolve()
    settings=project/'config/run-storage.json'
    if not settings.is_file():return project/'runs'
    config=json.loads(settings.read_text())
    mount=config.get('mount_point')
    if mount and not os.path.ismount(mount):
        raise RuntimeError(f'外置盘未挂载：{mount}；未启动采集，也不会回退写入系统盘。')
    return Path(config['run_root']).expanduser().resolve()


def runtime_directory(run):
    run=Path(run);run.parent.mkdir(parents=True,exist_ok=True)
    probe=run.parent/('.msim-link-probe-'+uuid.uuid4().hex)
    try:
        probe.symlink_to('missing-probe-target')
        return run
    except OSError as error:
        if error.errno in (errno.EROFS,errno.EACCES,errno.ENOSPC):
            raise RuntimeError(f'采集目录不可写或空间不足：{run.parent}: {error}') from error
        return Path(tempfile.mkdtemp(prefix='msim-runtime-'+str(os.getuid())+'-'))
    finally:
        probe.unlink(missing_ok=True)


def file_hash(path):
    result=hashlib.sha256()
    with Path(path).open('rb') as stream:
        while block:=stream.read(4*1024*1024):result.update(block)
    return result.hexdigest()


def compatibility_link(project,run):
    project=Path(project);run=Path(run).resolve()
    settings=project/'config/run-storage.json'
    config=json.loads(settings.read_text()) if settings.is_file() else {}
    root=Path(os.environ.get('MSIM_RUN_ROOT') or config.get('run_root') or project/'runs').expanduser().resolve()
    if root.resolve()==(project/'runs').resolve() or not run.is_relative_to(root):return
    recording_root(project)  # A managed external path must really be mounted.
    local=project/'runs'/run.relative_to(root)
    if local.exists() or local.is_symlink():return
    local.parent.mkdir(parents=True,exist_ok=True);local.symlink_to(run,target_is_directory=True)


def archive_runtime(runtime,run,cleanup=False):
    runtime=Path(runtime);run=Path(run)
    if runtime.resolve()==run.resolve():return {}
    if not runtime.name.startswith('msim-runtime-') or runtime.parent!=Path(tempfile.gettempdir()):
        raise ValueError('Refusing an unrecognized runtime directory')
    if not runtime.is_dir():return {}
    run.mkdir(parents=True,exist_ok=True)
    files=[];links=[]
    for folder,dirs,names in os.walk(runtime,followlinks=False):
        for name in dirs+names:
            source=Path(folder)/name;relative=source.relative_to(runtime);target=run/relative
            if source.is_symlink():
                links.append(dict(path=str(relative),target=os.readlink(source)))
            elif source.is_file():
                target.parent.mkdir(parents=True,exist_ok=True)
                temporary=target.with_name(target.name+'.archive-tmp')
                shutil.copyfile(source,temporary)
                with temporary.open('rb') as stream:os.fsync(stream.fileno())
                expected=file_hash(source)
                if file_hash(temporary)!=expected:raise IOError('Runtime archive checksum failed: '+str(relative))
                os.replace(temporary,target)
                files.append(dict(path=str(relative),bytes=source.stat().st_size,sha256=expected))
    result=dict(runtime_path=str(runtime),recording_path=str(run),archived=True,
                files=files,symlinks=links,cleaned=cleanup)
    (run/'runtime-storage.json').write_text(json.dumps(result,indent=2)+'\n')
    if cleanup:shutil.rmtree(runtime)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    root=sub.add_parser('root');root.add_argument('project',type=Path)
    runtime=sub.add_parser('runtime');runtime.add_argument('run',type=Path)
    archive=sub.add_parser('archive');archive.add_argument('runtime',type=Path);archive.add_argument('run',type=Path);archive.add_argument('--cleanup',action='store_true')
    link=sub.add_parser('link');link.add_argument('project',type=Path);link.add_argument('run',type=Path)
    args=parser.parse_args()
    if args.command=='root':print(recording_root(args.project))
    elif args.command=='runtime':print(runtime_directory(args.run))
    elif args.command=='link':compatibility_link(args.project,args.run)
    else:archive_runtime(args.runtime,args.run,args.cleanup)


if __name__=='__main__':main()
