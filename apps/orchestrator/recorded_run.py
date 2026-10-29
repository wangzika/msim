#!/usr/bin/env python3
"""Run a compatible executable and save a provenance manifest.

The executable receives --scenario, --seed and --output after its supplied
arguments. This wrapper does not parse scenarios or implement simulation.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def git(repo, *args):
    return subprocess.check_output(['git', '-C', str(repo), *args], text=True, stderr=subprocess.PIPE).strip()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', type=Path, required=True, help='Resolved scenario input')
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New run directory')
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--image', action='append', default=[], help='name@sha256:<digest>')
    parser.add_argument('--lock', type=Path, action='append', default=[])
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or not 0 <= args.seed < 2**64:
        parser.error('a command and unsigned 64-bit seed are required')
    if any(not re.fullmatch(r'[^@\s]+@sha256:[a-f0-9]{64}', image) for image in args.image):
        parser.error('image must be name@sha256:<64 lowercase hex>')
    try:
        scenario = args.scenario.resolve(strict=True)
        scenario_bytes = scenario.read_bytes()
        lock_inputs = [(p.resolve(strict=True), p.read_bytes()) for p in args.lock]
        locks = [{'path': str(path), 'sha256': hashlib.sha256(data).hexdigest(),
                  'captured_path': f'inputs/lock-{i}'}
                 for i, (path, data) in enumerate(lock_inputs)]
        revision = git(args.repo, 'rev-parse', '--verify', 'HEAD^{commit}')
        dirty = bool(git(args.repo, 'status', '--porcelain', '--untracked-files=all'))
        run = args.output.absolute()
        run.mkdir(parents=False, exist_ok=False)
        (run / 'logs').mkdir()
        (run / 'artifacts').mkdir()
        (run / 'inputs').mkdir()
        for record, (_, data) in zip(locks, lock_inputs):
            (run / record['captured_path']).write_bytes(data)
        resolved = run / 'scenario.resolved.yaml'
        resolved.write_bytes(scenario_bytes)
        invocation = command + ['--scenario', str(resolved), '--seed', str(args.seed),
                                '--output', str(run / 'artifacts')]
        error = None
        with (run / 'logs/stdout.log').open('wb') as stdout, (run / 'logs/stderr.log').open('wb') as stderr:
            try:
                process = subprocess.run(invocation, stdout=stdout, stderr=stderr, check=False)
                exit_code = process.returncode
            except OSError as exc:
                error = str(exc)
                stderr.write((error + '\n').encode())
                exit_code = 127
        files = []
        for path in sorted(run.rglob('*')):
            if path.is_symlink():
                raise ValueError(f'symbolic link not allowed in run: {path}')
            if path.is_file():
                files.append({'path': path.relative_to(run).as_posix(), 'sha256': digest(path)})
        manifest = {
            'schema_version': 1, 'kind': 'recorded_process_run', 'seed': args.seed,
            'scenario': {'source_path': str(scenario), 'sha256': hashlib.sha256(scenario_bytes).hexdigest()},
            'git': {'commit': revision, 'dirty': dirty}, 'dependency_locks': locks,
            'image_digests': args.image, 'container_identity': 'caller supplied' if args.image else 'not supplied', 'host': {'system': platform.system(),
                'release': platform.release(), 'machine': platform.machine(),
                'python': platform.python_version(), 'gpu': 'not collected'},
            'command': invocation, 'exit_code': exit_code, 'error': error, 'files': files,
            'limitations': ['Input is supplied as already resolved; no scenario validation is performed.',
                            'The wrapper cannot verify that the executable uses the passed seed.',
                            'A dirty Git tree is recorded but its contents are not captured.']}
        data = (json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False) + '\n').encode()
        fd = os.open(run / 'manifest.json', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        with os.fdopen(fd, 'wb') as dest:
            dest.write(data)
        return exit_code if 0 <= exit_code <= 125 else 1
    except (OSError, subprocess.CalledProcessError, ValueError) as exc:
        print(f'recorded run: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
