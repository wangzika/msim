#!/usr/bin/env python3
"""Create preliminary run provenance; this is not an orchestrator."""
import argparse
import hashlib
import json
import os
import platform
import re
from pathlib import Path
import subprocess
import sys


def git_info(repo):
    def git(*args):
        return subprocess.check_output(['git', '-C', str(repo), *args], text=True, stderr=subprocess.PIPE).strip()
    commit = git('rev-parse', '--verify', 'HEAD^{commit}')
    dirty = bool(git('status', '--porcelain', '--untracked-files=all'))
    return {'commit': commit, 'dirty': dirty}


def file_record(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    return {'path': str(path.resolve()), 'sha256': digest.hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', type=Path, required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--resolved-scenario', type=Path)
    parser.add_argument('--artifact', type=Path, action='append', default=[])
    parser.add_argument('--dependency-lock', type=Path, action='append', default=[])
    parser.add_argument('--image-digest', action='append', default=[],
                        help='Image reference in name@sha256:<64 hex> form')
    args = parser.parse_args()
    if not 0 <= args.seed < 2**64:
        parser.error('seed must be an unsigned 64-bit integer')
    if any(not re.fullmatch(r'[^@\s]+@sha256:[0-9a-f]{64}', item)
           for item in args.image_digest):
        parser.error('image digest must be an immutable name@sha256:<64 hex> reference')
    try:
        manifest = {'schema_version': 1, 'status': 'preliminary',
                    'scenario': file_record(args.scenario),
                    'seed': args.seed, 'git': git_info(args.repo),
                    'resolved_scenario': file_record(args.resolved_scenario) if args.resolved_scenario else None,
                    'artifacts': [file_record(p) for p in args.artifact],
                    'dependency_locks': [file_record(p) for p in args.dependency_lock],
                    'image_digests': args.image_digest,
                    'host': {'system': platform.system(), 'release': platform.release(),
                             'machine': platform.machine(), 'python': platform.python_version()},
                    'note': 'Preliminary record only; no orchestrator integration or completeness guarantee.'}
        data = (json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + '\n').encode()
        # Exclusive creation prevents replacing an existing record.
        fd = os.open(args.output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        with os.fdopen(fd, 'wb') as output:
            output.write(data)
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f'manifest: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
