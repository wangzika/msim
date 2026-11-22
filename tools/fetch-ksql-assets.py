#!/usr/bin/env python3
"""Fetch only the pinned KSQL assets, checking Git blob identity and SHA-256."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def identity(path):
    size = path.stat().st_size
    git = hashlib.sha1(f'blob {size}\0'.encode())
    sha = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            git.update(chunk); sha.update(chunk)
    return dict(size=size, git_blob_sha1=git.hexdigest(), sha256=sha.hexdigest())


def fetch(manifest, output, verify_only=False):
    output.mkdir(parents=True, exist_ok=True)
    receipt = dict(repository=manifest['repository'], commit=manifest['commit'],
                   declared_package_license=manifest['declared_package_license'], files={})
    for relative, expected in manifest['files'].items():
        path = Path(relative)
        if path.is_absolute() or '..' in path.parts:
            raise ValueError('Unsafe asset path')
        dest = output/'upstream'/path
        current = identity(dest) if dest.is_file() else {}
        if any(current.get(k) != expected[k] for k in ('size', 'git_blob_sha1')):
            if verify_only:
                raise ValueError(f'Missing or changed asset: {dest}; run fetch-ksql-assets.py first')
            dest.parent.mkdir(parents=True, exist_ok=True)
            url = f"https://raw.githubusercontent.com/osrf/uav_testing/{manifest['commit']}/{relative}"
            temporary = dest.with_suffix(dest.suffix+'.part')
            print(f'Downloading {relative} ({expected["size"]/1e6:.1f} MB)', flush=True)
            request = urllib.request.Request(url, headers={'User-Agent': 'msim-scene-import/1.0'})
            with urllib.request.urlopen(request, timeout=45) as response, temporary.open('wb') as stream:
                received = 0
                while chunk := response.read(1024*1024):
                    received += len(chunk)
                    if received > expected['size']:
                        raise ValueError('Asset exceeds pinned size: '+relative)
                    stream.write(chunk)
            current = identity(temporary)
            if any(current[k] != expected[k] for k in ('size', 'git_blob_sha1')):
                raise ValueError('Asset identity does not match pinned source: '+relative)
            temporary.replace(dest)
        receipt['files'][relative] = dict(path=str(dest.resolve()), **current)
    (output/'asset-manifest.json').write_text(json.dumps(receipt, indent=2)+'\n')
    print('Verified KSQL assets: '+str(output.resolve()), flush=True)
    return receipt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, default=ROOT/'build/scenes/ksql-airport')
    ap.add_argument('--verify-only', action='store_true')
    args = ap.parse_args()
    manifest = json.loads((ROOT/'scenes/ksql_airport.source.json').read_text())
    fetch(manifest, args.output.resolve(), args.verify_only)


if __name__ == '__main__':
    main()
