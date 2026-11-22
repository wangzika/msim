#!/usr/bin/env python3
"""Build an isolated LIO-SAM map optimizer for OpenCV 4.13 replay.

The external FAST_GLIO source and installed binary are left untouched. This
only corrects cv::eigen's current column-vector eigenvalue layout.
"""
import argparse
from pathlib import Path
import shlex
import subprocess


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True, type=Path)
    ap.add_argument('--fast-glio', type=Path, default=Path('/home/wzb/FAST_GLIO'))
    args = ap.parse_args()
    source_root = args.fast_glio
    output = args.run.resolve() / 'comparisons/lio-sam-patched'
    output.mkdir(parents=True, exist_ok=True)
    source = (source_root / 'LIO_SAM/src/mapOptmization.cpp').read_text()
    for old, new in (('matD1.at<float>(0, 1)', 'matD1.at<float>(1, 0)'),
                     ('matE.at<float>(0, i)', 'matE.at<float>(i, 0)')):
        if source.count(old) != 1:
            raise RuntimeError(f'Expected one occurrence of {old}')
        source = source.replace(old, new)
    patched_source = output / 'mapOptmization.cpp'
    patched_source.write_text(source)
    build = source_root / 'build/LIO_SAM/CMakeFiles/lio_sam_mapOptmization.dir'
    flags = (build / 'flags.make').read_text().splitlines()
    values = {line.split(' = ', 1)[0]: line.split(' = ', 1)[1]
              for line in flags if line.startswith('CXX_')}
    compiler = str(source_root / '.pixi/envs/default/bin/c++')
    command = [compiler]
    for key in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS'):
        command.extend(shlex.split(values[key]))
    obj = output / 'mapOptmization.o'
    command.extend(['-c', str(patched_source), '-o', str(obj)])
    subprocess.run(command, check=True)
    link = (build / 'link.txt').read_text().strip()
    link = link.replace('CMakeFiles/lio_sam_mapOptmization.dir/src/mapOptmization.cpp.o',
                        str(obj))
    binary = output / 'lio_sam_mapOptmization'
    link = link.replace(str(source_root / 'devel/lib/lio_sam/lio_sam_mapOptmization'),
                        str(binary))
    link = link.replace('-Wl,--dependency-file=CMakeFiles/lio_sam_mapOptmization.dir/link.d', '')
    subprocess.run(shlex.split(link), check=True, cwd=source_root / 'build/LIO_SAM')
    print(binary)


if __name__ == '__main__':
    main()
