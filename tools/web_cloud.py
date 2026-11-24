"""Small binary point-cloud packets and bounded PCD preview, without ROS."""
from array import array
from functools import lru_cache
import math
from pathlib import Path
import struct
import sys

MAX_MAP_POINTS = 90000
MAX_SCAN_POINTS = 12000


@lru_cache(maxsize=3)
def _pcd_preview(filename, modified_ns, size, limit):
    path = Path(filename)
    with path.open('rb') as stream:
        header = {}; consumed = 0
        while True:
            line = stream.readline(4096); consumed += len(line)
            if not line or consumed > 65536:
                raise ValueError('Invalid PCD header')
            text = line.decode('ascii').strip()
            if text and not text.startswith('#'):
                key, *value = text.split();header[key.upper()] = value
                if key.upper() == 'DATA':break
        if header.get('DATA') != ['binary']:
            raise ValueError('PCD preview currently requires uncompressed binary data')
        fields = header['FIELDS']; sizes = list(map(int,header['SIZE']))
        counts = list(map(int,header.get('COUNT',['1']*len(fields))))
        types = header['TYPE']
        if not len(fields) == len(sizes) == len(counts) == len(types):
            raise ValueError('Invalid PCD fields')
        offsets = {}; stride = 0
        for name,width,count,kind in zip(fields,sizes,counts,types):
            if width <= 0 or count <= 0:raise ValueError('Invalid PCD point size')
            if name in ('x','y','z'):
                if width != 4 or count != 1 or kind != 'F':
                    raise ValueError('Expected float32 PCD XYZ')
                offsets[name] = stride
            stride += width*count
        total = int(header['POINTS'][0])
        if total < 0 or stride <= 0 or total*stride > 128*1024*1024 or consumed+total*stride > size:
            raise ValueError('PCD is truncated or exceeds preview limits')
        if set(offsets) != {'x','y','z'}:raise ValueError('Missing PCD XYZ')
        payload = stream.read(total*stride)
    points = array('f')
    for i in range(min(total,limit)):
        base = (i*total//min(total,limit))*stride
        point = [struct.unpack_from('<f',payload,base+offsets[k])[0] for k in ('x','y','z')]
        if all(math.isfinite(v) for v in point):points.extend(point)
    if sys.byteorder != 'little':points.byteswap()
    return dict(data=points.tobytes(), count=len(points)//3, total=total,
                frame='camera_init', revision=f'pcd-{modified_ns}-{size}', source='map.pcd', age_s=None)


def pcd_preview(path, limit=MAX_MAP_POINTS):
    path = Path(path);stat = path.stat()
    return _pcd_preview(str(path.resolve()),stat.st_mtime_ns,stat.st_size,max(1,min(int(limit),MAX_MAP_POINTS)))
