#!/usr/bin/env python3
"""Measure sampling coverage before any estimated pose/map can influence it."""
import argparse
import json
from pathlib import Path
import numpy as np
import rosbag


def voxel_centroids(points, size=.5):
    keys = np.floor(points/size).astype(np.int64)
    unique, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    centroids = np.column_stack([np.bincount(inverse, weights=points[:, i])/counts for i in range(3)])
    return unique, centroids


def hash_mask(index, frame, stride=2, seed=17):
    with np.errstate(over='ignore'):
        x = (np.uint64(frame)*np.uint64(1000003)+index.astype(np.uint64)) ^ np.uint64(
            (seed*0x9e3779b97f4a7c15)&((1<<64)-1))
        x += np.uint64(0x9e3779b97f4a7c15)
        x = (x ^ (x >> np.uint64(30))) * np.uint64(0xbf58476d1ce4e5b9)
        x = (x ^ (x >> np.uint64(27))) * np.uint64(0x94d049bb133111eb)
        x ^= x >> np.uint64(31)
    return x % stride == 0


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    modes={'full':(1,0),'stride2-even':(2,0),'stride2-odd':(2,1),
           'stride3':(3,0),'stride4':(4,0),'hash-half':(2,None)}
    stats={key:[] for key in modes}
    with rosbag.Bag(str(args.bag)) as bag:
        for frame,(_,msg,_) in enumerate(bag.read_messages(topics=['/msim/velodyne_points_annotated']),1):
            dtype=np.dtype({'names':['x','y','z','ring'],
                'formats':['<f4','<f4','<f4','<u2'],
                'offsets':[next(f.offset for f in msg.fields if f.name==name) for name in ('x','y','z','ring')],
                'itemsize':msg.point_step})
            source=np.frombuffer(msg.data,dtype=dtype,count=msg.width*msg.height)
            points=np.column_stack([source[a] for a in ('x','y','z')])
            valid=np.sum(points**2,axis=1)>.25
            fullkeys,fullcenters=voxel_centroids(points[valid])
            fullmap={tuple(k):v for k,v in zip(fullkeys,fullcenters)}
            index=np.arange(len(points))
            for label,(stride,offset) in modes.items():
                mask=valid & (hash_mask(index,frame) if offset is None else index % stride==offset)
                chosen=points[mask]
                keys,centers=voxel_centroids(chosen)
                shifts=np.linalg.norm(centers-np.array([fullmap[tuple(k)] for k in keys]),axis=1)
                stats[label].append(dict(stamp_s=msg.header.stamp.to_sec(),points=len(chosen),
                    voxels=len(keys),voxel_retention=len(keys)/len(fullkeys),
                    centroid_shift_mean_m=float(np.mean(shifts)),
                    centroid_shift_p95_m=float(np.percentile(shifts,95)),
                    ring_counts=np.bincount(source['ring'][mask],minlength=16).tolist()))
    summary={}
    for label,rows in stats.items():
        rings=np.sum([r['ring_counts'] for r in rows],axis=0)
        summary[label]={key:float(np.mean([r[key] for r in rows]))
                        for key in ('points','voxels','voxel_retention','centroid_shift_mean_m','centroid_shift_p95_m')}
        summary[label]['ring_fraction']=(rings/rings.sum()).tolist()
    args.output.write_text(json.dumps(dict(summary=summary,frames=stats),indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
