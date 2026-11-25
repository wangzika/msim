"""Map registration and route geometry; no ROS, GPS or simulation-truth inputs."""
import json
import hashlib
import math
import struct
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


FLU_FRD=np.diag([1.,-1.,-1.])


def fresh_stamp(now, stamp, maximum=.5):
    """Allow one 10 Hz scan plus /clock delivery skew, retaining stale cutoff."""
    return -.15<=now-stamp<maximum


def param_int_wire(value):
    """PX4 PARAM_SET uses byte-wise INT32 encoding in the float transport field."""
    return struct.unpack('<f',struct.pack('<i',int(value)))[0]


def param_int_value(value):
    return struct.unpack('<i',struct.pack('<f',float(value)))[0]


def transform(points, matrix):
    return np.asarray(points)@matrix[:3,:3].T+matrix[:3,3]


def pose(position, quaternion):
    result=np.eye(4);result[:3,:3]=Rotation.from_quat(quaternion).as_matrix();result[:3,3]=position
    return result


def voxel(points, size=.5, maximum=120000):
    p=np.asarray(points,dtype=np.float64).reshape(-1,3);p=p[np.isfinite(p).all(axis=1)]
    if not len(p):return p
    _,ids=np.unique(np.floor(p/size).astype(np.int64),axis=0,return_index=True)
    p=p[np.sort(ids)]
    if len(p)>maximum:p=p[np.linspace(0,len(p)-1,maximum).astype(int)]
    return p


def descriptor(points, radius=80., rings=12, sectors=36):
    p=np.asarray(points);r=np.linalg.norm(p[:,:2],axis=1)
    ok=(r>.8)&(r<radius)&np.isfinite(p).all(axis=1);p=p[ok];r=r[ok]
    result=np.zeros((rings,sectors))
    if not len(p):return result
    rows=np.minimum((r/radius*rings).astype(int),rings-1)
    columns=((np.arctan2(p[:,1],p[:,0])+2*np.pi)%(2*np.pi)/(2*np.pi)*sectors).astype(int)
    # Height span plus occupancy. A Scan-Context-inspired retrieval baseline,
    # not an implementation of the published Scan Context++ algorithm.
    low=np.full_like(result,np.inf);high=np.full_like(result,-np.inf)
    np.minimum.at(low,(rows,columns),p[:,2]);np.maximum.at(high,(rows,columns),p[:,2])
    valid=np.isfinite(low);result[valid]=1.+np.minimum(20.,high[valid]-low[valid])
    return result


def descriptor_match(query, reference):
    denom=np.linalg.norm(query)*np.linalg.norm(reference)
    if denom<1e-9:return 1.,0.
    scores=[1.-np.sum(np.roll(query,k,axis=1)*reference)/denom for k in range(query.shape[1])]
    index=int(np.argmin(scores));return float(scores[index]),index*2*np.pi/query.shape[1]


def icp(source, target_tree, initial, limits=(3.,1.,.5), iterations=18):
    source=np.asarray(source);estimate=np.array(initial,dtype=float)
    if len(source)<60:raise ValueError('Insufficient query points')
    for maximum in limits:
        for _ in range(iterations):
            current=transform(source,estimate)
            distances,indices=target_tree.query(current)
            valid=distances<maximum
            if np.count_nonzero(valid)<40:break
            cutoff=np.quantile(distances[valid],.85);valid&=distances<=cutoff
            a=current[valid];b=target_tree.data[indices[valid]]
            ac=a.mean(0);bc=b.mean(0)
            u,_,vt=np.linalg.svd((a-ac).T@(b-bc));rotation=vt.T@u.T
            if np.linalg.det(rotation)<0:vt[-1]*=-1;rotation=vt.T@u.T
            delta=np.eye(4);delta[:3,:3]=rotation;delta[:3,3]=bc-rotation@ac
            estimate=delta@estimate
            if np.linalg.norm(delta[:3,3])<.002 and Rotation.from_matrix(rotation).magnitude()<.001:break
    distances,_=target_tree.query(transform(source,estimate));valid=distances<.5
    overlap=float(np.mean(valid));rmse=float(np.sqrt(np.mean(distances[valid]**2))) if valid.any() else math.inf
    return estimate,dict(overlap=overlap,rmse_m=rmse,score=overlap-.5*rmse)


class MapLocalizer:
    def __init__(self, bundle):
        path=Path(bundle);self.info=json.loads((path/'map.json').read_text())
        if self.info.get('sha256')!=hashlib.sha256((path/'map.npz').read_bytes()).hexdigest():raise ValueError('Map checksum mismatch')
        arrays=np.load(path/'map.npz',allow_pickle=False)
        self.points=arrays['points'];self.key_positions=arrays['positions'];self.descriptors=arrays['descriptors']
        if len(self.points)<100 or not len(self.key_positions):raise ValueError('Empty localization map')
        self.tree=cKDTree(self.points);self.last=None;self.confirmations=0

    def locate(self, cloud, position):
        local=voxel(np.asarray(cloud)-position,maximum=1200)
        if len(local)<100:return dict(accepted=False,reason='insufficient_points')
        query=descriptor(local);rank=[]
        for i,reference in enumerate(self.descriptors):
            distance,yaw=descriptor_match(query,reference);rank.append((distance,i,yaw))
        hypotheses=[]
        for _,index,yaw in sorted(rank)[:6]:
            for angle in (yaw,yaw+np.pi):
                initial=np.eye(4);initial[:3,:3]=Rotation.from_euler('z',angle).as_matrix();initial[:3,3]=self.key_positions[index]
                matrix,quality=icp(local,self.tree,initial)
                if quality['overlap']>.55 and quality['rmse_m']<.3:hypotheses.append((quality['score'],matrix,quality))
        if not hypotheses:self.confirmations=0;return dict(accepted=False,reason='no_geometric_match')
        hypotheses.sort(key=lambda x:x[0],reverse=True);score,matrix,quality=hypotheses[0]
        for other,alternative,_ in hypotheses[1:]:
            if score-other<.03 and np.linalg.norm(matrix[:3,3]-alternative[:3,3])>2.:
                self.confirmations=0;return dict(accepted=False,reason='ambiguous_candidates')
        # Convert query centred at the current LIO position into map <- odom.
        correction=matrix.copy();correction[:3,3]-=matrix[:3,:3]@position
        consistent=self.last is not None and np.linalg.norm(correction[:3,3]-self.last[:3,3])<.6 and Rotation.from_matrix(correction[:3,:3]@self.last[:3,:3].T).magnitude()<.12
        self.confirmations=self.confirmations+1 if consistent else 1;self.last=correction
        return dict(accepted=self.confirmations>=2,reason='confirmed' if self.confirmations>=2 else 'confirming',
                    transform=correction.tolist(),confirmations=self.confirmations,**quality)


def bounded_target(current, goal, step):
    current=np.asarray(current);delta=np.asarray(goal)-current;distance=np.linalg.norm(delta)
    return current+delta*min(1.,step/max(distance,1e-9))


def segment_clear(points, start, end, radius=.65):
    p=np.asarray(points)
    if len(p)<100:return False
    a=np.asarray(start);v=np.asarray(end)-a
    t=np.clip((p-a)@v/max(float(v@v),1e-9),0.,1.)
    return bool(np.min(np.linalg.norm(p-(a+t[:,None]*v),axis=1))>radius)
