#!/usr/bin/env python3
"""Offline sampled KSQL mesh clearance. Simulator truth is evaluation-only."""
import argparse
import json
from pathlib import Path
import numpy as np
from ksql_mesh import load_ksql


def triangle_distances(point, triangles):
    a,b,c=triangles[:,0],triangles[:,1],triangles[:,2]
    ab=b-a;ac=c-a;ap=point-a
    normal=np.cross(ab,ac);nn=np.einsum('ij,ij->i',normal,normal)
    dot=np.einsum('ij,ij->i',ap,normal)
    projected=ap-normal*(dot/np.maximum(nn,1.e-20))[:,None]
    d00=np.einsum('ij,ij->i',ab,ab);d01=np.einsum('ij,ij->i',ab,ac);d11=np.einsum('ij,ij->i',ac,ac)
    d20=np.einsum('ij,ij->i',projected,ab);d21=np.einsum('ij,ij->i',projected,ac)
    denom=d00*d11-d01*d01
    v=(d11*d20-d01*d21)/np.maximum(denom,1.e-20);w=(d00*d21-d01*d20)/np.maximum(denom,1.e-20)
    inside=(v>=0)&(w>=0)&(v+w<=1)&(nn>1.e-15)
    distances=[]
    for start,end in [(a,b),(b,c),(c,a)]:
        line=end-start;t=np.clip(np.einsum('ij,ij->i',point-start,line)/np.maximum(np.einsum('ij,ij->i',line,line),1.e-20),0,1)
        distances.append(np.linalg.norm(point-start-t[:,None]*line,axis=1))
    result=np.minimum.reduce(distances)
    result[inside]=np.abs(dot[inside])/np.sqrt(nn[inside])
    return result


def verify(run, assets):
    import rosbag
    nav=json.loads((run/'navigation-report.json').read_text())
    events=nav['events'];start=next(e['sim_time_s'] for e in events if e['state']=='outbound')
    end=next(e['sim_time_s'] for e in events if e['state']=='landing')
    samples=[];last=-float('inf')
    with rosbag.Bag(str(run/'sensors.bag')) as bag:
        for _,msg,_ in bag.read_messages(topics=['/ground_truth/state']):
            t=msg.header.stamp.to_sec()
            if start<=t<=end and t-last>=.2:
                p=msg.pose.pose.position;samples.append([p.x,p.y,p.z]);last=t
    points=np.asarray(samples)
    if len(points)<10:raise ValueError('No outbound/return truth samples')
    cap=3.
    meshes=load_ksql(assets);triangles=np.concatenate(list(meshes.values()))
    low=triangles.min(1);high=triangles.max(1)
    relevant=np.all(high>=points.min(0)-cap,axis=1)&np.all(low<=points.max(0)+cap,axis=1)
    triangles=triangles[relevant];low=low[relevant];high=high[relevant]
    minimum=cap;near_count=0
    for point in points:
        bbox=np.linalg.norm(np.maximum(np.maximum(low-point,point-high),0.),axis=1)
        candidates=triangles[bbox<cap]
        if len(candidates):
            nearest=float(triangle_distances(point,candidates).min());minimum=min(minimum,nearest)
            if nearest<.6:near_count+=1
    return dict(pass_sampled_clearance=minimum>.6,sample_period_s=.2,truth_samples=len(points),relevant_triangles=len(triangles),
                minimum_centre_clearance_lower_bound_m=minimum,search_cap_m=cap,samples_below_0_6_m=near_count,
                evaluation_only_truth=True,caveat='Sampled point-to-mesh distance, capped at 3 m; not continuous swept-volume collision certification.')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path,help='Report path; default is inside the recording directory')
    parser.add_argument('--assets',type=Path,default=Path(__file__).resolve().parents[1]/'build/scenes/ksql-airport')
    args=parser.parse_args()
    try:report=verify(args.run,args.assets)
    except (ValueError,StopIteration) as error:
        report=dict(pass_sampled_clearance=False,error=str(error) or 'missing outbound or landing phase',evaluation_only_truth=True)
    output=args.output or args.run/'mesh-clearance-verification.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if not report['pass_sampled_clearance']:raise SystemExit(1)

if __name__=='__main__':main()
