"""Evaluation-only truth matching, bounded buffers, no truth feedback."""
from collections import deque,Counter
import heapq
import math
import numpy as np


class PositionEvaluator:
    def __init__(self, write_row, metadata, maximum_delta_ns=50_000_000,
                 align_lio_translation=False, assume_aligned_axes=False, capacity=4096):
        if maximum_delta_ns<0 or capacity<2:raise ValueError('Invalid matching configuration')
        self.write_row=write_row;self.metadata=metadata;self.delta=maximum_delta_ns
        self.align=align_lio_translation;self.assume_axes=assume_aligned_axes
        self.truth=deque(maxlen=capacity);self.pending=[];self.capacity=capacity
        self.serial=0;self.stats={};self.skipped=Counter()

    def observe(self,event):
        if event.kind not in ('pose','gnss'):return
        if event.stream=='truth':
            if self.truth and event.stamp_ns<=self.truth[-1].stamp_ns:
                self.skipped['unordered_truth']+=1;return
            self.truth.append(event)
        else:
            if len(self.pending)>=self.capacity:
                heapq.heappop(self.pending);self.skipped['pending_capacity']+=1
            self.serial+=1;heapq.heappush(self.pending,(event.stamp_ns,self.serial,event))
        self.flush()

    def flush(self, final=False):
        while self.pending and (final or self.truth and self.truth[-1].stamp_ns>=self.pending[0][0]+self.delta):
            _,_,event=heapq.heappop(self.pending)
            if not self.truth:self.skipped['missing_truth']+=1;continue
            reference=min(self.truth,key=lambda p:(abs(p.stamp_ns-event.stamp_ns),p.stamp_ns))
            dt=abs(reference.stamp_ns-event.stamp_ns)
            if dt>self.delta:self.skipped['truth_time_gap']+=1;continue
            cross_frame=event.frame!=reference.frame
            translated=event.kind=='pose' and self.align
            if cross_frame and not (translated and self.assume_axes):
                self.skipped['unaligned_frames:'+event.stream]+=1;continue
            truth=np.asarray(reference.data['position_m'],float)
            if event.data['point']!=reference.data['point']:
                if event.data['point']!='antenna' or reference.data['point']!='body':
                    self.skipped['reference_point_mismatch']+=1;continue
                lever=np.asarray(self.metadata.get('antenna_offset_body_m',[0.,0.,0.]),float)
                q=reference.data.get('orientation_xyzw')
                if q is None and np.linalg.norm(lever)>0:
                    self.skipped['missing_truth_orientation']+=1;continue
                if q is not None:
                    v=np.asarray(q[:3]);truth=truth+lever+2*(q[3]*np.cross(v,lever)+np.cross(v,np.cross(v,lever)))
            position=np.asarray(event.data['position_m'],float)
            entry=self.stats.setdefault(event.stream,dict(count=0,squared=np.zeros(3),maximum=0.,last=None,
                maximum_match_delta_ns=0,translation=None,source_frame=event.frame,evaluation_frame=reference.frame))
            if translated and entry['translation'] is None:entry['translation']=truth-position
            if translated:position=position+entry['translation']
            error=position-truth;norm=float(np.linalg.norm(error))
            entry['count']+=1;entry['squared']+=error*error;entry['maximum']=max(entry['maximum'],norm)
            entry['last']=error.tolist();entry['maximum_match_delta_ns']=max(entry['maximum_match_delta_ns'],dt)
            self.write_row(dict(stamp_ns=event.stamp_ns,stream=event.stream,frame=reference.frame,
                **dict(zip(('east_m','north_m','up_m'),position.tolist())),
                **dict(zip(('truth_east_m','truth_north_m','truth_up_m'),truth.tolist())),
                **dict(zip(('error_east_m','error_north_m','error_up_m'),error.tolist()))))

    def summary(self):
        result={}
        for name,item in self.stats.items():
            variances=item['squared']/item['count']
            result[name]=dict(matched_pairs=item['count'],rmse_enu_m=np.sqrt(variances).tolist(),
                rmse_3d_m=math.sqrt(float(variances.sum())),maximum_3d_error_m=item['maximum'],
                final_enu_error_m=item['last'],maximum_match_delta_ms=item['maximum_match_delta_ns']/1e6,
                translation_m=item['translation'].tolist() if item['translation'] is not None else None,
                source_frame=item['source_frame'],evaluation_frame=item['evaluation_frame'],
                alignment='first matched translation; axes assumed aligned' if item['translation'] is not None else 'same frame; no fit')
        return dict(position=result,skipped=dict(self.skipped),truth_used_for_algorithm=False,
                    truth_matching='nearest timestamp within tolerance; ties prefer past; bounded buffer')
