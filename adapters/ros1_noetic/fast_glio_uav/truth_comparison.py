"""Display-only truth/LIO translation; never feeds measurements to estimation."""
from collections import deque
import math


class TruthComparison:
    def __init__(self,tolerance_ns=50_000_000,capacity=2000):
        self.truth=deque(maxlen=capacity);self.tolerance=tolerance_ns;self.translation=None
        self.anchor_stamp=None;self.matches=0;self.skipped=0;self.last_lio_stamp=None
        self.max_delta_ns=0

    def add_truth(self,stamp_ns,position,orientation=(0.,0.,0.,1.)):
        if len(position)!=3 or not all(math.isfinite(v) for v in position):raise ValueError('Invalid truth position')
        if self.truth and stamp_ns<self.truth[-1][0]:self.reset()
        if self.truth and stamp_ns==self.truth[-1][0]:return
        if len(orientation)!=4 or not all(math.isfinite(v) for v in orientation):raise ValueError('Invalid truth orientation')
        self.truth.append((stamp_ns,tuple(position),tuple(orientation)))

    def reset(self):
        self.truth.clear();self.translation=None;self.anchor_stamp=None
        self.matches=0;self.skipped=0;self.last_lio_stamp=None
        self.max_delta_ns=0

    def match(self,stamp_ns,position):
        if len(position)!=3 or not all(math.isfinite(v) for v in position):raise ValueError('Invalid LIO position')
        if self.last_lio_stamp is not None and stamp_ns<=self.last_lio_stamp:
            self.skipped+=1;return None
        if not self.truth:self.skipped+=1;return None
        # Finite local history, nearest stamp with past-favoring tie break.
        stamp,truth,orientation=min(self.truth,key=lambda r:(abs(r[0]-stamp_ns),r[0]))
        if abs(stamp-stamp_ns)>self.tolerance:self.skipped+=1;return None
        if self.translation is None:
            self.translation=tuple(position[i]-truth[i] for i in range(3));self.anchor_stamp=stamp_ns
        aligned=tuple(truth[i]+self.translation[i] for i in range(3))
        world=tuple(position[i]-self.translation[i] for i in range(3))
        self.last_lio_stamp=stamp_ns;self.matches+=1
        self.max_delta_ns=max(self.max_delta_ns,abs(stamp-stamp_ns))
        return dict(stamp_ns=stamp_ns,truth_stamp_ns=stamp,truth_aligned=aligned,lio_world=world,
                    truth_orientation=orientation,error_enu_m=tuple(position[i]-aligned[i] for i in range(3)),translation=self.translation)

    def report(self):
        return dict(method='first matched translation only; no rotation or scale fit',
                    axes_assumed_aligned=True,truth_used_by_estimator=False,
                    translation_truth_world_to_camera_init_m=self.translation,
                    anchor_stamp_ns=self.anchor_stamp,matched_pairs=self.matches,skipped=self.skipped,
                    matching_tolerance_ms=self.tolerance/1e6,maximum_match_delta_ms=self.max_delta_ns/1e6,
                    note='Display/evaluation only. LIO output is not modified or fed back. Valid for aligned ENU/map axes.')
