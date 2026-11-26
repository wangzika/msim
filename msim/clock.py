"""Availability-time clock for deterministic fast or wall-paced replay."""
import math
import time


class ReplayClock:
    def __init__(self, rate=0., sleeper=time.sleep, monotonic=time.monotonic):
        if not math.isfinite(rate) or rate<0:raise ValueError('Replay rate must be finite and nonnegative')
        self.rate=rate;self.now_ns=None;self.origin=None;self.wall=None
        self.sleep=sleeper;self.monotonic=monotonic

    def advance(self, arrival_ns):
        if type(arrival_ns) is not int or arrival_ns<0:raise ValueError('Invalid availability timestamp')
        if self.now_ns is not None and arrival_ns<self.now_ns:raise ValueError('Availability time moved backwards')
        if self.origin is None:self.origin=arrival_ns;self.wall=self.monotonic()
        if self.rate:
            wait=(arrival_ns-self.origin)/1e9/self.rate-(self.monotonic()-self.wall)
            if wait>0:self.sleep(wait)
        self.now_ns=arrival_ns
