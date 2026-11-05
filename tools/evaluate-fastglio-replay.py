#!/usr/bin/env python3
"""Compare replayed FAST_GLIO odometry with Gazebo truth after start alignment."""

import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path

import rosbag


def stamp_ns(msg):
    return msg.header.stamp.secs * 1_000_000_000 + msg.header.stamp.nsecs


def position(msg):
    p = msg.pose.pose.position
    return (p.x, p.y, p.z)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    samples = {'/Odometry': [], '/ground_truth/state': [],
               '/gnss_loose/diagnostics': []}
    with rosbag.Bag(str(args.bag)) as bag:
        for topic, msg, _ in bag.read_messages(topics=list(samples)):
            if topic == '/gnss_loose/diagnostics':
                samples[topic].append(msg.data)
            else:
                samples[topic].append((stamp_ns(msg), position(msg)))
    odom, truth = samples['/Odometry'], samples['/ground_truth/state']
    if not odom or not truth:
        ap.error('algorithm output or truth missing from replay bag')
    truth_stamps = [x[0] for x in truth]
    pairs = []
    for t, p in odom:
        index = bisect_left(truth_stamps, t)
        near = min((truth[j] for j in (index - 1, index) if 0 <= j < len(truth)),
                   key=lambda sample: abs(sample[0] - t), default=None)
        if near is not None and abs(near[0] - t) <= 50_000_000:
            pairs.append((t, p, near[1], abs(near[0] - t)))
    if len(pairs) < 10:
        ap.error(f'only {len(pairs)} odometry/truth pairs within 50 ms')
    first_odom, first_truth = pairs[0][1:3]
    errors = []
    for _, odom_p, truth_p, _ in pairs:
        delta = [odom_p[i] - first_odom[i] - (truth_p[i] - first_truth[i])
                 for i in range(3)]
        errors.append(delta)
    def rms(values):
        return math.sqrt(sum(x*x for x in values)/len(values))
    horizontal = [math.hypot(x, y) for x, y, _ in errors]
    spatial = [math.sqrt(x*x + y*y + z*z) for x, y, z in errors]
    report = dict(method='first matched pose translation; no rotation or scale fit',
                  odometry_messages=len(odom), truth_messages=len(truth),
                  matched_pairs=len(pairs),
                  max_match_time_delta_ms=max(p[3] for p in pairs)/1e6,
                  horizontal_rmse_m=rms(horizontal),
                  vertical_rmse_m=rms([e[2] for e in errors]),
                  three_d_rmse_m=rms(spatial),
                  final_horizontal_error_m=horizontal[-1],
                  final_three_d_error_m=spatial[-1])
    diagnostics = samples['/gnss_loose/diagnostics']
    if diagnostics:
        from collections import Counter
        report['gnss_diagnostic_status_counts'] = dict(Counter(
            str(int(row[1])) for row in diagnostics if len(row) > 1))
        report['gnss_alignment_reached'] = any(len(row) > 2 and row[2] == 1.0
                                               for row in diagnostics)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
