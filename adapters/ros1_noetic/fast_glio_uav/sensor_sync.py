"""Match asynchronous ROS sensor samples to a GNSS simulation epoch.

All timestamps passed here are ROS/Gazebo simulation nanoseconds. A live GNSS
epoch can only use measurements whose sensor timestamp is not in the future.
"""

from collections import deque


class PastSampleBuffer:
    def __init__(self, capacity=128):
        self.samples = deque(maxlen=capacity)

    def add(self, stamp_ns, value):
        if stamp_ns < 0:
            raise ValueError('sensor simulation timestamp must be non-negative')
        self.samples.append((int(stamp_ns), value))

    def match(self, epoch_ns, max_age_ns):
        """Return the newest sample at/before an epoch, with its age and status."""
        if max_age_ns < 0:
            raise ValueError('maximum sample age must be non-negative')
        eligible = (sample for sample in self.samples if sample[0] <= epoch_ns)
        sample = max(eligible, key=lambda item: item[0], default=None)
        if sample is None:
            return dict(status='missing', stamp_ns=None, age_ms=None, value=None)
        age_ns = epoch_ns - sample[0]
        return dict(status='matched' if age_ns <= max_age_ns else 'stale',
                    stamp_ns=sample[0], age_ms=age_ns / 1e6,
                    value=sample[1] if age_ns <= max_age_ns else None)


def summarize_lidar_alignment(records, warmup_ns=1_000_000_000,
                              max_age_ns=200_000_000):
    """Summarize online LiDAR/GNSS timestamp pairing from injection records."""
    if not records:
        raise ValueError('no GNSS injection records')
    origin = records[0]['sim_time_ns']
    rows = [row for row in records if row['sim_time_ns'] - origin >= warmup_ns]
    if not rows:
        rows = records
    matched = []
    ages = []
    for row in rows:
        scan_ns = row.get('lidar_stamp_ns')
        if row.get('lidar_sync_status') != 'matched' or scan_ns is None:
            continue
        age_ns = row['sim_time_ns'] - scan_ns
        if (0 <= age_ns <= max_age_ns and row.get('lidar_point_count', 0) > 0
                and row.get('lidar_age_ms') == age_ns / 1e6):
            matched.append(row)
            ages.append(age_ns / 1e6)
    return dict(lidar_epoch_count=len(rows), lidar_matched_epochs=len(matched),
                lidar_sync_fraction=len(matched) / len(rows),
                lidar_max_age_ms=max(ages, default=None),
                lidar_mean_age_ms=sum(ages) / len(ages) if ages else None)
