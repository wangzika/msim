#!/usr/bin/env python3
"""Check recorded KSQL LiDAR returns against independently loaded triangles.

Uses recorded truth only for evaluation, never for the localization algorithm.
Run inside the project's ROS 1 Python environment.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import rosbag
from scipy.spatial.transform import Rotation, Slerp

from ksql_mesh import load_ksql

ROOT = Path(__file__).resolve().parents[1]


def cloud_array(msg):
    fields = {f.name: f.offset for f in msg.fields}
    keys = ['x', 'y', 'z', 'time']
    dtype = np.dtype(dict(names=keys, formats=['<f4']*4,
                         offsets=[fields[k] for k in keys], itemsize=msg.point_step))
    if msg.is_bigendian or msg.height != 1:
        raise ValueError('Expected the archived unorganized little-endian msim cloud')
    return np.frombuffer(msg.data, dtype=dtype)


def ray_hit(triangles, origin, direction):
    """Nearest two-sided positive ray intersection: distance and triangle index."""
    a = triangles[:, 0]
    ab, ac = triangles[:, 1]-a, triangles[:, 2]-a
    p = np.cross(direction, ac)
    det = np.einsum('ij,ij->i', ab, p)
    valid = abs(det) > 1e-10
    inverse = np.divide(1., det, out=np.zeros_like(det), where=valid)
    delta = origin-a
    u = np.einsum('ij,ij->i', delta, p)*inverse
    q = np.cross(delta, ab)
    v = q @ direction*inverse
    distance = np.einsum('ij,ij->i', ac, q)*inverse
    valid &= (u >= -1e-7) & (v >= -1e-7) & (u+v <= 1+1e-7) & (distance > .3)
    candidates = np.where(valid, distance, np.inf)
    index = int(np.argmin(candidates))
    return float(candidates[index]), index


def audit(run, assets, truth_offset_s=None):
    automatic_phase = truth_offset_s is None
    if automatic_phase:
        # p3d records at WorldUpdateBegin; the scanner casts at WorldUpdateEnd.
        # Both carry the same SimTime, but physics advances between callbacks.
        # Thus end(t) is compared with recorded begin(t + one physics step).
        truth_offset_s = float(ET.parse(run/'world.sdf').findtext('.//physics/max_step_size'))
    truth = []; clouds = []; headers = []; counts = []; spans = []
    with rosbag.Bag(str(run/'sensors.bag')) as bag:
        total = bag.get_message_count('/velodyne_points')
        selected = set(np.linspace(2, total-3, min(16, total-4), dtype=int))
        for topic, msg, _ in bag.read_messages(topics=['/ground_truth/state', '/velodyne_points']):
            if topic == '/ground_truth/state':
                p = msg.pose.pose.position; q = msg.pose.pose.orientation
                truth.append([msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            else:
                index = len(headers); data = cloud_array(msg)
                headers.append(msg.header.stamp.to_sec()); counts.append(len(data))
                spans.append(float(np.ptp(data['time'])) if len(data) else 0.)
                if index in selected and len(data):
                    xyz = np.column_stack([data[k] for k in ('x', 'y', 'z')])
                    at = np.linspace(0, len(data)-1, min(96, len(data)), dtype=int)
                    clouds.append((msg.header.stamp.to_sec()+data['time'][at].astype(float), xyz[at]))
    truth = np.array(truth)
    if len(truth) < 2 or not clouds:
        raise ValueError('Missing recorded truth or clouds')
    truth = truth[np.r_[True, np.diff(truth[:, 0]) > 0]]
    rotation = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:8]))
    model = ET.parse(run/'vehicle.sdf')
    mount = np.fromstring(model.findtext(".//plugin[@name='msim_scanning_lidar']/pose"), sep=' ')
    mount_rotation = Rotation.from_euler('xyz', mount[3:])
    meshes = load_ksql(assets)
    triangles = np.concatenate([meshes['Terrain'], meshes['Buildings']])
    labels = np.r_[np.zeros(len(meshes['Terrain']), dtype=int), np.ones(len(meshes['Buildings']), dtype=int)]
    lower, upper = triangles.min(1), triangles.max(1)
    residual = []; hit_counts = [0, 0]; no_hit = 0; skipped = 0
    for times, xyz in clouds:
        times = times+truth_offset_s
        valid = (times >= truth[0, 0]) & (times <= truth[-1, 0])
        skipped += int((~valid).sum()); times, xyz = times[valid], xyz[valid]
        if not len(times):
            continue
        r = rotation(times)
        position = np.column_stack([np.interp(times, truth[:, 0], truth[:, k]) for k in range(1, 4)])
        origin = position+r.apply(np.tile(mount[:3], (len(times), 1)))
        measured = np.linalg.norm(xyz, axis=1)
        direction = r.apply(mount_rotation.apply(xyz/measured[:, None]))
        # Broad phase includes every triangle within the sensor's maximum range.
        mask = np.all(upper >= origin.min(0)-101., axis=1) & np.all(lower <= origin.max(0)+101., axis=1)
        local, local_labels = triangles[mask], labels[mask]
        for o, d, observed in zip(origin, direction, measured):
            expected, index = ray_hit(local, o, d)
            if not np.isfinite(expected) or expected > 100.1:
                no_hit += 1
            else:
                residual.append(float(observed-expected)); hit_counts[local_labels[index]] += 1
    residual = np.array(residual)
    if not len(residual):
        raise ValueError('No independently matched mesh returns')
    profile = json.loads((run/'scene-profile.json').read_text())
    in_region = np.all((truth[:, 1:3] >= [profile['test_region_enu_m']['east'][0], profile['test_region_enu_m']['north'][0]]) &
                       (truth[:, 1:3] <= [profile['test_region_enu_m']['east'][1], profile['test_region_enu_m']['north'][1]]), axis=1)
    report = dict(method='Interpolate recorded body truth at each sampled point time; apply archived sensor mounting; independently ray-cast original COLLADA triangles.',
                  truth_query_offset_s=truth_offset_s,
                  phase_basis='Archived physics step: p3d WorldUpdateBegin versus scanning_lidar WorldUpdateEnd' if automatic_phase else 'Explicit diagnostic offset',
                  clouds=len(headers), points_per_cloud_min_median_max=[int(min(counts)), float(np.median(counts)), int(max(counts))],
                  median_frame_interval_s=float(np.median(np.diff(headers))),
                  point_time_span_min_max_s=[min(spans), max(spans)],
                  truth_bounds_enu_m=[truth[:, 1:4].min(0).tolist(), truth[:, 1:4].max(0).tolist()],
                  truth_path_length_3d_m=float(np.linalg.norm(np.diff(truth[:, 1:4], axis=0), axis=1).sum()),
                  truth_max_displacement_m=float(np.linalg.norm(truth[:, 1:4]-truth[0, 1:4], axis=1).max()),
                  truth_final_displacement_enu_m=(truth[-1, 1:4]-truth[0, 1:4]).tolist(),
                  all_truth_inside_selected_region=bool(in_region.all()),
                  sampled_frames=len(clouds), matched_rays=len(residual), unmatched_rays=no_hit,
                  skipped_points_outside_truth_time=skipped,
                  terrain_hits=hit_counts[0], building_hits=hit_counts[1],
                  range_residual_mean_m=float(residual.mean()), range_residual_std_m=float(residual.std()),
                  range_residual_abs_median_m=float(np.median(abs(residual))),
                  range_residual_abs_p95_m=float(np.percentile(abs(residual), 95)),
                  fraction_range_residual_below_10cm=float(np.mean(abs(residual) < .1)),
                  caveat='A sparse recorded-ray check, not a proof for all rays. Edge rays can change triangle with truth interpolation; no GNSS multipath or occlusion model is added.')
    # Allow a small number of edge-ray differences, but reject wrong scale/frame.
    report['pass_recorded_mesh_check'] = bool(report['fraction_range_residual_below_10cm'] > .95 and
                                             no_hit/(len(residual)+no_hit) < .05 and
                                             report['building_hits'] > 0 and in_region.all())
    suffix = '' if automatic_phase else f'-offset-{truth_offset_s:+.3f}s'
    (run/f'ksql-sensor-audit{suffix}.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', type=Path, required=True)
    ap.add_argument('--assets', type=Path, default=ROOT/'build/scenes/ksql-airport')
    ap.add_argument('--truth-offset-s', type=float, help='Diagnostic override; default accounts for Begin/End callback phase using the archived physics step. Never rewrites the bag.')
    args = ap.parse_args()
    result = audit(args.run.resolve(), args.assets.resolve(), args.truth_offset_s)
    raise SystemExit(0 if result['pass_recorded_mesh_check'] else 1)
