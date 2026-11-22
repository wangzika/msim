#!/usr/bin/env python3
"""Prepare the pinned KSQL world with local assets and check the nominal route."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'adapters/ros1_noetic/fast_glio_uav'))
from scene_profile import load_scene_profile
from ksql_mesh import load_ksql, vertical_hits


def add(parent, name, value):
    ET.SubElement(parent, name).text = str(value)


def validate_route(meshes, profile, route):
    spawn = np.array(profile['spawn_enu_m'], dtype=float)
    heights = vertical_hits(meshes['Terrain'], *spawn[:2])
    if not len(heights):
        raise ValueError('Spawn has no supporting terrain')
    ground = float(heights.max())
    if not .054 < spawn[2]-ground < .5:
        raise ValueError('Spawn must be just above the actual terrain')
    # Use the settled body centre of the Iris box (half-height .055 m).
    flight_z = ground+.054+profile['flight_height_m']
    xy = [spawn[:2]+[p['east_m'], p['north_m']] for p in route]
    region = profile['test_region_enu_m']
    minimum = float('inf'); checked = 0
    for a, b in zip(xy, xy[1:]):
        for t in np.linspace(0, 1, max(2, int(np.linalg.norm(b-a)/4)+1)):
            point = a+(b-a)*t
            for offset in [(0, 0), (-3, 0), (3, 0), (0, -3), (0, 3)]:
                e, n = point+offset
                if not region['east'][0] <= e <= region['east'][1] or not region['north'][0] <= n <= region['north'][1]:
                    raise ValueError('Nominal route is outside the selected 1 km test region')
                terrain = vertical_hits(meshes['Terrain'], e, n)
                if not len(terrain):
                    raise ValueError('Route leaves terrain coverage')
                building = vertical_hits(meshes['Buildings'], e, n)
                top = max(float(terrain.max()), float(building.max()) if len(building) else -np.inf)
                clearance = flight_z-top
                minimum = min(minimum, clearance); checked += 1
                if clearance < 1.5:
                    raise ValueError(f'Route too close to a mesh at E={e:.1f}, N={n:.1f}: {clearance:.2f} m')
    return dict(spawn_ground_z_m=ground, nominal_flight_z_m=flight_z,
                samples_checked=checked, lateral_samples_m=3.,
                minimum_vertical_clearance_m=minimum,
                nominal_path_length_m=float(sum(np.linalg.norm(b-a) for a,b in zip(xy, xy[1:]))),
                caveat='Sampled vertical mesh intersections around the nominal path; not a swept-volume proof or a guarantee under GPS noise/spoofing.')


def prepare(asset_root, output, profile_path):
    receipt_path = asset_root/'asset-manifest.json'
    if not receipt_path.exists():
        raise ValueError('KSQL assets are missing; first run: python3 tools/fetch-ksql-assets.py')
    # Verify against the independently pinned source manifest, not just the receipt.
    spec = importlib.util.spec_from_file_location('ksql_fetch', Path(__file__).with_name('fetch-ksql-assets.py'))
    fetcher = importlib.util.module_from_spec(spec);spec.loader.exec_module(fetcher)
    source = json.loads((ROOT/'scenes/ksql_airport.source.json').read_text())
    fetcher.fetch(source, asset_root, verify_only=True)
    profile = load_scene_profile(profile_path)
    route = json.loads(Path(profile['route_plan']).read_text())
    meshes = load_ksql(asset_root)
    checks = validate_route(meshes, profile, route)
    upstream = asset_root/'upstream/ksql_airport'
    tree = ET.parse(upstream/'worlds/ksql.world')
    world = tree.getroot().find('world');world.set('name', 'ksql_airport')
    for gui in list(world.findall('gui')):
        world.remove(gui)  # The upstream GUI embeds an unrelated video plugin.
    for uri in world.findall('.//mesh/uri'):
        relative = uri.text.removeprefix('file://')
        path = (upstream/relative).resolve()
        if not path.is_file():
            raise ValueError('Mesh resource missing: '+str(path))
        uri.text = path.as_uri()
    physics = world.find('physics')
    physics.find('max_step_size').text = '0.002'
    physics.find('real_time_update_rate').text = '500'
    for tag in ('gravity', 'magnetic_field'):
        node = physics.find(tag)
        if node is not None:
            physics.remove(node);world.append(node)
    # An overview camera has no collision and does not contribute LiDAR points.
    model = ET.SubElement(world, 'model', name='msim_scene_camera')
    add(model, 'static', 'true');add(model, 'pose', '200 500 950 0 1.57079632679 0')
    link = ET.SubElement(model, 'link', name='camera_link')
    sensor = ET.SubElement(link, 'sensor', name='overview', type='camera')
    add(sensor, 'always_on', 'true');add(sensor, 'update_rate', .2)
    camera = ET.SubElement(sensor, 'camera');add(camera, 'horizontal_fov', 1.2)
    image = ET.SubElement(camera, 'image')
    for k,v in [('width',1280),('height',960),('format','R8G8B8')]:add(image,k,v)
    clip = ET.SubElement(camera, 'clip');add(clip,'near',.1);add(clip,'far',4000)
    plugin = ET.SubElement(sensor, 'plugin', name='scene_camera_ros', filename='libgazebo_ros_camera.so')
    for k,v in [('robotNamespace','/msim'),('cameraName','scene_overview'),
                ('imageTopicName','image_raw'),('cameraInfoTopicName','camera_info'),
                ('frameName','scene_camera'),('updateRate',.2)]:add(plugin,k,v)
    ET.indent(tree);output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding='unicode', xml_declaration=True)
    summary = dict(source=source, profile=profile, route_check=checks,
                   terrain_bounds_enu_m=[meshes['Terrain'].min((0,1)).tolist(),meshes['Terrain'].max((0,1)).tolist()],
                   triangles={k:len(v) for k,v in meshes.items()},
                   collision_meshes='Unmodified original terrain and building triangle geometry')
    (asset_root/'scene-validation.json').write_text(json.dumps(summary, indent=2)+'\n')
    print('KSQL world prepared: '+str(output))
    print(json.dumps(checks, indent=2))
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--assets', type=Path, default=ROOT/'build/scenes/ksql-airport')
    ap.add_argument('--output', type=Path, default=ROOT/'scenes/ksql_airport.world')
    ap.add_argument('--profile', type=Path, default=ROOT/'scenes/ksql_airport.profile.json')
    args = ap.parse_args()
    prepare(args.assets.resolve(), args.output.resolve(), args.profile.resolve())


if __name__ == '__main__':
    main()
