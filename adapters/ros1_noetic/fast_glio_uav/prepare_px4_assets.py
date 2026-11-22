#!/usr/bin/env python3
"""Add ROS 1 measurement outputs to PX4 Iris without modifying PX4 sources."""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'msim_sensors'))
from sensor_profiles import resolve
from scene_profile import from_environment


DEFAULT_SENSOR_PROFILE = (Path(__file__).resolve().parents[1] / 'msim_sensors' /
                          'config' / 'sensor_profile.json')


def add_xml(parent, fragment):
    parent.append(ET.fromstring(fragment))


def load_sensor_profile(path):
    original = json.loads(Path(path).read_text())
    if original.get('schema_version') not in (1, 2):
        raise ValueError('unsupported sensor profile schema')
    profile = resolve(path)
    imu = profile['imu_ros']
    px4_imu = profile['imu_px4']
    lidar = profile['lidar']
    expected_px4 = {
        'gyroscopeNoiseDensity', 'gyroscopeRandomWalk',
        'gyroscopeBiasCorrelationTime', 'gyroscopeTurnOnBiasSigma',
        'accelerometerNoiseDensity', 'accelerometerRandomWalk',
        'accelerometerBiasCorrelationTime', 'accelerometerTurnOnBiasSigma',
    }
    if set(px4_imu) != expected_px4:
        raise ValueError('PX4 IMU profile must specify all eight error parameters')
    for section in (imu, px4_imu):
        for name, value in section.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be a finite number')
    if not isinstance(imu['seed'], int):
        raise ValueError('IMU seed must be an integer')
    if any(value < 0 for name, value in imu.items() if name != 'seed'):
        raise ValueError('IMU error parameters must be nonnegative')
    if any(value < 0 for value in px4_imu.values()):
        raise ValueError('PX4 IMU error parameters must be nonnegative')
    if imu['rate_hz'] <= 0 or lidar['rate_hz'] <= 0:
        raise ValueError('sensor rates must be positive')
    if any(value <= 0 for name, value in px4_imu.items() if name.endswith('CorrelationTime')):
        raise ValueError('PX4 IMU bias correlation times must be positive')
    if any(value <= 0 for name, value in imu.items() if name.endswith('_correlation_time_s')):
        raise ValueError('ROS IMU bias correlation times must be positive')
    if (not isinstance(lidar['horizontal_samples'], int) or
            not isinstance(lidar['vertical_samples'], int) or
            lidar['horizontal_samples'] < 2 or not 2 <= lidar['vertical_samples'] <= 128):
        raise ValueError('LiDAR profile has invalid beam counts')
    if not 0 < lidar['range_min_m'] < lidar['range_max_m']:
        raise ValueError('invalid LiDAR range')
    if lidar['range_noise_sigma_m'] < 0 or lidar['range_resolution_m'] <= 0:
        raise ValueError('invalid LiDAR noise or resolution')
    return profile


def prepare_model(source, target, online_gnss=False, lidar_pitch_deg=0.0,
                  lidar_roll_deg=0.0, sensor_profile=None):
    profile = sensor_profile or load_sensor_profile(DEFAULT_SENSOR_PROFILE)
    imu = profile['imu_ros']
    lidar = profile['lidar']
    tree = ET.parse(source)
    model = tree.getroot().find('model')
    if model is None:
        raise ValueError('PX4 Iris SDF has no model')
    if online_gnss:
        gps = model.find("include[name='gps0']")
        joint = model.find("joint[@name='gps0_joint']")
        if gps is None or joint is None:
            raise ValueError('PX4 Iris GPS plugin layout changed; refusing dual GPS inputs')
        # Keep the native sensor and its noise/delay model, but isolate its
        # Transport output so MAVLink can receive only the relay's final sample.
        gps_tree = ET.parse(source.parent.parent / 'gps/gps.sdf')
        native = gps_tree.getroot().find('model')
        native.set('name', 'gps0')
        pose = gps.findtext('pose', '0.05 0 0.04 0 0 0')
        ET.SubElement(native, 'pose').text = pose
        plugin = native.find('.//plugin')
        ET.SubElement(plugin, 'topic').text = 'msim_gps_raw'
        plugin.find('gpsNoise').text = str(profile['gps']['native_noise']).lower()
        model.remove(gps)
        model.append(native)
        relay = ET.SubElement(model, 'plugin', name='msim_gps_relay', filename='libmsim_gps_relay.so')
        for key, value in profile['gps'].items():
            ET.SubElement(relay, key).text = str(value).lower() if isinstance(value, bool) else str(value)
    link = model.find("link[@name='base_link']")
    if link is None:
        raise ValueError('PX4 Iris SDF has no base_link')
    add_xml(link, f'''<sensor name="msim_imu" type="imu">
      <always_on>true</always_on><update_rate>{imu['rate_hz']}</update_rate>
      <plugin name="msim_ros_imu" filename="libgazebo_ros_imu_sensor.so">
        <topicName>/imu/ideal</topicName><bodyName>base_link</bodyName>
        <updateRateHZ>{imu['rate_hz']}</updateRateHZ><gaussianNoise>0</gaussianNoise>
        <frameName>base_link</frameName><initialOrientationAsReference>false</initialOrientationAsReference>
      </plugin>
    </sensor>''')
    px4_imu = model.find("plugin[@name='rotors_gazebo_imu_plugin']")
    if px4_imu is None:
        raise ValueError('PX4 Iris IMU plugin missing')
    for name, value in profile['imu_px4'].items():
        old = px4_imu.find(name)
        if old is not None:
            px4_imu.remove(old)
        ET.SubElement(px4_imu, name).text = str(value)
    lidar_pitch_rad = math.radians(lidar_pitch_deg)
    lidar_roll_rad = math.radians(lidar_roll_deg)
    scanner = ET.SubElement(model, 'plugin', name='msim_scanning_lidar', filename='libmsim_scanning_lidar.so')
    ET.SubElement(scanner, 'pose').text = f'0 0 0.15 {lidar_roll_rad:.12f} {lidar_pitch_rad:.12f} 0'
    for key, value in lidar.items():
        ET.SubElement(scanner, key).text = str(value)
    add_xml(scanner, f'''<ray><scan><horizontal><samples>1</samples><resolution>1</resolution>
      <min_angle>0</min_angle><max_angle>0</max_angle></horizontal></scan>
      <range><min>{lidar['range_min_m']}</min><max>{lidar['range_max_m']}</max>
      <resolution>{lidar['range_resolution_m']}</resolution></range></ray>''')
    add_xml(model, '''<plugin name="msim_truth" filename="libgazebo_ros_p3d.so">
      <alwaysOn>true</alwaysOn><updateRate>100</updateRate><bodyName>base_link</bodyName>
      <topicName>/ground_truth/state</topicName><gaussianNoise>0</gaussianNoise>
      <frameName>world</frameName><xyzOffsets>0 0 0</xyzOffsets><rpyOffsets>0 0 0</rpyOffsets>
    </plugin>''')
    ET.indent(tree)
    target.parent.mkdir(parents=True, exist_ok=True)
    tree.write(target, encoding='unicode', xml_declaration=True)


def prepare_world(source, target, gazebo_viz=False):
    scene = from_environment()
    origin = scene['origin_lla_deg_m']
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('urban world SDF has no world')
    old = world.find('spherical_coordinates')
    if old is not None:
        world.remove(old)
    add_xml(world, f'''<spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{origin[0]}</latitude_deg><longitude_deg>{origin[1]}</longitude_deg>
      <elevation>{origin[2]}</elevation><heading_deg>0</heading_deg>
    </spherical_coordinates>''')
    if gazebo_viz:
        gui = world.find('gui')
        if gui is None:
            gui = ET.SubElement(world, 'gui', fullscreen='0')
        for camera in list(gui.findall('camera')):
            gui.remove(camera)
        camera_pose = ' '.join(map(str, scene['camera_pose']))
        add_xml(gui, f'''<camera name="user_camera">
          <pose>{camera_pose}</pose>
          <view_controller>orbit</view_controller>
          <projection_type>perspective</projection_type>
        </camera>''')
    ET.indent(tree)
    target.parent.mkdir(parents=True, exist_ok=True)
    tree.write(target, encoding='unicode', xml_declaration=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--px4-root', type=Path, required=True)
    ap.add_argument('--fast-glio-root', type=Path, required=True)
    ap.add_argument('--output-dir', type=Path, required=True)
    ap.add_argument('--sensor-profile', type=Path, default=DEFAULT_SENSOR_PROFILE)
    ap.add_argument('--world-input', type=Path,
                    help='use a local Gazebo world instead of the default urban canyon')
    ap.add_argument('--online-gnss', action='store_true',
                    help='route native noisy Gazebo GPS through the attack relay')
    ap.add_argument('--gazebo-viz', action='store_true',
                    help='set a camera view of the flight corridor')
    ap.add_argument('--lidar-pitch-deg', type=float, default=0.0,
                    help='LiDAR pitch relative to base_link; positive points +X rays down')
    ap.add_argument('--lidar-roll-deg', type=float, default=0.0,
                    help='LiDAR roll relative to base_link; negative points +Y rays down')
    args = ap.parse_args()
    if not -30.0 <= args.lidar_pitch_deg <= 30.0:
        ap.error('--lidar-pitch-deg must be between -30 and +30 degrees')
    if not -30.0 <= args.lidar_roll_deg <= 30.0:
        ap.error('--lidar-roll-deg must be between -30 and +30 degrees')
    model = args.px4_root / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf'
    world = args.world_input or args.fast_glio_root / 'simulation/uav_simulator/worlds/urban_canyon.world'
    if not model.is_file():
        ap.error(f'PX4 Iris SDF missing: {model}; initialize the Gazebo Classic submodule')
    if not world.is_file():
        ap.error(f'FAST_GLIO world missing: {world}')
    profile = load_sensor_profile(args.sensor_profile)
    model_name = 'iris_msim_online_gnss.sdf' if args.online_gnss else 'iris_msim.sdf'
    prepare_model(model, args.output_dir / model_name, args.online_gnss,
                  args.lidar_pitch_deg, args.lidar_roll_deg, profile)
    prepare_world(world, args.output_dir / 'urban_canyon_msim.world', args.gazebo_viz)


if __name__ == '__main__':
    main()
