#!/usr/bin/env python3
"""Add ROS 1 measurement outputs to PX4 Iris without modifying PX4 sources."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


ORIGIN = (30.5312, 114.3601, 50.0)


def add_xml(parent, fragment):
    parent.append(ET.fromstring(fragment))


def prepare_model(source, target, online_gnss=False):
    tree = ET.parse(source)
    model = tree.getroot().find('model')
    if model is None:
        raise ValueError('PX4 Iris SDF has no model')
    if online_gnss:
        gps = model.find("include[name='gps0']")
        joint = model.find("joint[@name='gps0_joint']")
        if gps is None or joint is None:
            raise ValueError('PX4 Iris GPS plugin layout changed; refusing dual GPS inputs')
        model.remove(gps)
        model.remove(joint)
    link = model.find("link[@name='base_link']")
    if link is None:
        raise ValueError('PX4 Iris SDF has no base_link')
    add_xml(link, '''<sensor name="msim_imu" type="imu">
      <always_on>true</always_on><update_rate>200</update_rate>
      <plugin name="msim_ros_imu" filename="libgazebo_ros_imu_sensor.so">
        <topicName>/imu/data</topicName><bodyName>base_link</bodyName>
        <updateRateHZ>200</updateRateHZ><gaussianNoise>0.002</gaussianNoise>
        <frameName>base_link</frameName><initialOrientationAsReference>false</initialOrientationAsReference>
      </plugin>
    </sensor>''')
    add_xml(link, '''<sensor name="msim_lidar" type="ray">
      <pose>0 0 0.15 0 0 0</pose><always_on>true</always_on><update_rate>10</update_rate>
      <ray><scan>
        <horizontal><samples>800</samples><resolution>1</resolution><min_angle>-3.14159265</min_angle><max_angle>3.14159265</max_angle></horizontal>
        <vertical><samples>16</samples><resolution>1</resolution><min_angle>-0.26179938779</min_angle><max_angle>0.26179938779</max_angle></vertical>
      </scan><range><min>0.3</min><max>100</max><resolution>0.001</resolution></range>
      <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise></ray>
      <plugin name="msim_ros_lidar" filename="libgazebo_ros_block_laser.so">
        <topicName>/velodyne_points_raw</topicName><frameName>velodyne</frameName>
        <updateRate>10</updateRate><gaussianNoise>0.01</gaussianNoise>
      </plugin>
    </sensor>''')
    add_xml(model, '''<plugin name="msim_truth" filename="libgazebo_ros_p3d.so">
      <alwaysOn>true</alwaysOn><updateRate>100</updateRate><bodyName>base_link</bodyName>
      <topicName>/ground_truth/state</topicName><gaussianNoise>0</gaussianNoise>
      <frameName>world</frameName><xyzOffsets>0 0 0</xyzOffsets><rpyOffsets>0 0 0</rpyOffsets>
    </plugin>''')
    ET.indent(tree)
    target.parent.mkdir(parents=True, exist_ok=True)
    tree.write(target, encoding='unicode', xml_declaration=True)


def prepare_world(source, target):
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('urban world SDF has no world')
    old = world.find('spherical_coordinates')
    if old is not None:
        world.remove(old)
    add_xml(world, f'''<spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{ORIGIN[0]}</latitude_deg><longitude_deg>{ORIGIN[1]}</longitude_deg>
      <elevation>{ORIGIN[2]}</elevation><heading_deg>0</heading_deg>
    </spherical_coordinates>''')
    ET.indent(tree)
    target.parent.mkdir(parents=True, exist_ok=True)
    tree.write(target, encoding='unicode', xml_declaration=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--px4-root', type=Path, required=True)
    ap.add_argument('--fast-glio-root', type=Path, required=True)
    ap.add_argument('--output-dir', type=Path, required=True)
    ap.add_argument('--online-gnss', action='store_true',
                    help='remove Gazebo GPS so the MAVLink injector is the only GPS source')
    args = ap.parse_args()
    model = args.px4_root / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf'
    world = args.fast_glio_root / 'simulation/uav_simulator/worlds/urban_canyon.world'
    if not model.is_file():
        ap.error(f'PX4 Iris SDF missing: {model}; initialize the Gazebo Classic submodule')
    if not world.is_file():
        ap.error(f'FAST_GLIO world missing: {world}')
    model_name = 'iris_msim_online_gnss.sdf' if args.online_gnss else 'iris_msim.sdf'
    prepare_model(model, args.output_dir / model_name, args.online_gnss)
    prepare_world(world, args.output_dir / 'urban_canyon_msim.world')


if __name__ == '__main__':
    main()
