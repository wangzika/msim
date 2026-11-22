import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
ADAPTER = ROOT/'adapters/ros1_noetic/fast_glio_uav'
sys.path.insert(0, str(ADAPTER))
from scene_profile import load_scene_profile

spec = importlib.util.spec_from_file_location('scene_assets', ADAPTER/'prepare_px4_assets.py')
assets = importlib.util.module_from_spec(spec);spec.loader.exec_module(assets)


class SceneProfileTest(unittest.TestCase):
    def test_legacy_origin_and_spawn_remain_available(self):
        p = load_scene_profile()
        self.assertEqual(p['origin_lla_deg_m'], [30.5312, 114.3601, 50.])
        self.assertEqual(p['spawn_enu_m'], [0., -20., 0.])
        self.assertEqual(p['flight_height_m'], 8.)

    def test_reject_invalid_origins_and_heights(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)/'scene.json'
            for bad in [{'origin_lla_deg_m':[95,0,0]}, {'flight_height_m':-1},
                        {'spawn_enu_m':[0,float('nan'),0]}]:
                f.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):load_scene_profile(f)

    def test_world_uses_selected_origin_and_one_camera(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp);source=tmp/'source.world';target=tmp/'target.world'
            source.write_text('<sdf version="1.6"><world name="test"><gui><camera name="old"/></gui>'
                              '<spherical_coordinates><latitude_deg>0</latitude_deg></spherical_coordinates></world></sdf>')
            profile=tmp/'profile.json'
            profile.write_text(json.dumps(dict(origin_lla_deg_m=[37.52364,-122.255122,1.7],
                                                camera_pose=[-200,200,50,0,.5,.9],route_plan='route.json')))
            with patch.dict('os.environ', {'MSIM_SCENE_PROFILE':str(profile)}):
                assets.prepare_world(source,target,True)
            world=ET.parse(target).getroot().find('world')
            self.assertEqual(len(world.findall('spherical_coordinates')),1)
            self.assertEqual(world.findtext('spherical_coordinates/longitude_deg'),'-122.255122')
            self.assertEqual(len(world.findall('gui/camera')),1)
            self.assertEqual(world.findtext('gui/camera/pose'),'-200 200 50 0 0.5 0.9')
            self.assertEqual(load_scene_profile(profile)['route_plan'],str(tmp/'route.json'))


if __name__=='__main__':unittest.main()
