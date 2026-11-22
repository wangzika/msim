"""Check trail retirement with a renderer that retains deleted material names."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                       'adapters/ros1_noetic/fast_glio_uav'))
try:
    import gazebo_spoof_viz as visualizer
except ModuleNotFoundError as error:
    if error.name not in ('gazebo_msgs', 'geometry_msgs', 'nav_msgs', 'std_msgs', 'rospy'):
        raise
    visualizer = None


@unittest.skipIf(visualizer is None, 'ROS 1 Python packages are required')
class GazeboTrailTest(unittest.TestCase):
    def setUp(self):
        self.active = set()
        self.renderer_materials = set()
        self.deleted = []

        def spawn(name, xml, *_):
            self.assertNotIn(name, self.active)
            for item in ET.fromstring(xml).iter('visual'):
                material = (name, item.attrib['name'])
                self.assertNotIn(material, self.renderer_materials,
                                 'deleted model reused an Ogre material name')
                self.renderer_materials.add(material)
            self.active.add(name)
            return SimpleNamespace(success=True)

        def delete(name):
            self.active.remove(name)
            self.deleted.append(name)
            # Gazebo's client can retain a material after the server deletes it.
            return SimpleNamespace(success=True)

        with patch.dict('os.environ', {'MSIM_PX4_FLIGHT_PATTERN': 'hover'}):
            self.viz = visualizer.GazeboSpoofViz(
                spawn, lambda _: SimpleNamespace(success=True), delete)

    def update(self, second, north):
        stamp = second * 1_000_000_000
        self.viz.latest_truth = (stamp, 0., north, 8.)
        self.viz.latest_detection = dict(
            sim_time_ns=stamp, gps_east_m=1., gps_north_m=north,
            estimate_east_m=0., estimate_north_m=north, detector_status='normal',
            attack_active=True, attack_east_m=second * .18)
        self.assertTrue(self.viz.update())
        self.assertEqual(self.active, self.viz.models)
        self.assertLessEqual(sum('_trail_' in name for name in self.active),
                             4 * visualizer.MAX_TRAIL_POINTS)

    def test_multiple_history_wraps_never_reuse_material_names(self):
        self.viz.route_points = [dict(east_m=0, north_m=0), dict(east_m=0, north_m=50)]
        for second in range(3 * visualizer.MAX_TRAIL_POINTS + 5):
            self.update(second, float(second))
        self.assertGreater(len(self.deleted), 0)
        self.assertLessEqual(len(self.viz.trail_slots), visualizer.MAX_TRAIL_POINTS)

    def test_stationary_updates_retire_old_trails(self):
        for second in range(5):
            self.update(second, float(second))
        for second in range(5, 6 + visualizer.MAX_TRAIL_POINTS):
            self.update(second, 4.)
        self.assertFalse(any('_trail_' in name for name in self.active))


if __name__ == '__main__':
    unittest.main()
