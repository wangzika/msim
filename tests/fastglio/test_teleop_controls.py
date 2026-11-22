import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'adapters/ros1_noetic/fast_glio_uav'))
from teleop_controls import KeyInput,body_velocity,trajectory_window
from lio_map_recorder import VoxelMap


class TeleopControlsTest(unittest.TestCase):
    def test_body_directions_follow_yaw_and_up_is_negative_ned(self):
        np.testing.assert_allclose(body_velocity({'w'},0),[2,0,0,0])
        np.testing.assert_allclose(body_velocity({'w'},math.pi/2),[0,2,0,0],atol=1e-12)
        np.testing.assert_allclose(body_velocity({'d'},math.pi/2),[-2,0,0,0],atol=1e-12)
        self.assertLess(body_velocity({'r','q'},0)[2],0)
        self.assertLess(body_velocity({'r','q'},0)[3],0)

    def test_opposite_keys_cancel_and_diagonal_speed_is_bounded(self):
        np.testing.assert_allclose(body_velocity({'w','s','a','d','r','f','q','e'},1),[0,0,0,0])
        command=body_velocity({'w','d'},.6)
        self.assertAlmostEqual(math.hypot(*command[:2]),2)
        with self.assertRaises(ValueError):body_velocity({'w'},0,speed=float('nan'))

    def test_expired_or_cleared_keys_stop_motion(self):
        keys=KeyInput();keys.update(['w'],10)
        self.assertEqual(keys.active(10.2),{'w'})
        self.assertEqual(keys.active(10.36),set())
        keys.update(['d'],11);keys.clear()
        self.assertEqual(keys.active(11.1),set())
        with self.assertRaises(ValueError):keys.update(['invalid'],12)

    def test_duration_leaves_warmup_and_never_exceeds_truth(self):
        start,duration=trajectory_window(2_072_000_000,63_602_000_000)
        self.assertEqual(start,3_200_000_000)
        self.assertEqual(duration,60)
        self.assertLessEqual(start+duration*1_000_000_000,63_602_000_000)
        with self.assertRaises(ValueError):trajectory_window(1_000_000_000,2_000_000_000)

    def test_map_drops_nonfinite_deduplicates_and_caps_memory(self):
        grid=VoxelMap(1.,2)
        grid.add([[.1,.1,.1,1],[.2,.1,.1,2],[1.1,0,0,3],[float('nan'),0,0,0],[3,0,0,4]])
        self.assertEqual(len(grid.points),2)
        self.assertTrue(grid.capped)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'map.pcd';grid.save(path)
            data=path.read_bytes();header,payload=data.split(b'DATA binary\n')
            self.assertIn(b'POINTS 2',header)
            np.testing.assert_allclose(np.frombuffer(payload,dtype='<f4').reshape(-1,4),grid.array())


if __name__=='__main__':unittest.main()
