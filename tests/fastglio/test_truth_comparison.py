from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'adapters/ros1_noetic/fast_glio_uav'))
from truth_comparison import TruthComparison


class TruthComparisonTest(unittest.TestCase):
    def test_first_matched_translation_does_not_hide_later_error(self):
        c=TruthComparison();c.add_truth(1_000_000_000,[-210,240,.05])
        a=c.match(1_000_000_000,[0,0,0]);self.assertEqual(a['truth_aligned'],(0,0,0))
        c.add_truth(2_000_000_000,[-210,250,8.05])
        b=c.match(2_000_000_000,[2,10,10])
        self.assertEqual(b['truth_aligned'],(0,10,8))
        self.assertEqual(b['lio_world'],(-208,250,10.05))
        self.assertEqual(b['error_enu_m'],(2,0,2))
        self.assertFalse(c.report()['truth_used_by_estimator'])

    def test_time_gap_rejection_ties_and_reset(self):
        c=TruthComparison();c.add_truth(100_000_000,[1,2,3]);c.add_truth(120_000_000,[9,9,9])
        self.assertIsNone(c.match(500_000_000,[0,0,0]))
        a=c.match(110_000_000,[0,0,0]);self.assertEqual(a['truth_stamp_ns'],100_000_000)
        self.assertEqual(c.report()['maximum_match_delta_ms'],10.)
        c.add_truth(0,[0,0,0]);self.assertIsNone(c.translation)
        self.assertEqual(c.match(0,[0,0,0])['error_enu_m'],(0,0,0))

    def test_launch_default_and_demo_arg_use_global_scope(self):
        launch=ET.parse(ROOT/'adapters/ros1_noetic/fast_glio_uav/px4_live_loose.launch').getroot()
        self.assertEqual(launch.find("arg[@name='cube_side_length']").get('default'),'1000.0')
        self.assertEqual(launch.find("param[@name='/cube_side_length']").get('value'),'$(arg cube_side_length)')
        self.assertFalse(launch.findall("node/param[@name='cube_side_length']"))


if __name__=='__main__':unittest.main()
