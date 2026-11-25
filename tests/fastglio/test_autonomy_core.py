import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'adapters/ros1_noetic/fast_glio_uav'))
from autonomy_core import FLU_FRD,MapLocalizer,bounded_target,descriptor,icp,segment_clear,transform,voxel,param_int_wire,param_int_value,fresh_stamp


def scene():
    random=np.random.default_rng(14)
    return np.concatenate([random.normal(size=(220,3))*[1,.3,2]+center for center in [[4,2,1],[-3,7,4],[8,-4,2],[-7,-5,3]]])


def bundle(path,points,positions,descriptors):
    np.savez_compressed(path/'map.npz',points=points,positions=positions,descriptors=descriptors)
    (path/'map.json').write_text(json.dumps(dict(sha256=hashlib.sha256((path/'map.npz').read_bytes()).hexdigest(),home_position=[0,0,0])))


class AutonomyCoreTest(unittest.TestCase):
    def test_clock_delivery_skew_does_not_hide_stale_or_future_odometry(self):
        self.assertTrue(fresh_stamp(10.,10.02))
        self.assertTrue(fresh_stamp(10.,10.036))
        self.assertTrue(fresh_stamp(10.,9.91))
        self.assertFalse(fresh_stamp(10.,9.4))
        self.assertFalse(fresh_stamp(10.,10.2))

    def test_px4_integer_parameters_use_bytewise_transport(self):
        for value in [0,1,3,15,-1]:self.assertEqual(param_int_value(param_int_wire(value)),value)
        self.assertNotEqual(param_int_value(3.),3)

    def test_icp_recovers_six_dof_from_a_bounded_initial_error(self):
        points=scene();matrix=np.eye(4);matrix[:3,:3]=Rotation.from_euler('xyz',[.04,-.03,.15]).as_matrix();matrix[:3,3]=[.4,-.3,.2]
        query=transform(points,np.linalg.inv(matrix));actual,quality=icp(query,cKDTree(points),np.eye(4))
        np.testing.assert_allclose(actual,matrix,atol=.015);self.assertGreater(quality['overlap'],.99)

    def test_global_candidate_search_and_multiframe_confirmation(self):
        points=scene();matrix=np.eye(4);matrix[:3,:3]=Rotation.from_euler('z',np.pi/3).as_matrix();matrix[:3,3]=[4,2,1]
        query=transform(points,np.linalg.inv(matrix))
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);bundle(p,points,np.array([[4,2,1]]),np.array([descriptor(points-[4,2,1])]))
            locator=MapLocalizer(p);first=locator.locate(query,np.zeros(3));second=locator.locate(query,np.zeros(3))
            self.assertFalse(first['accepted']);self.assertTrue(second['accepted'],second)
            np.testing.assert_allclose(second['transform'],matrix,atol=.05)

    def test_repeated_geometry_is_rejected_instead_of_flying_to_wrong_building(self):
        points=scene();shift=np.array([35,0,0])
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);bundle(p,np.concatenate([points,points+shift]),np.array([[0,0,0],shift]),np.array([descriptor(points)]*2))
            result=MapLocalizer(p).locate(points,np.zeros(3))
            self.assertFalse(result['accepted']);self.assertEqual(result['reason'],'ambiguous_candidates')

    def test_mismatched_map_has_no_accepted_registration(self):
        points=scene()
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);bundle(p,points,np.zeros((1,3)),np.array([descriptor(points)]))
            result=MapLocalizer(p).locate(points+10000,np.zeros(3))
            self.assertFalse(result['accepted'])
            (p/'map.npz').write_bytes(b'corrupt')
            with self.assertRaises(ValueError):MapLocalizer(p)

    def test_target_is_bounded_and_obstacles_or_unknown_cloud_stop_motion(self):
        np.testing.assert_allclose(bounded_target([0,0,0],[10,0,0],1.2),[1.2,0,0])
        clear=np.tile([3,4,5],(120,1));self.assertTrue(segment_clear(clear,[0,0,0],[1,0,0]))
        self.assertFalse(segment_clear(np.r_[clear,[[.5,.1,0]]],[0,0,0],[1,0,0]))
        self.assertFalse(segment_clear(clear[:20],[0,0,0],[1,0,0]))

    def test_enu_body_frames_roundtrip_without_reflection(self):
        attitude=Rotation.from_euler('xyz',[.01,-.02,.5]).as_matrix();lio=Rotation.from_euler('xyz',[.02,.01,-.3]).as_matrix()
        rotation=attitude@FLU_FRD@lio.T
        np.testing.assert_allclose(rotation@lio@FLU_FRD,attitude,atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(rotation),1.)


if __name__=='__main__':unittest.main()
