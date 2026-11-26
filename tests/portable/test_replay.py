import csv
import json
import tempfile
from pathlib import Path
import unittest
import numpy as np

from msim.clock import ReplayClock
from msim.schema import Event
from msim.recording import Recording,RecordingWriter,safe_payload
from msim.demo import generate
from msim.replay import replay


def imu(t,arrival=None):
    return Event(t,t if arrival is None else arrival,'imu','imu','body_flu',dict(acceleration_mps2=[0,0,9.80665],angular_velocity_radps=[0,0,0],orientation_xyzw=[0,0,0,1]))


def pose(t,stream='truth',position=None,frame='enu',arrival=None):
    return Event(t,t if arrival is None else arrival,'pose',stream,frame,dict(position_m=position or [0,0,0],orientation_xyzw=[0,0,0,1],point='body'))


def gnss(t,arrival=None):
    return Event(t,t if arrival is None else arrival,'gnss','gnss','enu',dict(position_m=[0,0,0],point='antenna'))


class ReplayTest(unittest.TestCase):
    def test_fast_clock_does_not_sleep_and_paced_clock_uses_availability(self):
        wall=[100.];sleeps=[]
        def sleep(t):sleeps.append(t);wall[0]+=t
        c=ReplayClock(2.,sleep,lambda:wall[0]);c.advance(5_000_000_000);c.advance(7_000_000_000)
        self.assertEqual(sleeps,[1.]);self.assertEqual(c.now_ns,7_000_000_000)
        c.advance(7_000_000_000)
        with self.assertRaises(ValueError):c.advance(0)
        c=ReplayClock(0.,lambda _:self.fail('fast clock slept'));c.advance(0);c.advance(1_000_000_000)
        with self.assertRaises(ValueError):ReplayClock(float('nan'))

    def test_recording_preserves_large_integer_stamps_and_delayed_arrival(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'recording';t=1_795_000_000_000_000_001
            with RecordingWriter(p,{}) as w:w.write(gnss(t,t+200_000_000))
            actual=list(Recording(p).events())[0]
            self.assertEqual(actual.stamp_ns,t);self.assertEqual(actual.arrival_ns-t,200_000_000)
            with self.assertRaises(FileExistsError):RecordingWriter(p,{})

    def test_invalid_events_and_incomplete_recordings_are_rejected(self):
        with self.assertRaises(ValueError):Event(True,0,'imu','imu','body_flu',{}).validate()
        with self.assertRaises(ValueError):pose(0,position=[float('nan'),0,0]).validate()
        with self.assertRaises(ValueError):Event.from_row({'schema_version':2})
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'broken'
            with self.assertRaises(ValueError):
                with RecordingWriter(p,{}) as w:
                    w.write(imu(10));w.write(imu(9))
            with self.assertRaises(ValueError):Recording(p)

    def test_cloud_payload_keeps_rings_and_nonuniform_point_offsets(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'cloud'
            arrays=dict(x=np.array([1.,2.],dtype='f4'),y=np.array([0.,1.],dtype='f4'),z=np.array([3.,4.],dtype='f4'),ring=np.array([1,15],dtype='u2'),time=np.array([0.,.098],dtype='f4'))
            with RecordingWriter(p,{}) as w:w.cloud(100,500,'lidar','lidar',arrays)
            r=Recording(p);event=list(r.events())[0];actual=r.cloud(event)
            for key in arrays:np.testing.assert_array_equal(actual[key],arrays[key])
            with self.assertRaises(ValueError):safe_payload(p,'../outside.npz')
            with self.assertRaises(ValueError):safe_payload(p,'/outside.npz')
            with self.assertRaises(ValueError):safe_payload(p,'C:/outside.npz')
            with self.assertRaises(ValueError):safe_payload(p,'clouds\\outside.npz')

    def test_truth_and_attack_labels_cannot_change_detector_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);generate(root/'a',duration_s=6)
            original=Recording(root/'a')
            with RecordingWriter(root/'b',dict(labels={'attack_start_ns':None})) as w:
                for event in original.events():
                    if event.stream=='truth':event=pose(event.stamp_ns,position=[500,900,20],arrival=event.arrival_ns)
                    w.write(event)
            replay(root/'a',root/'ra');replay(root/'b',root/'rb')
            self.assertEqual((root/'ra/detector.jsonl').read_bytes(),(root/'rb/detector.jsonl').read_bytes())
            a=json.loads((root/'ra/summary.json').read_text());b=json.loads((root/'rb/summary.json').read_text())
            self.assertGreater(b['evaluation']['position']['lio']['rmse_3d_m'],a['evaluation']['position']['lio']['rmse_3d_m'])

    def test_deterministic_demo_detects_step_and_does_not_false_alarm_normal_fixture(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for name,attack in [('attack',True),('normal',False)]:
                generate(root/name,attack=attack)
                result=replay(root/name,root/(name+'-result'))
                self.assertEqual(result['detector']['false_alarm_fraction'],0.)
                if attack:self.assertEqual(result['detector']['first_alarm_stamp_ns'],12_000_000_000)
                else:self.assertEqual(result['detector']['alarm_count'],0)
                html=(root/(name+'-result')/'report.html').read_text()
                self.assertNotIn('__DATA__',html);self.assertNotIn('<script src=',html)
                self.assertEqual(result['evaluation']['position']['lio']['matched_pairs'],241)

    def test_delayed_gnss_uses_past_measurement_time_imu_never_future_imu(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            with RecordingWriter(root/'r',{}) as w:
                w.write(imu(100_000_000));w.write(pose(100_000_000))
                w.write(imu(300_000_000));w.write(gnss(100_000_000,300_000_000))
                w.write(gnss(300_000_001,300_000_001))
            result=replay(root/'r',root/'out')
            rows=[json.loads(line) for line in (root/'out/detector.jsonl').read_text().splitlines()]
            self.assertEqual(rows[0]['imu_stamp_ns'],100_000_000)
            self.assertEqual(rows[0]['arrival_ns'],300_000_000)
            self.assertEqual(result['detector']['valid_updates'],2)
            # A GNSS measurement with only a future IMU is explicitly unavailable.
            with RecordingWriter(root/'future',{}) as w:w.write(imu(101));w.write(gnss(100,200))
            result=replay(root/'future',root/'future-out')
            self.assertEqual(result['detector']['valid_updates'],0)
            self.assertEqual(result['detector']['skipped']['no_past_imu_within_50ms'],1)

    def test_cross_frame_error_is_skipped_until_axes_assumption_is_explicit(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            with RecordingWriter(root/'r',{}) as w:
                w.write(pose(0));w.write(pose(0,'lio',[10,5,1],'camera_init'))
                w.write(pose(100));w.write(pose(100,'lio',[11,5,1],'camera_init'))
            result=replay(root/'r',root/'raw')
            self.assertNotIn('lio',result['evaluation']['position'])
            result=replay(root/'r',root/'aligned',align_lio_translation=True,assume_aligned_axes=True)
            self.assertEqual(result['evaluation']['position']['lio']['translation_m'],[-10.,-5.,-1.])
            self.assertAlmostEqual(result['evaluation']['position']['lio']['rmse_enu_m'][0],2**-.5)

    def test_truth_matching_selects_nearest_future_sample_and_corrects_antenna_lever(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            with RecordingWriter(root/'r',dict(antenna_offset_body_m=[1,0,0])) as w:
                w.write(pose(0,position=[0,0,0]));w.write(gnss(9,9));w.write(pose(10,position=[2,0,0]))
            result=replay(root/'r',root/'o')
            self.assertEqual(result['evaluation']['position']['gnss']['final_enu_error_m'],[-3.,0.,0.])

    def test_unsupported_imu_frame_is_not_silently_treated_as_flu(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);event=imu(0)
            with RecordingWriter(root/'r',{}) as w:
                w.write(Event(0,0,'imu','imu','body_frd',event.data));w.write(gnss(0))
            result=replay(root/'r',root/'out')
            self.assertEqual(result['detector']['valid_updates'],0)
            self.assertEqual(result['detector']['skipped']['imu_frame_not_body_flu'],1)
            with self.assertRaises(ValueError):replay(root/'r',root/'bad',assume_aligned_axes=True)

    def test_replay_refuses_overwriting_previous_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);generate(root/'r',duration_s=4);replay(root/'r',root/'o')
            with self.assertRaises(FileExistsError):replay(root/'r',root/'o')


if __name__=='__main__':unittest.main()
