"""Physical and sensor invariants, not visual plausibility checks."""
import contextlib
import base64
import http.client
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

from msim.backends import gazebo_command
from msim.bullet_backend import BulletBackend, ReferenceController
from msim.cli import main
from msim.imu_model import ImuConfig, ImuNoiseModel
from msim.physics_scene import load_world, load_collada
from msim.physics_sensors import sensor_profile, GnssModel, RollingLidar, imu_sample
from msim.recording import Recording
from msim.physics_web import LiveView

ROOT=Path(__file__).resolve().parents[2]
HAS_BULLET=importlib.util.find_spec('pybullet') is not None


def empty_world(gravity=(0,0,-9.80665)):
    return dict(gravity=list(gravity),objects=[dict(kind='box',name='far-away',size=[1,1,1],
            matrix=[[1,0,0,1000],[0,1,0,0],[0,0,1,0],[0,0,0,1]])])


class BackendContractTest(unittest.TestCase):
    def test_live_view_png_saved_under_run_and_unknown_paths_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            view=LiveView(0,dict(objects=[]),tmp)
            try:
                png=base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+/l1sAAAAASUVORK5CYII=')
                connection=http.client.HTTPConnection('127.0.0.1',view.server.server_port,timeout=3)
                body=json.dumps(dict(image='data:image/png;base64,'+base64.b64encode(png).decode('ascii')))
                connection.request('POST','/api/export/png',body,{'Content-Type':'application/json','Origin':view.url.rstrip('/')})
                response=connection.getresponse();self.assertEqual(response.status,200);url=json.loads(response.read())['url']
                connection.request('GET',url);response=connection.getresponse();self.assertEqual(response.status,200);self.assertEqual(response.read(),png)
                connection.request('GET','/exports/../../README.md');response=connection.getresponse();self.assertEqual(response.status,404);response.read()
                connection.request('POST','/api/export/png',body,{'Content-Type':'application/json','Origin':'https://example.com'})
                response=connection.getresponse();self.assertEqual(response.status,400);response.read();connection.close()
                self.assertEqual(len(list((Path(tmp)/'exports').glob('*.png'))),1)
            finally:view.close()

    def test_gazebo_forwarding_preserves_every_original_argument(self):
        if sys.platform!='linux':
            with self.assertRaises(ValueError):gazebo_command([],ROOT)
            return
        args=['--web','--scene','ksql_airport','--goal-enu','100','300','12','--point-filter-num','2']
        command,root=gazebo_command(args,ROOT)
        self.assertEqual(command[2:],args);self.assertEqual(root,ROOT)
        self.assertTrue(command[1].endswith('/tools/px4-uav-capture.sh'))

    def test_gazebo_rejects_silently_ignored_native_flags(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as exit:
                main(['simulate','--backend','gazebo','--dry-run','--web'])
        self.assertEqual(exit.exception.code,2)

    def test_imu_ros_adapter_reuses_exact_same_model_class(self):
        p=ROOT/'adapters/ros1_noetic/msim_sensors/scripts/imu_noise_model.py'
        if not p.is_file():self.skipTest('source checkout not available')
        spec=importlib.util.spec_from_file_location('legacy_imu_wrapper',p);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.assertIs(module.ImuNoiseModel,ImuNoiseModel)

    def test_packaged_sensor_parameters_match_original_defaults(self):
        path=ROOT/'adapters/ros1_noetic/msim_sensors/sensor_profiles.py'
        if not path.is_file():self.skipTest('source checkout not available')
        spec=importlib.util.spec_from_file_location('original_sensor_profiles',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        for name in ('vlp16','hdl32','os1-64','mid360'):
            actual=sensor_profile(lidar=name);original=module.resolve(lidar=name)
            self.assertEqual(actual['imu_ros'],original['imu_ros']);self.assertEqual(actual['gps'],original['gps'])
            self.assertEqual(actual['lidar'],original['lidar'])

    def test_gnss_transition_and_marginal_variance(self):
        g=sensor_profile()['gps'];model=GnssModel(g)
        model.rng.gauss=lambda *_:1.
        first=model.sample(0,[0,0,0]);second=model.sample(200_000_000,[0,0,0])
        white=np.array(g['position_sigma_enu_m']);bias=np.array(g['bias_sigma_enu_m']);phi=np.exp(-.2/np.array(g['bias_tau_enu_s']))
        np.testing.assert_allclose(first,bias+white,atol=1e-15)
        np.testing.assert_allclose(second,phi*bias+bias*np.sqrt(1-phi*phi)+white,atol=1e-15)
        np.testing.assert_allclose(np.square(white)+np.square(bias),g['reported_variance_enu_m2'],atol=1e-15)
        with self.assertRaises(ValueError):model.sample(200_000_000,[0,0,0])

    def test_custom_profile_lidar_switch_preserves_imu_and_clears_livox_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            profile=sensor_profile(lidar='mid360');profile['imu_ros']['accel_noise_density']*=.5
            path=Path(tmp)/'sensors.json';path.write_text(json.dumps(profile))
            switched=sensor_profile(path,lidar='hdl32')
            self.assertEqual(switched['imu_ros'],profile['imu_ros']);self.assertEqual(switched['lidar']['vertical_samples'],32)
            self.assertNotIn('pattern_file',switched['lidar']);self.assertNotIn('point_rate_hz',switched['lidar'])

    def test_scene_import_rejects_unsupported_collisions_and_dynamic_models(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'bad.world'
            for body in ['<static>true</static><link name="l"><collision name="c"><geometry><heightmap/></geometry></collision></link>',
                         '<link name="l"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision></link>']:
                p.write_text('<sdf version="1.6"><world name="w"><model name="bad">'+body+'</model></world></sdf>')
                with self.assertRaises(ValueError):load_world(p)


@unittest.skipUnless(HAS_BULLET,'optional physics extra not installed')
class PhysicalInvariantTest(unittest.TestCase):
    def test_freefall_matches_gravity_and_semiimplicit_integral(self):
        with BulletBackend(empty_world(),spawn=(0,0,10)) as b:
            n=100
            for _ in range(n):state=b.step([0]*4)
            self.assertAlmostEqual(state['velocity'][2],-9.80665*n*b.dt,places=10)
            self.assertAlmostEqual(state['position'][2],10-9.80665*b.dt**2*n*(n+1)/2,places=10)

    def test_hover_uses_forces_and_moments_and_converges(self):
        with BulletBackend(empty_world(),spawn=(0,0,1)) as b:
            c=ReferenceController(b)
            for _ in range(6000):b.step(c.motors([0,0,5]))
            np.testing.assert_allclose(b.state()['position'],[0,0,5],atol=.001)
            self.assertLess(np.linalg.norm(b.state()['velocity']),.001)
            self.assertTrue(np.all(b.omega>0))

    def test_motor_step_uses_sdf_time_constant(self):
        with BulletBackend(empty_world(),spawn=(0,0,10)) as b:
            b.step([500]*4)
            np.testing.assert_allclose(b.omega,[500*(1-math.exp(-b.dt/.0125))]*4,atol=1e-12)
            self.assertGreater(b.state()['acceleration'][2],-9.80665)

    def test_body_stops_at_actual_wall_collision(self):
        scene=empty_world((0,0,0));scene['objects']=[dict(kind='box',name='wall',size=[.2,10,10],
            matrix=[[1,0,0,2],[0,1,0,0],[0,0,1,5],[0,0,0,1]])]
        with BulletBackend(scene,spawn=(0,0,5)) as b:
            b.p.resetBaseVelocity(b.vehicle,linearVelocity=[4,0,0],physicsClientId=b.client)
            contacted=False
            for _ in range(500):b.step([0]*4);contacted|=bool(b.contacts())
            self.assertTrue(contacted);self.assertLess(b.state()['position'][0],1.9)

    def test_east_position_bias_causes_west_compensation(self):
        with BulletBackend(empty_world(),spawn=(0,0,5)) as b:
            c=ReferenceController(b)
            for _ in range(5000):b.step(c.motors([0,0,5],position_bias=[2,0,0]))
            self.assertLess(b.state()['position'][0],-1.9)
            self.assertAlmostEqual(b.state()['position'][2],5,places=2)

    def test_rolling_returns_have_real_motion_and_original_acquisition_times(self):
        scene=empty_world((0,0,0));scene['objects']=[dict(kind='plane',name='ground',normal=[0,0,1],matrix=np.eye(4).tolist())]
        cfg=sensor_profile()['lidar'];cfg.update(horizontal_samples=100,range_noise_sigma_m=0.)
        with BulletBackend(scene,spawn=(0,0,5)) as b:
            b.p.resetBaseVelocity(b.vehicle,linearVelocity=[1,0,0],physicsClientId=b.client)
            scanner=RollingLidar(cfg,b.dt);ready=None
            for i in range(51):
                ready=scanner.update(b)
                if i<50:b.step([0]*4)
            self.assertIsNotNone(ready);fields=ready['fields']
            self.assertEqual(ready['stamp_ns'],0);self.assertEqual(ready['end_ns'],100_000_000)
            self.assertAlmostEqual(float(fields['time'].max()),.098,places=6)
            points=np.column_stack([fields[k] for k in ('x','y','z')])
            expected=points+np.column_stack([fields['time'],np.zeros(len(points)),np.full(len(points),5.15)])
            np.testing.assert_allclose(ready['world_points'],expected,atol=2e-5)
            self.assertLess(np.max(np.abs(ready['world_points'][:,2])),.002)
            # A fake single-pose snapshot cannot satisfy this across a moving scan.
            self.assertGreater(np.max(np.abs(ready['world_points'][:,0]-(points[:,0]+.1))),.05)

    def test_imu_specific_force_freefall_zero_hover_gravity(self):
        cfg=sensor_profile()['imu_ros']
        for key in cfg:
            if any(s in key for s in ('noise_density','sigma','random_walk')):cfg[key]=0.
        model=ImuNoiseModel(ImuConfig.from_dict(cfg))
        state=dict(stamp_ns=0,orientation=[0,0,0,1],angular_velocity=np.zeros(3),angular_acceleration=np.zeros(3),acceleration=np.array([0,0,-9.80665]))
        np.testing.assert_allclose(imu_sample(model,state,np.array([0,0,-9.80665]))['linear_acceleration'],[0,0,0],atol=1e-12)
        state.update(stamp_ns=5_000_000,acceleration=np.zeros(3))
        np.testing.assert_allclose(imu_sample(model,state,np.array([0,0,-9.80665]))['linear_acceleration'],[0,0,9.80665],atol=1e-12)

    def test_end_to_end_physical_recording_preserves_delay_without_fake_lio(self):
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'run'
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(['simulate','--backend','bullet','--world',str(ROOT/'scenes/simple.world'),
                    '--output',str(out),'--duration-s','1','--real-time-factor','0','--spoof-start-s','.4','--spoof-rate-enu','.18','0','0']),0)
            recording=Recording(out/'recording');events=list(recording.events())
            self.assertTrue(recording.metadata['complete']);self.assertEqual(recording.metadata['antenna_offset_body_m'],[.05,0,.04])
            gps=[e for e in events if e.stream=='gnss'];self.assertEqual(len(gps),6)
            self.assertTrue(all(e.arrival_ns-e.stamp_ns==200_000_000 for e in gps))
            normal={e.stamp_ns:e for e in events if e.stream=='gnss_normal'}
            for e in gps:
                expected=.18 if e.stamp_ns>=400_000_000 else 0.
                self.assertAlmostEqual(e.data['velocity_mps'][0]-normal[e.stamp_ns].data['velocity_mps'][0],expected)
            clouds=[e for e in events if e.kind=='cloud'];self.assertEqual(len(clouds),10)
            for e in clouds:self.assertTrue(np.isfinite(recording.cloud(e)['x']).all())
            self.assertFalse(any(e.stream in ('lio','loose') for e in events))
            self.assertTrue((out/'evaluation/report.html').is_file())

    def test_ksql_mesh_matches_existing_independent_parser_and_physical_rays(self):
        p=ROOT/'build/scenes/ksql-airport/upstream/ksql_airport/media/SanCarlosAirport_Terrain.DAE'
        if not p.is_file():self.skipTest('optional large KSQL assets absent')
        spec=importlib.util.spec_from_file_location('existing_ksql_parser',ROOT/'tools/ksql_mesh.py')
        old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
        np.testing.assert_array_equal(load_collada(p),old.load_mesh(p))
        scene=load_world(ROOT/'scenes/ksql_airport.world')
        triangles=old.load_ksql(ROOT/'build/scenes/ksql-airport')['Terrain']
        with BulletBackend(scene,spawn=(-210,240,20)) as b:
            distance=b.rays([[0,0,-1]],minimum=.3,maximum=100)[0]
            hits=old.vertical_hits(triangles,-210,240)
            expected=20.15-float(hits.max())
            self.assertAlmostEqual(distance,expected,places=3)


if __name__=='__main__':unittest.main()
