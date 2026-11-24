import importlib.util
import base64
import io
import json
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('web_sim_console', ROOT / 'tools/web-sim-console.py')
web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(web)
from web_cloud import pcd_preview


def request(handler, path, method='GET', value=None):
    body=value if isinstance(value,bytes) else json.dumps(value).encode() if value is not None else b''
    message=(f'{method} {path} HTTP/1.0\r\nHost: localhost\r\nContent-Length: {len(body)}\r\n\r\n').encode()+body
    class Socket:
        def __init__(self):self.output=bytearray()
        def makefile(self,*_):return io.BytesIO(message)
        def sendall(self,data):self.output.extend(data)
    socket=Socket()
    handler(socket,('127.0.0.1',1000),SimpleNamespace(server_name='localhost',server_port=8766))
    header,body=bytes(socket.output).split(b'\r\n\r\n',1)
    lines=header.decode().split('\r\n')
    return int(lines[0].split()[1]),dict(line.split(': ',1) for line in lines[1:]),body


class WebConsoleTest(unittest.TestCase):
    def test_launch_script_keeps_reference_gps_preset(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'sensor_profile.json').write_text(json.dumps(dict(models=dict(gps='f9p-dgnss-ucr'))))
            self.assertIn(b'--gps-model f9p-dgnss-ucr',web.launch_script(root,'simple','teleop'))

    def test_pose_stream_is_small_and_keeps_true_world_orientation(self):
        pose=dict(position=[-210,240,12],orientation=[0,0,0,1])
        bridge=SimpleNamespace(pose_snapshot=lambda:dict(pose=dict(truth=pose),frames=dict(truth='world')))
        with tempfile.TemporaryDirectory() as tmp:
            status,headers,body=request(web.make_handler(Path(tmp),bridge),'/api/poses')
            self.assertEqual(status,200);self.assertEqual(json.loads(body)['pose']['truth'],pose)
            self.assertEqual(json.loads(body)['frames']['truth'],'world')
            self.assertLess(len(body),512)

    def test_failed_offline_capture_is_a_preview_with_no_flight_controls(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'capture-failure.json').write_text('{"error":"disk full"}')
            (root/'capture-settings.json').write_text('{"interactive":true}')
            status,headers,body=request(web.make_handler(root),'/api/state')
            state=json.loads(body)
            self.assertTrue(state['complete']);self.assertFalse(state['streaming']);self.assertFalse(state['control_available'])
            self.assertEqual(state['failure']['error'],'disk full')

    def test_downloaded_launch_script_preserves_sensors_and_manual_mode(self):
        import shlex
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'sensor_profile.json').write_text(json.dumps(dict(models=dict(lidar='vlp16',gps='standalone',imu='low-noise'),lidar=dict(timing='rolling'))))
            (root/'capture-settings.json').write_text('{"point_filter_num":2,"scene_key":"simple"}')
            status,headers,body=request(web.make_handler(root),'/api/launch.sh?scene=ksql_airport&mode=teleop')
            self.assertEqual(status,200);self.assertIn('attachment;',headers['Content-Disposition'])
            arguments=shlex.split(body.decode().splitlines()[-1])
            self.assertEqual(arguments[:3],['exec',str(ROOT/'tools/px4-uav-capture.sh'),'--web'])
            for option,value in [('--teleop-height-m','12'),('--scene','ksql_airport'),('--gps-model','standalone'),('--imu-model','low-noise'),('--point-filter-num','2')]:
                self.assertEqual(arguments[arguments.index(option)+1],value)
            self.assertIn('--teleop',arguments)
            self.assertEqual(request(web.make_handler(root),'/api/launch.sh?scene=invalid&mode=teleop')[0],400)

    def test_png_export_is_saved_and_served_without_ros(self):
        png=base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=')
        with tempfile.TemporaryDirectory() as tmp:
            handler=web.make_handler(Path(tmp))
            status,headers,body=request(handler,'/api/export/png','POST',png)
            self.assertEqual(status,200)
            saved=json.loads(body)
            self.assertEqual(Path(saved['file']).read_bytes(),png)
            status,headers,body=request(handler,saved['url'])
            self.assertEqual(status,200);self.assertEqual(body,png)
            self.assertEqual(headers['Content-Type'],'image/png')
            self.assertEqual(request(handler,'/api/export/png','POST',b'not a png')[0],400)
            self.assertEqual(request(handler,'/exports/../../README.md')[0],404)

    def test_binary_cloud_protocol_and_unchanged_revision(self):
        payload=struct.pack('<ffffff',1,2,3,4,5,6)
        bridge=SimpleNamespace(cloud_packet=lambda layer:dict(data=payload,count=2,total=20,
            frame='camera_init',revision='v1',source='/msim/lio_map',age_s=.2))
        with tempfile.TemporaryDirectory() as tmp:
            handler=web.make_handler(Path(tmp),bridge)
            status,headers,body=request(handler,'/api/cloud?layer=map')
            self.assertEqual(status,200);self.assertEqual(body,payload)
            self.assertEqual(headers['X-Point-Total'],'20')
            self.assertEqual(headers['X-Cloud-Frame'],'camera_init')
            status,headers,body=request(handler,'/api/cloud?layer=map&revision=v1')
            self.assertEqual(status,204);self.assertEqual(body,b'')
            self.assertEqual(request(handler,'/api/cloud?layer=unknown')[0],400)

    def test_completed_run_rejects_control_even_if_ros_is_present(self):
        commands=[]
        bridge=SimpleNamespace(command=lambda value:commands.append(value))
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'capture-settings.json').write_text('{"interactive":true}')
            handler=web.make_handler(root,bridge)
            self.assertEqual(request(handler,'/api/teleop/command','POST',{'op':'hold'})[0],200)
            (root/'capture-report.json').write_text('{}')
            self.assertEqual(request(handler,'/api/teleop/command','POST',{'op':'takeoff','height_m':5})[0],409)
            self.assertEqual(commands,[{'op':'hold'}])

    def test_pcd_preview_respects_field_order_and_reports_full_count(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'map.pcd'
            header=b'VERSION 0.7\nFIELDS intensity z x y\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nPOINTS 4\nDATA binary\n'
            p.write_bytes(header+b''.join(struct.pack('<ffff',1,i+3,i+1,i+2) for i in range(4)))
            packet=pcd_preview(p,limit=2)
            self.assertEqual(packet['count'],2);self.assertEqual(packet['total'],4)
            self.assertEqual(struct.unpack('<ffffff',packet['data']),(1,2,3,3,4,5))
            handler=web.make_handler(Path(tmp))
            status,headers,body=request(handler,'/api/cloud?layer=map')
            self.assertEqual(status,200);self.assertEqual(headers['X-Point-Count'],'4')
            self.assertEqual(len(body),4*12)
            p.write_bytes(header+b'broken')
            with self.assertRaises(ValueError):pcd_preview(p)

    def test_scene_identity_uses_archive_and_static_paths_stay_local(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            self.assertEqual(web.scene_metadata(root,dict(scene_key='industrial_yard'))['name'],'工业场地')
            self.assertEqual(web.scene_metadata(root,dict(scene_key='custom',world_source='/simple/custom.world'))['key'],'custom')
            self.assertEqual(web.scene_metadata(root,dict(world_source='/build/ksql_airport-case/world.sdf'))['key'],'ksql_airport')
            handler=web.make_handler(root)
            self.assertEqual(request(handler,'/vendor/../'+'../README.md')[0],404)
            self.assertEqual(request(handler,'/vendor/three/OrbitControls.js')[0],200)

    def test_scene_geometry_extracts_city_collisions(self):
        shapes = web.scene_geometry(ROOT / 'scenes/complex_urban.world')
        self.assertGreater(len(shapes), 20)
        self.assertTrue(any(shape['name'] == 'urban_building_0' for shape in shapes))
        self.assertTrue(all(min(shape['size']) > 0 for shape in shapes))

    def test_samples_ignore_partial_line_and_limit_history(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'injected-gps.jsonl'
            with path.open('w') as output:
                for index in range(web.MAX_SAMPLES + 3):
                    output.write(json.dumps(dict(sim_time_ns=index, east_m=index,
                                                 north_m=0, up_m=0)) + '\n')
                output.write('{"sim_time_ns":')
            rows = web.read_samples(path)
            self.assertEqual(len(rows), web.MAX_SAMPLES)
            self.assertEqual(rows[0]['east_m'], 3)

    def test_teleop_commands_are_bounded(self):
        self.assertEqual(web.validate_command({'op': 'keys', 'keys': ['w', 'w', 'a']}),
                         {'op': 'keys', 'keys': ['a', 'w']})
        for command in ({'op': 'keys', 'keys': ['x']}, {'op': 'takeoff', 'height_m': 100},
                        {'op': 'speed', 'value': True}, {'op': 'shutdown'}):
            with self.assertRaises(ValueError):
                web.validate_command(command)


if __name__ == '__main__':
    unittest.main()
