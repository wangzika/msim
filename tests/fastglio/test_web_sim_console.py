import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('web_sim_console', ROOT / 'tools/web-sim-console.py')
web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(web)


class WebConsoleTest(unittest.TestCase):
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
