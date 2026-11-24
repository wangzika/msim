from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
import web_scene


class WebSceneTest(unittest.TestCase):
    def test_pose_composition_uses_gazebo_rpy_and_column_major_translation(self):
        matrix=web_scene.multiply(web_scene.pose_matrix('10 20 30 0 0 1.5707963267948966'),web_scene.pose_matrix('1 0 2 0 0 0'))
        values=web_scene.column_major(matrix)
        self.assertAlmostEqual(values[12],10);self.assertAlmostEqual(values[13],21);self.assertAlmostEqual(values[14],32)

    def test_collada_texture_stays_local_and_sdf_height_is_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);media=root/'media';(media/'textures').mkdir(parents=True)
            (media/'textures/satellite.png').write_bytes(b'png fixture')
            mesh=media/'terrain.dae'
            mesh.write_text('<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema"><asset><unit meter="0.01"/><up_axis>Z_UP</up_axis></asset><library_images><image id="sat"><init_from>file://media/textures/satellite.png</init_from></image></library_images></COLLADA>')
            world=root/'world.sdf'
            world.write_text(f'<sdf><world><model name="ground"><pose>0 0 -3.5 0 0 0</pose><link name="body"><visual name="surface"><geometry><mesh><uri>{mesh.as_uri()}</uri></mesh></geometry></visual></link></model></world></sdf>')
            with patch.object(web_scene,'ROOT',root):resources=web_scene.SceneResources(world)
            manifest=resources.manifest();self.assertEqual(manifest['mesh_count'],1)
            item=manifest['objects'][0];self.assertEqual(item['matrix'][14],-3.5);self.assertEqual(item['up_axis'],'Z_UP')
            payload=resources.materialized[item['url'].split('/')[-1]]
            self.assertNotIn(b'file://',payload)
            parsed=ET.fromstring(payload);ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
            texture=parsed.findtext('c:library_images/c:image/c:init_from',namespaces=ns)
            self.assertEqual(resources.assets[texture],media/'textures/satellite.png')
            self.assertEqual(parsed.find('c:asset/c:unit',ns).get('meter'),'0.01')
            with patch.object(web_scene,'ROOT',root):self.assertFalse(resources.allowed(Path('/etc/passwd')))
