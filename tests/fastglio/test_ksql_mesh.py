import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from ksql_mesh import load_mesh, vertical_hits

spec=importlib.util.spec_from_file_location('prepare_ksql',ROOT/'tools/prepare-ksql-scene.py')
scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)


class KsqlMeshTest(unittest.TestCase):
    def test_collada_centimetres_and_node_scale_cancel_correctly(self):
        xml='''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">
        <asset><unit meter="0.01"/><up_axis>Z_UP</up_axis></asset>
        <library_geometries><geometry id="g"><mesh>
        <source id="positions"><float_array>0 0 2 1 0 2 0 1 2</float_array></source>
        <vertices id="v"><input semantic="POSITION" source="#positions"/></vertices>
        <triangles count="1"><input semantic="VERTEX" source="#v" offset="0"/><p>0 1 2</p></triangles>
        </mesh></geometry></library_geometries><library_visual_scenes><visual_scene>
        <node><matrix>100 0 0 0 0 100 0 0 0 0 100 0 0 0 0 1</matrix>
        <instance_geometry url="#g"/></node></visual_scene></library_visual_scenes></COLLADA>'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'mesh.dae';p.write_text(xml);tri=load_mesh(p)
        np.testing.assert_allclose(tri[0],[[0,0,2],[1,0,2],[0,1,2]])
        np.testing.assert_allclose(vertical_hits(tri,.2,.3),[2])
        self.assertEqual(len(vertical_hits(tri,2,2)),0)

    def test_route_rejects_roof_intersection(self):
        floor=np.array([[[-20,-20,0],[20,-20,0],[20,20,0]],
                        [[-20,-20,0],[20,20,0],[-20,20,0]]],dtype=float)
        roof=floor.copy();roof[:,:,2]=10
        profile=dict(spawn_enu_m=[0,0,.2],flight_height_m=8,
                     test_region_enu_m=dict(east=[-20,20],north=[-20,20]))
        route=[dict(east_m=0,north_m=0),dict(east_m=5,north_m=0)]
        with self.assertRaisesRegex(ValueError,'too close'):
            scene.validate_route(dict(Terrain=floor,Buildings=roof),profile,route)


if __name__=='__main__':unittest.main()
