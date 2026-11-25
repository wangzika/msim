import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools'))
spec=importlib.util.spec_from_file_location('mesh_clearance',ROOT/'tools/evaluate-mesh-clearance.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class MeshClearanceTest(unittest.TestCase):
    def test_triangle_face_edge_and_vertex_distances(self):
        t=np.array([[[0,0,0],[2,0,0],[0,2,0]]],dtype=float)
        self.assertAlmostEqual(module.triangle_distances([.5,.5,3],t)[0],3.)
        self.assertAlmostEqual(module.triangle_distances([1,-2,0],t)[0],2.)
        self.assertAlmostEqual(module.triangle_distances([-3,-4,0],t)[0],5.)
        self.assertAlmostEqual(module.triangle_distances([.5,.5,0],t)[0],0.)

if __name__=='__main__':unittest.main()
