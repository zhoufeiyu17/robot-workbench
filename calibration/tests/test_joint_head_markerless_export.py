import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import numpy as np

class MeshExportCheck(unittest.TestCase):
    def test_link_local_origin_scale_and_faces(self):
        script = Path(__file__).resolve().parents[1]/'examples/scripts/debug/export_markerless_mesh.py'
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'meshes').mkdir()
            (root/'meshes/t.stl').write_bytes(b' '*80 + struct.pack('<I',1) + struct.pack('<12fH',0,0,1,0,0,0,1,0,0,0,1,0,0))
            (root/'robot.urdf').write_text('<robot><link name="base_link"><visual><origin xyz="1 2 3"/><geometry><mesh filename="package://acone/meshes/t.stl" scale="0.1 0.1 0.1"/></geometry></visual></link><link name="camera"><visual><origin xyz="0 0 1"/><geometry><box size="0.025 0.045 0.045"/></geometry></visual></link></robot>')
            subprocess.run([sys.executable,str(script),'--urdf',str(root/'robot.urdf'),'--package',str(root),'--output',str(root/'mesh.npz')],check=True,capture_output=True)
            with np.load(root/'mesh.npz',allow_pickle=False) as data:
                self.assertEqual(data['names'].tolist(),['base_link','camera'])
                np.testing.assert_allclose(data['vertices_0'].min(0),[1,2,3])
                np.testing.assert_allclose(data['vertices_0'].max(0),[1.1,2.1,3])
                self.assertEqual(data['faces_0'].shape,(1,3))
                np.testing.assert_allclose(data['vertices_1'].max(0),[.0125,.0225,1.0225])
                self.assertEqual(data['faces_1'].shape,(12,3))
                tri=data['vertices_1'][data['faces_1']]
                outward=np.einsum('ij,ij->i',np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),tri.mean(1)-[0,0,1])
                self.assertTrue((outward>0).all())
            self.assertIn('urdf_sha256',json.loads((root/'mesh.json').read_text()))

if __name__ == '__main__':
    unittest.main()
