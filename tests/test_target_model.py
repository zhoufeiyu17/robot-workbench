from pathlib import Path
from types import SimpleNamespace as NS
import json,tempfile,unittest
import numpy as np
from robot_workbench.target_model import export_model,ModelAssets

class TargetModelTest(unittest.TestCase):
    def test_compact_owned_assets_and_tampering(self):
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);urdf=root/'robot.urdf';urdf.write_text('<robot/>')
            vertices=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]])
            geometry=NS(config=dict(urdf=str(urdf)),model=NS(root='base',base='base',names=['j'+str(i)for i in range(12)],joints=[],mimics={}),
                aliases={s:{s+'_link6':'link6'}for s in('left','right')},full_side_meshes={s:[('link6',vertices,np.array([[0,1,2]]))]for s in('left','right')},
                static_meshes=[],frames=lambda q:{s:dict(link6=np.eye(4))for s in('left','right')})
            out=root/'assets';manifest=export_model(geometry,out,'test');assets=ModelAssets(out/'manifest.json','test')
            self.assertEqual(assets.model_id,manifest['model_id']);self.assertEqual(assets.metadata['binary_bytes'],2*3*3*4*2)
            self.assertNotIn('positions',assets.metadata);self.assertTrue(assets.metadata['display_only'])
            with self.assertRaises(ValueError):ModelAssets(out/'manifest.json','another-device')
            path=out/'model.json';value=json.loads(path.read_text());value['binary_bytes']+=4;path.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError,'digest'):ModelAssets(out/'manifest.json','test')
            path.chmod(0o644)
            with self.assertRaisesRegex(ValueError,'private owned'):ModelAssets(out/'manifest.json','test')

if __name__=='__main__':unittest.main()
