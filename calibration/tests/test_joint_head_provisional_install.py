"""A user exception installs exact results without laundering failed acceptance."""
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

from joint_head_workbench import Workbench
from arx_env.joint_head_calibration_store import CalibrationStore


class ProvisionalInstallTest(unittest.TestCase):
    def test_provisional_replay_is_readonly_and_remains_batch_bound(self):
        with TemporaryDirectory() as directory:
            root=Path(directory).resolve();(root/'batch').mkdir()
            matrix=np.eye(4).tolist()
            row={'id':'pair','kind':'wrist','file':'batch/wrist-fit.json',
                 'pair_status':'incomplete_both','deployment_status':'user_accepted_provisional',
                 'left':{'T_link6_camera':matrix},'right':{'T_link6_camera':matrix}}
            frame={'image':'frame.png','camera_images':{'left':'left.png','right':'right.png'},
                   'q_reported_rad':[0.]*12}
            contract={'head_snapshot':{'joint_offsets_rad':{'left':[0.]*6,'right':[0.]*6}},
                      'cameras':{side:{'width':2,'height':2,'rectified':True} for side in ('left','right')}}
            app=SimpleNamespace(output=root,review_batch=lambda batch:{'frames':[frame]},
                wrist_results=lambda:[row],wrist_contract=lambda r:contract,wrist_overlay_error=lambda r:None,
                calibration_asset_matches=lambda r:True,
                geometry=SimpleNamespace(frames=lambda q:{side:{'link6':np.eye(4)} for side in ('left','right')}))
            with patch('cv2.imread',return_value=np.zeros((2,2,3),np.uint8)), \
                 patch('cv2.imwrite',return_value=True), \
                 patch('arx_env.joint_head_workbench_overlay.mesh_overlay',return_value=np.zeros((2,2,3),np.uint8)):
                for camera in ('left','right'):
                    body={'batch_id':'batch','frame_index':0,'camera':camera,'result_id':'pair'}
                    self.assertEqual(Workbench.historical_overlay(app,body)['camera'],camera)
                    with self.assertRaises(ValueError):
                        Workbench.historical_overlay(app,{**body,'batch_id':'other'})
                del row['deployment_status']
                self.assertEqual(Workbench.historical_overlay(app,body)['camera'],'right')
            self.assertEqual(row['pair_status'],'incomplete_both')

    def test_exact_pair_and_numerics_remain_required_and_failure_is_preserved(self):
        with TemporaryDirectory() as directory:
            root=Path(directory).resolve()
            store=CalibrationStore(root/'library','robot-test','hardware')
            head={'id':'head-id','robot_source':'physical','simulation_only':False}
            h=store.register('head',head['id'],{'row':head},'head-batch','hardware')
            matrix=[[1.,0,0,0],[0,1.,0,0],[0,0,1.,0],[0,0,0,1.]]
            side={'numerically_valid':True,'optimizer_success':True,'jacobian_rank':6,
                  'fit':{'positive_depth':True},'holdout':{'positive_depth':True,'rms_px':7.7},
                  'T_link6_camera':matrix,'accepted':False,'acceptance':{'max_holdout_rms_px':2.}}
            raw=json.dumps({'left':side,'right':side}).encode()
            (root/'wrist.json').write_bytes(raw)
            row={'id':hashlib.sha256(b'paired'+raw).hexdigest(),'kind':'wrist','file':'wrist.json',
                 'environment':'hardware','robot_source':'physical','simulation_only':False,
                 'pair_status':'incomplete_both','accepted':False,'left':{'accepted':False},
                 'right':{'accepted':False},'capture_coverage':{'captured_count':27,'planned_count':32}}
            app=SimpleNamespace(idle=lambda:None,calibration_results=lambda:[head],wrist_results=lambda:[row],
                require_calibration_environment=lambda r:None,require_joint_head_dependency=lambda r:None,
                wrist_contract=lambda r:{'head_snapshot':{'id':head['id']}},output=root,
                store_for=lambda env:store,register_result=lambda r:h,geometry_id='geometry',
                robot_source=lambda:'physical',installed_pair=store.active)
            announced=[]
            app.on_calibration_installed=announced.append
            app.notify_installation=lambda:Workbench.notify_installation(app)
            body={'head_result_id':head['id'],'wrist_result_id':row['id']}
            with self.assertRaisesRegex(ValueError,'confirmation'):
                Workbench.install_provisional_pair(app,body)
            body['confirmation']='INSTALL_USER_ACCEPTED_PROVISIONAL'
            with self.assertRaisesRegex(ValueError,'exact head'):
                Workbench.install_provisional_pair(app,{**body,'wrist_result_id':'wrong'})
            bad=json.loads(raw);bad['left']['jacobian_rank']=5
            bad_raw=json.dumps(bad).encode();original_id=row['id']
            row['id']=hashlib.sha256(b'paired'+bad_raw).hexdigest()
            (root/'wrist.json').write_bytes(bad_raw)
            with self.assertRaisesRegex(ValueError,'observable'):
                Workbench.install_provisional_pair(app,{**body,'wrist_result_id':row['id']})
            row['id']=original_id;(root/'wrist.json').write_bytes(raw)
            result=Workbench.install_provisional_pair(app,body)
            self.assertTrue(result['installed']['verified'])
            self.assertEqual(announced,[result['installed']])
            active=json.loads((store.root/'active/wrist.json').read_text())
            self.assertFalse(active['payload']['row']['accepted'])
            self.assertEqual(active['payload']['row']['pair_status'],'incomplete_both')
            self.assertEqual(active['payload']['row']['left']['T_link6_camera'],matrix)
            self.assertFalse(result['installed']['acceptance']['normal_acceptance_passed'])
            self.assertEqual((root/'wrist.json').read_bytes(),raw)
            self.assertIsNone(row['left'].get('T_link6_camera'))
            (root/'wrist.json').write_text('{}')
            with self.assertRaisesRegex(ValueError,'result changed'):
                Workbench.install_provisional_pair(app,body)


if __name__=='__main__':unittest.main()
