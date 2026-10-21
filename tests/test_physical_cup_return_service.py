"""Prepared physical recovery publication; no SDK or motion."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from physical_cup_maintenance import derive_return
from test_physical_cup_maintenance import fixture
from robot_workbench.cup_return import prepare
from robot_workbench.physical_cup_return import mark_submitted,stage_candidate


class PhysicalReturnTest(unittest.TestCase):
    def test_current_checked_candidate_publishes_once_and_rejects_changed_intent_or_owner(self):
        parent,old,receipt,source,binding,offset,geometry,_=fixture()
        plan=derive_return(parent,old,receipt,source,offset,geometry)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);active=root/'active';active.mkdir()
            (active/'manifest.json').write_text('{}')
            binding.update(active_sha256={'manifest.json':hashlib.sha256(b'{}').hexdigest()},head_id='h',wrist_id='w')
            current=root/'binding.json';current.write_text(json.dumps(binding))
            manifest=dict(schema='physical_cup_return_candidate.v1',runtime_nonce='nonce',command_epoch=1)
            for key,value in (('source',source),('plan',plan),('binding',binding)):
                path=root/(key+'-candidate.json');path.write_text(json.dumps(value)+(' '*20_000_000 if key=='plan' else ''))
                manifest[key]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            candidate=root/'candidate.json';candidate.write_text(json.dumps(manifest))
            with patch('robot_workbench.control_plan.MAX_INPUT',1):
                with self.assertRaisesRegex(ValueError,'bounded file limit|size budget|执行文件大小上限'):
                    stage_candidate(candidate,root/'rejected-stage')
            self.assertFalse((root/'rejected-stage').exists())
            state=dict(runtime_nonce='nonce',command_epoch=1,holder='collection',allowed_operations={'preview':{'allowed':True}})
            device=NS(environment='physical',physical_cup_return_candidate=str(candidate),cup_return_root=root/'returns',
                runtime=NS(owner=NS(hardware_binding=str(current)),lock=threading.RLock()),workflows={},
                collection_preparation=NS(snapshot=lambda:dict(installed=dict(verified=True,head_id='h',wrist_id='w',path=str(active)))),
                snapshot=lambda _:deepcopy(state),submit=Mock(return_value=dict(token='t',runtime_nonce='nonce',command_epoch=1)))
            initial=source['initial_state']
            sample=(deepcopy(initial['planned_reported']),dict(initial['owner_samples'][-1],disabled_grippers=['right']))
            decode=json.loads
            def bounded_owner_decode(raw,*args,**kwargs):
                self.assertLessEqual(len(raw),256*1024,'large JSON must stay in the CPU child')
                return decode(raw,*args,**kwargs)
            with patch.dict(os.environ,{'PYTHONPATH':''}), patch('real_observation.read_initial_sample',return_value=sample), patch(
                    'robot_workbench.physical_cup_return.json.loads',side_effect=bounded_owner_decode):
                with self.assertRaisesRegex(ValueError,'偏移'):prepare(device,[20,-20])
                state['runtime_nonce']='new'
                with self.assertRaisesRegex(ValueError,'控制权'):prepare(device,[-20,20])
                state['runtime_nonce']='nonce';sample[0]['left_arm_joints'][0]+=.1
                with self.assertRaisesRegex(ValueError,'初态'):prepare(device,[-20,20])
                sample[0]['left_arm_joints'][0]-=.1
                device.submit.assert_not_called()
                result=prepare(device,[-20,20])
                self.assertIn('61 mm',result['summary']);self.assertNotIn('仿真秒',result['summary'])
                self.assertEqual(device.submit.call_args.args[1]['action'],'preview_frozen')
                mark_submitted(device,'review')
                with self.assertRaisesRegex(ValueError,'已提交'):prepare(device,[-20,20])
                with self.assertRaises(FileExistsError):mark_submitted(device,'another-review')
            from robot_workbench.control_plan import _read_file
            staged=device.submit.call_args.args[1].paths['plan']
            packaged=json.loads(_read_file(staged))
            from physical_cup_maintenance import validate_evidence
            validate_evidence(packaged,source)
            for key in ('ticks','native_actions','native_substeps','timeline_digest','control_validation','planning_validation'):
                self.assertEqual(packaged[key],plan[key])
            self.assertNotIn('parent_plan',packaged['maintenance_derivation'])
            original=Path(manifest['plan']['path']).read_bytes()
            self.assertEqual(hashlib.sha256(original).hexdigest(),manifest['plan']['sha256'])


if __name__=='__main__':unittest.main()
