"""Declared profile and deployed source fingerprints; no hardware or remote access."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from robot_workbench.machine_profile import MachineProfile,Deployment


class MachineProfileTest(unittest.TestCase):
    def test_declared_machine_and_current_index_digest_never_claim_hardware_verification(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);manifest=root/'manifest.json';manifest.write_text('{"head":{"id":"declared-head"}}')
            value=dict(device_id='arx-configured',environment='physical',machine_profile=dict(label='ARX fixture',host_name='declared-host',
                hardware=dict(robot_model='A1',robot_serial='declared-serial',can_interfaces={'left':'can3','right':'can1'},
                    cameras={'head':{'serial':'123','intrinsics_index':str(root/'head.json')}},disabled_grippers=['right']),
                calibration=dict(active_manifest_index=str(manifest),library_index=str(root),configuration_index=str(root/'missing.json')),
                known_degradations=[{'capability':'right_gripper','reason':'deployment disables its SDK output'}]))
            profile=MachineProfile(value);value['machine_profile']['hardware']['robot_serial']='changed-after-init'
            first=profile.snapshot()
            self.assertEqual(first['declared']['hardware']['robot_serial'],'declared-serial')
            self.assertFalse(first['hardware_verified'])
            self.assertEqual(first['calibration_indexes']['active_manifest_index']['sha256'],hashlib.sha256(manifest.read_bytes()).hexdigest())
            self.assertEqual(first['calibration_indexes']['library_index']['status'],'directory_present')
            self.assertEqual(first['calibration_indexes']['configuration_index']['status'],'unavailable')
            manifest.write_text('{"head":{"id":"new-installation"}}')
            self.assertEqual(profile.snapshot()['calibration_indexes']['active_manifest_index']['sha256'],
                                first['calibration_indexes']['active_manifest_index']['sha256'])

    def test_profile_rejects_unknown_secret_fields_and_undeclared_index_routes(self):
        base=dict(device_id='device',environment='fake')
        self.assertFalse(MachineProfile(base).snapshot()['configured'])
        for declaration in ({'token':'must-never-render'}, {'hardware':{'password':'must-never-render'}},
                {'calibration':{'active_manifest_index':'../private'}}, {'hardware':{'disabled_grippers':[{}]}},
                {'hardware':{'cameras':{'other':{'serial':'123'}}}}, {'known_degradations':[{'reason':'missing capability'}]}):
            with self.subTest(declaration=declaration),self.assertRaises(ValueError):MachineProfile(dict(base,machine_profile=declaration))

    def test_source_manifest_excludes_runtime_results_and_detects_changed_entrypoints(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'web').mkdir();(root/'scripts').mkdir();(root/'.evidence').mkdir()
            ui=root/'web/operations.js';ui.write_text('// release one\n')
            script=root/'scripts/service.py';script.write_text('print("inert fixture")\n')
            (root/'.evidence/runtime.py').write_text('never part of deployed code manifest')
            (root/'scripts/private.env').write_text('PRIVATE_FIXTURE=do-not-display')
            services={'workbench':SimpleNamespace(spec=SimpleNamespace(expected_script=str(script)))}
            deployed=Deployment(root,services);original=deployed.snapshot()
            self.assertEqual(original['source_verification'],'captured_at_startup');self.assertEqual(len(original['sha256']),64)
            self.assertNotIn('PRIVATE_FIXTURE',json.dumps(original))
            (root/'.evidence/runtime.py').write_text('updated historical output')
            self.assertEqual(Deployment(root,services).sha256,original['sha256'])
            ui.write_text('// release two\n')
            changed=deployed.snapshot()
            self.assertNotIn('files_unchanged',changed)
            self.assertEqual(changed['sha256'],original['sha256'])
            self.assertNotEqual(Deployment(root,services).sha256,original['sha256'])


if __name__=='__main__':unittest.main()
