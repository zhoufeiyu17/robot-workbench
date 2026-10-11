"""Frozen child derivation and lease gates, with no GPU or remote commands."""
import json
from pathlib import Path
import shlex
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from serve_fake_session import FullFakeProfile,save
import serve_physx_device


class ProfileTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name).resolve()
        self.deadline=time.time_ns()+int(600e9);code=self.root/'code';code.mkdir()
        self.uuids=['GPU-a','GPU-b']
        argv=['env','CUDA_VISIBLE_DEVICES=1','timeout','-k','30','900','/isaac-sim/python.sh','-B',str(code/'scripts/serve_physx_device.py'),
              '--gpu','1','--base',str(code/'base.json'),'--nominal-config',str(code/'nominal.json'),'--output',str(self.root/'old'),
              '--port','43759','--duration-wall-s','600','--collection-contract',str(code/'contract.json')]
        self.contract=dict(identity={'session_id':'test-session'},code={'root':str(code)},
            resources={'cuda_visible_devices':[1],'exclusive_host_gpu_uuids':self.uuids},browser_ports=[18866],
            filesystem={'writable_roots':[str(self.root/'old')],'readonly_roots':[]},execution={'smoke_commands':[shlex.join(argv)]},
            provenance={'code_sha256':{}},camera_spool_root=str(self.root/'spools'))
        save(code/'contract.json',self.contract);save(code/'base.json',{});save(code/'nominal.json',{})
        self.lease=self.root/'lease.json';save(self.lease,dict(schema='fake_session_lease.v1',session_id='test-session',status='active',
            gpu_record='pre-reserved-test',deadline_wall_ns=self.deadline,gpu_uuids=self.uuids))
        self.profile=FullFakeProfile(contract=code/'contract.json',base=code/'base.json',nominal=code/'nominal.json',
            state_dir=self.root/'manager',lease=self.lease,deadline_wall_ns=self.deadline,browser_ports=[43759])

    def test_prepared_child_is_new_bounded_and_private_and_keeps_original_contract(self):
        with patch('serve_fake_session.subprocess.run') as run:
            spec=self.profile.prepare(1,450)
        path=Path(spec['run_path']);self.assertFalse(Path(spec['output_path']).exists())
        contract=json.loads((path/'contract.json').read_text());argv=spec['argv']
        self.assertEqual(argv[argv.index('--port')+1],'0');self.assertEqual(argv[argv.index('--deadline-wall-ns')+1],str(self.deadline))
        self.assertEqual(contract['browser_ports'],[18866,43759]);self.assertEqual(contract['resources']['service_lifetime_seconds'],450)
        self.assertNotIn(str(self.root/'old'),contract['filesystem']['writable_roots'])
        self.assertTrue(run.call_args.args[0][-1]=='--dry-run')
        self.assertEqual(json.loads(self.profile.contract_path.read_text()),self.contract)
        self.assertNotIn('calibration-auto-confirm',str(argv))

    def test_resource_lease_and_actual_gpu_identity_are_both_required(self):
        empty='<nvidia_smi_log><gpu><uuid>GPU-a</uuid><processes/></gpu><gpu><uuid>GPU-b</uuid><processes/></gpu></nvidia_smi_log>'
        with patch('serve_fake_session.subprocess.check_output',return_value=empty):self.assertTrue(self.profile.guard(1,450))
        busy=empty.replace('<processes/>','<processes><process_info><pid>9</pid></process_info></processes>',1)
        with patch('serve_fake_session.subprocess.check_output',return_value=busy),self.assertRaisesRegex(ValueError,'GPU'):self.profile.guard(1,450)
        with self.assertRaisesRegex(ValueError,'时段'):self.profile.guard(1,179)
        lease=json.loads(self.lease.read_text());lease['status']='released';save(self.lease,lease)
        with self.assertRaisesRegex(ValueError,'时段'):self.profile.guard(1,450)

    def test_cleanup_never_reuses_unconfirmed_run_or_resumes_controls(self):
        with patch('serve_fake_session.subprocess.run'):spec=self.profile.prepare(1,400)
        output=Path(spec['output_path']);output.mkdir();save(output/'run.json',{})
        self.profile.stopped(spec,dict(all_owned_exited=False));self.assertIsNone(self.profile.sources['last_output'])
        self.profile.stopped(spec,dict(all_owned_exited=True));self.assertEqual(self.profile.sources['last_output'],str(output))
        self.assertEqual(set(json.loads(self.profile.source_file.read_text())),{'resume','history','catalogs','last_output'})

    def test_early_stopped_start_keeps_latest_prepared_calibration_fallback(self):
        with patch('serve_fake_session.subprocess.run'):spec=self.profile.prepare(1,400)
        path=Path(spec['run_path'])/'sources.json';sources=json.loads(path.read_text())
        sources['resume']='/latest-verified-source.json';save(path,sources)
        output=Path(spec['output_path']);output.mkdir();save(output/'run.json',{})
        self.profile.stopped(spec,dict(all_owned_exited=True))
        self.assertEqual(self.profile.sources['resume'],'/latest-verified-source.json')

    def test_expired_absolute_deadline_fails_before_dry_run_or_output_creation(self):
        output=self.root/'expired-output'
        argv=['serve','--base',str(self.profile.base),'--nominal-config',str(self.profile.nominal),
              '--output',str(output),'--gpu','1','--deadline-wall-ns','1','--dry-run']
        with patch.object(sys,'argv',argv),self.assertRaisesRegex(ValueError,'deadline expired'):
            serve_physx_device.main()
        self.assertFalse(output.exists())


if __name__=='__main__':unittest.main()
