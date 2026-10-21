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
    def test_migration_scene_override_keeps_current_scene_only_for_matching_source_output(self):
        last=self.root/'last';last.mkdir()
        def pose(x):return dict(position=[x,0,.8],orientation_wxyz=[1,0,0,0])
        current=dict(environment_from_cup=pose(.51),environment_from_machine_root=pose(.63))
        historical=dict(schema='scene_recovery_state.v1',origin=pose(.4),latest_parent={'receipt':'original-parent'},
            last_observed_scene=dict(cup=pose(.42),machine=pose(.60)))
        save(self.profile.base,dict(initial_suite=dict(episodes=[current])))
        save(last/'scene-recovery-state.json',historical);original=(last/'scene-recovery-state.json').read_bytes()
        self.profile.sources['last_output']=str(last)
        self.profile.contract['maintenance_restore']=dict(scene=current,scene_override_source_output=str(last),cup_return_parent={'receipt':'keep-parent'})
        with patch('serve_fake_session.subprocess.run'):spec=self.profile.prepare(1,450)
        restored=json.loads((Path(spec['run_path'])/'contract.json').read_text())['maintenance_restore']
        self.assertEqual(restored['scene'],current)
        self.assertEqual(restored['cup_return_parent'],{'receipt':'keep-parent'})
        self.assertEqual(restored['recovery_state_source']['path'],str(last/'scene-recovery-state.json'))
        self.assertEqual((last/'scene-recovery-state.json').read_bytes(),original,'original recovery anchors and history remain intact')

        output=Path(spec['output_path']);output.mkdir();save(output/'run.json',{})
        latest=dict(historical,last_observed_scene=dict(cup=pose(.56),machine=pose(.66)))
        save(output/'scene-recovery-state.json',latest)
        self.profile.stopped(spec,dict(all_owned_exited=True))
        with patch('serve_fake_session.subprocess.run'):following=self.profile.prepare(2,400)
        next_restore=json.loads((Path(following['run_path'])/'contract.json').read_text())['maintenance_restore']
        self.assertEqual(next_restore['scene']['environment_from_cup'],latest['last_observed_scene']['cup'])
        self.assertEqual(next_restore['scene']['environment_from_machine_root'],latest['last_observed_scene']['machine'])
        self.assertEqual(next_restore['cup_return_parent'],restored['cup_return_parent'])
        self.assertEqual((last/'scene-recovery-state.json').read_bytes(),original)

    def test_scene_override_rejects_relative_path_and_missing_or_invalid_explicit_scene(self):
        scene=dict(environment_from_cup=dict(position=[.5,0,.8],orientation_wxyz=[1,0,0,0]),
            environment_from_machine_root=dict(position=[.6,0,.8],orientation_wxyz=[1,0,0,0]))
        for index,maintenance in enumerate([
                dict(scene=scene,scene_override_source_output='relative'),
                dict(scene=scene,scene_override_source_output=str(self.root/'../other')),
                dict(scene_override_source_output=str(self.root/'last')),
                dict(scene={},scene_override_source_output=str(self.root/'last')),
                dict(scene={**scene,'environment_from_cup':dict(position=[float('nan'),0,.8],orientation_wxyz=[1,0,0,0])},scene_override_source_output=str(self.root/'last'))]):
            with self.subTest(maintenance=maintenance):
                self.profile.contract['maintenance_restore']=maintenance
                with patch('serve_fake_session.subprocess.run') as preflight,self.assertRaises(ValueError):self.profile.prepare(index+1,450)
                preflight.assert_not_called()

    def test_restart_keeps_failed_motion_endpoint_and_error_settings(self):
        last=self.root/'last';last.mkdir()
        machine={'position':[.6,0,.78]}
        save(self.profile.base,dict(initial_suite=dict(episodes=[dict(environment_from_machine_root=machine)])))
        state=dict(schema='scene_recovery_state.v1',last_completed=dict(cup={'position':[.5,0,.8]},machine=machine),
            last_observed_scene=dict(cup={'position':[.43,.18,.78]},machine=machine,task_outcome='failed'))
        save(last/'scene-recovery-state.json',state)
        settings=last/'owner-lock/device.execution-settings.json'
        settings.parent.mkdir()
        save(settings,dict(schema='execution_settings.v1',device_id='fake-test',environment='fake',revision=2,policies={}))
        self.profile.sources['last_output']=str(last)
        with patch('serve_fake_session.subprocess.run'):spec=self.profile.prepare(1,450)
        saved=json.loads((Path(spec['run_path'])/'contract.json').read_text())
        self.assertEqual(saved['maintenance_restore']['scene']['environment_from_cup'],state['last_observed_scene']['cup'])
        self.assertEqual(saved['maintenance_restore']['execution_settings_source']['path'],str(settings))
        early=Path(spec['output_path']);early.mkdir();save(early/'run.json',{})
        self.profile.stopped(spec,dict(all_owned_exited=True))
        with patch('serve_fake_session.subprocess.run'):following=self.profile.prepare(2,400)
        restored=json.loads((Path(following['run_path'])/'contract.json').read_text())
        self.assertEqual(restored['maintenance_restore'],saved['maintenance_restore'])

    def test_restart_preserves_latest_recovery_scene_and_pins_original_anchor(self):
        last=self.root/'last';last.mkdir()
        scene=dict(environment_from_cup={'position':[.4,.2,.78]},environment_from_machine_root={'position':[.6,0,.78]})
        save(self.profile.base,dict(initial_suite=dict(episodes=[scene])))
        state=dict(schema='scene_recovery_state.v1',origin={'position':[.4,.2,.78]},latest_parent={},
            last_completed=dict(cup={'position':[.43,.18,.78]},machine=scene['environment_from_machine_root']))
        save(last/'scene-recovery-state.json',state)
        self.profile.sources['last_output']=str(last)
        self.profile.contract['provenance']['input_sha256']={}
        with patch('serve_fake_session.subprocess.run'):
            spec=self.profile.prepare(1,450)
        saved=json.loads((Path(spec['run_path'])/'contract.json').read_text())
        maintenance=saved['maintenance_restore']
        self.assertEqual(maintenance['scene']['environment_from_cup'],state['last_completed']['cup'])
        self.assertEqual(maintenance['recovery_state_source']['path'],str(last/'scene-recovery-state.json'))
        self.assertIn(str(last),saved['filesystem']['readonly_roots'])
        self.assertEqual(saved['provenance']['input_sha256'][str(last/'scene-recovery-state.json')],maintenance['recovery_state_source']['sha256'])
        early=Path(spec['output_path']);early.mkdir();save(early/'run.json',{})
        self.profile.stopped(spec,dict(all_owned_exited=True))
        with patch('serve_fake_session.subprocess.run'):
            next_spec=self.profile.prepare(2,400)
        following=json.loads((Path(next_spec['run_path'])/'contract.json').read_text())
        self.assertEqual(following['maintenance_restore'],saved['maintenance_restore'])

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

    def test_persistent_profile_requires_explicit_lease_and_removes_all_child_expiry(self):
        self.contract['resources'].update(persistent_service=True,service_lifetime_seconds=None,
            resident_deadline_wall_ns=None,max_wall_time_minutes=120)
        save(self.profile.contract_path,self.contract)
        profile=FullFakeProfile(contract=self.profile.contract_path,base=self.profile.base,nominal=self.profile.nominal,
            state_dir=self.root/'persistent',lease=self.lease,deadline_wall_ns=None)
        empty='<nvidia_smi_log><gpu><uuid>GPU-a</uuid><processes/></gpu><gpu><uuid>GPU-b</uuid><processes/></gpu></nvidia_smi_log>'
        with patch('serve_fake_session.subprocess.check_output',return_value=empty):
            with self.assertRaisesRegex(ValueError,'资源'):profile.guard(1,None)
            lease=json.loads(self.lease.read_text());lease.update(persistent_service=True,deadline_wall_ns=None);save(self.lease,lease)
            self.assertTrue(profile.guard(1,None))
        with patch('serve_fake_session.subprocess.run') as preflight:spec=profile.prepare(1,None)
        argv=spec['argv'];derived=json.loads((Path(spec['run_path'])/'contract.json').read_text())
        self.assertIn('--persistent',argv)
        for obsolete in ('timeout','--duration-wall-s','--deadline-wall-ns'):self.assertNotIn(obsolete,argv)
        self.assertIsNone(derived['resources']['service_lifetime_seconds'])
        self.assertEqual(derived['resources']['max_wall_time_minutes'],120)
        self.assertEqual(preflight.call_args.kwargs['timeout'],120,'startup remains bounded')
        self.assertEqual(json.loads(profile.contract_path.read_text()),self.contract)

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

    def test_managed_attempt_has_fixed_port_and_compatible_receipts_without_overwriting_legacy(self):
        attempts=self.root/'attempts'
        profile=FullFakeProfile(contract=self.profile.contract_path,base=self.profile.base,nominal=self.profile.nominal,
            state_dir=self.profile.root,lease=self.lease,deadline_wall_ns=self.deadline,backend_port=43961,attempt_root=attempts)
        legacy=self.profile.root/'cleanup-1.json';legacy.write_text('old immutable receipt')
        with patch('serve_fake_session.subprocess.run'):spec=profile.prepare(1,450)
        output=Path(spec['output_path']);self.assertEqual(output.parent,attempts)
        self.assertRegex(output.name,r'^attempt-[0-9]+$');self.assertEqual(spec['argv'][spec['argv'].index('--port')+1],'43961')
        identity=dict(pid=123,start_ticks='456',argv=spec['argv'],started_wall_ns=time.time_ns(),output=str(output),log=spec['log_path'])
        profile.started(spec,identity)
        self.assertEqual(json.loads((attempts/'current.json').read_text()),identity)
        profile.stopped(spec,dict(all_owned_exited=False,child_exit_code=None,checked_wall_ns=time.time_ns()))
        exited=attempts/(output.name+'-exit.json');self.assertEqual(json.loads(exited.read_text())['status'],'unknown')
        self.assertIsNone(profile.sources['last_output'])
        output.mkdir();save(output/'run.json',{})
        profile.stopped(spec,dict(all_owned_exited=True,child_exit_code=0,checked_wall_ns=time.time_ns()))
        receipt=json.loads(exited.read_text());self.assertEqual(receipt['status'],'exited');self.assertEqual(receipt['exit_code'],0)
        self.assertEqual(profile.sources['last_output'],str(output));self.assertEqual(legacy.read_text(),'old immutable receipt')
        self.assertEqual(json.loads(profile.contract_path.read_text()),self.contract)

    def test_inference_settings_and_history_pins_follow_last_run_and_survive_early_stop(self):
        last=self.root/'retained';infer=last/'inference';infer.mkdir(parents=True)
        save(infer/'execution-settings.json',dict(execute_steps=45,revision=4))
        video=infer/'old-run/videos/source-base_0.mp4';video.parent.mkdir(parents=True);video.write_bytes(b'original small playback')
        save(infer/'history.json',dict(schema='inference_history.v1',entries=[dict(path='old-run/videos/source-base_0.mp4',
            result=dict(id='old-result',run_id='old-run',details={'receipt':str(infer/'old-run/receipt.json')}))]))
        self.profile.sources['last_output']=str(last)
        with patch('serve_fake_session.subprocess.run'):spec=self.profile.prepare(1,450)
        contract=json.loads((Path(spec['run_path'])/'contract.json').read_text())
        source=contract['maintenance_restore']['inference_source']
        self.assertEqual(source['root'],str(infer));self.assertEqual(len(source['files']),3)
        self.assertIn(str(infer),contract['filesystem']['readonly_roots'])
        self.assertEqual(contract['provenance']['input_sha256'][str(video)],source['files']['old-run/videos/source-base_0.mp4']['sha256'])
        early=Path(spec['output_path']);early.mkdir();save(early/'run.json',{})
        self.profile.stopped(spec,dict(all_owned_exited=True))
        with patch('serve_fake_session.subprocess.run'):following=self.profile.prepare(2,400)
        restored=json.loads((Path(following['run_path'])/'contract.json').read_text())
        self.assertEqual(restored['maintenance_restore']['inference_source'],source)

    def test_single_attempt_cli_has_one_local_launcher_and_no_http_management_server(self):
        import serve_fake_session
        argv=['serve_fake_session','--contract',str(self.profile.contract_path),'--base',str(self.profile.base),
              '--nominal-config',str(self.profile.nominal),'--state-dir',str(self.profile.root),'--lease',str(self.lease),
              '--deadline-wall-ns',str(self.deadline),'--single-attempt','--backend-port','43961',
              '--attempt-root',str(self.root/'attempts')]
        with patch.object(sys,'argv',argv),patch.object(serve_fake_session,'FakeSession') as session,\
                patch.object(serve_fake_session,'run_single_attempt',return_value=0) as run,\
                patch.object(serve_fake_session,'serve_session') as http,patch.object(serve_fake_session.signal,'signal'):
            self.assertEqual(serve_fake_session.main(),0)
        http.assert_not_called();run.assert_called_once()
        options=session.call_args.kwargs;self.assertTrue(options['operations_managed'])
        self.assertNotEqual(options['state_dir'],self.profile.root)
        self.assertEqual(session.call_args.args[0].__self__.root,self.profile.root)
        self.assertEqual(session.call_args.args[0].__self__.backend_port,43961)


if __name__=='__main__':unittest.main()
