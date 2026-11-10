"""Common assembly contracts and physical preflight; never imports a hardware SDK."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),
              str(ROOT/'calibration/tests'),str(ROOT/'rule_based_workbench')]
from robot_workbench.device_bootstrap import (read_profile,preflight,create_device,install_business_workflows,
    physical_runtime_factory,build_physical_application,close_deployment,_attach_remote_collection)
from robot_workbench.service import Application
from test_owner_clock import ClockBackend,Safety,calibration,jaw


class BootstrapTests(unittest.TestCase):
    def test_production_profile_requires_ros_without_legacy_live_sources(self):
        with self.assertRaisesRegex(ValueError,'explicit ROS'):read_profile(self.path,require_ros=True)
        plane=dict(schema='workbench.ros-data-plane.v1',namespace='/workbench/cpu',domain_id=126)
        value=dict(self.profile,data_plane=plane)
        self.path.write_text(json.dumps(value))
        self.assertEqual(read_profile(self.path,require_ros=True)['data_plane'],plane)
        for change in (dict(recording_backend=None),dict(inference={'native_rgb':{'directory':'/old'}})):
            self.path.write_text(json.dumps(dict(value,**change)))
            with self.assertRaises(ValueError):read_profile(self.path,require_ros=True)
    def test_explicit_initial_hold_and_maintenance_finish_options_preserve_defaults(self):
        from robot_workbench.device_bootstrap import _inference_options
        profile=dict(self.profile,disabled_grippers=[],inference={})
        self.assertNotIn('normal_initial_mode',_inference_options(profile))
        self.assertNotIn('normal_finish_policy',_inference_options(profile))
        profile['preserve_begin_jaws']=True
        profile['inference']=dict(action_chunks=True,recording_enabled=False,normal_modes={'recording_required':False},
            normal_initial_mode='HOLD',normal_finish_kind='maintenance_home_v1',normal_finish_policy={'cpu':'Owner validates the real policy'})
        options=_inference_options(profile)
        self.assertEqual(options['normal_initial_mode'],'HOLD')
        self.assertEqual(options['normal_finish_kind'],'maintenance_home_v1')
        options['normal_finish_policy']['cpu']='changed copy'
        self.assertNotEqual(options['normal_finish_policy'],profile['inference']['normal_finish_policy'])
        for fields in ({'normal_initial_mode':'TEACH'},{'normal_finish_kind':'dagger_reset_v1'},
                       {'normal_finish_policy':None},{'recording_context_policy':'per_episode_review'}):
            with self.subTest(fields=fields),self.assertRaises(ValueError):_inference_options(dict(profile,inference=dict(profile['inference'],**fields)))

    def test_module_consumer_options_are_explicit_and_do_not_enable_raw_gate_by_default(self):
        from robot_workbench.device_bootstrap import _inference_options
        profile=dict(self.profile,disabled_grippers=[],inference={})
        self.assertNotIn('action_chunks',_inference_options(profile))
        self.assertNotIn('normal_modes',_inference_options(profile))
        profile['inference']=dict(action_chunks=True,recording_enabled=False,normal_modes={'recording_required':False})
        with self.assertRaisesRegex(ValueError,'preserved begin'):_inference_options(profile)
        profile['preserve_begin_jaws']=True
        value=_inference_options(profile)
        self.assertTrue(value['action_chunks']);self.assertIs(value['normal_modes']['recording_required'],False)
        profile['inference']['recording_enabled']=True
        with self.assertRaisesRegex(ValueError,'recording disabled'):_inference_options(profile)

    def test_machine_pedal_mapping_reaches_listener_and_keeps_legacy_default(self):
        from types import ModuleType
        for key in (None,46):
            value=dict(self.profile)
            if key is not None:value['pedal_stop_key_code']=key
            self.path.write_text(json.dumps(value));read_profile(self.path)
            module=ModuleType('arx_env.pedal_stop');module.PedalStopListener=Mock()
            backend=Mock();runtime=NS(owner=NS(backend=backend))
            with patch.dict(sys.modules,{'arx_env.pedal_stop':module}),patch('real_control.controller_conflicts',return_value=[]),patch(
                    'robot_workbench.device_bootstrap.preflight',return_value={'configuration_ready':True}),patch(
                    'examples.scripts.debug.joint_head_real_owner.legacy_modules',return_value=NS()),patch(
                    'robot_workbench.runtime.Runtime',return_value=runtime):
                physical_runtime_factory(value)()
            module.PedalStopListener.assert_called_once_with(key_code=5 if key is None else key)
            backend.send.assert_not_called()
        for bad in (True,0,768,'46',None):
            self.path.write_text(json.dumps(dict(self.profile,pedal_stop_key_code=bad)))
            with self.assertRaisesRegex(ValueError,'pedal_stop_key_code'):read_profile(self.path)

    def test_initial_hold_mode_reaches_existing_stop_without_motion(self):
        from types import ModuleType
        module=ModuleType('arx_env.pedal_stop');module.PedalStopListener=Mock
        for mode,manual,strict in [('manual_gravity',True,False),('retained_hold',False,False),('retained_hold',False,True)]:
            backend=Mock();runtime=NS(owner=NS(backend=backend))
            value=dict(self.profile,initial_hold_mode=mode,preserve_begin_jaws=strict)
            with patch.dict(sys.modules,{'arx_env.pedal_stop':module}),patch('real_control.controller_conflicts',return_value=[]),patch(
                    'robot_workbench.device_bootstrap.preflight',return_value={'configuration_ready':True}),patch(
                    'examples.scripts.debug.joint_head_real_owner.legacy_modules',return_value=NS()),patch(
                    'robot_workbench.runtime.Runtime',return_value=runtime) as factory:
                self.assertIs(physical_runtime_factory(value)(),runtime)
            if strict:self.assertIs(factory.call_args.kwargs['preserve_begin_jaws'],True)
            else:self.assertNotIn('preserve_begin_jaws',factory.call_args.kwargs)
            backend.connect.assert_called_once_with()
            backend.set_damping_hold.assert_called_once_with(manual=manual)
            backend.send.assert_not_called()

    def test_collection_handoff_must_be_stopped_before_sdk_acquisition(self):
        value=dict(self.profile,robot_id='bench-b',can_interfaces=dict(left='bus_l',right='bus_r'),
                   exclusive_idle_units=['collection-b.service'])
        for active,pid,expected in [('inactive','0',True),('active','12',False),('failed','0',False),('inactive','12',False)]:
            result=NS(returncode=0,stdout=f'ActiveState={active}\nSubState={"dead" if active=="inactive" else "running"}\nMainPID={pid}\n')
            with patch('robot_workbench.device_bootstrap.subprocess.run',return_value=result):
                row=next(r for r in preflight(value,net_root=self.net)['checks']if r['name']=='exclusive collection handoff')
                self.assertEqual(row['passed'],expected)
        self.path.write_text(json.dumps(value))
        self.assertEqual(read_profile(self.path)['exclusive_idle_units'],['collection-b.service'])
        for units in ([],['../collection.service'],['collection.service','collection.service'],'collection.service'):
            self.path.write_text(json.dumps(dict(value,exclusive_idle_units=units)))
            with self.assertRaisesRegex(ValueError,'handoff unit'):read_profile(self.path)

    def test_declared_mapping_and_commissioning_are_inert_and_device_bound(self):
        value=dict(self.profile,robot_id='bench-b',can_interfaces=dict(left='bus_l',right='bus_r'),
                   acquisition_blocked_reason='new device parameters not qualified')
        self.path.write_text(json.dumps(value))
        with patch('subprocess.Popen',side_effect=AssertionError('must not start SDK')):
            self.assertEqual(read_profile(self.path)['can_interfaces'],value['can_interfaces'])
            report=preflight(value,net_root=self.net)
            self.assertFalse(report['configuration_ready'])
            self.assertTrue(any(r['name']=='device commissioning' and not r['passed'] for r in report['checks']))
        for changes in (dict(robot_id='fake-bench'),dict(can_interfaces=dict(left='bus_l',right='bus_l')),
                        dict(acquisition_blocked_reason='')):
            self.path.write_text(json.dumps(dict(value,**changes)))
            with self.assertRaises(ValueError):read_profile(self.path)

    def test_preserved_native_reference_is_explicit_and_requires_pinned_shared_profile(self):
        self.assertNotIn('native_reference_api',read_profile(self.path))
        for mode in (None,True,'automatic'):
            self.path.write_text(json.dumps(dict(self.profile,native_reference_api=mode)))
            with self.assertRaisesRegex(ValueError,'native_reference_api'):read_profile(self.path)
        value=dict(self.profile,native_reference_api='preserve_reference_v1')
        self.path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError,'pinned SDK'):read_profile(self.path)
        value.update(prepared_reference_enabled=True,target_interpreter_profile='x5_control50_damping_v1',
                     sdk_quiesce={'library_build_id':'candidate validated in preflight'})
        self.path.write_text(json.dumps(value))
        with patch('subprocess.Popen',side_effect=AssertionError('profile parsing must remain inert')):
            self.assertEqual(read_profile(self.path)['native_reference_api'],'preserve_reference_v1')

    def test_prepared_reference_output_requires_explicit_normal_sdk_profile(self):
        from robot_workbench.device_bootstrap import _inference_options
        value=dict(self.profile,disabled_grippers=['right'],inference=dict(prepared_reference_execution=True))
        with self.assertRaises(ValueError):_inference_options(value)
        value['prepared_reference_enabled']=True
        self.assertTrue(_inference_options(value)['prepared_reference_execution'])
        value['inference']['reference_integral_feedback']=True
        self.assertTrue(_inference_options(value)['reference_integral_feedback'])
        value['inference']['prepared_reference_execution']=False
        with self.assertRaisesRegex(ValueError,'integral'):_inference_options(value)
        value['inference']['prepared_reference_execution']=True
        value['prepared_reference_enabled']=1;self.path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError,'boolean'):read_profile(self.path)

    def test_reference_planning_is_explicit_copied_profile_input_not_audit_limits(self):
        from robot_workbench.device_bootstrap import _inference_options
        policy=dict(schema='workbench.infer-reference-policy.v1',curve='cubic_c2_quintic_ends',
            velocity_limit_rad_s=1.,acceleration_limit_rad_s2=40.,jaw_velocity_limit_mm_s=40.,
            boundary_scope='complete_prepared_segment')
        profile=dict(self.profile,disabled_grippers=[],inference=dict(action_chunks=True,reference_planning=policy))
        result=_inference_options(profile)
        self.assertEqual(result['reference_planning'],policy)
        policy['velocity_limit_rad_s']=2.
        self.assertEqual(result['reference_planning']['velocity_limit_rad_s'],1.)
        profile['inference']['action_chunks']=False
        with self.assertRaisesRegex(ValueError,'action chunks'):_inference_options(profile)

    def test_single_motion_gain_profile_is_exact_and_inert(self):
        for gain,jaws in [(60,['right']),(80,['right']),(30,['right']),(True,['right']),(float('inf'),['right']),(60,[]),(80,[]),(80.1,['right'])]:
            value=dict(self.profile,right_j4_diagnostic_kp=gain,disabled_grippers=jaws)
            self.path.write_text(json.dumps(value))
            with self.subTest(gain=gain,jaws=jaws):
                if gain in (60,80) and jaws==['right']:
                    with patch('subprocess.Popen',side_effect=AssertionError('profile read must remain inert')):
                        self.assertEqual(read_profile(self.path)['right_j4_diagnostic_kp'],gain)
                else:
                    with self.assertRaises(ValueError):read_profile(self.path)

        for kp,kd in ((60,1),(80,1),(80,1.6),(60,1.6),(80,1.7),(80,True),(80,None),(80,float('nan')),(None,1.6)):
            value=dict(self.profile,disabled_grippers=['right'],right_j4_diagnostic_kd=kd)
            if kp is not None:value['right_j4_diagnostic_kp']=kp
            self.path.write_text(json.dumps(value))
            with self.subTest(kp=kp,kd=kd):
                if type(kd) in (int,float) and (kp,kd) in ((60,1),(80,1),(80,1.6)):
                    with patch('subprocess.Popen',side_effect=AssertionError('profile read must remain inert')):
                        self.assertEqual(read_profile(self.path)['right_j4_diagnostic_kd'],kd)
                else:
                    with self.assertRaises(ValueError):read_profile(self.path)

    def test_left_j6_diagnostic_is_fixed_exclusive_and_not_a_prepared_profile(self):
        for extra,valid in [({},True),({'left_j6_diagnostic':1},False),({'disabled_grippers':[]},False),
                            ({'right_j4_diagnostic_kp':60},False),({'prepared_reference_enabled':True},False)]:
            value=dict(self.profile,left_j6_diagnostic=True,disabled_grippers=['right']);value.update(extra)
            self.path.write_text(json.dumps(value))
            if valid:self.assertTrue(read_profile(self.path)['left_j6_diagnostic'])
            else:
                with self.assertRaises(ValueError):read_profile(self.path)

    def setUp(self):
        self.operation_number=0
        self.temp=TemporaryDirectory(prefix='rwb-',dir='/tmp');self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name).resolve()
        source=self.root/'legacy/infra/arx_env'
        for name in ('coffee_safety.py','pedal_stop.py','backends/arx5_backend.py'):
            path=source/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('')
        (self.root/'jaw.json').write_text('{}')
        cameras={}
        for index,role in enumerate(('head','left','right')):
            path=self.root/(role+'.json');serial=str(900+index)
            path.write_text(json.dumps(dict(serial=serial,width=640,height=480,fx=400.,fy=400.,cx=320.,cy=240.)))
            cameras[role]=dict(serial=serial,intrinsics=str(path))
        self.profile=dict(schema='robot_workbench.physical_device.v1',environment='physical',robot_id='A1-cpu-profile',
            legacy_source=str(self.root/'legacy'),storage_root=str(self.root/'storage'),jaw_mapping_file=str(self.root/'jaw.json'),
            can_interfaces=dict(left='can3',right='can1'),cameras=cameras)
        self.path=self.root/'profile.json';self.path.write_text(json.dumps(self.profile))
        self.net=self.root/'net';self.net.mkdir()
        for name in ('can3','can1'):(self.net/name).mkdir()

    def maintenance(self,app,action,**fields):
        from robot_workbench.maintenance import target
        self.operation_number+=1;device=app.devices['physical']
        return app.maintenance_action(action,dict(environment='physical',request_id='bootstrap-'+str(self.operation_number),
            expected_target=target(app,device),**fields))

    def managed(self,app,action,**fields):
        window=self.maintenance(app,'begin',purpose=action)
        try:return self.maintenance(app,action,maintenance_token=window['token'],**fields)
        finally:self.maintenance(app,'end',maintenance_token=window['token'])

    def test_preflight_does_not_load_sdk_open_cameras_or_spawn_processes(self):
        self.profile['disabled_grippers']=['right']
        self.path.write_text(json.dumps(self.profile))
        with patch('subprocess.Popen',side_effect=AssertionError('no startup in preflight')):
            value=read_profile(self.path);report=preflight(value,net_root=self.net)
            factory=physical_runtime_factory(value)
        self.assertEqual(value['disabled_grippers'],['right'])
        self.assertTrue(report['configuration_ready'])
        self.assertFalse(report['hardware_connected']);self.assertFalse(report['hardware_verified'])
        self.assertFalse(report['models_started']);self.assertFalse(report['cameras_started'])
        self.assertTrue(callable(factory))
        self.assertNotIn('pyrealsense2',sys.modules)
        self.assertNotIn('arx_env.backends.arx5_backend',sys.modules)
        self.assertFalse((self.root/'storage').exists())

    def test_wrist_seed_is_checked_in_the_current_release_before_hardware_start(self):
        from robot_workbench.device_bootstrap import calibration_configuration
        config=self.root/'calibration-config.json';self.profile['calibration_config']=str(config)
        seed=self.root/'release/calibration/configs/wrist-motion-seed-v7.json'
        config.write_text(json.dumps(dict(wrist_seed_file='configs/wrist-motion-seed-v7.json')))
        with patch('robot_workbench.device_bootstrap.ROOT',self.root/'release'),patch('subprocess.Popen',side_effect=AssertionError('preflight must not start hardware')):
            missing=preflight(self.profile,net_root=self.net)
            self.assertFalse(missing['configuration_ready'])
            failed=next(row for row in missing['checks'] if row['name']=='calibration wrist_seed_file')
            self.assertIn('calibration.wrist_seed_file',failed['detail']);self.assertIn(str(seed),failed['detail'])
            seed.parent.mkdir(parents=True);seed.write_text('{}')
            self.assertTrue(preflight(self.profile,net_root=self.net)['configuration_ready'])
            self.assertEqual(calibration_configuration(config)['wrist_seed_file'],str(seed))
            # Explicit external paths remain explicit; never substitute another seed.
            config.write_text(json.dumps(dict(wrist_seed_file=str(seed))))
            self.assertEqual(calibration_configuration(config)['wrist_seed_file'],str(seed))
            config.write_text(json.dumps(dict(wrist_seed_file=str(seed)+'-absent')))
            with self.assertRaisesRegex(ValueError,'calibration.wrist_seed_file'):calibration_configuration(config)

    def test_profile_rejects_wrong_can_duplicate_serial_and_fake_identity(self):
        for edit in (lambda p:p.update(can_interfaces=dict(left='../bus_l',right='bus_r')),
                     lambda p:p.update(robot_id='fake-something'),
                     lambda p:p['cameras']['left'].update(serial=p['cameras']['head']['serial']),
                     lambda p:p.update(inference=dict(model_endpoint='http://robot:1234'))):
            value=deepcopy(self.profile);edit(value);self.path.write_text(json.dumps(value))
            with self.assertRaises(ValueError):read_profile(self.path)
        value=preflight(self.profile,net_root=self.root/'missing-net')
        self.assertFalse(value['configuration_ready'])
        self.assertEqual(sum(not row['passed'] for row in value['checks']),2)

    def inference_profile(self):
        self.profile['disabled_grippers']=['right']
        config=dict(model_endpoint='http://127.0.0.1:49379',catalog_root='/approved/model-host/checkpoints')
        for key in ('model_token_file','profile_path','binding_path'):
            path=self.root/(key+'.json');path.write_text('{}');path.chmod(0o600);config[key]=str(path)
        self.profile['inference']=config
        self.path.write_text(json.dumps(self.profile))
        return read_profile(self.path)

    def test_inference_profile_defaults_epoch7_without_contacting_model_or_device(self):
        from robot_workbench.inference_model import BASE566_ID,model_descriptor
        with patch('subprocess.Popen',side_effect=AssertionError('preflight must not start a model')),\
                patch('http.client.HTTPConnection',side_effect=AssertionError('preflight must not contact model endpoint')):
            value=self.inference_profile();report=preflight(value,net_root=self.net)
        self.assertEqual(value['inference']['model_id'],BASE566_ID)
        self.assertTrue(report['configuration_ready'])
        self.assertEqual(report['inference']['disabled_grippers'],['right'])
        self.assertFalse(report['inference']['model_prepare_requires_device_connection'])
        self.assertFalse(report['inference']['model_endpoint_checked'])
        descriptor=model_descriptor('base566-epoch005-step42130')
        value['inference'].update(model_id=descriptor['model_id'],model_bundle=descriptor)
        self.path.write_text(json.dumps(value))
        self.assertEqual(read_profile(self.path)['inference']['model_bundle'],descriptor)
        for change in (lambda p:p.pop('disabled_grippers'),lambda p:p.update(disabled_grippers=['left']),
                lambda p:p['inference'].update(model_id='other-family'),
                lambda p:p['inference'].update(model_bundle='/untrusted/model.json'),
                lambda p:p['inference'].update(catalog_root='../checkpoints'),
                lambda p:p['inference'].update(model_endpoint='http://10.17.0.27:49379')):
            invalid=deepcopy(value);change(invalid);self.path.write_text(json.dumps(invalid))
            with self.assertRaises(ValueError):read_profile(self.path)

    def test_physical_inference_boot_select_and_prepare_stay_sdk_disconnected(self):
        from robot_workbench.inference_model import BASE566_ID,model_descriptor
        profile=self.inference_profile();clients=[]
        def client(*args,**kwargs):
            instance=Mock();instance.prepare.return_value={'model_id':kwargs['model_id']}
            instance.status.return_value=dict(model_id=kwargs['model_id'],status='ready',loaded=True,
                warmup={'completed':True},service_pid=123,worker_pid=456)
            instance.list_models.return_value=[dict(model_id=ident,label=ident,status='ready',selectable=True,
                model_descriptor=model_descriptor(ident)) for ident in (BASE566_ID,'base566-epoch005-step42130')]
            clients.append((instance,args,kwargs));return instance
        with patch('robot_workbench.inference_workflow.RemoteBaseClient',side_effect=client) as transport,\
                patch('robot_workbench.calibration_profile.load_device_profile',side_effect=AssertionError('model-only must not load device calibration')):
            deployment=build_physical_application(profile,output=self.root/'infer-service')
            try:
                flow=deployment.inference
                deployment.device.connect=Mock(side_effect=AssertionError('no automatic connect'))
                deployment.device.submit=Mock(side_effect=AssertionError('no model-only motion'))
                transport.assert_not_called()
                self.assertIsNone(deployment.device.runtime)
                self.assertEqual(flow.selected_model_id,BASE566_ID)
                self.assertEqual(flow.disabled_grippers,('right',))
                self.assertIsNone(flow.event)
                self.assertEqual(flow.catalog_root,profile['inference']['catalog_root'])
                loaded=self.managed(deployment.application,'prepare_model',model_id='base566-epoch005-step42130')
                self.assertTrue(loaded['model_ready']);self.assertFalse(loaded['control_ready'])
                self.assertEqual(loaded['model_id'],'base566-epoch005-step42130')
                prepared=self.managed(deployment.application,'prepare_model',expected_model_id='base566-epoch005-step42130')
                self.assertTrue(prepared['model_ready']);self.assertFalse(prepared['control_ready'])
                self.assertIsNone(deployment.device.runtime);self.assertIsNone(flow.cache_thread)
                self.assertEqual(clients[-1][1],('http://127.0.0.1:49379',))
                self.assertEqual(clients[-1][2]['model_id'],'base566-epoch005-step42130')
                self.assertEqual(clients[-1][2]['conversion_identity']['disabled_grippers'],['right'])
                self.assertIsNone(clients[-1][2]['adapter'])
                with self.assertRaisesRegex(ValueError,'先连接设备'):
                    flow.action('inference_preview',dict(seed=42))
                deployment.device.connect.assert_not_called();deployment.device.submit.assert_not_called()
            finally:close_deployment(deployment)

    def test_wam_profile_provider_binds_deployment_without_installed_extrinsics(self):
        from test_wam import WamInputWorkflowTests
        profile=self.inference_profile();config=profile['inference']
        bundle=WamInputWorkflowTests().descriptor()
        config.update(model_id=bundle['model_id'],model_bundle=bundle)
        for key in ('profile_path','binding_path','control_mapping_pending'):config.pop(key,None)
        expected=({'native':'WAM profile'},{'native':'WAM binding'})
        with patch('robot_workbench.wam.native_binding',return_value=expected)as load:
            deployment=build_physical_application(profile,output=self.root/'wam-native-provider')
            try:
                self.assertEqual(deployment.inference.profile_provider(),expected)
                self.assertEqual(load.call_args.args,(profile,))
                self.assertIsNone(deployment.device.runtime)
            finally:close_deployment(deployment)

    def test_pedal_double_press_window_is_explicit_before_binding_the_source(self):
        from robot_workbench.service import Application
        for index,window in enumerate((None,False,0.,float('nan'),1.1,.5)):
            profile=self.inference_profile()
            controls=dict(home_velocity_rad_s=.1,timeout_s=20.,focus_timeout_s=8.,
                pedal_source_descriptor=str(self.root/'pedal-source.json'),event_max_age_s=1.)
            if window is not None:controls['double_press_window_s']=window
            profile['inference']['operator_controls']=controls
            with self.subTest(window=window),patch.object(Application,'install_operator_pedal')as install:
                if window==.5:
                    deployment=build_physical_application(profile,output=self.root/f'pedal-window-{index}')
                    try:
                        self.assertEqual(deployment.inference.operator_controls['double_press_window_s'],.5)
                        install.assert_called_once();self.assertIsNone(deployment.device.runtime)
                    finally:close_deployment(deployment)
                else:
                    with self.assertRaisesRegex(ValueError,'double-press window'):
                        build_physical_application(profile,output=self.root/f'pedal-window-{index}')
                    install.assert_not_called()

    def test_physical_inference_record_event_adapts_real_cli_kwargs_signature(self):
        recorded=[]
        def event(name,**fields):recorded.append((name,fields))
        deployment=build_physical_application(self.inference_profile(),output=self.root/'infer-events',event=event)
        try:
            flow=deployment.inference;directory=flow.output/'cpu-event-fixture';directory.mkdir()
            run=dict(directory=directory,event_bytes=0)
            value=dict(execution_instance_id='cpu-only-no-motion',sequence=1)
            flow._record_event(run,'prediction_requested',value)
            self.assertEqual(recorded,[('prediction_requested',value)])
            self.assertEqual(json.loads((directory/'policy-events.jsonl').read_text()),
                             dict(event='prediction_requested',**value))
            self.assertGreater(run['event_bytes'],0)
            self.assertIsNone(deployment.device.runtime)
        finally:close_deployment(deployment)

    def test_inference_history_survives_a_new_service_run_directory(self):
        profile=self.inference_profile()
        first=build_physical_application(profile,output=self.root/'first-service')
        result=dict(id='prior-run',status='invalid',label='preserved failure')
        output=first.inference.output
        self.assertEqual(output,Path(profile['storage_root'])/'inference')
        first.inference.history.append([(result,None)])
        close_deployment(first)
        second=build_physical_application(profile,output=self.root/'second-service')
        try:
            self.assertEqual(second.inference.results(),[result])
            self.assertIsNone(second.device.runtime)
        finally:close_deployment(second)

    def test_native_profile_provider_uses_deployment_not_legacy_local_profile(self):
        profile=self.inference_profile();config=profile['inference']
        config.update(native_binding={'path':'/cpu/native.json','sha256':'a'*64},task='coffee')
        for key in ('profile_path','binding_path','control_mapping_pending'):config.pop(key,None)
        expected=({'native':'profile'},{'native':'binding'})
        with patch('robot_workbench.mixed366_control.load_binding',return_value=expected)as load:
            deployment=build_physical_application(profile,output=self.root/'native-provider')
            try:
                self.assertEqual(deployment.inference.profile_provider(),expected)
                self.assertEqual(load.call_args.args,(config['native_binding'],profile))
                self.assertIsNone(deployment.device.runtime)
            finally:close_deployment(deployment)

    def test_first_physical_acquire_checks_execution_storage_before_sdk_or_owner(self):
        execution=Path(self.profile['storage_root'])/'execution'
        acquire=physical_runtime_factory(self.profile)
        self.assertFalse(execution.exists())  # Factory construction stays inert.
        class OwnerBoundary(Exception):pass
        def owner_boundary(*args,**kwargs):
            self.assertEqual(kwargs['execution_root'],execution)
            self.assertTrue(execution.is_dir())
            self.assertEqual(list(execution.iterdir()),[])  # Write probe cleaned up.
            with (execution/'execution-first.json').open('x') as stream:stream.write('{}')
            raise OwnerBoundary
        with patch('real_control.controller_conflicts',return_value=[]), \
             patch('robot_workbench.device_bootstrap.preflight',return_value={'configuration_ready':True}), \
             patch.object(calibration,'legacy_modules',return_value=NS()) as legacy, \
             patch.object(calibration,'hardware_backend',side_effect=AssertionError('no SDK in CPU storage check')) as sdk, \
             patch('robot_workbench.runtime.Runtime',side_effect=owner_boundary) as runtime:
            with self.assertRaises(OwnerBoundary):acquire()
            self.assertTrue((execution/'execution-first.json').is_file())
            sdk.assert_not_called()
            legacy.reset_mock();runtime.reset_mock()
            # Explicit EACCES also works when the test user is root; chmod alone does not.
            with patch('tempfile.NamedTemporaryFile',side_effect=PermissionError('directory is not writable')):
                with self.assertRaisesRegex(ValueError,'execution evidence directory unavailable'):
                    acquire()
            legacy.assert_not_called();runtime.assert_not_called();sdk.assert_not_called()
            self.assertEqual((execution/'execution-first.json').read_text(),'{}')

    def test_physical_attempt_recorder_and_timeline_match_existing_export_path_guards(self):
        from real_workflow import resolve_raw_source
        from playground_pipeline import execution_runtime
        from test_recording_wall_clock import plan_fixture
        output=self.root/'path-contract-attempt'
        profile=dict(self.profile,collection_setup={'cpu_assembly_only':True},recording_backend=None)
        with patch('robot_workbench.device_bootstrap._attach_physical_businesses'),\
                patch('robot_workbench.device_bootstrap.physical_runtime_factory',wraps=physical_runtime_factory) as factory:
            deployment=build_physical_application(profile,output=output)
        self.addCleanup(close_deployment,deployment)
        self.assertIsNone(deployment.device.runtime)
        recording=factory.call_args.kwargs['recording_factory']
        execution=factory.call_args.kwargs['execution_root']
        self.assertEqual(execution,output/'execution')
        with patch('robot_workbench.recording_process.ProcessRecordingSession',side_effect=lambda path,**kw:NS(root=path)):
            recorded=recording({'plan_ref':{}},lambda _:None)
        configured={'environment':'physical','local_recording_root':str(output/'collection-raw')}
        self.assertEqual(recorded.root.parent,Path(configured['local_recording_root']))
        recorded.root.mkdir(parents=True)
        (recorded.root/'manifest.json').write_text(json.dumps(dict(schema_version='physical_raw_dataset.v1',
            sealed_wall_ns=1,capture_status='incomplete')))
        self.assertEqual(resolve_raw_source(configured,dict(directory=str(recorded.root),done=True)),recorded.root)
        old_raw=output/'raw/old-run';old_raw.mkdir(parents=True)
        with self.assertRaisesRegex(ValueError,'local_recording_root'):
            resolve_raw_source(configured,dict(directory=str(old_raw),done=True))
        _,plan,_,timeline=plan_fixture(4,'physical')
        path=execution/'validated-plan-current/execution-current.json';path.parent.mkdir(parents=True)
        path.write_text(json.dumps(timeline));exported=output/'playground/jobs/current';exported.mkdir(parents=True)
        state=dict(connected=True,execution_timeline=dict(path=str(path),timeline_digest=timeline['timeline_digest'],
            parent_timeline_digest=plan['timeline_digest']))
        lane=NS(config=configured,owner=lambda:deepcopy(state))
        self.assertEqual(execution_runtime(lane,plan,exported,None),timeline)
        old_execution=Path(profile['storage_root'])/'execution';old_execution.mkdir(parents=True)
        outside=old_execution/'execution-old.json';outside.write_text(json.dumps(timeline))
        state['execution_timeline']['path']=str(outside)
        with self.assertRaisesRegex(ValueError,'outside the owned control workspace'):
            execution_runtime(lane,plan,exported,None)
        class RuntimeBoundary(Exception):pass
        def boundary(*args,**kwargs):
            self.assertEqual(kwargs['execution_root'],execution)
            raise RuntimeBoundary
        with patch('real_control.controller_conflicts',return_value=[]),\
                patch('robot_workbench.device_bootstrap.preflight',return_value={'configuration_ready':True}),\
                patch.object(calibration,'legacy_modules',return_value=NS()),\
                patch.object(calibration,'hardware_backend',side_effect=AssertionError('no SDK')) ,\
                patch('robot_workbench.runtime.Runtime',side_effect=boundary):
            with self.assertRaises(RuntimeBoundary):deployment.device.factory()

    def test_both_environments_use_identical_runtime_and_explicit_physical_gate(self):
        jaw.configure_mapping(None)
        for environment in ('fake','physical'):
            calls=[];clock=[10.]
            def factory():
                calls.append('constructed')
                return ClockBackend(lambda:clock[0],source_clock_id='cpu-fake-clock' if environment=='fake' else None)
            def drive(runtime,closed):
                # Test-owned deterministic driver, same Runtime used on both sides.
                for _ in range(42):clock[0]+=.01;runtime.tick()
                closed.wait()
            device=create_device('cpu-'+environment,environment,ref=calibration,safety=Safety,backend_factory=factory,
                lock_root=self.root/environment,initialize_backend=lambda b:b.set_damping_hold(manual=True),
                owner_options={'control_clock':lambda:clock[0]},drive=drive)
            self.assertEqual(calls,[])
            if environment=='physical':
                with self.assertRaisesRegex(ValueError,'支撑'):device.connect({})
                self.assertEqual(calls,[])
            device.connect({'arms_supported':True})
            self.assertEqual(calls,['constructed'])
            self.assertEqual(set(device.credentials),{'calibration','collection','inference','research'})
            self.assertIs(device.runtime.ref,calibration)
            device.closed.set();device.thread.join(1)
            device.runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
            device.runtime.tick();device.runtime.close()

    def test_install_only_actual_components_does_not_invent_business_readiness(self):
        device=create_device('cpu-device','fake',runtime_factory=lambda:None)
        app=Application({'fake':device})
        install_business_workflows(device,app)
        self.assertEqual(device.workflows,{})
        flow=Mock();install_business_workflows(device,app,inference=flow)
        self.assertIs(device.workflows['inference'],flow)
        self.assertNotIn('calibration',device.workflows)
        self.assertNotIn('collection',device.workflows)

    def test_real_common_camera_backend_can_boot_without_opening_any_sensor_or_controller(self):
        deployment=build_physical_application(self.profile,output=self.root/'service')
        try:
            self.assertIsNone(deployment.device.runtime)
            self.assertEqual(deployment.camera_processes,[])
            self.assertEqual(set(deployment.camera_ports),{'head','left','right'})
            self.assertEqual(deployment.cameras.descriptor['environment'],'physical')
            self.assertNotIn('pyrealsense2',sys.modules)
            self.assertNotIn('arx_env.backends.arx5_backend',sys.modules)
            self.assertEqual(deployment.device.workflows,{})
            process=deployment.cameras.process
        finally:close_deployment(deployment)
        self.assertIsNotNone(process.poll())

    def test_physical_workbench_attaches_camera_only_service_without_opening_or_stopping_sources(self):
        from robot_workbench.rgbd_service import CommonCameraService
        intrinsics={name:json.loads(Path(self.profile['cameras'][role]['intrinsics']).read_text())
                    for role,name in {'head':'base_0','left':'left_wrist_0','right':'right_wrist_0'}.items()}
        service=CommonCameraService.start(intrinsics,self.root/'shared-cameras','physical')
        try:
            self.profile['camera_service_descriptor']=str(self.root/'shared-cameras/camera-service.json')
            with patch('robot_workbench.rgbd_service.start_capture_sources',side_effect=AssertionError('no duplicate SDK source')):
                deployment=build_physical_application(self.profile,output=self.root/'attached',start_cameras=True)
            self.assertIsNone(deployment.device.runtime)
            self.assertIsNone(deployment.cameras.process)
            close_deployment(deployment)
            self.assertIsNone(service.process.poll())
            self.assertEqual(service.call('identity')['instance_id'],service.descriptor['instance_id'])
        finally:service.close()

    def test_listener_is_closed_once_only_after_successful_supported_release(self):
        from robot_workbench.runtime import Runtime
        now=[10.]
        runtime=Runtime(calibration,Safety,lambda:ClockBackend(lambda:now[0]),device_id='listener',environment='physical',
            lock_root=self.root/'listener-lock',initialize_backend=lambda b:b.set_damping_hold(manual=True),
            control_clock=lambda:now[0])
        listener=Mock();runtime.owner.listener=listener
        with self.assertRaises(RuntimeError):runtime.close()
        listener.close.assert_not_called()
        # This fixture bypasses pedal input parsing; only lifecycle close is exercised.
        runtime.owner.listener=None
        runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'));runtime.tick()
        runtime.owner.listener=listener
        runtime.close();runtime.close()
        listener.close.assert_called_once_with()

    def physical_preparation(self):
        root=self.root/'prepare';root.mkdir()
        active=root/'calibrations/hardware/active';active.mkdir(parents=True)
        manifest=active/'manifest.json';manifest.write_text('{}')
        install=dict(verified=True,head_id='physical-head',wrist_id='physical-wrists')
        calibration_app=NS(config={'calibration_root':str(root/'calibrations')},output=root/'captures',
            installed_pair=lambda:deepcopy(install));calibration_app.output.mkdir()
        jaw.configure_mapping(None)
        device=create_device('A1-physical-test','physical',ref=calibration,safety=Safety,
            backend_factory=lambda:ClockBackend(time.monotonic),lock_root=root/'cpu-control',
            initialize_backend=lambda backend:backend.set_damping_hold(manual=True))
        app=Application({'physical':device});app.operations_managed=True
        flow=NS(install_lock=threading.RLock(),closed=threading.Event(),invalidate=Mock(),snapshot=lambda:dict(busy=False))
        flow.calibration_installed=Mock(side_effect=lambda value:app.invalidate('physical'))
        device.workflows['calibration']=flow
        cameras=NS(directories={name:str(root/name) for name in ('base_0','left_wrist_0','right_wrist_0')})
        deployment=NS(output=root,device=device,application=app,calibration=calibration_app,cameras=cameras,
            camera_ports=dict(head=1,left=2,right=3),gateway=NS(server_port=4))
        details=dict(compute={'endpoint':{'host':'compute-test','port':22,'key':'/task/key'},
            'deployment_path':'/task/compute.json','deployment_sha256':'a'*64},
            registration=str(root/'registration.json'),profile_template=str(root/'template.json'))
        prep=_attach_remote_collection({'collection_setup':details},deployment,{'collection':root/'token'})
        # Explicitly connect the CPU protocol Owner through the production API.
        # Preparation then has fresh feedback but cannot acquire or command it.
        self.managed(app,'connect_control',arms_supported=True)
        self.assertTrue(device.fact_snapshot()['feedback_healthy']);self.assertTrue(device.fact_snapshot()['stationary'])
        self.assertLessEqual(device.fact_snapshot()['sample_age_s'],.1)
        device.connect=Mock(side_effect=AssertionError('no control acquisition during preparation'))
        device.submit=Mock(side_effect=AssertionError('no motion during preparation'))
        def close_control():
            runtime=device.runtime
            if runtime is not None:
                runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
                device.thread.join(2);self.assertFalse(device.thread.is_alive());runtime.close()
        self.addCleanup(close_control)
        # Only these test-owned mock worker objects receive teardown admission.
        self.addCleanup(prep.close,lifecycle_admission=lambda action:action in ('stop:detector','stop:planner','stop:scene'))
        return prep,device,app,calibration_app,install,manifest

    @staticmethod
    def await_preparation(prep):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            if not prep.snapshot()['busy']:return prep.snapshot()
            time.sleep(.01)
        raise AssertionError('preparation failed to converge')

    def prepared_reply(self,kwargs):
        root=kwargs['output'];root.mkdir();client=root/'client';client.mkdir()
        binding=root/'binding.json';binding.write_text(json.dumps({'robot_id':'A1-physical-test'}))
        configured=dict(setup_path=str(client/'setup.json'),trusted_binding_path=str(kwargs['trusted_binding_path']),
            initial_trusted_binding_source=str(binding))
        (client/'configured.json').write_text(json.dumps(configured))
        return configured

    def attachment_reply(self,path,expected,**kwargs):
        configured=json.loads(Path(path).read_text())
        return dict(configured=configured,installed_binding=json.loads(Path(configured['initial_trusted_binding_source']).read_text()),
            attachment_sha256={str(path):__import__('hashlib').sha256(Path(path).read_bytes()).hexdigest()})

    @staticmethod
    def attach_parts(parts):
        def construct(path,device,app,*,validated):
            parts.configured_path=Path(path);parts.attachment_identity=validated['attachment_sha256']
            return parts
        return construct

    def prepared_parts(self):
        state=dict(busy=False,status='ready',environment={'ready':True},warmup={'status':'ready'},progress={})
        playground=NS(lock=threading.Lock(),state=state,cancel_event=threading.Event(),thread=None,persist=Mock(),
            start=Mock(return_value={}),snapshot=lambda:deepcopy(state))
        playground.start.side_effect=lambda *a,**kw:state.update(status='ready',warmup={'status':'ready'}) or {}
        workers=[]
        for i in range(3):
            manager=Mock(spec=['status','start','recover','probe','close','last_retirement'])
            facts=dict(status='stopped',model_initialized=False,pid=100+i,process_start_ticks=100+i)
            manager.status.side_effect=lambda f=facts:deepcopy(f)
            manager.last_retirement=None
            manager.probe.side_effect=lambda f=facts:dict(service=deepcopy(f),needs_warmup=False,evidence_type='cpu_protocol_fixture')
            def start(*args,m=manager,f=facts,**kwargs):
                def effect():m.config={};f.update(status='ready',model_initialized=True)
                kwargs.get('launch',lambda create:create())(effect)
            manager.start.side_effect=start
            workers.append(manager)
        return NS(playground=playground,workflow=NS(closed=threading.Event(),invalidate=Mock(),snapshot=lambda:dict(busy=False)),
            detector=workers[0],planner=workers[1],scene=workers[2],pipeline_config=Path('/cpu/config'),worker_state_directory=Path('/cpu/workers'))

    def test_physical_prepare_button_reuses_common_startup_and_warm_cache(self):
        parts=self.prepared_parts()
        def reply(**kwargs):
            self.assertTrue(kwargs['still_current']());self.assertFalse(kwargs['cancel']())
            return self.prepared_reply(kwargs)
        with patch('robot_workbench.collection_setup.prepare_remote_collection_process',side_effect=reply) as remote, \
                patch('robot_workbench.collection_setup.prepare_collection_attachment_process',side_effect=self.attachment_reply), \
                patch('robot_workbench.collection_setup.construct_collection',side_effect=self.attach_parts(parts)) as construct:
            prep,device,app,calibration,install,manifest=self.physical_preparation()
            self.assertEqual(prep.snapshot()['status'],'awaiting_prepare')
            remote.assert_not_called();construct.assert_not_called()
            self.managed(app,'prepare_collection');state=self.await_preparation(prep)
            self.assertEqual(state['status'],'ready',state)
            self.assertEqual(remote.call_args.kwargs['expected_install']['head_id'],install['head_id'])
            self.assertEqual(remote.call_args.kwargs['local_recording_root'],prep.output/'collection-raw')
            self.assertFalse(remote.call_args.kwargs['still_current'](),'ended operations cannot authorize late replies')
            self.assertTrue(remote.call_args.kwargs['cancel']())
            self.assertEqual(json.loads((prep.output/'collection-binding.json').read_text()),{'robot_id':'A1-physical-test'})
            for manager in (parts.detector,parts.planner,parts.scene):manager.start.assert_called_once()
            self.assertIs(device.workflows['collection'],parts.workflow)
            self.assertTrue(self.managed(app,'prepare_collection')['reused']);remote.assert_called_once();construct.assert_called_once()
            self.assertEqual(device.runtime.holder,'collection')
            self.assertFalse(any(row[0] in ('begin','send') for row in device.runtime.owner.backend.events))
            device.connect.assert_not_called();device.submit.assert_not_called()
            install['head_id']='new-physical-head';calibration.on_calibration_installed(deepcopy(install))
            self.assertEqual(prep.snapshot()['status'],'awaiting_prepare')
            self.assertFalse(remote.call_args.kwargs['still_current']())
            remote.assert_called_once()

    def test_physical_late_remote_reply_cannot_publish_after_stop_or_install_change(self):
        for change in ('stop','manifest'):
            with self.subTest(change=change):
                # Each case has an independent task-owned run root.
                self.root=self.root/change;self.root.mkdir()
                prep,device,app,calibration,install,manifest=self.physical_preparation()
                def reply(kwargs):
                    self.assertTrue(kwargs['still_current']())
                    if change=='stop':self.maintenance(app,'stop')
                    else:manifest.write_text('{"changed":true}')
                    self.assertFalse(kwargs['still_current']());self.assertTrue(kwargs['cancel']())
                    return self.prepared_reply(kwargs)
                with patch('robot_workbench.collection_setup.prepare_remote_collection_process',side_effect=lambda **kw:reply(kw)) as remote, \
                        patch('robot_workbench.collection_setup.prepare_collection_attachment_process',side_effect=self.attachment_reply), \
                        patch('robot_workbench.collection_setup.construct_collection') as construct:
                    with self.assertRaises(ValueError):self.managed(app,'prepare_collection')
                    state=self.await_preparation(prep)
                    self.assertEqual(state['status'],'failed',state)
                    self.assertIn('not installed',state['error'])
                    self.assertFalse((prep.output/'collection-binding.json').exists())
                    construct.assert_not_called();device.connect.assert_not_called();device.submit.assert_not_called()
                prep.close()

    def test_shared_activation_stops_before_next_worker_after_revocation(self):
        prep,device,app,calibration,install,manifest=self.physical_preparation()
        parts=self.prepared_parts()
        start=parts.detector.start.side_effect
        def stop_after_start(*args,**kwargs):
            start(*args,**kwargs)
            self.maintenance(app,'stop')
        parts.detector.start.side_effect=stop_after_start
        with patch('robot_workbench.collection_setup.prepare_remote_collection_process',side_effect=lambda **kw:self.prepared_reply(kw)), \
                patch('robot_workbench.collection_setup.prepare_collection_attachment_process',side_effect=self.attachment_reply), \
                patch('robot_workbench.collection_setup.construct_collection',side_effect=self.attach_parts(parts)):
            with self.assertRaises(ValueError):self.managed(app,'prepare_collection')
            state=self.await_preparation(prep)
        self.assertEqual(state['status'],'failed',state)
        self.assertIn('撤销',state['error'])
        parts.detector.start.assert_called_once();parts.planner.start.assert_not_called();parts.scene.start.assert_not_called()
        parts.detector.close.assert_called_once()
        parts.planner.close.assert_not_called();parts.scene.close.assert_not_called()
        self.assertNotIn('collection',device.workflows)
        device.submit.assert_not_called();device.connect.assert_not_called()

    def test_attachment_validation_is_outside_install_locks_and_stop_revokes_publication(self):
        prep,device,app,calibration,install,manifest=self.physical_preparation()
        def validate(path,expected,**kwargs):
            self.assertFalse(app.lock._is_owned())
            self.assertFalse(device.workflows['calibration'].install_lock._is_owned())
            self.maintenance(app,'stop')
            return self.attachment_reply(path,expected,**kwargs)
        with patch('robot_workbench.collection_setup.prepare_remote_collection_process',side_effect=lambda **kw:self.prepared_reply(kw)),\
                patch('robot_workbench.collection_setup.prepare_collection_attachment_process',side_effect=validate),\
                patch('robot_workbench.collection_setup.construct_collection') as construct:
            with self.assertRaises(ValueError):self.managed(app,'prepare_collection')
            state=self.await_preparation(prep)
        self.assertEqual(state['status'],'failed',state)
        self.assertFalse((prep.output/'collection-binding.json').exists())
        construct.assert_not_called();device.submit.assert_not_called();device.connect.assert_not_called()

    def test_physical_remote_prepare_profile_is_explicit_and_preflight_never_contacts_compute(self):
        config=self.root/'calibration.json';config.write_text('{}')
        template=self.root/'template.json';template.write_text('{}')
        registration=self.root/'registration.json';registration.write_text('{}')
        self.profile.update(calibration_config=str(config),collection_setup=dict(registration=str(registration),
            profile_template=str(template),compute=dict(endpoint=dict(host='compute-test',port=22,key='/task/key'),
                deployment_path='/task/deployment.json',deployment_sha256='a'*64)))
        self.path.write_text(json.dumps(self.profile))
        with patch('subprocess.Popen',side_effect=AssertionError('no remote or sensor launch')):
            profile=read_profile(self.path);report=preflight(profile,net_root=self.net)
        self.assertTrue(report['configuration_ready']);self.assertTrue(report['businesses']['collection'])
        self.profile['collection_setup']['compute']['deployment_sha256']='unpinned'
        self.path.write_text(json.dumps(self.profile))
        with self.assertRaisesRegex(ValueError,'pinned remote'):read_profile(self.path)


if __name__=='__main__':unittest.main()
