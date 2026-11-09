"""CPU study buttons never connect hardware and use the actual shared runner."""
from pathlib import Path
from copy import deepcopy
import hashlib,io,json,sys,time,unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
import test_shared_target_routes as fixture
from robot_workbench.precision_study import PrecisionStudy
from robot_workbench.control_reference import SCHEMA
from robot_workbench.control_owner import ControlRuntimeClient
from robot_workbench.execution_settings import study_defaults,validate_study


class PrecisionStudyTest(unittest.TestCase):
    profile='x5_control50_damping_v1'
    setUp=fixture.SharedTargetRoutesTest.setUp
    cleanup=fixture.SharedTargetRoutesTest.cleanup
    create=fixture.SharedTargetRoutesTest.create
    advance=fixture.SharedTargetRoutesTest.advance

    def test_single_speed_deployment_does_not_offer_unqualified_references(self):
        study=self.study();config=deepcopy(study.config)
        config['references']={'1.5':config['references']['1.5']}
        selected=PrecisionStudy(study.device,config)
        state=selected.snapshot(study.device.snapshot('research'),'research')
        self.assertEqual(state['runtime_facts']['trajectory_speed_choices'],[1.5])
        with self.assertRaises(ValueError):selected.prepare('research','trajectory',1.)
        for refs in ({},{'3.0':config['references']['1.5']}):
            with self.assertRaises(ValueError):PrecisionStudy(study.device,dict(config,references=refs))

    def test_streamed_monitor_preserves_between_poll_peak_and_requires_fresh_terminal(self):
        import gzip
        study=self.study();clock=[1.];polls=[];event_calls=[];reference='a'*64
        review=dict(kind='trajectory',duration_s=2.,plan_sha256='plan',prepared_reference_id=reference,
                    binding=dict(runtime_nonce='cpu',command_epoch=1))
        directory=study.root/'record';directory.mkdir()
        study.latest=dict(id='record',summary='CPU',speed=.5,settings_revision=0,started_wall_ns=1,
                          minimum_reference_start_s=1.)
        def snapshot(business):
            self.assertFalse((directory/'samples.jsonl.gz').exists(),'disk output must wait for monitoring to finish')
            self.assertFalse((directory/'events.jsonl.gz').exists())
            polls.append(clock[0]);return dict(runtime_nonce='cpu',connected=True,state='READY',command_epoch=1,
                stationary=True,motion=dict(plan_sha256='plan',status='READY'if clock[0]>=1.3 else'RUNNING'))
        def events(*args,**kwargs):
            event_calls.append(clock[0]);index=len(event_calls)
            speed=.7 if index==6 else .1
            value=dict(reference_id=reference,reference_started_s=1.,raw=dict(sample_time_s=clock[0],
                sample_wall_ns=index+1,qvel=[speed]*14,qpos=[0.]*14))
            foreign=dict(value,reference_id='b'*64,raw=dict(value['raw'],qvel=[10.]*14))
            stale=dict(value,reference_started_s=.5,raw=dict(value['raw'],qvel=[20.]*14))
            rows=[dict(sequence=index*4+j,event='control_study_feedback',value=v)
                for j,v in enumerate((foreign,stale,value))]
            rows.append(dict(sequence=index*4+3,event='owner_loop_trace',value=dict(wall_s=.035)))
            return dict(incomplete=False,latest=index*4+3,events=rows)
        runtime=SimpleNamespace(nonce='cpu',channel=SimpleNamespace(call=events))
        study.device.runtime=runtime;study.device.snapshot=snapshot
        study.device.request_stop=Mock()
        with patch('robot_workbench.precision_study.time.monotonic',side_effect=lambda:clock[0]), \
                patch('robot_workbench.precision_study.time.sleep',side_effect=lambda seconds:clock.__setitem__(0,clock[0]+seconds)):
            study._monitor(runtime,'research',review,directory,0)
        self.assertEqual(study.latest['status'],'completed');self.assertEqual(study.latest['feedback_peak_rad_s'],.7)
        self.assertEqual(study.latest['feedback_samples'],len(event_calls))
        self.assertLess(len(polls),len(event_calls)/5)
        self.assertTrue(all(b-a>=.1-1e-9 for a,b in zip(polls,polls[1:])))
        self.assertEqual(polls[-1],event_calls[-1],'terminal decision must use the current full snapshot')
        rows=[json.loads(line)for line in gzip.open(directory/'samples.jsonl.gz','rt')]
        self.assertEqual(len(rows),len(event_calls));self.assertTrue(all(r['reference_id']==reference for r in rows))
        events=[json.loads(line)for line in gzip.open(directory/'events.jsonl.gz','rt')]
        self.assertEqual(sum(r['event']=='owner_loop_trace'for r in events),len(event_calls))
        study.device.request_stop.assert_not_called()

    def test_numeric_buffer_budget_is_shared_and_failure_requests_stop(self):
        from robot_workbench.precision_study import _NumericBuffer
        budget=[0];first=_NumericBuffer(budget);second=_NumericBuffer(budget)
        try:
            with patch('robot_workbench.precision_study.NUMERIC_MEMORY_LIMIT',32):
                first.write(b'a'*24)
                with self.assertRaisesRegex(OSError,'budget exceeded'):second.write(b'b'*9)
                self.assertEqual(budget,[24]);self.assertEqual(second.getvalue(),b'')
        finally:first.close();second.close()
        study=self.study();directory=study.root/'budget';directory.mkdir()
        runtime=SimpleNamespace(nonce='cpu',channel=SimpleNamespace(call=Mock()))
        study.device.runtime=runtime;study.device.request_stop=Mock()
        study.latest=dict(id='budget',summary='CPU',speed=.5,settings_revision=0,started_wall_ns=1,minimum_reference_start_s=1.)
        review=dict(kind='trajectory',duration_s=2.,prepared_reference_id='a'*64,binding=dict(runtime_nonce='cpu'))
        with patch('robot_workbench.precision_study.NUMERIC_MEMORY_LIMIT',0):
            study._monitor(runtime,'research',review,directory,0)
        self.assertEqual(study.latest['status'],'result_unknown')
        self.assertIn('budget exceeded',study.latest['reason'])
        study.device.request_stop.assert_called_once()

    def test_streamed_event_loss_keeps_result_unconfirmed_and_requests_stop(self):
        study=self.study();directory=study.root/'missing';directory.mkdir()
        runtime=SimpleNamespace(nonce='cpu',channel=SimpleNamespace(call=Mock(return_value=dict(incomplete=True))))
        study.device.runtime=runtime;study.device.request_stop=Mock()
        study.device.snapshot=lambda _:dict(runtime_nonce='cpu')
        study.latest=dict(id='missing',summary='CPU',speed=.5,settings_revision=0,started_wall_ns=1,minimum_reference_start_s=1.)
        review=dict(kind='trajectory',duration_s=2.,prepared_reference_id='a'*64,binding=dict(runtime_nonce='cpu'))
        study._monitor(runtime,'research',review,directory,0)
        self.assertEqual(study.latest['status'],'result_unknown');self.assertIn('事件丢失',study.latest['reason'])
        study.device.request_stop.assert_called_once()

    def study(self):
        self.create();self.creds['research']=self.runtime.register_business('research');self.runtime.handoff(self.creds['calibration'],'research');root=Path(self.temp.name)
        from arx_env.joint_head_real_motion import feedback_to_command
        start=feedback_to_command(self.runtime.owner.cache['calibration_feedback']['qpos'],self.runtime.owner.cache['joint_limits'])
        refs={}
        for speed in(.5,1.,1.5,2.):
            b=np.asarray([0.,1.,2.]);c=np.zeros((2,4,12));c[0,:,0]=[-.08,.12,0.,0.];c[1,:,0]=[.08,-.12,0.,.04]
            jaws=np.tile(np.asarray(start)[[6,13]],(3,1))
            m=dict(schema='prepared_motion_reference.v1',study_schema=SCHEMA,source_range_s=[0.,50.],
                speed_multiplier=speed,duration_s=2.,event_times_s=[0.,2.])
            h=hashlib.sha256(json.dumps(m,sort_keys=True).encode())
            for array in(b,c,jaws):h.update(array.tobytes())
            m['reference_id']=h.hexdigest();f=io.BytesIO();np.savez(f,breaks=b,coefficients=c,jaws=jaws,metadata=json.dumps(m))
            file=root/f'{speed}.npz';file.write_bytes(f.getvalue());refs[str(speed)]=dict(path=str(file),sha256=hashlib.sha256(f.getvalue()).hexdigest(),size_bytes=len(f.getvalue()))
        device=SimpleNamespace(runtime=self.runtime,workflows={},snapshot=lambda business:self.runtime.snapshot(self.creds[business]),
            submit=lambda business,command:self.runtime.submit(self.creds[business],command),request_stop=self.runtime.request_stop)
        return PrecisionStudy(device,dict(schema='workbench.precision_study.v1',controller_profile=self.profile,root=str(root/'study'),references=refs))

    def test_prepare_execute_consume_and_stop_preserve_numeric_result(self):
        study=self.study();before=len(self.raw.packets)
        study.prepare('research','trajectory',1.)
        self.assertEqual(len(self.raw.packets),before,'preview cannot execute or initialize hardware')
        state=study.snapshot(study.device.snapshot('research'),'research');self.assertTrue(state['execute_allowed'])
        review=state['review']['id'];study.execute('research',review)
        with self.assertRaises(ValueError):study.execute('research',review)
        self.advance(50);self.runtime.request_stop();self.advance(50)
        study.thread.join(3);self.assertFalse(study.thread.is_alive())
        self.assertEqual(study.latest['status'],'stopped');self.assertIsNone(self.raw.window)
        self.assertEqual(study.latest['measurement_contract'],'spatial_path.v2')
        restored=PrecisionStudy(study.device,study.config)
        self.assertEqual(restored.latest['id'],study.latest['id']);self.assertEqual(restored.latest['camera_calls'],0)
        self.assertIsNone(restored.review);self.assertFalse(restored.snapshot(study.device.snapshot('research'),'research')['execute_allowed'])

    def test_explicit_feedforward_binds_configuration_and_stop_cancels_both(self):
        study=self.study();artifact=study.config['references']['1.0']
        with np.load(artifact['path'])as f:reference=json.loads(str(f['metadata']))['reference_id']
        ff=dict(schema='reference_feedforward.v1',source_sha256=reference,controller_profile=self.profile,
            phase=[0.,1.],torque_bias_sdk6=[[.05]*6]*2,
            provenance=dict(study_settings=study.settings,speed_multiplier=1.))
        raw=json.dumps(ff).encode();digest=hashlib.sha256(raw).hexdigest()
        file=Path(self.temp.name)/(digest+'.json');file.write_bytes(raw)
        pin=dict(path=str(file),size_bytes=len(raw),sha256=digest)
        for kind,speed in (('home',1.),('start',1.),('trajectory',.5)):
            with self.subTest(kind=kind,speed=speed),self.assertRaises(ValueError):study.prepare('research',kind,speed,pin)
        with self.assertRaisesRegex(ValueError,'SHA256'):study.prepare('research','trajectory',1.,dict(pin,sha256='0'*64))
        settings=json.loads(json.dumps(study.settings));settings['gains']['left']['kp'][0]-=1
        study.update_settings(settings,study.revision)
        with self.assertRaisesRegex(ValueError,'增益'):study.prepare('research','trajectory',1.,pin)
        study.update_settings(ff['provenance']['study_settings'],study.revision)
        self.raw.native_reference_api='preserve_reference_v1';self.advance(1)
        with self.assertRaisesRegex(ValueError,'原生执行接口'):study.prepare('research','trajectory',1.,pin)
        self.raw.native_reference_api='legacy_recompute';self.advance(1)
        before=len(self.raw.packets);study.prepare('research','trajectory',1.,pin)
        self.assertEqual(len(self.raw.packets),before)
        view=study.snapshot(study.device.snapshot('research'),'research')['review']
        self.assertEqual(view['feedforward_artifact'],pin);self.assertIn('前馈',view['summary'])
        study.execute('research',view['id']);self.advance(100)
        interpreter=self.runtime.owner.runner.target_interpreter
        self.assertIsNotNone(interpreter.feedforward);self.assertTrue(interpreter.integrals)
        self.runtime.request_stop();self.advance(50);study.thread.join(3)
        self.assertFalse(study.thread.is_alive());self.assertTrue(interpreter.cancelled)
        self.assertIsNone(self.raw.window);self.assertIsNone(interpreter.feedforward)
        self.assertEqual(study.latest['feedforward_artifact'],pin)
        self.assertEqual(study.latest['native_reference_api'],'legacy_recompute')

    def test_feedforward_only_full_review_execution_and_stop(self):
        study=self.study();study.update_settings(dict(study.settings,integral_enabled=False),study.revision)
        artifact=study.config['references']['1.0']
        with np.load(artifact['path'])as f:reference=json.loads(str(f['metadata']))['reference_id']
        value=dict(schema='reference_feedforward.v1',source_sha256=reference,controller_profile=self.profile,
            phase=[0.,1.],torque_bias_sdk6=[[.05]*6]*2,
            provenance=dict(study_settings=study.settings,speed_multiplier=1.))
        raw=json.dumps(value).encode();digest=hashlib.sha256(raw).hexdigest()
        path=study.root/(digest+'.json');path.write_bytes(raw)
        pin=dict(path=str(path),size_bytes=len(raw),sha256=digest)
        study.prepare('research','trajectory',1.,pin)
        self.assertIn('无残差积分',study.review['summary'])
        study.execute('research',study.review['id']);self.advance(100)
        interpreter=self.runtime.owner.runner.target_interpreter
        self.assertFalse(interpreter.integrals)
        self.assertGreater(np.max(np.asarray(interpreter.last_packet['torque_bias_sdk14'])[:,:6]),.01)
        self.runtime.request_stop();self.advance(50);study.thread.join(3)
        self.assertTrue(interpreter.cancelled);self.assertIsNone(interpreter.feedforward)
        self.assertIsNone(self.raw.window);self.assertFalse(study.thread.is_alive())

    def test_guard_profile_holder_settings_and_bad_reference_are_checked(self):
        study=self.study();s=study.device.snapshot('research')
        self.assertFalse(study.snapshot(s,'collection')['prepare_allowed'])
        with self.assertRaises(ValueError):study.prepare('research','trajectory',True)
        with self.assertRaises(ValueError):study.prepare('research','shell',1.)
        study.prepare('research','home',.5);review=study.review['id']
        from robot_workbench.execution_settings import defaults
        policy=defaults();policy['speed_protection']['enabled']=False
        self.runtime.update_execution_settings(self.creds['research'],policy)
        with self.assertRaisesRegex(ValueError,'峰值保护开启且正在监测'):study.execute('research',review)
        policy['speed_protection']['enabled']=True;self.runtime.update_execution_settings(self.creds['research'],policy)
        with self.assertRaisesRegex(ValueError,'失效'):study.execute('research',review)
        artifact=study.config['references']['1.0'];Path(artifact['path']).write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'artifact changed'):study.prepare('research','trajectory',1.)

    def test_control_config_catalog_uses_actual_profiles_without_applying_gains(self):
        from robot_workbench.target_interpreter import gains
        study=self.study();before=len(self.raw.packets)
        rows=study.snapshot(study.device.snapshot('research'),'research')['settings_presets']
        self.assertEqual([r['id']for r in rows],['base-v1','v8','v10'])
        for side in ('left','right'):
            self.assertEqual(rows[0]['settings']['gains'][side],dict(kp=[80.,70.,70.,30.,30.,20.],kd=[2.,2.,2.,1.,1.,.7]))
            self.assertEqual(rows[0]['settings']['integral_gains'][side],[0.]*6)
        rows=rows[1:]
        for row,profile in zip(rows,('x5_control50_pd30_feedback50_v1','x5_control50_damping_v1')):
            self.assertEqual(row['source_profile'],profile)
            self.assertEqual(validate_study(row['settings']),row['settings'])
            for side in('left','right'):
                self.assertEqual(row['settings']['gains'][side],dict(zip(('kp','kd'),gains(profile,side))))
        study.update_settings(rows[0]['settings'],0)
        self.assertEqual(study.snapshot(study.device.snapshot('research'),'research')['settings'],rows[0]['settings'])
        self.assertEqual(len(self.raw.packets),before)
        self.assertIsNone(getattr(self.raw,'study_gain_override',None))

    def test_research_caps_include_approved_all_joint_p400_without_changing_defaults(self):
        study=self.study();snapshot=study.snapshot(study.device.snapshot('research'),'research')
        self.assertEqual(snapshot['settings']['gains']['left']['kp'][2],210.)
        self.assertEqual(snapshot['settings_caps']['gains']['left']['kp'][2],400.)
        for kp in (180.,210.,240.,270.,330.,400.):
            settings=study_defaults();settings['gains']['left']['kp'][2]=kp
            self.assertEqual(validate_study(settings),settings)
        for kp in (100.,150.,200.,300.,400.,490.):
            settings=study_defaults();settings['gains']['left']['kp'][1]=kp
            self.assertEqual(validate_study(settings),settings)
        self.assertEqual(study_defaults()['gains']['left']['kp'][1],210.)
        for side,name,joint,value in [('left','kp',2,400.01),('left','kp',1,490.01),
                                      ('right','kp',2,400.01),('left','kd',2,5.01)]:
            settings=study_defaults();settings['gains'][side][name][joint]=value
            with self.subTest(side=side,name=name,joint=joint),self.assertRaises(ValueError):
                validate_study(settings)
        self.assertEqual(study_defaults()['gains']['left']['kp'][2],210.)
        settings=study_defaults()
        settings['gains']={side:dict(kp=[400.]*6,kd=[0.]*6)for side in ('left','right')}
        settings['integral_gains']={side:[0.]*6 for side in ('left','right')}
        self.assertEqual(validate_study(settings),settings)

    def test_zero_research_damping_is_valid_but_zero_stiffness_is_not(self):
        settings=study_defaults();settings['gains']['left']['kp'][1]=400.
        settings['gains']['left']['kd'][1]=0.
        self.assertEqual(validate_study(settings),settings)
        for name,value in [('kd',-.001),('kp',0.),('kd',float('nan')),('kd',True)]:
            bad=deepcopy(settings);bad['gains']['left'][name][1]=value
            with self.subTest(name=name,value=value),self.assertRaises(ValueError):validate_study(bad)

    def test_saved_settings_freeze_gain_and_integral_application_and_notes(self):
        study=self.study();settings=study_defaults();settings['gains']['left']['kp'][3]=70.;settings['integral_enabled']=False
        study.prepare('research','home',.5);old=study.review['id']
        study.update_settings(settings,0)
        self.assertIsNone(study.review);self.assertIsNone(getattr(self.raw,'study_gain_override',None),'saving never changes hardware')
        restored=PrecisionStudy(study.device,study.config);self.assertEqual(restored.settings,settings);self.assertEqual(restored.revision,1)
        study.prepare('research','trajectory',1.);review=study.review['id'];study.execute('research',review)
        self.advance(100);self.assertEqual(self.raw.study_gain_override['left']['kp'][3],70.)
        self.assertFalse(self.runtime.owner.runner.target_interpreter.integrals)
        self.assertEqual(self.runtime.execution_calibration['study_settings'],settings)
        self.runtime.request_stop();self.advance(50);study.thread.join(3)
        identity=study.latest['id'];study.notes(identity,'先降低J4刚度对照，未作精度通过',0)
        restored=PrecisionStudy(study.device,study.config);restored.select(identity)
        self.assertIn('降低J4',restored.selected['notes']);self.assertEqual(restored.selected['settings'],settings)
        with self.assertRaisesRegex(ValueError,'结论版本'):study.notes(identity,'old revision',0)
        with self.assertRaises(ValueError):study.select('../outside')
        def analysis_reply(*args,**kwargs):
            study.notes(identity,'分析期间新补充的结论',1)
            return SimpleNamespace(returncode=0,stdout=json.dumps({'passed':False}),stderr='')
        with patch('subprocess.run',side_effect=analysis_reply):study.analyze()
        self.assertEqual(study.latest['notes'],'分析期间新补充的结论','analysis cannot overwrite a newer note')
        for value in (0,-1,float('nan'),float('inf'),True,400.01):
            bad=study_defaults();bad['gains']['left']['kp'][3]=value
            with self.subTest(value=value),self.assertRaises(ValueError):validate_study(bad)

    def test_gain_changes_revoke_previous_hold_and_ordinary_motion_resets_override(self):
        study=self.study();state=study.device.snapshot('research');command=dict(action='preview_workbench',
            waypoints=[dict(q12=[0.]*12,speeds12=[.1]*12)],study_settings=study_defaults())
        review=study.device.submit('research',command)
        fixture.SharedTargetRoutesTest.confirm(self,'research',review);self.advance(350)
        previous=self.runtime.owner.runner.target_interpreter
        self.assertFalse(previous.cancelled)
        custom=study_defaults();custom['gains']['left']['kp'][2]=400.;command['study_settings']=custom
        review=study.device.submit('research',command);fixture.SharedTargetRoutesTest.confirm(self,'research',review);self.advance(350)
        self.assertTrue(previous.cancelled);self.assertEqual(self.raw.study_gain_override['left']['kp'][2],400.)
        command.pop('study_settings');review=study.device.submit('research',command)
        fixture.SharedTargetRoutesTest.confirm(self,'research',review);self.advance(50)
        self.assertIsNone(self.raw.study_gain_override,'ordinary commands cannot inherit study gains')


class RetirementLeaseTest(unittest.TestCase):
    def test_success_consumes_lease_but_permission_error_does_not(self):
        client=object.__new__(ControlRuntimeClient);client.lease={'token':'cpu'};client.fd=1
        client._call=Mock(return_value={'worker_exit_confirmed':True});client.refresh=Mock()
        result=client.retire_protected();self.assertTrue(result['worker_exit_confirmed']);self.assertIsNone(client.lease)
        client._call.reset_mock();client.end_maintenance();client._call.assert_not_called()
        client.lease={'token':'cpu'};client._call=Mock(side_effect=PermissionError('wrong capability'))
        with self.assertRaises(PermissionError):client.retire_protected()
        self.assertIsNotNone(client.lease,'cannot swallow arbitrary authorization failures')


class NumericStudyAnalysisTest(unittest.TestCase):
    def test_spatial_contract_allows_delay_and_preserves_legacy_time_verdict(self):
        import gzip,tempfile
        from robot_workbench.precision_study import analyze
        with tempfile.TemporaryDirectory()as temp:
            root=Path(temp);breaks=np.array([0.,2.]);c=np.zeros((1,4,12));c[0,:,0]=[-.01,.03,0.,0.]
            jaws=np.zeros((2,2));meta=dict(schema='prepared_motion_reference.v1',study_schema=SCHEMA,
                duration_s=2.,source_range_s=[0.,50.],speed_multiplier=1.,event_times_s=[0.,2.])
            h=hashlib.sha256(json.dumps(meta,sort_keys=True).encode())
            for array in(breaks,c,jaws):h.update(array.tobytes())
            meta['reference_id']=h.hexdigest();ref=root/'reference.npz'
            np.savez(ref,breaks=breaks,coefficients=c,jaws=jaws,metadata=json.dumps(meta))
            config=dict(urdf=str(root/'model.urdf'),zero_offsets_rad=[0.]*12,tcp_m=[0.]*3)
            Path(config['urdf']).write_text('mock geometry');config['urdf_sha256']=hashlib.sha256(b'mock geometry').hexdigest()
            original=np.zeros((101,2,3));original[:,0,0]=np.linspace(0.,.04,101)
            geom=root/'geometry.json';geom.write_text(json.dumps(dict(schema='workbench.research_path_geometry.v1',
                paired_tcp_m=original.tolist(),source_range_s=[0.,50.],**{k:config[k]for k in('urdf_sha256','zero_offsets_rad','tcp_m')})))
            pin=lambda p:dict(path=str(p),size_bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
            result=dict(status='completed',kind='trajectory',feedback_peak_rad_s=.1,reference_artifact=pin(ref),
                measurement_contract='spatial_path.v2',source_path_geometry=pin(geom),scoring_model=config)
            with gzip.open(root/'events.jsonl.gz','wt')as stream:
                stream.write(json.dumps(dict(event='control_study_feedback',value=dict(reference_started_s=None,
                    raw=dict(sample_time_s=99.99,qpos=[0.]*14))))+'\n')
                for t in np.arange(0.,2.21,.01):
                    u=np.clip((t-.1)/2.,0.,1.);q=np.zeros(14);q[0]=.04*u*u*(3.-2.*u)
                    stream.write(json.dumps(dict(event='control_study_feedback',value=dict(reference_started_s=100.,
                        raw=dict(sample_time_s=100.+t,qpos=q.tolist()))))+'\n')
            def frames(q):
                left=np.eye(4);left[0,3]=q[0]
                return dict(left_link6=left,right_link16=np.eye(4))
            with patch('arx_env.joint_head_isaac_fit.Model')as model:
                model.return_value.frames.side_effect=frames
                (root/'result.json').write_text(json.dumps(result))
                report=analyze(root,config)
                self.assertTrue(report['precision_passed']);self.assertFalse(report['same_time_precision_passed'])
                self.assertIsNone(report['smoothness_accepted'])
                result.pop('measurement_contract');(root/'result.json').write_text(json.dumps(result))
                legacy=analyze(root,config);self.assertFalse(legacy['passed'])
                self.assertEqual(legacy['measurement_contract'],'same_time.v1')
                result['measurement_contract']='spatial_path.v2';result.pop('source_path_geometry')
                (root/'result.json').write_text(json.dumps(result))
                self.assertFalse(analyze(root,config)['passed'],'cannot silently substitute a smoothed path for the original')

    def test_owner_trip_is_in_numeric_peak_even_if_business_sampler_misses_it(self):
        import tempfile
        for epoch,expected in [(3,.824177),(2,.1)]:
            with self.subTest(epoch=epoch),tempfile.TemporaryDirectory()as temp:
                root=Path(temp);runtime=SimpleNamespace(nonce='cpu');study=object.__new__(PrecisionStudy)
                study.lock=__import__('threading').RLock();study.root=root;study.config={};study.history=[]
                study.latest=dict(id='a'*24,status='submitted',summary='cpu',speed=.5,settings_revision=1,started_wall_ns=100)
                state=dict(runtime_nonce='cpu',connected=True,state='FAULT_STOP_UNCONFIRMED',stationary=True,
                    command_epoch=4,calibration_feedback=dict(sample_time_s=1.,qvel=[.1]*14),
                    speed_protection=dict(last_trip=dict(command_epoch=epoch,sample_wall_ns=101,peak_rad_s=.824177)))
                study.device=SimpleNamespace(runtime=runtime,snapshot=lambda business:state,request_stop=Mock())
                review=dict(duration_s=.1,plan_sha256='none',binding=dict(runtime_nonce='cpu',command_epoch=3))
                study._monitor(runtime,'research',review,root,None)
                self.assertAlmostEqual(study.latest['feedback_peak_rad_s'],expected)


if __name__=='__main__':unittest.main()
