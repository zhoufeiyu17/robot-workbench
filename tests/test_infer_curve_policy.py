"""Explicit infer-only curve policy, whole-path extrema and physical derivatives."""
from copy import deepcopy
import unittest
import numpy as np
from robot_workbench.motion_reference import prepare_reference,_extrema,_extrema_batched,_check_c2

POLICY=dict(schema='workbench.infer-reference-policy.v1',curve='cubic_c2_quintic_ends',
    velocity_limit_rad_s=1.,acceleration_limit_rad_s2=40.,jaw_velocity_limit_mm_s=40.,
    boundary_scope='complete_prepared_segment',timing_policy='retime_allowed',max_execution_duration_s=2.)


class InferCurvePolicyTests(unittest.TestCase):
    def fixture(self,duration=.5):
        q=np.zeros((16,14));q[:,0]=np.linspace(0,.1,16)
        limits=np.tile([-1.,1.],(14,1));limits[[6,13]]=[0,.082]
        return dict(kind='online_prediction_chunk',device_targets14=q.tolist(),duration_s=duration,active_arms=['left','right']),limits

    def prepare(self,source,limits,**options):
        kwargs=dict(source_sha256='a'*64,planning_policy=POLICY,
            initial_velocity_sdk14=[0.]*14,initial_acceleration_sdk14=[0.]*14,
            sdk_targets14=source['device_targets14'],jaw_scales={'left':1000.,'right':1000.})
        kwargs.update(options)
        return prepare_reference(source,source['device_targets14'][0],limits,**kwargs)

    def test_resident_planner_matches_fresh_process_reuses_worker_and_preserves_rejection(self):
        import os
        from robot_workbench.motion_reference import create_reference_executor,prepare_reference_process
        source,limits=self.fixture();initial=source['device_targets14'][0]
        options=dict(source_sha256='a'*64,planning_policy=POLICY,
            initial_velocity_sdk14=[0.]*14,initial_acceleration_sdk14=[0.]*14,
            sdk_targets14=source['device_targets14'],jaw_scales={'left':1000.,'right':1000.})
        cold={};expected=prepare_reference_process(source,initial,limits.tolist(),timing=cold,**options)
        executor=create_reference_executor()
        try:
            pid=executor.submit(os.getpid).result(5);self.assertNotEqual(pid,os.getpid())
            for _ in range(2):
                measured={};actual=prepare_reference_process(source,initial,limits.tolist(),executor=executor,timing=measured,**options)
                self.assertEqual(actual.metadata,expected.metadata)
                for name in ('breaks','coefficients','jaws'):np.testing.assert_array_equal(getattr(actual,name),getattr(expected,name))
                self.assertEqual(measured['worker_pid'],pid)
                for row in (cold,measured):
                    times=[row['worker_'+name+'_ns']for name in ('started','decoded','prepared','finished')]
                    self.assertEqual(times,sorted(times));self.assertGreater(row['worker_total_s'],0)
                    self.assertGreater(row['fit_s'],0);self.assertGreater(row['extrema_s'],0)
            invalid=deepcopy(options);invalid['sdk_targets14']=deepcopy(source['device_targets14'])
            invalid['sdk_targets14'][1][0]=2.
            for pool in (None,executor):
                with self.assertRaisesRegex(ValueError,'source outside SDK command limits'):
                    prepare_reference_process(source,initial,limits.tolist(),executor=pool,**invalid)
            self.assertEqual(executor.submit(os.getpid).result(5),pid)
        finally:executor.shutdown(wait=True,cancel_futures=True)
        with self.assertRaises(RuntimeError):executor.submit(os.getpid)

    def test_local_jaw_retiming_preserves_all_other_interval_durations(self):
        source,limits=self.fixture(32/30)
        q=np.zeros((33,14));q[16:,6]=.04;source['device_targets14']=q.tolist()
        reference=self.prepare(source,limits,planning_policy=dict(POLICY,max_execution_duration_s=None))
        widths=np.diff(reference.breaks);normal=np.delete(widths,15)
        np.testing.assert_allclose(normal,1/30,rtol=0,atol=1e-14)
        self.assertAlmostEqual(widths[15],1.02)
        self.assertEqual([row['interval']for row in reference.metadata['local_retiming']],[15])
        self.assertEqual(reference.metadata['local_retiming'][0]['constraints'],['left_jaw'])
        self.assertLessEqual(reference.metadata['max_jaw_velocity_mm_s'],40.)
        _check_c2(reference.coefficients,reference.breaks)

    def test_local_joint_velocity_change_preserves_remote_safe_intervals(self):
        source,limits=self.fixture(32/30)
        q=np.zeros((33,14));q[:,0]=np.r_[np.zeros(12),np.linspace(0,.1,9),np.full(12,.1)]
        source['device_targets14']=q.tolist()
        reference=self.prepare(source,limits,planning_policy=dict(POLICY,
            velocity_limit_rad_s=.1,max_execution_duration_s=None))
        widths=np.diff(reference.breaks)
        np.testing.assert_allclose(widths[:5],1/30,atol=1e-14,rtol=0)
        np.testing.assert_allclose(widths[-5:],1/30,atol=1e-14,rtol=0)
        self.assertTrue(np.any(widths[12:20]>1/30))
        self.assertLessEqual(reference.metadata['max_velocity_rad_s'],.1+1e-9)
        self.assertLessEqual(reference.metadata['max_acceleration_rad_s2'],40.+1e-9)
        _check_c2(reference.coefficients,reference.breaks)

    def test_local_brake_reserve_keeps_safe_prefix_timing_and_all_knots(self):
        from robot_workbench.motion_reference import retime_for_brake_reserve,check_brake_reserve
        source,limits=self.fixture(32/30)
        q=np.zeros((33,14));q[:,1]=np.linspace(.23,-.049,33);q[:,[6,13]]=.04
        source['device_targets14']=q.tolist();limits[1]=[-.05,3.5]
        policy=dict(POLICY,velocity_limit_rad_s=4.,jaw_velocity_limit_mm_s=60.,max_execution_duration_s=None)
        reference=self.prepare(source,limits,planning_policy=policy)
        result=retime_for_brake_reserve(reference,limits,duration_s=.2,velocity_limit=4.,
            acceleration_limit=40.,maximum_duration_s=4120.)
        np.testing.assert_array_equal(result.breaks[:28],reference.breaks[:28])
        np.testing.assert_array_equal(result.coefficients[:26],reference.coefficients[:26])
        self.assertLess(result.duration_s,3.)
        self.assertEqual([row['interval']for row in result.metadata['normal_brake_local_retiming']],[27,28,29,30,31])
        for chunk in np.array_split(result.breaks,2):
            positions,_,_=result.sample(chunk)
            for stamp,position in zip(chunk,positions):
                index=int(np.flatnonzero(result.breaks==stamp)[0]);np.testing.assert_allclose(position,q[index],atol=1e-12)
        _check_c2(result.coefficients,result.breaks)
        check_brake_reserve(result,limits,duration_s=.2,velocity_limit=4.,acceleration_limit=40.)
        # The run budget may admit or reject a curve; it cannot set its speed.
        other=retime_for_brake_reserve(reference,limits,duration_s=.2,velocity_limit=4.,
            acceleration_limit=40.,maximum_duration_s=10.)
        np.testing.assert_array_equal(result.breaks,other.breaks)
        np.testing.assert_array_equal(result.coefficients,other.coefficients)

    def test_per_interval_extrema_reduce_to_existing_complete_curve_proof(self):
        source,limits=self.fixture();r=self.prepare(source,limits)
        local=_extrema_batched(r.coefficients,r.breaks,per_interval=True)
        complete=_extrema_batched(r.coefficients,r.breaks)
        reduced=[local[0].min(axis=0),local[1].max(axis=0),local[2].max(axis=0),local[3].max(axis=0)]
        for actual,expected in zip(reduced,complete):np.testing.assert_array_equal(actual,expected)

    def test_internal_derivatives_are_not_forced_to_zero_and_only_outer_ends_stop(self):
        source,limits=self.fixture();r=self.prepare(source,limits)
        q,v,a=r.sample(r.breaks)
        self.assertGreater(max(abs(v[1:-1,0])),.1);self.assertGreater(max(abs(a[1:-1,0])),.1)
        np.testing.assert_allclose(v[[0,-1]],0,atol=1e-10);np.testing.assert_allclose(a[[0,-1]],0,atol=1e-10)
        np.testing.assert_allclose(q,np.asarray(source['device_targets14']),atol=1e-12)
        _check_c2(r.coefficients,r.breaks)
        self.assertEqual(r.metadata['velocity_limit_rad_s'],1.);self.assertEqual(r.metadata['acceleration_limit_rad_s2'],40.)

    def test_retiming_preserves_nonzero_start_velocity_and_acceleration(self):
        source,limits=self.fixture(.05);v=np.zeros(14);a=np.zeros(14);v[0]=.2;a[0]=-.5
        r=self.prepare(source,limits,initial_velocity_sdk14=v,initial_acceleration_sdk14=a)
        self.assertGreater(r.duration_s,.05);self.assertGreater(r.metadata['retiming_refits'],0)
        q,dq,ddq=r.sample([0,r.duration_s]);np.testing.assert_allclose(dq[0],v,atol=1e-10);np.testing.assert_allclose(ddq[0],a,atol=1e-10)
        np.testing.assert_allclose(dq[-1],0,atol=1e-10);np.testing.assert_allclose(ddq[-1],0,atol=1e-9)
        _,_,pv,pa=_extrema(r.coefficients,r.breaks);self.assertLessEqual(max(pv),1+1e-9);self.assertLessEqual(max(pa),40+1e-9)
        with self.assertRaises(ValueError):self.prepare(source,limits,initial_velocity_sdk14=v,initial_acceleration_sdk14=a,timing_policy='strict_duration')

    def test_batched_full_extrema_match_scalar_and_cover_interior_roots(self):
        rng=np.random.default_rng(20261103)
        for degree in (3,4,5):
            widths=rng.uniform(.01,.3,8);b=np.r_[0,np.cumsum(widths)]
            c=rng.normal(size=(8,degree+1,12))*.02/widths[:,None,None]**np.arange(degree,-1,-1)[None,:,None]
            c[0,:,0]=0;c[1,:-2,1]=0
            for left,right in zip(_extrema(c,b),_extrema_batched(c,b)):
                np.testing.assert_allclose(left,right,rtol=1e-10,atol=1e-8)
        c=np.zeros((1,4,12));c[0,:,0]=[0,-4,4,0]
        lo,hi,_,_=_extrema_batched(c,[0,1]);self.assertAlmostEqual(hi[0],1.);self.assertAlmostEqual(lo[0],0.)

    def test_full_chunk_brakes_preserve_prefix_and_all_polynomial_bounds(self):
        from robot_workbench.motion_reference import prepare_brake_after,JOINTS
        source,limits=self.fixture(32/30)
        q=np.zeros((33,14));t=np.linspace(0,1,33)
        q[:,list(JOINTS)]=.01*np.sin(t[:,None]*np.arange(1,13)[None,:])
        q[:,[6,13]]=[.03,.04];source['device_targets14']=q.tolist()
        reference=self.prepare(source,limits,planning_policy=dict(POLICY,max_execution_duration_s=32/30*4))
        for fraction in (.1,.5,.9):
            boundary=reference.duration_s*fraction
            brake=prepare_brake_after(reference,boundary,limits,duration_s=.2,velocity_limit=1.,acceleration_limit=40.)
            for times in np.array_split(np.linspace(0,boundary,97),4):
                for old,new in zip(reference.sample(times),brake.sample(times)):
                    np.testing.assert_allclose(old,new,rtol=1e-10,atol=1e-10)
            scalar=_extrema(brake.coefficients,brake.breaks)
            for old,new in zip(scalar,_extrema_batched(brake.coefficients,brake.breaks)):
                np.testing.assert_allclose(old,new,rtol=1e-10,atol=1e-8)
            self.assertAlmostEqual(brake.metadata['max_velocity_rad_s'],max(scalar[2]),places=9)
            self.assertAlmostEqual(brake.metadata['max_acceleration_rad_s2'],max(scalar[3]),places=9)
            _check_c2(brake.coefficients,brake.breaks)
            narrow=limits.copy();narrow[0,1]=scalar[1][0]-1e-6
            with self.assertRaises(ValueError):
                prepare_brake_after(reference,boundary,narrow,duration_s=.2,velocity_limit=1.,acceleration_limit=40.)

    def test_overshoot_and_unknown_boundary_reject_instead_of_clipping_or_zeroing(self):
        source,limits=self.fixture();source['device_targets14'][1][0]=-.01936916448175907
        source['device_targets14'][0][0]=.00171661376953125
        for row in source['device_targets14'][2:]:row[0]=-.007
        limits[0]=[-.0195,1.]
        with self.assertRaisesRegex(ValueError,'joint command limits'):self.prepare(source,limits)
        source,limits=self.fixture()
        with self.assertRaisesRegex(ValueError,'start q/v/a'):self.prepare(source,limits,initial_velocity_sdk14=None)
        invalid=deepcopy(POLICY);invalid['jaw_velocity_limit_mm_s']=80.
        self.assertEqual(self.prepare(source,limits,planning_policy=invalid).metadata['planning_policy']['jaw_velocity_limit_mm_s'],80.)
        invalid['jaw_velocity_limit_mm_s']=-1.
        with self.assertRaises(ValueError):self.prepare(source,limits,planning_policy=invalid)

    def test_legacy_default_is_unchanged_and_profile_is_scoped_and_frozen(self):
        from robot_workbench.inference_owner import configuration
        from robot_workbench.inference_model import MIXED366_ID,BASE566_ID
        source,limits=self.fixture()
        old=prepare_reference(source,source['device_targets14'][0],limits,source_sha256='a'*64)
        self.assertEqual(old.metadata['velocity_limit_rad_s'],.25);self.assertEqual(old.metadata['acceleration_limit_rad_s2'],1.)
        self.assertNotIn('planning_policy',old.metadata)
        cfg=dict(model_id=MIXED366_ID,reference_planning=deepcopy(POLICY),duration_s=.5,
            finite_chunk_count=3,task='coffee',execute_steps=15,target_execution='shared_reference_windows',recording_enabled=False,
            maximum_observation_age_s=.5,reference_supply_timeout_s=.5)
        frozen=configuration(cfg);cfg['reference_planning']['velocity_limit_rad_s']=2.
        self.assertEqual(frozen['reference_planning'],POLICY)
        with self.assertRaisesRegex(ValueError,'scoped'):configuration(dict(frozen,model_id=BASE566_ID))
        with self.assertRaisesRegex(ValueError,'diagnostic prefix'):configuration(dict(frozen,bounded_live_prediction=True))

    def test_old_owner_cannot_silently_ignore_new_planning_policy(self):
        from robot_workbench.control_owner import ControlRuntimeClient
        client=object.__new__(ControlRuntimeClient);client.state={'control_owner':{'protocol_features':[]}}
        client._call=lambda *a,**k:self.fail('old Owner must reject before configure')
        with self.assertRaisesRegex(ValueError,'protocol'):
            client.configure_remote_inference({'reference_planning':POLICY},{},created=None,metric=None,record=None,predict=None)

    def test_moving_boundary_and_brake_prefix_keep_their_analytic_derivatives(self):
        from robot_workbench.motion_reference import prepare_brake_after
        source,limits=self.fixture();first=self.prepare(source,limits);boundary=first.duration_s*.4
        q,v,a=first.sample([boundary]);source['device_targets14'][0]=q[0].tolist()
        following=self.prepare(source,limits,initial_velocity_sdk14=v[0],initial_acceleration_sdk14=a[0])
        q2,v2,a2=following.sample([0.]);np.testing.assert_allclose(q2,q,atol=1e-12)
        np.testing.assert_allclose(v2,v,atol=1e-12);np.testing.assert_allclose(a2,a,atol=1e-10)
        brake=prepare_brake_after(first,boundary,limits,duration_s=.3,velocity_limit=1.,acceleration_limit=40.)
        for old,new in zip(first.sample([0,boundary/2]),brake.sample([0,boundary/2])):np.testing.assert_allclose(old,new,atol=1e-12)
        _check_c2(brake.coefficients,brake.breaks)

    def test_two_second_cap_is_infer_only_and_measured_guard_conflict_rejects(self):
        from types import SimpleNamespace
        from robot_workbench.inference_owner import InferenceRunner
        from robot_workbench.inference_model import MIXED366_ID
        source,limits=self.fixture()
        for row in source['device_targets14'][1:]:row[0]=.8
        limits[0]=[-2,2]
        with self.assertRaisesRegex(ValueError,'2s bound'):self.prepare(source,limits)
        owner=SimpleNamespace(preserve_begin_jaws=True,environment='physical',
            motion_error_settings=lambda:{'speed_protection':{'enabled':True,'limit_rad_s':.8}})
        cfg=dict(model_id=MIXED366_ID,reference_planning=POLICY,finite_chunk_count=3,duration_s=.5,
            execute_steps=15,task='coffee',target_execution='shared_reference_windows',recording_enabled=False,
            maximum_observation_age_s=.5,reference_supply_timeout_s=.5)
        with self.assertRaisesRegex(ValueError,'measured-speed protection'):
            InferenceRunner(owner,None,None,cfg,predict=None,observe=None,adapter=None)

    def test_first_target_knot_cannot_be_silently_skipped(self):
        from types import SimpleNamespace
        from robot_workbench.target_interpreter import TargetInterpreter
        source,limits=self.fixture();reference=self.prepare(source,limits);packets=[]
        clock=lambda:1.
        interpreter=TargetInterpreter(SimpleNamespace(queue_reference=lambda p:packets.append(p)),reference,
            active_arm='both',start_s=0.,epoch=1,current_epoch=lambda:1,gate=lambda:None,clock=clock)
        with self.assertRaisesRegex(ValueError,'first target knot already passed'):
            interpreter.tick(1.,{'sample_time_s':1.,'qpos':[0.]*14})
        self.assertEqual(packets,[])


import test_strict_initial_hold as strict
import test_normal_modes as normal


class FiniteInferTests(unittest.TestCase):
    profile='normal_X5'
    setUp=normal.NormalModeTests.setUp
    cleanup=normal.NormalModeTests.cleanup
    setup_modes=normal.NormalModeTests.setup_modes
    advance=normal.NormalModeTests.advance
    until=strict.StrictInitialHoldTests.until
    prepare=strict.StrictInitialHoldTests.prepare

    def confirm(self,business,review):
        from robot_workbench.inference_model import MIXED366_ID
        config=deepcopy(self.runtime.inference_configuration);factory=self.runtime.inference_factory
        for name in('normal_modes','normal_initial_mode','normal_finish_kind','normal_finish_policy','_normal_finish'):config.pop(name,None)
        config.update(model_id=MIXED366_ID,task='coffee',reference_planning=deepcopy(POLICY),finite_chunk_count=3,
            duration_s=.5,maximum_observation_age_s=.5,execute_steps=15,target_execution='shared_reference_windows',recording_enabled=False)
        config['reference_supply_timeout_s']=.5
        self.runtime.configure_inference(config,factory)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=.5))
        return normal.NormalModeTests.confirm(self,business,review)

    def run_to_stop(self,runner,maximum=1500):
        for _ in range(maximum):
            self.advance()
            if runner.status in('STOPPED','FAULT'):return
        self.fail('finite synchronous batch did not finish')

    def test_three_full_retimed_segments_have_fresh_queries_and_one_final_stop(self):
        runner=self.prepare();predict=runner.predict;observe=runner.observe
        def model(request):
            response=predict(request);response['action14'][:,0]=request['observation']['device_state14'][0]+np.linspace(.03,.43,60)
            return response
        runner.predict=model
        runner.observe=lambda now,fb:None if(runner.finite_stage=='waiting_observation'and now-runner.finite_observation_floor<.3)else observe(now,fb)
        self.run_to_stop(runner)
        self.assertEqual(runner.status,'STOPPED',runner.reason)
        rows=runner.finite_completed;self.assertEqual(len(rows),3);self.assertEqual(len(self.requests),3)
        self.assertEqual(len({row['request_id']for row in rows}),3)
        self.assertTrue(all(.5<row['execution_duration_s']<=2 for row in rows))
        self.assertTrue(all(row['complete_source_steps']==15 for row in rows))
        self.assertTrue(all(rows[i]['observation_source_s']>rows[i-1]['completed_source_s']for i in(1,2)))
        self.assertTrue(all(rows[i]['waiting_s']>=.3 for i in(1,2)))
        self.assertEqual(len(self.raw.begin_calls),2,'one initial begin per arm, never per chunk')
        self.assertEqual(len([e for e in self.events if e[0]=='stop']),1)
        self.assertAlmostEqual(runner.snapshot()['finite_chunks']['completed_execution_s'],sum(row['execution_duration_s']for row in rows))

    def test_stop_while_waiting_does_not_reuse_a_prediction_or_start_another_segment(self):
        runner=self.prepare();observe=runner.observe
        runner.observe=lambda now,fb:None if runner.finite_stage=='waiting_observation'else observe(now,fb)
        self.until(lambda:len(runner.finite_completed)==1)
        self.runtime.request_stop();self.advance(45)
        self.assertEqual(len(self.requests),1);self.assertEqual(len(runner.finite_completed),1)
        self.assertEqual(runner.status,'STOPPED');self.assertTrue(self.runtime.stop_receipt['stop_hold_return_confirmed'])

    def test_infeasible_second_segment_aborts_without_truncation_or_third_query(self):
        runner=self.prepare();predict=runner.predict
        def model(request):
            response=predict(request)
            if len(self.requests)==2:response['action14'][:,0]=2.
            return response
        runner.predict=model;self.run_to_stop(runner)
        self.assertEqual(runner.status,'FAULT');self.assertEqual(len(self.requests),2)
        self.assertEqual(len(runner.finite_completed),1);self.assertIn('2s bound',runner.reason)

    def test_endpoint_output_mismatch_cannot_count_a_segment_or_start_the_next(self):
        runner=self.prepare();read=self.raw.read_execution_output
        def changed():
            value=read()
            if runner.finite_stage=='verifying_endpoint':value['q_sdk14'][0]+=.4
            return value
        self.raw.read_execution_output=changed;self.run_to_stop(runner)
        self.assertEqual(runner.status,'FAULT');self.assertIn('native endpoint mismatch',runner.reason)
        self.assertEqual(len(runner.finite_completed),0);self.assertEqual(len(self.requests),1)

    def test_initial_native_velocity_is_not_silently_reset_to_zero(self):
        runner=self.prepare();read=self.raw.read_execution_output
        def moving():
            value=read();value['dq_sdk14'][0]=.2;return value
        self.raw.read_execution_output=moving;self.run_to_stop(runner)
        self.assertEqual(runner.status,'FAULT');self.assertIn('moving boundary cannot be zeroed',runner.reason)
        self.assertEqual(runner.actual_submission_count,0);self.assertFalse(self.requests)


if __name__=='__main__':unittest.main()
