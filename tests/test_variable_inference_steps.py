"""CPU-only checks for configurable complete model segments; no SDK or model calls."""
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import json
import threading
import unittest
import numpy as np
from robot_workbench.inference_owner import InferenceRunner, configuration, execution_steps
from robot_workbench.inference_workflow import InferenceWorkflow
from robot_workbench.motion_reference import infer_reference_policy, prepare_reference, _check_c2


def config(steps=15):
    return dict(model_id='mixed366-nohistory-epoch010-step34250',execute_steps=steps,
        reference_planning=dict(schema='workbench.infer-reference-policy.v1',curve='cubic_c2_quintic_ends',
            velocity_limit_rad_s=1.,acceleration_limit_rad_s2=40.,jaw_velocity_limit_mm_s=40.,
            boundary_scope='complete_prepared_segment',timing_policy='retime_allowed',max_execution_duration_s=steps/30.*4),
        normal_modes=dict(brake_duration_s=.2,velocity_limit=1.,acceleration_limit=40.,gain_transition_s=.05,
            teach_kd_scale=.1,recording_required=False,recorder_ids=[],recording_clock_id='host_wall_time_ns'),
        normal_initial_mode='HOLD',task='coffee',target_execution='shared_reference_windows',
        maximum_observation_age_s=.5,reference_supply_timeout_s=.5,duration_s=120.)


class VariableStepsTests(unittest.TestCase):
    def test_config_boundaries_and_duration_policy(self):
        self.assertEqual(execution_steps(),15)
        for n in range(1,61):
            frozen=configuration(config(n))
            self.assertEqual(frozen['execute_steps'],n)
            self.assertAlmostEqual(frozen['reference_planning']['max_execution_duration_s'],n/30.*4)
        for n in (True,False,0,61,15.,'15'):
            bad=config();bad['execute_steps']=n
            with self.assertRaises(ValueError):configuration(bad)
        wrong=config(60);wrong['reference_planning']['max_execution_duration_s']=2.
        with self.assertRaises(ValueError):configuration(wrong)

    def test_settings_persist_invalidate_and_do_not_change_frozen_run(self):
        with TemporaryDirectory() as folder:
            flow=InferenceWorkflow.__new__(InferenceWorkflow)
            flow.lock=threading.RLock();flow.output=Path(folder);flow.execute_steps=15;flow.execution_revision=0
            flow.reference_planning=config()['reference_planning'];flow.finite_chunk_count=None
            flow.snapshot=Mock(return_value={'busy':False});flow.invalidate=Mock()
            frozen=deepcopy(flow.reference_planning)
            self.assertTrue(flow.execution_settings()['allowed'])
            result=flow.set_execution_steps(60,0)
            self.assertEqual((result['execute_steps'],result['revision']),(60,1))
            self.assertEqual(flow.reference_planning['max_execution_duration_s'],8.)
            self.assertEqual(frozen['max_execution_duration_s'],2.)
            self.assertEqual(json.loads((flow.output/'execution-settings.json').read_text())['execute_steps'],60)
            flow.invalidate.assert_called_once()
            with self.assertRaises(ValueError):flow.set_execution_steps(15,0)
            flow.snapshot.return_value={'busy':True}
            self.assertFalse(flow.execution_settings()['allowed'])
            with self.assertRaises(ValueError):flow.set_execution_steps(15,1)

    def test_actual_prepare_admit_complete_for_selected_steps(self):
        for n in (1,2,15,30,60):
            with self.subTest(steps=n),patch('robot_workbench.motion_reference.prepare_reference_process',side_effect=prepare_reference):
                runner=InferenceRunner.__new__(InferenceRunner);runner.config=configuration(config(n))
                runner.owner=NS(control_time=lambda:100.,epoch=2,execution_clock_id='cpu')
                runner._epoch=2;runner.initial_device14=[0.]*14;runner.target_interpreter=None
                runner.initialization_source_s=100.;runner.previous_native_hold=None
                runner.sdk_limits=[[-3.,3.]]*6+[[0.,.082]]+[[-3.,3.]]*6+[[0.,.082]]
                runner.reference_executor=None;runner.normal_config=runner.config['normal_modes'];runner.normal_planned=True
                runner.adapter=NS(decode=lambda row,held:dict(final_device14=row.tolist()))
                runner._record=Mock();runner._emit=Mock();runner._queue_native=Mock()
                rows=np.zeros((60,14));rows[:,0]=np.linspace(.00001,.006,60)
                request=dict(request_id='cpu-'+str(n),observation=dict(source_time=100.))
                response=runner._prepare_native_response(dict(request_id=request['request_id'],action14=rows),request)
                ref=response['_prepared_reference'];_check_c2(ref.coefficients,ref.breaks)
                self.assertEqual(len(ref.breaks),n+1)
                self.assertAlmostEqual(ref.metadata['source_duration_s'],n/30.)
                self.assertLessEqual(ref.duration_s,n/30.*4)
                q,v,a=ref.sample([ref.duration_s]);self.assertAlmostEqual(q[0,0],rows[n-1,0])
                np.testing.assert_allclose(v,0,atol=1e-10);np.testing.assert_allclose(a,0,atol=1e-9)
                runner.normal_single_segment=False;runner.started=100.;runner.normal_segment=None
                runner.bounded_request=request;runner._bind_normal=Mock()
                self.assertTrue(runner._install_normal_segment(ref,100.,{}))
                decisions=[args[1] for args,_ in runner._emit.call_args_list if args[0]=='guard_decision']
                self.assertTrue({'G13','G15','G16','G21','G22'}<={row['rule_id'] for row in decisions})
                self.assertEqual({row['policy_hash'] for row in decisions},{runner.guard_policy['policy_hash']})
                self.assertTrue(all(row['verdict']=='pass' for row in decisions))
                self.assertEqual(runner.normal_segment['source_steps'],n)
                self.assertAlmostEqual(runner.normal_segment['source_duration_s'],n/30.)
                end=runner.normal_segment['end_source_s'];runner.normal_segment['status']='verifying_endpoint'
                runner.normal_segment['endpoint_verification_after_s']=end
                runner.loop=NS();runner.native_queued_until=end+.1
                runner.target_interpreter=NS(reference=ref,last_packet=dict(host_times_s=[end+.1]))
                runner._normal_output=lambda:dict(q_sdk14=q[0],dq_sdk14=v[0],observations=dict(left=dict(sample_monotonic_ns=int((end+.1)*1e9))))
                feedback=dict(sample_time_s=end+.1,qpos=q[0],qvel=v[0]);runner.owner.stationary=True
                self.assertTrue(runner._tick_normal_segment(end+.1,feedback))
                self.assertEqual(runner.normal_segment['complete_source_steps'],n)
                runner.config['execute_steps']=1 if n!=1 else 2
                with self.assertRaises(ValueError):runner._install_normal_segment(ref,end+.1,feedback)

    def test_duration_budget_rejects_excessive_motion(self):
        # Large but in-limit first-step motion cannot fit 4x the source time.
        runner=InferenceRunner.__new__(InferenceRunner);runner.config=configuration(config(1))
        source=dict(kind='online_prediction_chunk',device_targets14=[[0.]*14,[2.]+[0.]*13],duration_s=1/30.,active_arms=['left','right'])
        limits=[[-3.,3.]]*6+[[0.,.082]]+[[-3.,3.]]*6+[[0.,.082]]
        with self.assertRaisesRegex(ValueError,'bound'):
            prepare_reference(source,[0.]*14,limits,source_sha256='cpu',planning_policy=runner.config['reference_planning'],
                initial_velocity_sdk14=[0.]*14,initial_acceleration_sdk14=[0.]*14)

if __name__=='__main__':unittest.main()
