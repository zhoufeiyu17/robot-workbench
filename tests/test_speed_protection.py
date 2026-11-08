"""Measured-speed guard through the real shared Owner, CPU protocol only."""
import json
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
import test_shared_target_routes as fixture
from robot_workbench.execution_settings import defaults,validate,measured_joint_speed


class SpeedProtectionTest(unittest.TestCase):
    profile='normal_X5'
    setUp=fixture.SharedTargetRoutesTest.setUp
    cleanup=fixture.SharedTargetRoutesTest.cleanup
    def create(self,**kwargs):
        fixture.SharedTargetRoutesTest.create(self,**kwargs)
        self.creds['research']=self.runtime.register_business('research')
    advance=fixture.SharedTargetRoutesTest.advance
    confirm=fixture.SharedTargetRoutesTest.confirm

    def inject_velocity(self,slot,value):
        self.velocity=[0.]*14;self.velocity[slot]=value
        read=self.raw.read_calibration_state
        def feedback():
            result=read();result['qvel']=self.velocity[:];return result
        self.raw.read_calibration_state=feedback

    def test_defaults_validation_and_all_joint_directions(self):
        for business in ('calibration','collection','inference','research'):
            self.assertEqual(defaults(business)['speed_protection'],dict(enabled=True,limit_rad_s=.8))
        for value in (False,0,-1,float('nan'),float('inf'),'0.8',None):
            settings=defaults();settings['speed_protection']['limit_rad_s']=value
            with self.subTest(value=value),self.assertRaises(ValueError):validate(settings)
        settings=defaults();settings['speed_protection']['enabled']=1
        with self.assertRaises(ValueError):validate(settings)
        legacy=defaults();legacy.pop('speed_protection')
        with self.assertRaises(ValueError):validate(legacy)
        self.assertEqual(validate(legacy,allow_legacy=True)['speed_protection'],defaults()['speed_protection'])
        for slot in (*range(6),*range(7,13)):
            for sign in (-1,1):
                velocity=[0.]*14;velocity[slot]=sign*.81;velocity[6]=velocity[13]=99.
                measured=measured_joint_speed(dict(qvel=velocity,sample_time_s=10.),defaults()['speed_protection'])
                self.assertEqual(measured['joint'],('left'if slot<7 else'right')+f'_J{slot%7+1}')
                self.assertEqual(measured['velocity_rad_s'],sign*.81)

    def test_equal_limit_and_jaw_are_excluded_but_single_peak_trips_before_confirm(self):
        self.create();self.inject_velocity(12,-.8);self.velocity[6]=self.velocity[13]=99.
        self.advance();self.assertNotEqual(self.runtime.owner.control.state,self.runtime.owner.states.FAULT)
        self.velocity=[0.]*14;self.advance(40)
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_both_home'))
        self.confirm('calibration',review);epoch=self.runtime.owner.epoch
        self.velocity[12]=-.800001;self.advance()
        state=self.runtime.snapshot(self.creds['calibration'])
        self.assertEqual(state['state'],'FAULT_STOP_UNCONFIRMED');self.assertTrue(state['feedback_healthy'])
        self.assertFalse(state['automatic_enabled']);self.assertGreater(self.runtime.owner.epoch,epoch)
        self.assertFalse(self.raw.packets,'overspeed must win over queued confirm in the same tick')
        trip=state['speed_protection']['last_trip']
        self.assertEqual(trip['joint'],'right_J6');self.assertEqual(trip['velocity_rad_s'],-.800001)
        self.assertTrue(state['speed_protection']['latched']);self.assertIn('中断恢复',state['reason'])
        self.velocity=[0.]*14;self.advance(50)
        self.assertFalse(self.raw.packets);self.assertEqual(self.runtime.owner.control.state,self.runtime.owner.states.FAULT)

    def test_every_business_execution_and_home_cancel_future_native_windows(self):
        for business in ('calibration','collection','inference','research'):
            for home in (False,True):
                if home and business!='calibration':continue
                with self.subTest(business=business,home=home):
                    self.cleanup();q=[0.]*14;q[0]=.02;self.create(q=q)
                    if business!='calibration':self.runtime.handoff(self.creds['calibration'],business)
                    settings=defaults(business);settings.update(arrival_enabled=False,tracking_enabled=False)
                    self.runtime.update_execution_settings(self.creds[business],settings)
                    action=(dict(action='preview_both_home')if home else dict(action='preview_workbench',
                        waypoints=[dict(q12=[.1]+[0.]*11,speeds12=[.1]*12)]))
                    review=self.runtime.submit(self.creds[business],action);self.confirm(business,review)
                    for _ in range(500):
                        self.advance()
                        if self.raw.window is not None:break
                    self.assertIsNotNone(self.raw.window,self.runtime.owner.snapshot())
                    self.inject_velocity(3,1.);self.advance();count=len(self.raw.packets)
                    self.assertIsNone(self.raw.window);self.assertEqual(self.runtime.owner.runner.status,'FAULT')
                    self.assertTrue(self.runtime.owner.runner.target_interpreter.cancelled)
                    self.velocity=[0.]*14;self.advance(50)
                    self.assertEqual(len(self.raw.packets),count)
                    self.assertEqual(self.runtime.owner.control.state,self.runtime.owner.states.FAULT)

    def test_hold_guard_and_explicit_recovery_never_auto_resume(self):
        self.create();review=self.runtime.submit(self.creds['calibration'],dict(action='preview',active_arm='left',q_target=[.02]+[0.]*13))
        self.confirm('calibration',review)
        for _ in range(1500):
            self.advance()
            if self.runtime.owner.runner.status=='READY':break
        self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.motion_record())
        self.inject_velocity(0,.9);self.advance(40)
        owner=self.runtime.owner
        self.assertFalse(owner.recovery_ready(),'continuing overspeed must prohibit recovery')
        recovery=lambda:dict(action='recover',confirmation='ACKNOWLEDGE_MOTION_FAULT',owner_pid=os.getpid(),command_epoch=owner.epoch)
        with self.assertRaisesRegex(ValueError,'fresh stationary'):self.runtime.submit(self.creds['calibration'],recovery())
        self.velocity=[0.]*14;self.advance(40);self.assertTrue(owner.recovery_ready())
        stop=self.runtime.snapshot(self.creds['calibration'])['speed_protection']['last_trip']['stop']
        self.assertTrue(stop['hold_return_confirmed']);self.assertTrue(stop['feedback_stationary_confirmed'])
        self.assertEqual(owner.control.state,owner.states.FAULT)
        count=len(self.raw.packets)
        self.runtime.submit(self.creds['calibration'],recovery());self.velocity[0]=.9;self.advance()
        self.assertEqual(owner.control.state,owner.states.FAULT,'new overspeed must revoke queued recovery')
        self.velocity=[0.]*14;self.advance(40)
        self.runtime.submit(self.creds['calibration'],recovery())
        self.advance(40)
        self.assertEqual(owner.control.state,owner.states.STOPPED);self.assertIsNone(owner.runner)
        self.assertEqual(len(self.raw.packets),count);self.assertIsNone(self.raw.window)

    def test_stop_failure_stays_unconfirmed_and_nonrecoverable(self):
        self.create();self.inject_velocity(0,1.)
        with patch.object(self.raw,'set_damping_hold',side_effect=RuntimeError('CPU native stop failed')):
            self.advance()
        self.velocity=[0.]*14;self.advance(50)
        state=self.runtime.snapshot(self.creds['calibration'])
        self.assertIn('damping failed',state['reason']);self.assertIsNone(state['recoverable_fault'])
        self.assertTrue(state['speed_protection']['latched']);self.assertFalse(self.runtime.owner.recovery_ready())

    def test_rule_collection_and_online_infer_share_the_peak_guard(self):
        from test_collection_physx_protocol import protocol_fixture
        _,source,plan,_,mapping=protocol_fixture(Path(self.temp.name),source_clock_id='shared-interpreter-cpu-clock')
        fixture.jaw.configure_mapping(mapping);first=plan['ticks'][0]['target']
        q=[*first['left_arm_joints'],fixture.jaw.feedback_sdk('left',first['left_gripper']),
           *first['right_arm_joints'],fixture.jaw.feedback_sdk('right',first['right_gripper'])]
        self.create(q=q,mapping=mapping,binding=Path(self.temp.name)/'binding.json')
        self.runtime.handoff(self.creds['calibration'],'collection')
        review=self.runtime.submit(self.creds['collection'],dict(action='preview_frozen',source=source,plan=plan))
        self.confirm('collection',review)
        for _ in range(500):
            self.advance()
            if self.raw.window is not None:break
        self.assertIsNotNone(self.raw.window);self.inject_velocity(8,-.9);self.advance()
        count=len(self.raw.packets);self.velocity=[0.]*14;self.advance(50)
        self.assertEqual(self.runtime.owner.runner.status,'FAULT');self.assertIsNone(self.raw.window)
        self.assertEqual(len(self.raw.packets),count)
        self.cleanup();self.create();self.runtime.handoff(self.creds['calibration'],'inference')
        fixture.inference_fixture.InferenceOwnerTests.configure(self,execute_steps=10)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference'))
        self.confirm('inference',review);self.advance(5);runner=self.runtime.owner.runner
        self.assertTrue(runner.is_inference)
        self.inject_velocity(1,.9);self.advance();count=len(self.raw.packets)
        self.velocity=[0.]*14;self.advance(150)
        self.assertEqual(runner.status,'FAULT');self.assertIsNone(self.raw.window)
        self.assertEqual(len(self.raw.packets),count,'late model result must not emit after the speed trip')

    def test_per_business_persistence_and_legacy_restore(self):
        self.create()
        for business,limit in (('calibration',.9),('collection',1.1),('inference',.7)):
            if business!=self.runtime.holder:self.runtime.handoff(self.creds[self.runtime.holder],business)
            settings=defaults(business);settings['speed_protection']['limit_rad_s']=limit
            self.runtime.update_execution_settings(self.creds[business],settings)
        path=self.runtime.execution_settings_path;self.cleanup();self.create()
        self.assertEqual(self.runtime.execution_settings_revision,3)
        for business,limit in (('calibration',.9),('collection',1.1),('inference',.7)):
            self.assertEqual(self.runtime.execution_settings_snapshot(business)['settings']['speed_protection']['limit_rad_s'],limit)
        self.cleanup();saved=json.loads(path.read_text());saved['policies']['calibration'].pop('speed_protection')
        path.write_text(json.dumps(saved));self.create()
        self.assertEqual(self.runtime.execution_settings_snapshot('calibration')['settings']['speed_protection'],dict(enabled=True,limit_rad_s=.8))

    def test_explicit_disable_does_not_implicitly_lower_threshold(self):
        self.create();settings=defaults();settings['speed_protection']=dict(enabled=False,limit_rad_s=.9)
        self.runtime.update_execution_settings(self.creds['calibration'],settings)
        self.inject_velocity(0,1.);self.advance()
        self.assertNotEqual(self.runtime.owner.control.state,self.runtime.owner.states.FAULT)
        self.assertFalse(self.runtime.snapshot(self.creds['calibration'])['speed_protection']['enabled'])

    def test_live_guard_follows_committed_business_policy_and_isolated_snapshots(self):
        self.create()
        sample = dict(qvel=[0.] * 14, qpos=[0.] * 14, sample_time_s=1.)
        for business, limit in [('calibration', 1.1), ('collection', 1.3), ('inference', .9)]:
            if business != self.runtime.holder:
                self.runtime.handoff(self.creds[self.runtime.holder], business)
            self.runtime.owner.check_speed_protection(sample)
            self.assertEqual(self.runtime.owner.speed_protection_observation['limit_rad_s'], .8)
            settings = defaults(business)
            settings['speed_protection']['limit_rad_s'] = limit
            self.runtime.update_execution_settings(self.creds[business], settings)
            settings['speed_protection']['limit_rad_s'] = 99.
            snapshot = self.runtime.execution_settings_snapshot(business)
            snapshot['settings']['speed_protection']['limit_rad_s'] = 88.
            self.runtime.owner.check_speed_protection(sample)
            self.assertEqual(self.runtime.owner.speed_protection_observation['limit_rad_s'], limit)
        self.runtime.handoff(self.creds['inference'], 'calibration')
        self.runtime.owner.check_speed_protection(sample)
        self.assertEqual(self.runtime.owner.speed_protection_observation['limit_rad_s'], 1.1)
        policy=self.runtime._guard_policy('calibration')
        with patch('robot_workbench.runtime.policy_hash',side_effect=AssertionError('no hot-path hashing')):
            for _ in range(10):self.runtime.owner.check_speed_protection(sample)
        decision=self.runtime.owner.speed_protection_observation['guard_decision']
        self.assertEqual(decision['policy_hash'],policy['policy_hash'])
        self.assertEqual(decision['revision'],self.runtime.execution_settings_revision)


if __name__=='__main__':unittest.main()
