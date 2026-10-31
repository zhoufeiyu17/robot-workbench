"""CPU-only explicit Ki: actual study route, unchanged legacy, bounded state."""
from copy import deepcopy
from pathlib import Path
import hashlib,json,sys,unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
import test_precision_study as fixture
from robot_workbench.execution_settings import study_defaults,validate_study
from robot_workbench.motion_reference import ReferenceIntegral,INTEGRAL_ERROR_GAINS


def coefficients(j3=30.72):
    v=dict(left=[8*x for x in INTEGRAL_ERROR_GAINS],right=list(INTEGRAL_ERROR_GAINS))
    v['left'][2]=j3
    return v


class StudyIntegralGainsTest(unittest.TestCase):
    profile='x5_control50_damping_v1'
    setUp=fixture.PrecisionStudyTest.setUp
    cleanup=fixture.PrecisionStudyTest.cleanup
    create=fixture.PrecisionStudyTest.create
    advance=fixture.PrecisionStudyTest.advance
    study=fixture.PrecisionStudyTest.study

    def test_pd_only_terminal_change_preserves_unmodified_arm_compensation(self):
        study=self.study();self.raw.position_bias[8]=.002
        settings=dict(study.settings,integral_gains=coefficients(0))
        settings['integral_gains']['left'][1]=0.
        command=dict(action='preview_workbench',waypoints=[dict(q12=[0.]*12,speeds12=[.1]*12)],study_settings=settings)
        def execute():
            review=study.device.submit('research',command)
            fixture.fixture.SharedTargetRoutesTest.confirm(self,'research',review);self.advance(350)
            self.assertEqual(self.runtime.owner.runner.status,'READY')
            return self.runtime.owner.runner.target_interpreter
        old=execute();before=len(self.raw.packets);begins=sum(row[0]=='begin'for row in self.raw.events)
        bias=old.last_packet['torque_bias_sdk14'][-1][8];self.assertLess(bias,-.05)
        command['study_settings']=deepcopy(settings)
        command['study_settings']['gains']['left']['kp'][2]=.5
        command['study_settings']['gains']['left']['kd'][2]=.2
        new=execute()
        self.assertEqual(sum(row[0]=='begin'for row in self.raw.events),begins)
        self.assertTrue(any(row[0]=='hold_gains'for row in self.raw.events))
        self.assertTrue(old.cancelled);self.assertEqual(new.compensation_parent,old.reference.metadata['reference_id'])
        self.assertLess(self.raw.packets[before]['torque_bias_sdk14'][0][8],-.05)
        self.runtime.request_stop();self.advance(50)
        self.assertTrue(new.cancelled);self.assertIsNone(self.raw.window)

    def test_legacy_unchanged_and_explicit_values_are_bounded(self):
        old=study_defaults();self.assertEqual(validate_study(old),old)
        self.assertNotIn('integral_gains',validate_study(old))
        new=dict(old,integral_gains=coefficients());self.assertEqual(validate_study(new),new)
        for value in (True,-1.,480.01,float('nan'),float('inf')):
            bad=deepcopy(new);bad['integral_gains']['left'][2]=value
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'left J3 integral gain'):validate_study(bad)
        bad=deepcopy(new);bad['integral_gains']['right']=[]
        with self.assertRaisesRegex(ValueError,'six right'):validate_study(bad)

    def test_real_integrator_scales_only_requested_joint_and_zero_preserves_bias(self):
        legacy=ReferenceIntegral(rapid=True,rapid_scale=8,profile=self.profile)
        explicit=ReferenceIntegral(rapid=True,rapid_scale=8,profile=self.profile,gains_sdk6=coefficients(480)['left'])
        small=ReferenceIntegral(rapid=True,rapid_scale=8,profile=self.profile,gains_sdk6=coefficients()['left'])
        frozen=ReferenceIntegral(rapid=True,rapid_scale=8,profile=self.profile,gains_sdk6=coefficients(0)['left']);frozen.bias[2]=.4
        for k in range(30):
            now=100.+k*.02;times=now+.03+np.arange(11)*.01;e=[.001]*6
            a=np.array(legacy.packet(now,now,e,times));b=np.array(explicit.packet(now,now,e,times));c=np.array(small.packet(now,now,e,times))
            np.testing.assert_array_equal(a,b)
            np.testing.assert_allclose(c[:,2],a[:,2]*(30.72/480),atol=1e-12)
            np.testing.assert_array_equal(c[:,[0,1,3,4,5]],a[:,[0,1,3,4,5]])
            np.testing.assert_allclose(np.array(frozen.packet(now,now,e,times))[:,2],.4)
        self.assertAlmostEqual(small.effective_rate[2],.03072)
        self.assertFalse(small.gains.flags.writeable)

    def test_custom_ki_reaches_runner_is_recorded_and_stop_clears_it(self):
        study=self.study();settings=dict(study.settings,integral_gains=coefficients())
        study.update_settings(settings,0);study.prepare('research','trajectory',1.)
        study.execute('research',study.review['id']);self.advance(120)
        old=self.runtime.owner.runner.target_interpreter
        self.assertAlmostEqual(old.integrals['left'].gains[2],30.72)
        self.assertEqual(old.snapshot()['integral_gains_sdk'],settings['integral_gains'])
        self.assertEqual(self.runtime.execution_calibration['study_settings'],settings)
        self.runtime.request_stop();self.advance(50);study.thread.join(3)
        self.assertFalse(study.thread.is_alive());self.assertTrue(old.cancelled);self.assertFalse(old.integrals)
        self.assertIsNone(self.raw.window)
        settings=deepcopy(settings);settings['integral_gains']['left'][2]=0.
        study.update_settings(settings,study.revision);study.prepare('research','home',1.)
        study.execute('research',study.review['id'])
        try:
            self.advance(350)
            new=self.runtime.owner.runner.target_interpreter;self.assertIsNot(new,old)
            self.assertEqual(new.integrals['left'].gains[2],0.)
        finally:
            self.runtime.request_stop();self.advance(50);study.thread.join(3)

    def test_direct_reference_review_rejects_old_feedforward_ki_provenance(self):
        study=self.study();self.raw.native_reference_api='preserve_reference_v1';self.advance(1)
        settings=dict(study.settings,integral_gains=coefficients());pin=study.config['references']['1.0']
        with np.load(pin['path'])as f:identity=json.loads(str(f['metadata']))['reference_id']
        ff=dict(schema='reference_feedforward.v1',source_sha256=identity,controller_profile=self.profile,
            phase=[0.,1.],torque_bias_sdk6=[[0.]*6]*2,provenance=dict(study_settings=study.settings,native_reference_api='preserve_reference_v1'))
        raw=json.dumps(ff).encode();sha=hashlib.sha256(raw).hexdigest();p=Path(self.temp.name)/(sha+'.json');p.write_bytes(raw)
        command=dict(action='preview_workbench',reference_artifact=pin,study_settings=settings,
            feedforward_artifact=dict(path=str(p),sha256=sha,size_bytes=len(raw)))
        before=len(self.raw.packets)
        with self.assertRaisesRegex(ValueError,'完整积分增益'):self.runtime.submit(self.creds['research'],command)
        self.assertEqual(len(self.raw.packets),before)

    def test_changing_only_ki_rebuilds_the_active_terminal_hold(self):
        study=self.study();settings=dict(study.settings,integral_gains=coefficients())
        command=dict(action='preview_workbench',waypoints=[dict(q12=[0.]*12,speeds12=[.1]*12)],study_settings=settings)
        def execute():
            review=study.device.submit('research',command)
            fixture.fixture.SharedTargetRoutesTest.confirm(self,'research',review);self.advance(350)
            return self.runtime.owner.runner.target_interpreter
        first=execute();self.assertFalse(first.cancelled)
        command['study_settings']=deepcopy(settings);command['study_settings']['integral_gains']['left'][2]=0.
        second=execute();self.assertTrue(first.cancelled);self.assertIsNot(second,first)
        self.assertEqual(second.integrals['left'].gains[2],0.)
        self.assertEqual(self.raw.study_gain_override,settings['gains'])


if __name__=='__main__':unittest.main()
