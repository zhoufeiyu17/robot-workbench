"""Continuous reference preparation, derivatives, timing and bounded packets."""
import unittest
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import numpy as np
from robot_workbench.motion_reference import prepare_reference,ReferenceIntegral,TORQUE_BIAS_LIMITS,load_feedforward,ReferenceFeedforward,FEEDFORWARD_SLEW_LIMITS,native_holds


class MotionReferenceTests(unittest.TestCase):
    def test_frozen_feedforward_identity_bounds_and_total_ramp(self):
        with TemporaryDirectory()as temporary:
            value=dict(schema='reference_feedforward.v1',source_sha256='a'*64,controller_profile='normal_X5',
                phase=[0.,1.],torque_bias_sdk6=[[0.,0.,.6,0.,0.,0.]]*2,provenance=dict(run='cpu'))
            raw=json.dumps(value).encode();sha=hashlib.sha256(raw).hexdigest();p=Path(temporary)/(sha+'.json');p.write_bytes(raw)
            profile=load_feedforward(temporary,sha,'a'*64)
            with self.assertRaisesRegex(ValueError,'source'):load_feedforward(temporary,sha,'b'*64)
            follower=ReferenceFeedforward(profile,101.5,2.,100.);last=None
            for i in range(50):
                now=100+i*.1;times=now+np.arange(24)*.01+.02
                actual=np.asarray(follower.packet(times,np.zeros((24,6))))
                self.assertTrue(np.all(np.abs(actual)<=np.asarray(TORQUE_BIAS_LIMITS)))
                self.assertTrue(np.all(np.abs(np.diff(actual,axis=0))<=np.diff(times)[:,None]*np.asarray(FEEDFORWARD_SLEW_LIMITS)+1e-9))
                if last is not None:
                    prior=[np.interp(times[0],last[0],last[1][:,j])for j in range(6)]
                    np.testing.assert_allclose(actual[0],prior)
                last=(times,actual)
            self.assertAlmostEqual(actual[-1,2],.6)
            p.write_bytes(b'{}')
            with self.assertRaisesRegex(ValueError,'digest'):load_feedforward(temporary,sha,'a'*64)
        self.assertEqual(native_holds(dict(reference_integral_feedback=True)),(.04,4.))
        self.assertEqual(native_holds(dict(reference_integral_feedback=True,reference_feedforward={'sha256':'a'*64})),(4.,4.))

    def test_integral_is_bounded_slew_limited_fresh_and_has_no_q_reference_changes(self):
        controller=ReferenceIntegral();now=100.
        first=np.asarray(controller.packet(now,now,[1.]*6,[now+.02,now+.25]))
        np.testing.assert_allclose(first[0],0.)
        for i in range(1,501):
            now=100+i*.1;value=np.asarray(controller.packet(now,now,[1.]*6,[now+.02,now+.25]))
            self.assertTrue(np.all(np.abs(value)<=np.asarray(TORQUE_BIAS_LIMITS)))
        np.testing.assert_allclose(value[-1],TORQUE_BIAS_LIMITS)
        now+=.1;previous=controller.bias.copy();value=np.asarray(controller.packet(now,now,[-1.]*6,[now+.02,now+.25]))
        self.assertTrue(np.all(value[-1]<previous))
        with self.assertRaisesRegex(ValueError,'fresh'):controller.packet(now+.1,now-.1,[0.]*6,[now+.12,now+.3])
        with self.assertRaisesRegex(ValueError,'clock'):controller.packet(now,now,[0.]*6,[now+.02,now+.25])
        # A simple static plant check verifies sign/anti-windup, not robot stability.
        controller=ReferenceIntegral();bias=np.zeros(6)
        for i in range(200):
            now=200+i*.1;error=np.array([0.,0.,.008,0.,0.,0.])-.01*bias
            bias=np.asarray(controller.packet(now,now,error,[now+.02,now+.12]))[0]
        self.assertLess(abs(.008-.01*bias[2]),np.radians(.2))

    def fixture(self):
        q=np.zeros((61,14));q[:,0]=.2*(np.arange(61)/60)**2
        source=dict(device_targets14=q.tolist(),model_id='cpu',calibration_profile_digest='c'*64)
        limits=np.asarray([[-3.,3.]]*14);limits[[6,13]]=[0,.082]
        return source,np.zeros(14),limits

    def test_curve_derivatives_endpoints_time_policy_and_bounded_packets(self):
        source,initial,limits=self.fixture()
        reference=prepare_reference(source,initial,limits,source_sha256='a'*64)
        self.assertGreater(reference.duration_s,2.)
        scale=reference.duration_s/2
        for i in range(0,61,30):
            q,dq,ddq=reference.sample(np.arange(i,min(i+30,61))/30*scale)
            np.testing.assert_allclose(q[:,0],np.asarray(source['device_targets14'])[i:i+len(q),0],atol=1e-10)
        q,dq,_=reference.sample([-1.,0.,reference.duration_s,reference.duration_s+1])
        np.testing.assert_allclose(dq,0.,atol=1e-10);self.assertAlmostEqual(q[-1,0],.2)
        t=reference.duration_s*.43;h=1e-5
        q,dq,ddq=reference.sample([t-h,t,t+h])
        np.testing.assert_allclose((q[2]-q[0])/(2*h),dq[1],atol=1e-7)
        np.testing.assert_allclose((dq[2]-dq[0])/(2*h),ddq[1],atol=1e-7)
        with self.assertRaises(ValueError):reference.coefficients[0,0,0]=1
        packet=reference.packet(.5)
        self.assertLessEqual(len(packet['relative_times_s']),32)
        self.assertAlmostEqual(packet['relative_times_s'][0],.52)
        self.assertAlmostEqual(packet['relative_times_s'][-1],.75)
        with self.assertRaises(ValueError):reference.packet(.5,horizon_s=1)
        with self.assertRaisesRegex(ValueError,'strict 2s'):prepare_reference(source,initial,limits,source_sha256='a'*64,timing_policy='strict_duration')
        self.assertLessEqual(reference.metadata['max_velocity_rad_s'],.25)
        self.assertLessEqual(reference.metadata['max_acceleration_rad_s2'],1.)

    def test_initial_hold_and_identity_are_bound_to_actual_start(self):
        source,initial,limits=self.fixture()
        a=prepare_reference(source,initial,limits,source_sha256='a'*64)
        initial[0]=.001;initial[7]=.2
        b=prepare_reference(source,initial,limits,source_sha256='a'*64)
        self.assertNotEqual(a.metadata['reference_id'],b.metadata['reference_id'])
        q,_,_=b.sample([0.,b.duration_s])
        self.assertEqual(q[0,0],.001);np.testing.assert_allclose(q[:,7],.2)
        with self.assertRaises(ValueError):b.sample([float('nan')])
        initial[0]=4
        with self.assertRaisesRegex(ValueError,'outside limits'):prepare_reference(source,initial,limits,source_sha256='a'*64)


if __name__=='__main__':unittest.main()
