"""The actual common Runtime follows a reviewed curve, including a zero-displacement loop."""
from pathlib import Path
import hashlib
import io
import json
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_shared_target_routes import SharedTargetRoutesTest
from robot_workbench.control_reference import SCHEMA, decode, load


class ControlReferenceTest(unittest.TestCase):
    profile='x5_shared_precision_v1'
    setUp=SharedTargetRoutesTest.setUp
    create=SharedTargetRoutesTest.create
    cleanup=SharedTargetRoutesTest.cleanup
    advance=SharedTargetRoutesTest.advance
    confirm=SharedTargetRoutesTest.confirm

    def test_smooth_preparation_derivatives_bounds_identity_and_terminal_hold(self):
        from robot_workbench.control_reference import smooth
        from robot_workbench.motion_reference import PreparedReference,_extrema
        b=np.array([0.,1.,2.]);c=np.zeros((2,4,12));c[0,:,0]=[-.08,.12,0.,0.];c[1,:,0]=[.08,-.12,0.,.04]
        m=dict(schema='prepared_motion_reference.v1',study_schema=SCHEMA,source_range_s=[0.,50.],
            speed_multiplier=1.,duration_s=2.,event_times_s=[0.,2.],reference_id='a'*64)
        ref=PreparedReference(b,c,np.zeros((3,2)),m);new=smooth(ref,sigma_s=.1)
        self.assertEqual(new.coefficients.shape[1:],(8,12))
        q,v,a=new.sample([0.,2.,2.1]);np.testing.assert_allclose(q[:,0],0.,atol=1e-12)
        np.testing.assert_allclose(v,0.,atol=1e-12);np.testing.assert_allclose(a,0.,atol=1e-9)
        t=.613;h=1e-5;q,v,a=new.sample([t-h,t,t+h])
        np.testing.assert_allclose((q[2]-q[0])/(2*h),v[1],atol=1e-8)
        np.testing.assert_allclose((v[2]-v[0])/(2*h),a[1],atol=1e-8)
        # All derivative extrema must include interior roots, not just knots.
        low,high,peak_v,peak_a=_extrema(new.coefficients,new.breaks)
        grid=np.linspace(0.,2.,1001);values=[new.sample(grid[i:i+32])for i in range(0,len(grid),32)]
        for bound,slot in ((peak_v,1),(peak_a,2)):
            self.assertGreaterEqual(bound[0]+1e-10,max(abs(np.vstack([x[slot]for x in values])[:,0])))
        raw=io.BytesIO();np.savez(raw,breaks=new.breaks,coefficients=new.coefficients,jaws=new.jaws,metadata=json.dumps(new.metadata))
        decoded=decode(raw.getvalue(),np.tile([-3.,3.],(14,1)))
        self.assertEqual(decoded.metadata['reference_id'],new.metadata['reference_id'])
        for index,at in ((0,0.),(-1,np.diff(new.breaks)[-1])):
            self.assertAlmostEqual(np.polyval(np.polyder(new.coefficients[index,:,0],3),at),0.,places=6)
        # Seventh-degree joins include continuous jerk, checked from both sides.
        for i,width in enumerate(np.diff(new.breaks)[:-1]):
            for order in range(5):
                left=np.polyval(np.polyder(new.coefficients[i,:,0],order),width)
                right=np.polyval(np.polyder(new.coefficients[i+1,:,0],order),0.)
                self.assertAlmostEqual(left,right,places=6)

    def test_long_recording_is_explicitly_diagnostic_and_never_an_ordinary_policy_run(self):
        from test_recording_wall_clock import plan_fixture
        from integration import _digest
        from real_recording import validate_policy_source
        from robot_workbench.sample_clock import clock_spec
        parent,_,_,_=plan_fixture(4,'physical')
        source=dict(schema_version='policy_execution_source.v1',environment='physical',
            robot_id=parent['robot_id'],camera_serials=parent['camera_serials'],calibration_profile=parent['calibration_profile'],
            clock=clock_spec('physical'),execution_instance_id='control-reference-test',duration_s=103.)
        binding=dict(calibration_accepted=True,robot_id=source['robot_id'],camera_serials=source['camera_serials'],calibration_profile_digest=_digest(source['calibration_profile']))
        with self.assertRaises(ValueError):validate_policy_source(source,binding,'physical')
        source.update(execution_kind='control_reference_study',training_observation=False)
        validate_policy_source(source,binding,'physical')
    def test_reviewed_reference_loop_uses_shared_output_and_stop_revokes_it(self):
        self.create()
        q=np.asarray(self.raw.q,dtype=float)
        q[6]=q[13]=0.
        # q(t)=0.04*sin-like excursion, returning to the same endpoint. A point
        # proxy cannot reproduce this, so the test detects a hidden point fallback.
        b=np.asarray([0.,1.,2.]);c=np.zeros((2,4,12))
        c[0,:,0]=[-.08,.12,0.,0.];c[1,:,0]=[.08,-.12,0.,.04]
        jaws=np.tile(q[[6,13]],(3,1))
        m=dict(schema='prepared_motion_reference.v1',study_schema=SCHEMA,source_range_s=[0.,50.],
               speed_multiplier=1.,duration_s=2.,event_times_s=[0.,2.])
        h=hashlib.sha256(json.dumps(m,sort_keys=True).encode())
        for a in (b,c,jaws):h.update(a.tobytes())
        m['reference_id']=h.hexdigest();f=io.BytesIO()
        np.savez(f,breaks=b,coefficients=c,jaws=jaws,metadata=json.dumps(m))
        raw=f.getvalue();reference=decode(raw,self.runtime.owner.cache['joint_limits'])
        artifact=Path(self.temp.name)/'reference.npz';artifact.write_bytes(raw)
        pin=dict(path=str(artifact),sha256=hashlib.sha256(raw).hexdigest(),size_bytes=len(raw))
        actual=load(pin,self.runtime.owner.cache['joint_limits'])
        self.assertEqual(actual.metadata['reference_id'],m['reference_id'])
        with self.assertRaisesRegex(ValueError,'artifact changed'):
            load(dict(pin,sha256='0'*64),self.runtime.owner.cache['joint_limits'])
        changed=c.copy();changed[0,3,0]=.001;corrupted=io.BytesIO()
        np.savez(corrupted,breaks=b,coefficients=changed,jaws=jaws,metadata=json.dumps(m))
        with self.assertRaisesRegex(ValueError,'digest'):decode(corrupted.getvalue(),self.runtime.owner.cache['joint_limits'])
        ff=dict(schema='reference_feedforward.v1',source_sha256=m['reference_id'],controller_profile=self.profile,
                phase=[0.,1.],torque_bias_sdk6=[[0.,0.,.4,0.,0.,0.]]*2,provenance=dict(scope='CPU boundary test'))
        ffraw=json.dumps(ff).encode();ffsha=hashlib.sha256(ffraw).hexdigest()
        ffpath=Path(self.temp.name)/(ffsha+'.json');ffpath.write_bytes(ffraw)
        ffpin=dict(path=str(ffpath),sha256=ffsha,size_bytes=len(ffraw))
        def slow_preparation(*_):
            self.advance(25)  # Latest live cache advances while the compiler snapshot ages.
            return reference
        with patch('robot_workbench.control_reference.load',side_effect=slow_preparation):
            review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',
                reference_artifact=dict(path='/unused.npz',sha256='a'*64,size_bytes=len(raw)),feedforward_artifact=ffpin))
        self.assertEqual(review['prepared_reference_id'],m['reference_id'])
        self.assertEqual(review['prepared_feedforward_sha256'],ffsha)
        self.confirm('calibration',review)
        self.advance(150)
        self.assertGreater(max(row['q_sdk14'][-1][0]for row in self.raw.packets),.025)
        self.assertEqual(self.runtime.owner.runner.target_interpreter.reference.metadata['reference_id'],m['reference_id'])
        bias=np.asarray(self.raw.packets[-1]['torque_bias_sdk14'])
        self.assertGreater(bias[-1,2],.3)
        np.testing.assert_allclose(bias[:,7:],0.)
        self.runtime.request_stop();self.advance(3)
        self.assertTrue(self.runtime.owner.runner.target_interpreter.cancelled)
        self.assertIsNone(self.raw.window)


if __name__=='__main__':
    import unittest
    unittest.main()
