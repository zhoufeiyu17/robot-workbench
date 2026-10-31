"""Actual decoder/translator qualification for the explicit left-arm response study."""
from copy import deepcopy
import hashlib,io,json,unittest
from types import SimpleNamespace as NS
import numpy as np
from robot_workbench.control_reference import decode,joint_response
from robot_workbench.execution_settings import study_defaults,study_caps,validate_study
from robot_workbench.motion_reference import JOINTS,_extrema
from robot_workbench.target_interpreter import TargetInterpreter,ReferenceRevokedError

class JointResponseProbeTest(unittest.TestCase):
    def test_all_axes_exact_delay_continuity_bounds_and_stop(self):
        initial=np.zeros(14);initial[[1,2,8,9]]=np.deg2rad(6.5);initial[[6,13]]=.04
        limits=[[-3.,3.]]*14;limits[6]=limits[13]=[0.,.082]
        for slot in range(6):
            for amplitude in (1,3,5):
                for direction in (-1,1):
                    r=joint_response(initial,limits,slot=slot,amplitude_deg=amplitude,direction=direction)
                    b,c=r.breaks,r.coefficients;self.assertAlmostEqual(b[3]-b[2],1.)
                    q,v,a=r.sample([b[2],b[3],b[4],b[5]])
                    np.testing.assert_allclose(q[:,slot]-initial[slot],np.deg2rad(direction*np.array([amplitude,amplitude,amplitude+.15,amplitude+.15])),atol=1e-12)
                    np.testing.assert_allclose(np.delete(q-initial,slot,axis=1),0,atol=1e-12)
                    np.testing.assert_allclose(v,0,atol=1e-12);np.testing.assert_allclose(a,0,atol=1e-10)
                    _,_,peak_v,peak_a=_extrema(c,b);self.assertLessEqual(max(peak_v),.03+1e-10);self.assertLessEqual(max(peak_a),.2+1e-10)
                    column=JOINTS.index(slot)
                    for i,width in enumerate(np.diff(b)):
                        jerk=np.polyder(c[i,:,column],3);t=np.linspace(0,width,101)
                        self.assertLessEqual(max(abs(np.polyval(jerk,t))),2.+1e-9)
                        for derivative in (1,2,3):
                            self.assertAlmostEqual(np.polyval(np.polyder(c[i,:,column],derivative),0),0,places=8)
                            self.assertAlmostEqual(np.polyval(np.polyder(c[i,:,column],derivative),width),0,places=8)
        packets=[];now=100.
        x=TargetInterpreter(NS(queue_reference=lambda p:packets.append(p)or{}),r,active_arm='both',start_s=101.,epoch=1,
            current_epoch=lambda:1,gate=lambda:None,clock=lambda:now,profile='x5_control50_damping_v1')
        for stamp in np.arange(100.,104.,.02):
            now=float(stamp);x.tick(now,dict(sample_time_s=now,qpos=initial.tolist()))
        self.assertGreater(len(packets),10);x.revoke();count=len(packets)
        with self.assertRaises(ReferenceRevokedError):x.tick(now+.02,dict(sample_time_s=now+.02,qpos=initial.tolist()))
        self.assertEqual(len(packets),count)

    def test_rehashed_modified_trajectory_rejected_and_caps_not_defaults(self):
        initial=np.zeros(14);limits=[[-3,3]]*14
        r=joint_response(initial,limits,slot=1,amplitude_deg=1,direction=1)
        for field in ('curve','wait','axis'):
            c=r.coefficients.copy();m=deepcopy(r.metadata);m.pop('reference_id')
            if field=='curve':c[2,-1,0]+=.001
            if field=='wait':m['probe_wait_s']=.9
            if field=='axis':m['probe_joint_index']=True
            h=hashlib.sha256(json.dumps(m,sort_keys=True).encode())
            for a in (r.breaks,c,r.jaws):h.update(a.tobytes())
            m['reference_id']=h.hexdigest();f=io.BytesIO();np.savez(f,breaks=r.breaks,coefficients=c,jaws=r.jaws,metadata=np.array(json.dumps(m)))
            with self.assertRaises(ValueError):decode(f.getvalue(),limits)
        self.assertEqual(study_defaults()['gains']['right']['kp'][4:6],[30.,20.])
        for side in ('left',):
            for j in range(6):
                for p in (1,10,30,50):
                    settings=study_defaults();settings['gains'][side]['kp'][j]=p;settings['gains'][side]['kd'][j]=.2
                    validate_study(settings)
        for joint in (4,5):
            self.assertEqual(study_caps()['gains']['right']['kp'][joint],400.)
            settings=study_defaults();settings['gains']['right']['kp'][joint]=400.
            validate_study(settings)
            settings['gains']['right']['kp'][joint]=400.01
            with self.assertRaises(ValueError):validate_study(settings)

if __name__=='__main__':unittest.main()
