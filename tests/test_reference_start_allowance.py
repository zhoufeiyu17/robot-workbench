from types import SimpleNamespace as NS
from threading import RLock
from copy import deepcopy
import math,unittest
import numpy as np
from robot_workbench.control_reference import check_initial_tracking,start_allowance_matches,study_input_sha256
class StartAllowanceTest(unittest.TestCase):
 def test_experimental_settings_and_feedforward_must_match_exact_binding(self):
  from copy import deepcopy
  study=dict(gains={s:dict(kp=[80,70,70,30,30,20],kd=[2,2,2,1,1,.7])for s in('left','right')},integral_enabled=True,integral_gains={s:[0.]*6 for s in('left','right')})
  study['gains']['left']['kd'][0]=1.5;command={'feedforward_artifact':{'path':'/x','sha256':'a'*64,'size_bytes':42}}
  owner=NS(backend=NS(reference_start_allowance=dict(study_bindings=[])))
  self.assertFalse(start_allowance_matches(owner,study,command));owner.backend.reference_start_allowance['study_bindings']=[study_input_sha256(study,command)]
  self.assertTrue(start_allowance_matches(owner,study,command))
  other=deepcopy(study);other['gains']['left']['kd'][0]=1.6
  self.assertFalse(start_allowance_matches(owner,other,command));self.assertFalse(start_allowance_matches(owner,study,{}))
 def test_only_exact_reference_and_completed_continuous_command(self):
  ref=NS(metadata={'reference_id':'a'*64},sample=lambda t:(np.zeros((1,14)),))
  packet=dict(q_sdk14=np.zeros((3,14)).tolist(),dq_sdk14=np.zeros((3,14)).tolist())
  prior=NS(cancelled=False,epoch=4,active_arm='both',profile='p',start_s=0.,reference=NS(duration_s=1.,metadata={'reference_id':'parent'}),last_packet=packet)
  owner=NS(backend=NS(reference_start_allowance={'reference_id':'a'*64,'maximum_error_rad':math.radians(2.5)}),lock=RLock(),runner=NS(status='READY',target_interpreter=prior),epoch=4,shared_target_profile='p',control_time=lambda:2.,feedback_healthy=True,cache=dict(stationary=True,calibration_feedback=dict(sample_time_s=1.99,controller_telemetry=dict(arms={s:dict(output_qpos=[0.]*6)for s in ('left','right')}))))
  measured=np.zeros(14);measured[2]=math.radians(2.1)
  self.assertTrue(check_initial_tracking(owner,ref,measured)['native_output_matched'])
  for key,value in [('cancelled',True),('epoch',3),('active_arm','left')]:
   old=getattr(prior,key);setattr(prior,key,value)
   with self.assertRaises(ValueError):check_initial_tracking(owner,ref,measured)
   setattr(prior,key,old)
  for field in ('q_sdk14','dq_sdk14'):
   packet[field][0][2]=.001
   with self.assertRaises(ValueError):check_initial_tracking(owner,ref,measured)
   packet[field][0][2]=0.
  owner.cache['calibration_feedback']['controller_telemetry']['arms']['left']['output_qpos'][2]=.001
  with self.assertRaises(ValueError):check_initial_tracking(owner,ref,measured)
  owner.cache['calibration_feedback']['controller_telemetry']['arms']['left']['output_qpos'][2]=0.
  measured[2]=math.radians(2.51)
  with self.assertRaises(ValueError):check_initial_tracking(owner,ref,measured)
  measured[2]=math.radians(2.1);owner.backend.reference_start_allowance['reference_id']='b'*64
  with self.assertRaises(ValueError):check_initial_tracking(owner,ref,measured)
  owner.backend.reference_start_allowance=None
  with self.assertRaises(ValueError):check_initial_tracking(owner,ref,measured)
  measured[2]=.005;self.assertEqual(check_initial_tracking(owner,ref,measured)['limit_rad'],math.radians(2.))
if __name__=='__main__':unittest.main()
