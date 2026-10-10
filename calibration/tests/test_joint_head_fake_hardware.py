import json,unittest
from examples.scripts.debug.joint_head_fake_hardware import FakePlant,FakeOwner,FakeSafety,camera_metadata
from arx_env.joint_head_jaw_mapping import command_sdk,feedback_gaps
class FakeHardwareTest(unittest.TestCase):
 def test_completed_fake_motion_survives_idle_scheduler_pause(self):
  from unittest.mock import patch
  from arx_env.joint_head_real_motion import make_plan
  clock=[10000.]
  with patch('time.monotonic',side_effect=lambda:clock[0]):
   plant=FakePlant({'q':[0.]*12,'jaw_gap_mm':[80.,80.]},[[-4.,4.]]*14)
   owner=FakeOwner(plant,FakeSafety,hardware=True)
   for _ in range(12):owner.tick();clock[0]+=.04
   feedback=owner.snapshot()['calibration_feedback']
   plan=make_plan('both',feedback['qpos'],feedback['qpos'],owner.snapshot()['joint_limits'],kind='workbench',created_at_s=clock[0],environment='fake')
   owner.active_epoch=owner.epoch;owner.review_press=owner.pedal_snapshot()[0]
   owner.capture_sample='sample-0'
   owner.start_motion(plan,feedback)
   for _ in range(80):
    clock[0]+=.04;owner.tick()
    if owner.runner and owner.runner.status=='READY':break
   self.assertIsNotNone(owner.runner,'pending capture must retain the completed runner')
   owner.capture_sample=None;owner.capture_wait=None;owner.arrival_pending=None
   clock[0]+=.04;owner.tick()
   self.assertIsNone(owner.runner,'finished fake action must leave the idle motion monitor')
   clock[0]+=.106;owner.tick()
   self.assertEqual(owner.snapshot()['state'],'READY')

 def test_nominal_camera_bootstrap_needs_no_solved_calibration(self):
  import tempfile,numpy as np
  from pathlib import Path
  from joint_head_wrist_fixture_bringup import nominal_fixture_contract
  from joint_head_workbench_live import validate_wrist_contract,Session
  from types import SimpleNamespace
  with tempfile.TemporaryDirectory() as temp:
   path=Path(temp)/'candidate.json';T=np.eye(4).tolist()
   path.write_text(json.dumps({'T_world_optical':T,'T_link6_marker':{'24':T,'30':T}}))
   k={'fx':500,'fy':500,'cx':320,'cy':240,'width':640,'height':480}
   c={'model_family':'ac_one','candidate':str(path),'asset_geometry_id':'test-a1',
      'wrist_seed_file':str(Path(__file__).resolve().parents[1]/'configs/wrist-motion-seed-v7.json'),
      'wrist_nominal_mounts':{s:T for s in ('left','right')},'wrist_simulation_cameras':{s:k for s in ('left','right')}}
   contract=validate_wrist_contract(nominal_fixture_contract(c))
   self.assertTrue(contract['display_only']);self.assertTrue(contract['simulation_only'])
   self.assertEqual(len(contract['planned_frames']),32)
   session=Session(SimpleNamespace(velocity_limits=np.ones(12),limits=np.array([[-3.,3.]]*12),k=k),Path(temp)/'output',c)
   self.assertEqual(session.pending_wrist_config,contract)
   self.assertIsNone(session.calibration);self.assertIsNone(session.pending)

 def test_mapped_feedback_and_immutable_frame_timestamp(self):
  plant=FakePlant({'q':[0.]*12,'jaw_gap_mm':[83.,83.]},[[-4,4]]*14);plant.command_guard=lambda:None
  q=plant.q[:];q[6]=command_sdk('left',83.);plant.send('left',q)
  self.assertAlmostEqual(feedback_gaps(plant.read_calibration_state()['qpos'])[0],83.)
  h={'source':'dsw_rtx_kinematic_simulation','view':'head','frame_sequence':9,'q_reported_rad':[0.]*12,'timestamp_s':123.5,'camera':{'rectified':True,'coefficients':[0.]*5}}
  m=camera_metadata(json.dumps(h),'head','fake-head',7)
  self.assertEqual(m['host_receive_ns'],123500000007);self.assertEqual(m['frame_number'],9)
  self.assertFalse(m['intrinsics']['rectified']);self.assertTrue(m['simulation_only'])
  with self.assertRaises(ValueError):camera_metadata(json.dumps(h),'left','fake-left')

 def test_float_noise_near_a_hard_limit_is_not_reported_as_out_of_limit(self):
  # Parked joints are computed (quaternion/trig round trips), so a value of 0 arrives
  # as -2.22e-16. The owner's motion monitor validates measured qpos against the plan's
  # SDK limits with `low <= value`; on a limit of exactly 0 that noise latched
  # FAULT_STOP_UNCONFIRMED ("measured qpos[8] outside SDK limit") and blocked the
  # workbench. The device boundary cleans it, so the safety check stays untouched.
  noise=[-2.220446049250313e-16,2.220446049250313e-16,4.336808689942018e-19]
  plant=FakePlant({'q':[0.]*8+noise+[0.],'jaw_gap_mm':[83.,83.]},[[-4,4]]*14);plant.command_guard=lambda:None
  self.assertEqual(plant.q[8:11],[0.,0.,0.])
  qpos=plant.read_calibration_state()['qpos']
  self.assertTrue(all(v>=0. for v in qpos[8:11]),qpos[8:11])
  # The threshold is float noise, not a physical tolerance: a real violation survives.
  q=plant.q[:];q[8]=-0.01;plant.send('right',q)
  self.assertEqual(plant.q[8],-0.01)
  self.assertEqual(FakePlant.clean([1e-13,-1e-13,1e-12,-1e-12]),[0.,0.,1e-12,-1e-12])
