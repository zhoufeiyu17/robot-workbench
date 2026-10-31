"""Manual planning uses actual SDK command bounds and separate feedback allowance."""
import math
from types import SimpleNamespace
import unittest
import numpy as np
from examples.scripts.debug.joint_head_workbench import Workbench
from robot_workbench.joint_bounds import JointPositionBoundaryError

class ManualLimitsTest(unittest.TestCase):
    def test_measured_start_does_not_reuse_nominal_or_command_noise_bounds(self):
        app=Workbench.__new__(Workbench)
        app.config={'environment':'physical','simulation_only':False,'robot_source':'physical'}
        app.geometry=SimpleNamespace(limits=np.array([[0.,1.]]*12),_q=lambda v:np.asarray(v,float))
        app.hardware_feedback={'owner':{'joint_limits':[[-.05,1.]]*14}}
        q=[-.02]*12
        self.assertEqual(app.bounded_joint_pose(q,feedback=True)[0].tolist(),q)
        q=[-.05-math.radians(1.5)]*12
        bounded,receipt=app.bounded_joint_pose(q,feedback=True)
        self.assertEqual(bounded.tolist(),[-.05]*12)
        self.assertEqual(receipt['requested_q'],q)
        with self.assertRaises(JointPositionBoundaryError):app.bounded_joint_pose(q)
        with self.assertRaises(JointPositionBoundaryError):app.bounded_joint_pose([-.05-math.radians(2.1)]*12,feedback=True)
        app.config['command_joint_limits']=app.manual_joint_limits()
        app.hardware_feedback=None
        self.assertEqual(app.manual_joint_limits(),[[-.05,1.]]*12)
        app.config['command_joint_limits']=[[0.,float('nan')]]*12
        with self.assertRaises(ValueError):app.manual_joint_limits()

if __name__=='__main__':unittest.main()
