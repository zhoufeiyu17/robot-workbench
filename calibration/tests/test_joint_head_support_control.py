import threading
import unittest
from types import SimpleNamespace

from joint_head_workbench import Workbench


class SupportControlTest(unittest.TestCase):
    def app(self, plans, check=None):
        app=Workbench.__new__(Workbench)
        app.config={'model_family':'ac_one'}
        app.control_mode='hardware_sync'
        app.mode_lock=threading.RLock()
        app.hardware_tokens=set()
        app._control_request=lambda action,body=None: {'token':'review','plans':plans}
        app.geometry=SimpleNamespace(support_path_check=check or (lambda a,b:{'ok':True}))
        return app

    def test_every_motion_preview_checks_frozen_owner_path_before_token(self):
        start=list(range(14));target=[v+.1 for v in start]
        plans=[{'q_start':start,'q_target':target}]
        for action in ('home-preview','arm-home-preview','both-home-preview',
                       'motion-preview','jaw-preview','trajectory-preview'):
            with self.subTest(action=action):
                calls=[]
                app=self.app(plans,lambda a,b:calls.append((a,b)) or {'ok':True})
                result=app.control_request(action,{})
                self.assertEqual(calls,[(start[:6]+start[7:13],target[:6]+target[7:13])])
                self.assertIn('review',app.hardware_tokens)
                self.assertEqual(result['support_checks'],[{'ok':True}])

    def test_calibration_trajectory_skips_support_checks_but_keeps_review_token(self):
        app=self.app(None,lambda *args:self.fail('calibration must not check collision'))
        result=app.control_request('trajectory-preview',{'waypoints':[]},calibration=True)
        self.assertEqual(result['token'],'review')
        self.assertIn('review',app.hardware_tokens)
        self.assertNotIn('support_checks',result)

    def test_rejected_later_segment_cannot_be_confirmed(self):
        plans=[{'q_start':[0.]*14,'q_target':[0.]*14},
               {'q_start':[0.]*14,'q_target':[1.]*14}]
        app=self.app(plans,lambda a,b:{'ok':b[0]==0,'details':{'depth_m':.0013}})
        with self.assertRaisesRegex(ValueError,'第 2 段'):
            app.control_request('trajectory-preview',{})
        self.assertFalse(app.hardware_tokens)
        with self.assertRaisesRegex(ValueError,'预览过期'):
            app.control_request('motion-confirm',{'token':'review'})

    def test_missing_or_invalid_frozen_plan_fails_closed(self):
        for plans in (None,[],[{}],[{'q_start':[float('nan')]*14,'q_target':[0.]*14}]):
            with self.subTest(plans=plans):
                app=self.app(plans)
                with self.assertRaises(ValueError):app.control_request('home-preview',{})
                self.assertFalse(app.hardware_tokens)

    def test_slow_check_does_not_return_expired_token(self):
        from unittest.mock import patch
        plans=[{'q_start':[0.]*14,'q_target':[0.]*14}]
        app=self.app(plans)
        with patch('joint_head_workbench.time.monotonic',side_effect=[0.,31.]):
            with self.assertRaisesRegex(ValueError,'预览已过期'):
                app.control_request('home-preview',{})
        self.assertFalse(app.hardware_tokens)

    def test_legacy_geometry_keeps_existing_control_contract(self):
        app=self.app(None)
        app.config={}
        app.control_request('home-preview',{})
        self.assertIn('review',app.hardware_tokens)


if __name__=='__main__':unittest.main()
