import threading
import unittest
from copy import deepcopy
from types import SimpleNamespace
import numpy as np

from joint_head_workbench import Workbench
from robot_workbench.joint_bounds import feedback_position_margin_rad


class SupportControlTest(unittest.TestCase):
    def app(self, plans, check=None):
        app=Workbench.__new__(Workbench)
        app.config={'model_family':'ac_one'}
        app.control_mode='hardware_sync'
        app.mode_lock=threading.RLock()
        app.hardware_tokens=set()
        app._control_request=lambda action,body=None: {'token':'review','plans':plans}
        app.geometry=SimpleNamespace(_q=lambda values:np.asarray(values,float),limits=np.tile([-2.,2.],(12,1)),
            support_path_check=lambda *args:self.fail('fine support gate must not run'))
        app.manual_self_collision=SimpleNamespace(check_path=check or (lambda a,b,**kwargs:{'ok':True}),description={'scope':'robot_self_spheres'})
        return app

    def test_every_motion_preview_checks_frozen_owner_path_before_token(self):
        start=[i/100 for i in range(14)];target=[v+.1 for v in start]
        plans=[{'q_start':start,'q_target':target}]
        for action in ('home-preview','arm-home-preview','both-home-preview',
                       'motion-preview','jaw-preview','trajectory-preview'):
            with self.subTest(action=action):
                calls=[]
                app=self.app(plans,lambda a,b,**kwargs:calls.append((list(a),list(b))) or {'ok':True})
                result=app.control_request(action,{})
                self.assertEqual(calls,[(start[:6]+start[7:13],target[:6]+target[7:13])])
                self.assertIn('review',app.hardware_tokens)
                self.assertEqual(result['collision_checks'],[{'ok':True}])
                self.assertNotIn('support_checks',result)

    def test_calibration_trajectory_skips_collision_checks_but_keeps_review_token(self):
        app=self.app(None,lambda *args:self.fail('calibration must not check collision'))
        result=app.control_request('trajectory-preview',{'waypoints':[]},calibration=True)
        self.assertEqual(result['token'],'review')
        self.assertIn('review',app.hardware_tokens)
        self.assertNotIn('collision_checks',result)

    def test_rejected_later_segment_cannot_be_confirmed(self):
        plans=[{'q_start':[0.]*14,'q_target':[0.]*14},
               {'q_start':[0.]*14,'q_target':[1.]*14}]
        app=self.app(plans,lambda a,b,**kwargs:{'ok':bool(b[0]==0),'collisions':[['left/link3','right/link4']]})
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

    def test_missing_coarse_checker_cannot_register_token(self):
        app=self.app([{'q_start':[0.]*14,'q_target':[0.]*14}])
        del app.manual_self_collision
        with self.assertRaisesRegex(ValueError,'检查器未就绪'):
            app.control_request('trajectory-preview',{})
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

    def boundary_preview(self, environment, raw_value):
        start=[0.]*14;start[1]=raw_value;start[6]=start[13]=.04
        target=[.1]*14;target[6]=target[13]=.04
        plans=[{'q_start':start,'q_target':target}];before=deepcopy(plans);calls=[]
        def strict_check(a,b,**kwargs):
            calls.append((list(a),list(b)))
            return {'ok':bool(a[1]>=0 and b[1]>=0),'reason':'joint limits'}
        app=self.app(plans,strict_check)
        app.config.update(environment=environment,simulation_only=environment=='fake')
        app.geometry.limits[1]=[0.,2.]
        result=app.control_request('trajectory-preview',{'waypoints':[{'q':[.1]*12}]})
        self.assertEqual(calls[0][0][1],0.)
        self.assertEqual(plans,before,'the Owner frozen plan must retain original feedback')
        self.assertEqual(result['plans'][0]['q_start'][1],raw_value)
        boundary=result['collision_checks'][0]['position_boundary']
        self.assertEqual(boundary['environment'],environment)
        self.assertEqual(boundary['q_start']['requested_q'][1],raw_value)
        self.assertEqual(boundary['q_start']['command_q'][1],0.)
        self.assertEqual(boundary['q_start']['corrections'][0]['original_rad'],raw_value)
        self.assertIn('review',app.hardware_tokens)

    def test_fake_frozen_start_accepts_existing_position_noise(self):
        self.boundary_preview('fake',-.004)

    def test_physical_frozen_start_accepts_existing_position_noise(self):
        self.boundary_preview('physical',-.03)

    def test_frozen_endpoints_beyond_position_noise_never_register_token(self):
        for environment,value in (('fake',-.006),('physical',-.011)):
            for endpoint in ('q_start','q_target'):
                with self.subTest(environment=environment,endpoint=endpoint):
                    plans=[{'q_start':[0.]*14,'q_target':[0.]*14}]
                    plans[0][endpoint][1]=-(feedback_position_margin_rad(environment)+.001) if endpoint=='q_start' else value
                    before=deepcopy(plans)
                    app=self.app(plans,lambda *args,**kwargs:self.fail('over-noise endpoint reached sphere check'))
                    app.config.update(environment=environment,simulation_only=environment=='fake')
                    app.geometry.limits[1]=[0.,2.]
                    with self.assertRaisesRegex(ValueError,'position-boundary noise allowance'):
                        app.control_request('trajectory-preview',{})
                    self.assertEqual(plans,before)
                    self.assertFalse(app.hardware_tokens)


if __name__=='__main__':unittest.main()
