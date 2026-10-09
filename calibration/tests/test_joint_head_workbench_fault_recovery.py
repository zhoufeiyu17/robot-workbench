"""Fault explanations and fake-only service recovery; never starts systemd."""
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from joint_head_workbench import Workbench


class FaultRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.app=Workbench.__new__(Workbench)
        self.app.control_mode='hardware_sync'
        self.app.config={'simulation_only':True}
        self.owner=dict(pid=123,command_epoch=7,state='FAULT_STOP_UNCONFIRMED',
                        reason='control tick gap exceeds 0.1 s',simulation_only=True,
                        physical_hardware_connected=False,automatic_enabled=False,connected=True,feedback_healthy=True,
                        last_fault_diagnostic={'tick_gap_s':.1057,'wall_ns':123456})
        self.app.control_status={'owner':self.owner,'motion_enabled':False}
        self.app.control_received=time.monotonic()
        self.app.hardware_feedback={'owner':self.owner}
        self.app.hardware_received=time.monotonic()
        self.app.hardware_error=None;self.app.control_error=None
        self.app.fake_restart_lock=threading.Lock()
        self.app.fake_restart_pending={};self.app.fake_restart_in_progress=False
        self.app.pending_capture=None;self.app.importing=False;self.app.pipeline_running=False
        self.app.tokens={'old':{}};self.app.hardware_tokens={'old'};self.app.calibration_previews={'old':{}}
        self.app.idle=lambda:None
        self.app.dsw_connection=lambda:'connected'
        self.app.control_request=lambda path:self.app.control_status
        self.app.remote_request=lambda path:{'status':'READY','mirror':None}

    def test_fault_is_not_other_owner_and_recovery_is_source_scoped(self):
        self.assertEqual(self.app.robot_control(),'owned_by_workbench')
        blocker=self.app.execution_blocker()
        self.assertEqual((blocker['code'],blocker['action']),('owner_fault_fake_restart','restart_fake'))
        self.assertIn('105.7 ms',blocker['reason'])
        self.assertEqual(blocker['since_ns'],123456)
        self.app.dsw_connection=lambda:'stale'
        self.assertEqual(self.app.execution_blocker()['action'],'reconnect_dsw')
        self.app.dsw_connection=lambda:'connected'
        self.owner['recoverable_fault']='tracking deviation'
        self.assertEqual(self.app.execution_blocker()['action'],'recover')
        self.owner['recoverable_fault']=None
        self.app.config={'simulation_only':False};self.owner['simulation_only']=False
        self.owner['physical_hardware_connected']=True
        self.assertEqual((self.app.execution_blocker()['code'],self.app.execution_blocker()['action']),
                         ('owner_fault_physical',None))
        self.owner['state']='READY';self.app.control_status['motion_enabled']=True
        self.assertEqual(self.app.execution_blocker()['code'],'ready')
        self.app.hardware_feedback={'owner':dict(self.owner,pid=999)}
        self.assertEqual(self.app.execution_blocker()['code'],'control_other_owner')

    def test_fake_restart_checks_unit_identity_and_consumes_review_once(self):
        commands=[]
        def systemctl(argv,**kwargs):
            self.assertEqual(argv[:2],['/usr/bin/systemctl','--user'])
            commands.append(argv[2:])
            if argv[2]=='show':return SimpleNamespace(stdout='123\n')
            if argv[2]=='start':
                self.app.control_status={'owner':dict(self.owner,pid=456,state='READY',reason='fake device'),
                                         'motion_enabled':True,'host_sample_recent':True}
            return SimpleNamespace(stdout='')
        with patch('joint_head_workbench.subprocess.run',side_effect=systemctl):
            preview=self.app.fake_controller_restart_preview({})
            self.assertEqual((preview['owner_pid'],preview['command_epoch']),(123,7))
            reply=self.app.fake_controller_restart_confirm({k:preview[k] for k in ('token','owner_pid','command_epoch')})
            self.assertEqual(reply['owner_pid'],456)
            self.assertEqual(commands[-2:],[['stop','joint-head-fake-device.service'],
                                            ['start','joint-head-fake-device.service']])
            self.assertEqual((self.app.tokens,self.app.hardware_tokens,self.app.calibration_previews),({},set(),{}))
            with self.assertRaisesRegex(ValueError,'过期|变化'):
                self.app.fake_controller_restart_confirm({k:preview[k] for k in ('token','owner_pid','command_epoch')})

    def test_nonfake_and_moving_owner_cannot_restart(self):
        self.app.config={'simulation_only':False}
        with self.assertRaisesRegex(ValueError,'Fake'):
            self.app.fake_controller_restart_preview({})
        self.app.config={'simulation_only':True}
        self.owner['automatic_enabled']=True
        with self.assertRaisesRegex(ValueError,'空闲'):
            self.app.fake_controller_restart_preview({})
        self.owner['automatic_enabled']=False;self.owner['recoverable_fault']='tracking'
        with self.assertRaisesRegex(ValueError,'空闲'):
            self.app.fake_controller_restart_preview({})

    def test_restarting_overrides_cached_ready_and_failure_revokes_old_tokens(self):
        self.owner['state']='READY';self.app.control_status['motion_enabled']=True
        self.app.fake_restart_in_progress=True
        self.assertEqual(self.app.execution_blocker()['code'],'owner_restarting')
        self.app.fake_restart_in_progress=False
        self.owner['state']='FAULT_STOP_UNCONFIRMED';self.app.control_status['motion_enabled']=False
        def fail_start(argv,**kwargs):
            if argv[2]=='show':return SimpleNamespace(stdout='123')
            raise OSError('service unavailable')
        with patch('joint_head_workbench.subprocess.run',side_effect=fail_start):
            preview=self.app.fake_controller_restart_preview({})
            with self.assertRaisesRegex(ValueError,'停止.*失败'):
                self.app.fake_controller_restart_confirm({k:preview[k] for k in ('token','owner_pid','command_epoch')})
        self.assertFalse(self.app.fake_restart_in_progress)
        self.assertEqual(self.app.hardware_tokens,set())
        self.assertIn('上次恢复失败',self.app.execution_blocker()['detail'])
        self.assertNotEqual(self.app.execution_blocker()['code'],'ready')

    def test_changed_owner_expired_token_and_wrong_service_refuse_before_stop(self):
        with patch('joint_head_workbench.subprocess.run',return_value=SimpleNamespace(stdout='123')) as command:
            p=self.app.fake_controller_restart_preview({})
            self.owner['command_epoch']+=1
            with self.assertRaisesRegex(ValueError,'变化'):
                self.app.fake_controller_restart_confirm({k:p[k] for k in ('token','owner_pid','command_epoch')})
            self.assertTrue(all(c.args[0][2]=='show' for c in command.call_args_list))
            p=self.app.fake_controller_restart_preview({})
            self.app.fake_restart_pending[p['token']]=(time.monotonic()-1,123,8)
            with self.assertRaisesRegex(ValueError,'过期'):
                self.app.fake_controller_restart_confirm({k:p[k] for k in ('token','owner_pid','command_epoch')})
        with patch('joint_head_workbench.subprocess.run',return_value=SimpleNamespace(stdout='999')):
            with self.assertRaisesRegex(ValueError,'不一致'):
                self.app.fake_controller_restart_preview({})


if __name__=='__main__':unittest.main()
