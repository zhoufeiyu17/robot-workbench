"""CPU-only interruption and checkpoint rollback; no camera, DSW or robot."""
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event,Thread
from types import SimpleNamespace
from unittest.mock import patch
import json,sys,time
from cancellable_process import run
from playground import Playground
from playground_pipeline import run_stages


class Bridge:
    config={'playground':{'enabled':True}}
    def __init__(self):self.automatic=False;self.commands=[]
    def owner(self):return {'automatic_enabled':self.automatic}
    def control(self,command):
        self.commands.append(command)
        if command=={'action':'stop'}:self.automatic=False
    def status(self):
        return dict(connected=True,owner=dict(mode='hardware',feedback_healthy=True,
            state='READY',stationary=True,automatic_enabled=self.automatic,recording_configured=True,
            pedal={'input_healthy':True,'physically_down':False}),
            cameras={name:{'ready':True} for name in ('base_0','left_wrist_0','right_wrist_0')})


def main():
    with TemporaryDirectory() as folder:
        bridge=Bridge();service=Playground(folder,bridge,lambda:{'status':'ready'})
        service.state['environment']={'ready':True,'checks':[dict(key='dsw',status='ready')]}
        service.state['warmup']={'status':'ready','message':'model initialized'}
        service.state.update(status='failed',plan={'validated':False,'timeline_digest':'old'})
        service.checkpoint(dict(validated=True,timeline_digest='old'),execution_pending=True,execution_output=Path(folder)/'jobs/old')
        assert service.start({'action':'interrupt'})=={'status':'ready'}
        assert service.state['status']=='ready' and not service.state['plan']['validated']
        assert json.loads((Path(folder)/'prepared-plan.json').read_text())['invalidated_by_interrupt']
        entered=Event()
        def pending_plan(cfg,out,bridge,speed,progress=None,cancel=None):
            entered.set()
            while not cancel():time.sleep(.01)
            raise InterruptedError('cancelled')
        with patch('playground_pipeline.plan',pending_plan):
            service.start({'action':'plan','speed_scale':.3})
            assert entered.wait(1)
            assert service.start({'action':'interrupt'})=={'status':'stopping'}
            service.thread.join(2)
        assert not service.thread.is_alive() and service.state['status']=='ready'
        assert not service.state['busy'] and not service.state['plan']['validated']
        assert service.state['environment']['ready'] and service.state['progress']['stage_status']=='interrupted'
        # The stopped client waits for its bounded request without discarding a healthy worker.
        recovering=Event();allow_recovery=Event();callbacks=[]
        def recover(*,progress):
            callbacks.append('wait-owned-planner');recovering.set()
            assert allow_recovery.wait(2)
            callbacks.append('planner-ready')
        service.recover_planner=recover;entered.clear()
        with patch('playground_pipeline.plan',pending_plan):
            service.start({'action':'plan','speed_scale':.3});assert entered.wait(1)
            service.start({'action':'interrupt'});assert recovering.wait(1)
            assert service.busy and not service.snapshot()['can_execute']
            allow_recovery.set();service.thread.join(2)
        assert callbacks==['wait-owned-planner','planner-ready'] and service.state['status']=='ready'
        assert service.state['warmup']['status']=='ready'
        assert bridge.commands==[] and not service.state['plan']['validated']
        # Transport/receipt code may wrap InterruptedError as RuntimeError.
        def wrapped_plan(cfg,out,bridge,speed,progress=None,cancel=None):
            try:pending_plan(cfg,out,bridge,speed,progress,cancel)
            except InterruptedError as error:raise RuntimeError('transport interrupted') from error
        entered.clear();callbacks.clear()
        with patch('playground_pipeline.plan',wrapped_plan):
            service.start({'action':'plan','speed_scale':.3});assert entered.wait(1)
            service.start({'action':'interrupt'});service.thread.join(2)
        assert callbacks==['wait-owned-planner','planner-ready'] and service.state['status']=='ready'
        def failed_recovery(**kwargs):raise RuntimeError('owned planner recovery failed')
        service.recover_planner=failed_recovery;entered.clear()
        with patch('playground_pipeline.plan',pending_plan):
            service.start({'action':'plan','speed_scale':.3});assert entered.wait(1)
            service.start({'action':'interrupt'});service.thread.join(2)
        assert service.state['status']=='failed' and not service.state['environment']['ready']
        assert 'recovery failed' in service.state['error'] and not service.snapshot()['can_execute']
        # Explicit environment preparation may recover the planner alone after
        # a failed recovery. Existing camera/control owners are left connected.
        init={'status':'failed','planner':{'status':'failed'}}
        service.initialization=lambda:dict(init)
        def prepare_recovery(**kwargs):init.update(status='ready',planner={'status':'ready','model_initialized':False})
        def warmup(*args,**kwargs):
            init['planner']['model_initialized']=True
            return {'model_initialized':True}
        service.recover_planner=prepare_recovery
        with patch('playground_pipeline.prepare_environment',return_value=[dict(key='dsw',status='ready')]),patch('playground_pipeline.restore_plan',return_value=None),patch('playground_pipeline.warmup',warmup):
            service.start({'action':'prepare'});service.thread.join(2)
        assert service.state['status']=='ready' and service.snapshot()['environment']['ready']
        assert bridge.commands==[] and not service.state['plan']['validated']
        # An in-flight physical action must stop the owner before cancellation.
        bridge.automatic=True;service.state.update(busy=True,action='execute')
        assert service.start({'action':'interrupt'})=={'status':'stopping'}
        assert bridge.commands==[{'action':'stop'}] and not bridge.automatic
    cancelled=Event()
    Thread(target=lambda:(time.sleep(.1),cancelled.set()),daemon=True).start()
    started=time.monotonic()
    try:run([sys.executable,'-c','import time;time.sleep(10)'],cancel=cancelled.is_set,timeout=12)
    except InterruptedError:pass
    else:raise AssertionError('owned process survived interruption')
    assert time.monotonic()-started<2
    with TemporaryDirectory() as folder:
        cancelled=Event();entered=Event()
        def progress(*args,**kwargs):
            if kwargs.get('stage_status')=='running':entered.set()
        Thread(target=lambda:(entered.wait(1),time.sleep(.1),cancelled.set()),daemon=True).start()
        with patch('playground_pipeline.bounded_command',return_value=(
                [sys.executable,'-c','import time;time.sleep(10)'],12)):
            started=time.monotonic()
            try:run_stages([dict(name='playground-plan')],folder,
                           {'resources':{'max_wall_time_minutes':1}},progress,cancelled.is_set)
            except InterruptedError:pass
            else:raise AssertionError('planning subprocess survived interruption')
            assert time.monotonic()-started<2
    print('PASS stage interruption, checkpoint invalidation, environment rollback, owned subprocess cancellation; no motion')

if __name__=='__main__':main()
