"""CPU-only check against a supplied checked plan; no hardware or network."""
from copy import deepcopy
import json
from pathlib import Path
import sys
from frozen_executor import FrozenExecutor
from half_speed_plan import resume_initial_open
from integration import _digest
from real_contract import validate_real_plan


def main(folder):
    root=Path(folder)
    source,plan,binding=[json.loads((root/(n+'.json')).read_text()) for n in ('source','plan','hardware-binding')]
    original=deepcopy(plan)
    resumed=resume_initial_open(plan,source,binding,39.89,minimum_tick=482)
    cut=resumed['execution_resume']['skipped_ticks']
    assert cut>=482 and plan==original
    assert [t['target'] for t in resumed['ticks']]==[t['target'] for t in plan['ticks'][cut:]]
    assert resumed['native_actions']==plan['native_actions'][cut:]
    assert resumed['ticks'][0]['target']['left_gripper']>=39.89
    assert resumed['stages'][0]==dict(name='initial_open',start_tick=0,start_s=0.)
    assert resumed['ticks'][-1]['target']==plan['ticks'][-1]['target']
    for change in ('left_arm_joints','right_gripper'):
        bad=deepcopy(resumed)
        if change.endswith('joints'):bad['ticks'][0]['target'][change][0]+=.001
        else:bad['ticks'][0]['target'][change]+=.001
        bad['timeline_digest']=_digest(bad['ticks'])
        bad['execution_resume']['first_target_digest']=_digest(bad['ticks'][0]['target'])
        bad['planning_validation']['execution_resume_digest']=_digest(bad['execution_resume'])
        try:validate_real_plan(bad,source,binding)
        except ValueError:pass
        else:raise AssertionError('non-gripper initial state changed')
    for candidate in (plan,resumed):
        sent=[];stops=[];now=0.
        def sink(target):
            if not sent or sent[-1][0]!=executor.command_index:
                sent.append((executor.command_index,now,target))
        executor=FrozenExecutor(candidate,source,sink=sink,gate=lambda:True,stop=stops.append,
            source_validator=lambda p,s:validate_real_plan(p,s,binding))
        def feedback():return dict(sample_monotonic_s=now,state=sent[-1][2] if sent else candidate['ticks'][0]['target'])
        executor.start(now,feedback());slips=[]
        for i in range(1,40000):
            now+=.05041973799 if i%997==50 else .01
            old=executor.schedule_slips;result=executor.tick(now,feedback())
            if executor.schedule_slips>old:slips.append(executor.last_tick)
            assert result['status']!='failed',result
            if result['status']=='completed':break
        assert result['status']=='completed' and not stops and slips
        assert [i for i,_,_ in sent]==list(range(len(candidate['ticks'])))
        assert [q for _,_,q in sent]==[t['target'] for t in candidate['ticks']]
        assert all(sent[i+1][1]-sent[i][1]>=1/30-1e-9 for i in slips if i+1<len(sent))
        print(json.dumps(dict(targets=len(sent),duration_s=candidate['duration_s'],
            schedule_slips=len(slips),schedule_delay_s=executor.scheduling_delay_s,all_ordered=True,hardware=False)))
    from tempfile import TemporaryDirectory
    from unittest.mock import patch
    import shlex
    from playground_pipeline import prepare_remote_resume
    with TemporaryDirectory() as folder:
        remote=Path(folder);(remote/'detection-bridge').mkdir()
        for name,data in [('source',source),('hardware-binding',binding)]:
            (remote/'detection-bridge'/(name+'.json')).write_text(json.dumps(data))
        parent=remote/'parent.json';parent.write_text(json.dumps(plan))
        cfg=dict(remote_run_root=str(remote),workbench_code=str(Path(__file__).resolve().parent))
        with patch('playground_pipeline.ssh',side_effect=lambda side,cmd:[sys.executable,*shlex.split(cmd)[1:]]):
            destination=prepare_remote_resume(cfg,resumed,str(parent))
            assert json.loads(Path(destination).read_text())==resumed
            assert prepare_remote_resume(cfg,resumed,str(parent))==destination
            Path(destination).write_text('{}')
            try:prepare_remote_resume(cfg,resumed,str(parent))
            except Exception:pass
            else:raise AssertionError('corrupt cached suffix accepted')
    print('PASS initial-opening suffix, native parity, mutation guards, complete jitter runs, remote derivation/cache digest')

if __name__=='__main__':main(sys.argv[1])
