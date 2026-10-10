"""CPU fake-native checks; these are interface tests, never collision evidence."""
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest.mock import patch
import sys

from integration import _digest
from joint_space import convert_state
from playground_plan import build, audit
from test_frozen_executor import fixture
from test_initial_state import initial_fixture
from test_joint_space import a1_profile


class Pose:
    def __init__(self, position=(0.,0.,0.), orientation_wxyz=(1,0,0,0), **_):
        self.position=tuple(position);self.orientation_wxyz=tuple(orientation_wxyz)
    def compose(self,other): return Pose(tuple(a+b for a,b in zip(self.position,other.position)))
    def inverse(self): return Pose(tuple(-a for a in self.position))


class Arm:
    def __init__(self,q,opened): self.joint_positions_rad=q; self.gripper_open=opened
class Dual:
    def __init__(self,left,right): self.left=left; self.right=right
class Spec:
    left=right=NS(arm=NS(soft_lower_limits_rad=(-2.,)*6,soft_upper_limits_rad=(2.,)*6))
    def encode(self,q): return [v/2 for v in q.left.joint_positions_rad]+[1 if q.left.gripper_open else -1]+[v/2 for v in q.right.joint_positions_rad]+[1 if q.right.gripper_open else -1]
    def decode(self,raw): return Dual(Arm(tuple(v*2 for v in raw[:6]),raw[6]==1),Arm(tuple(v*2 for v in raw[7:13]),raw[13]==1))
class Row(list):
    def tolist(self): return list(self)
def trajectory(rows,*_,**__): return NS(positions=[Row(row) for row in rows])


def main():
    source,_=fixture()
    for key in ('lane','adapter','instance_id','gpu'):source.pop(key,None)
    source.update(schema_version='physical_collection_source.v1',evidence_source='physical_rgbd',robot_id='cpu-fixture',camera_serials=dict(head='h',left='l',right='r'))
    source['calibration_profile']=a1_profile(source['calibration_profile'])
    source['calibration_profile']['joints']['zero_offsets_rad']=[.01]*12
    source['initial_state']=initial_fixture(source['calibration_profile'])
    model=source['initial_state']['model']
    start={**dict(zip((f'left_joint{i}' for i in range(1,7)),model['left_arm_joints'])),**dict(zip((f'right_joint1{i}' for i in range(1,7)),model['right_arm_joints'])), 'left_joint7':.041496,'left_joint8':.041496,'right_joint17':.041496,'right_joint18':.041496}
    contact={**start,'left_joint7':.038,'left_joint8':.038}
    q=model['left_arm_joints']; row=dict(q=q)
    initial=dict(passed=True,approach={'approach_000':row,'approach_001':row},selected=dict(closing=dict(terminal_state=contact),lift=dict(checks=[row]),placement=dict(stages={n:[row] for n in ('raise','transport','lower','separate','retreat')},candidate=dict(world_from_cup=dict(position=(5,6,7))))))
    initial['selected']['closing'].update(passed=True,world_from_cup=dict(position=(15,26.001,37)),samples=[dict(jaws={k:start[k] for k in ('left_joint7','left_joint8')},world_from_cup=dict(position=(15,26,37))),dict(jaws={k:contact[k] for k in ('left_joint7','left_joint8')},world_from_cup=dict(position=(15,26.001,37)))])
    calls=[]
    def corridor(fk,previous,state,**kw):
        calls.append((deepcopy(previous),deepcopy(state),kw))
        return dict(free=True,actual_samples=2)
    geo=NS(mount=Pose((10,20,30)),corridor=corridor,audit=lambda *a,**k:NS(free=True,violations=[]))
    contexts=[]
    class Reuse:
        def __init__(self,fk,**options):
            self.fk,self.options=fk,options;self.closed=False;self.passed=set()
            self.stats=dict.fromkeys(('visits','hits','misses','stored','capacity_bypasses','peak_entries'),0)
            contexts.append(self)
        def __enter__(self):return self
        def __exit__(self,*args):self.closed=True;self.passed.clear()
        def check(self,start,goal,**resolution):
            assert not self.closed
            result=geo.corridor(self.fk,start,goal,**self.options,**resolution)
            for state in (start,goal):
                key=tuple(sorted(state.items()));self.stats['visits']+=1
                if key in self.passed:self.stats['hits']+=1
                else:
                    self.stats['misses']+=1
                    if result['free']:self.passed.add(key);self.stats['stored']+=1
            self.stats['peak_entries']=max(self.stats['peak_entries'],len(self.passed))
            return result
    geo.trajectory_state_reuse=Reuse
    fk=NS(forward=lambda state:{'left_link6':Pose((1,2,3))},pose=lambda x:x)
    planner=NS(open_position_m=.041496,identity=dict(chassis_collision={'schema':'fake-test'}),fk=fk,geometry=lambda obs:(geo,deepcopy(start)))
    obs=NS(sample=NS(arm_positions=model['left_arm_joints']+model['right_arm_joints'],world_from_cup=Pose((15,26,37))))
    modules={'robot_sim.asset_cfg.objects.coffee_machine':NS(RigidTransform=Pose),'robot_sim.policies.continuous_motion':NS(MotionLimits=lambda *x:x,continuous_trajectory=trajectory),'robot_sim.specs.action':NS(ArmCommand=Arm,DualArmCommand=Dual,ac_one_dual_arm_zero_home_action_spec=Spec)}
    with patch.dict(sys.modules,modules):
        plan=build(planner,obs,initial,source,dict(control=dict(physics_hz=240,substeps=8)),1.)
        assert len(calls)==len(plan['ticks'])
        assert contexts and all(c.closed and not c.passed for c in contexts)
        reuse=plan['planning_validation']['state_reuse']
        assert reuse['totals']['hits']>0 and reuse['totals']['visits']==2*len(calls)
        assert len(contexts)>1, 'different contact/cup semantics must have separate caches'
        assert calls[0][2]['initial_table_contact']
        assert all(c[2]['maximum_joint_step']==.01 for c in calls)
        stages=plan['stages']; close=next(i for i,s in enumerate(stages) if s['name']=='close_and_hold')
        assert stages[close+1]['start_tick']-stages[close]['start_tick']==18
        for tick in plan['ticks']:
            held=next(s['name'] for s in reversed(stages) if s['start_tick']<=tick['tick']) in {'close_and_hold','lift','raise','transport','lower','support_settle'}
            i=tick['tick']; state=calls[i][1]; options=calls[i][2]
            assert all(abs(a-b)<1e-12 for a,b in zip(tick['target']['right_arm_joints'],source['initial_state']['planned_reported']['right_arm_joints']))
            assert abs(plan['native_actions'][i][6]+.008-tick['target']['left_gripper'])<1e-10
            if held: assert tick['target']['left_gripper']==65.
            if held:
                assert all(abs(a-b)<1e-10 for a,b in zip(options['tool_from_cup'].position,(4,4.001,4))), 'centered cup or world mount missing from attachment'
                assert state['left_joint7']==state['left_joint8']==.038, 'preload command used as geometric contact'
            stage_name=next(s['name'] for s in reversed(stages) if s['start_tick']<=i)
            if stage_name in ('open','release_hold','separate','retreat','home','home_hold'):
                assert options['tool_from_cup'] is None, 'empty hand departure still carries cup'
                assert options['world_from_cup'].position==(5,6,7)
                assert options['grasp_contact']==(stage_name in ('open','release_hold','separate'))
        assert plan['planning_validation']['samples']>2*len(calls)
        assert plan['simulation']==dict(validated=False,outcome='not_run')
        opening=next(i for i,s in enumerate(stages) if s['name']=='open')
        first,last=stages[opening]['start_tick'],stages[opening+1]['start_tick']
        gaps=[t['target']['left_gripper'] for t in plan['ticks'][first:last]]
        assert gaps[0]==plan['ticks'][first-1]['target']['left_gripper']==65.
        assert abs(gaps[-1]-83.)<1e-10, (gaps[0],gaps[-1])
        assert all(0 <= b-a <= .20000001 for a,b in zip(gaps,gaps[1:]))
        # Fake actuator: both the observed ~71.5 mm and a wider 77 mm contact.
        # Neither a compressed/absent cup nor rigid contact may cause a release jump.
        # Timestamped feedback is one 120 Hz cycle behind the submitted target.
        from frozen_executor import FrozenExecutor
        from real_contract import validate_real_plan
        binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id=source['robot_id'],camera_serials=source['camera_serials'],source_digest=_digest(source),calibration_profile_digest=_digest(source['calibration_profile']),initial_state=source['initial_state'])
        def run(stuck=False,contact_gap=77.,value=plan):
            desired=deepcopy(value['ticks'][0]['target']); actual=deepcopy(desired)
            def sink(target):desired.update(deepcopy(target))
            executor=FrozenExecutor(value,source,sink=sink,gate=lambda:True,stop=lambda _:None,source_validator=lambda p,s:validate_real_plan(p,s,binding))
            def feedback(now):return dict(sample_monotonic_s=now,state=deepcopy(actual))
            result=executor.start(0.,feedback(0.))
            assert result['status']=='running',result
            release=next(s['start_s'] for s in value['stages'] if s['name']=='open')
            for step in range(1,int(value['duration_s']*120)+50):
                now=step/120
                old=actual['left_gripper']; goal=max(contact_gap,desired['left_gripper'])
                actual.update(deepcopy(desired))
                actual['left_gripper']=old+max(-.1,min(.1,goal-old))
                if stuck and now>=release:actual['left_gripper']=contact_gap
                result=executor.tick(now,feedback(now))
                if result['status']!='running':return result
            raise AssertionError('fake follower did not terminate')
        for contact_gap in (71.,71.5,77.):
            assert run(contact_gap=contact_gap)['status']=='completed', 'rate-limited follower rejected opening'
        assert run(stuck=True)['status']=='failed', 'opening tracking protection bypassed'
        original_checks=len(calls)
        calls.clear()
        slow=build(planner,obs,initial,source,dict(control=dict(physics_hz=240,substeps=8)),.3)
        assert len(calls)==original_checks<len(slow['ticks']), 'speed interpolation repeated collision checks'
        assert slow['planning_validation']['timeline_digest']==slow['timeline_digest']
        assert slow['planning_validation']['input_ticks']==original_checks
        for contact_gap in (71.,71.5,77.):
            assert run(contact_gap=contact_gap,value=slow)['status']=='completed'
            assert run(stuck=True,contact_gap=contact_gap,value=slow)['status']=='failed'
        asymmetric=deepcopy(source)
        for field in ('measured_reported','planned_reported','after_observation_reported'):
            asymmetric['initial_state'][field]['left_gripper']=11.48
        asymmetric['initial_state']['model']=convert_state(asymmetric['initial_state']['planned_reported'],asymmetric['calibration_profile'],to_reported=False)
        calls.clear()
        with patch.object(planner,'geometry',lambda obs:(geo,{**start,'left_joint7':.005736,'left_joint8':.005736})):
            prepared=build(planner,obs,initial,asymmetric,dict(control=dict(physics_hz=240,substeps=8)),1.)
        assert abs(prepared['ticks'][0]['target']['left_gripper']-11.48)<1e-9
        assert abs(prepared['ticks'][-1]['target']['left_gripper']-11.48)<1e-9
        for name in ('initial_open','home_gripper_restore'):
            index=next(i for i,s in enumerate(prepared['stages']) if s['name']==name)
            lo,hi=prepared['stages'][index]['start_tick'],prepared['stages'][index+1]['start_tick']
            values=[t['target']['left_gripper'] for t in prepared['ticks'][lo:hi]]
            assert all(abs(b-a)<=.20000001 for a,b in zip(values,values[1:]))
            assert all(t['target']['left_arm_joints']==prepared['ticks'][0]['target']['left_arm_joints']
                       for t in prepared['ticks'][lo:hi])
            assert all(c[2]['initial_table_contact']==(name=='initial_open') for c in calls[lo:hi])
        assert all(t['target']['right_gripper']==83. for t in prepared['ticks'])
        bad_initial=deepcopy(initial);bad_initial['selected']['closing']['terminal_state']['left_joint7']-=.001
        try:build(planner,obs,bad_initial,source,dict(control=dict(physics_hz=240,substeps=8)),.3)
        except ValueError:pass
        else:raise AssertionError('old independently stopped fingers accepted')
        geo.audit=lambda *a,**k:NS(free=False,violations=['synthetic swept cup collision'])
        swept=deepcopy(plan)
        try:audit(swept,source,planner,obs,initial)
        except ValueError as error:assert 'coupled closing sweep collision' in str(error)
        else:raise AssertionError('cup centering sweep skipped')
        assert 'planning_validation' not in swept
        assert all(c.closed and not c.passed for c in contexts), 'failed audit retained reusable state'
        geo.audit=lambda *a,**k:NS(free=True,violations=[])
        # Audit actual right targets, not a frozen start substituted for them.
        changed=deepcopy(plan); changed['ticks'][0]['target']['right_arm_joints'][0]+=.2
        changed['timeline_digest']=_digest(changed['ticks']);calls.clear()
        audit(changed,source,planner,obs,initial)
        assert abs(calls[0][1]['right_joint11']-(start['right_joint11']+.2))<1e-12
        geo.corridor=lambda *a,**k:dict(free=False,actual_samples=1,failures=['synthetic collision'])
        try:audit(changed,source,planner,obs,initial)
        except ValueError:pass
        else:raise AssertionError('collision accepted')
        assert all(c.closed and not c.passed for c in contexts)
        assert 'planning_validation' not in changed, 'stale approval retained after failed audit'
        try:build(planner,obs,initial,source,dict(control=dict(physics_hz=120,substeps=8)))
        except ValueError:pass
        else:raise AssertionError('15 Hz incorrectly accepted')
    print('Playground native-interface, full-state audit, frame and fixed-hold CPU checks: PASS')

if __name__=='__main__':main()
