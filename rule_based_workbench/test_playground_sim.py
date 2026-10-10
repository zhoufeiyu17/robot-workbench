"""Pure Python distance decoder checks. No Isaac imports, GPU, or robot access."""
from copy import deepcopy
from types import SimpleNamespace
from tempfile import TemporaryDirectory
from pathlib import Path
from unittest.mock import patch
from dataclasses import dataclass
import io
import json
import sys
from contract import demo_state
from joint_space import convert_state
from playground_sim import (decode_target, normalize_initial_arm_limits, validate_native_limits, setup,
                            validate_inputs, runtime_replay_plan, runtime_initial_model, replay,
                            validate_l3_replay_derivation)
from test_frozen_executor import fixture
from test_joint_space import a1_profile


class Spec:
    left=right=SimpleNamespace(arm=SimpleNamespace(soft_lower_limits_rad=(-2.,)*6,soft_upper_limits_rad=(2.,)*6))
    def encode(self,command):
        return [*(q/2 for q in command.left.joint_positions_rad),1.,
                *(q/2 for q in command.right.joint_positions_rad),1.]
    def decode(self,raw):
        assert raw[6]==raw[13]==1
        if any(not -1<=v<=1 for v in raw):raise ValueError('normalized limit')
        return SimpleNamespace(left=SimpleNamespace(joint_positions_rad=tuple(2*v for v in raw[:6])),
                               right=SimpleNamespace(joint_positions_rad=tuple(2*v for v in raw[7:13])))


class NativeSpec:
    """CPU codec for the zero-home native arm contract; no Isaac dependency."""
    left=right=SimpleNamespace(arm=SimpleNamespace(
        soft_lower_limits_rad=(-1.83222,0.,0.,-1.5039,-1.5039,-1.8846),
        soft_upper_limits_rad=(2.87982,3.48175,3.078,1.5039,1.5039,1.8846)))
    def encode(self,command):
        result=[]
        for side in ('left','right'):
            arm=getattr(self,side).arm
            for value,lo,hi in zip(getattr(command,side).joint_positions_rad,
                                   arm.soft_lower_limits_rad,arm.soft_upper_limits_rad):
                if not lo<=value<=hi:raise ValueError('native soft limit')
                result.append(2*(value-lo)/(hi-lo)-1)
            result.append(1.)
        return result
    def decode(self,raw):
        if raw[6]!=1 or raw[13]!=1 or any(not -1<=v<=1 for v in raw):raise ValueError('normalized limit')
        values={}
        for side,start in (('left',0),('right',7)):
            arm=getattr(self,side).arm
            values[side]=SimpleNamespace(joint_positions_rad=tuple(lo+(q+1)*(hi-lo)/2 for q,lo,hi
                in zip(raw[start:start+6],arm.soft_lower_limits_rad,arm.soft_upper_limits_rad)))
        return SimpleNamespace(**values)


def action_module(spec):
    return SimpleNamespace(ArmCommand=lambda joints,opened:SimpleNamespace(joint_positions_rad=joints),
        DualArmCommand=lambda left,right:SimpleNamespace(left=left,right=right),
        ac_one_dual_arm_zero_home_action_spec=lambda:spec)


def native_action(target,source,spec):
    model=convert_state(target,source['calibration_profile'],to_reported=False)
    raw=spec.encode(SimpleNamespace(**{side:SimpleNamespace(joint_positions_rad=model[side+'_arm_joints'])
                                      for side in ('left','right')}))
    raw[6],raw[13]=model['left_gripper'],model['right_gripper']
    return raw


def derivation_fixture(environment='fake',overflow=None):
    """Warm-153813's measured prefix and once-converted right J3 failure."""
    from execution_timeline import build
    from integration import _digest
    from real_contract import namespace
    from real_observation import read_initial_sample,seal_initial_state
    from test_initial_state import owner
    from test_physical_motiongen import fake_inputs
    source,plan,_=fake_inputs();profile=source['calibration_profile']
    profile['joints']['zero_offsets_rad']=[.001798129906836862,.0038220501680543673,.002793825889624288,
        .00940147851670417,.005506399504360409,.006774389877607065,-.004353344895872246,
        -.0022818841467342706,-.0009886907126574016,.007953737754499916,-.007416092073027134,.0015374572394383271]
    measured=demo_state()
    measured.update(left_arm_joints=[-.02603786811232567,.012441361322999,.0023289674427360296,
        -.01677761971950531,.008740301243960857,.08624594658613205],
        right_arm_joints=[.01921522058546543,.01878148317337036,.0003104299248661846,
        -.023985443636775017,-.007784130983054638,.05566935986280441],
        left_gripper=71.23463408517837,right_gripper=83.00000236797332)
    if overflow is not None:
        profile['joints']['zero_offsets_rad'][8]=-overflow;measured['right_arm_joints'][2]=0.
    before=read_initial_sample(lambda:owner(measured),wall_ns=lambda:1_003_000_000)
    after=read_initial_sample(lambda:owner(measured,stamp=9_000_000_000),wall_ns=lambda:9_003_000_000)
    source.update(schema_version=namespace(environment)['source_schema'],environment=environment,
        evidence_source=namespace(environment)['evidence'],initial_state=seal_initial_state(before,after,profile))
    if environment=='fake':source['simulation_only']=True
    target=deepcopy(source['initial_state']['planned_reported'])
    target['right_arm_joints'][2]=-profile['joints']['zero_offsets_rad'][8]
    plan.update(ticks=[dict(tick=i,time_s=i/30,target=deepcopy(target)) for i in range(3)],
        stages=[dict(name='initial_hold',start_tick=0)],native_substeps=[8]*3,duration_s=.1)
    spec=NativeSpec()
    plan['native_actions']=[native_action(row['target'],source,spec) for row in plan['ticks']]
    seal_geometric(plan,source)
    limits={side+'_arm_joints':list(zip(spec.left.arm.soft_lower_limits_rad,spec.left.arm.soft_upper_limits_rad))
            for side in ('left','right')}
    limits.update(left_gripper=[.008,83.],right_gripper=[.008,83.])
    timeline=build(plan,current_state=measured,limits=limits,environment=environment)
    return source,plan,timeline,spec


def seal_geometric(plan,source):
    from integration import _digest
    plan['timeline_digest']=_digest(plan['ticks'])
    plan['planning_validation']=dict(schema_version='geometric_path_validation.v1',passed=True,
        timeline_digest=plan['timeline_digest'],source_digest=_digest(source),
        collision_coverage=['base_link','left','right'],max_sample_step_rad=.01)


def reject(call,reason):
    try:call()
    except ValueError as error:
        assert reason in str(error),(reason,str(error));return
    raise AssertionError('invalid L3 derivation accepted: '+reason)


def l3_derivation_check():
    from execution_timeline import build
    from integration import _digest
    source,parent,timeline,spec=derivation_fixture()
    before=deepcopy((source,parent,timeline))
    with patch.dict(sys.modules,{'robot_sim.specs.action':action_module(spec)}):
        view=validate_l3_replay_derivation(parent,source,timeline,spec)
        saved=view['l3_derivation']
        assert saved['prefix_count']==1 and saved['source_environment']=='fake' and saved['limit_allowance_rad']==.005
        assert saved['adjustments']==[dict(tick=0,source_tick=None,joint='right_3',
            parent_reported_rad=.0003104299248661846,derived_reported_rad=.0009886907126574016,
            parent_model_rad=-.000678260787791217,derived_model_rad=0.,delta_rad=.000678260787791217)]
        assert saved['replay_matches_shared_commands'] is False
        assert view['native_actions'][0][9]==-1.
        assert view['ticks'][1:]==timeline['commands'][1:]
        assert json.dumps(view['native_actions'][1:])==json.dumps(parent['native_actions'])
        assert view['native_substeps'][1:]==parent['native_substeps']
        assert validate_l3_replay_derivation(parent,source,timeline,spec,json.loads(json.dumps(saved)))==view
        changed=deepcopy(saved);changed['adjustments'][0]['delta_rad']+=.0001
        changed['derivation_digest']=_digest({k:v for k,v in changed.items() if k!='derivation_digest'})
        reject(lambda:validate_l3_replay_derivation(parent,source,timeline,spec,changed),'differs')
        for environment,bound in (('fake',.005),('physical',.01)):
            for overflow,accepted in ((bound,True),(bound+.000001,False)):
                s,p,t,sp=derivation_fixture(environment,overflow)
                if accepted:
                    receipt=validate_l3_replay_derivation(p,s,t,sp)['l3_derivation']
                    assert receipt['limit_allowance_rad']==bound and receipt['source_environment']==environment
                else:reject(lambda:validate_l3_replay_derivation(p,s,t,sp),'position-boundary noise allowance')
        # A suffix is decoded unchanged; even a tiny native overflow is never clamped.
        bad=deepcopy(parent);bad['native_actions'][1][9]=-1.0000001
        reject(lambda:validate_l3_replay_derivation(bad,source,timeline,spec),'normalized limit')
        for key,value in (('max_target_velocity_rad_s',.001),('max_target_acceleration_rad_s2',.1)):
            bad=deepcopy(parent);bad['ticks'][1]['target']['left_arm_joints'][0]+=.001
            bad['native_actions'][1]=native_action(bad['ticks'][1]['target'],source,spec)
            bad['control'][key]=value;seal_geometric(bad,source)
            derived=build(bad,current_state=timeline['initialization']['actual_start'],
                limits=timeline['initialization']['command_limits'],environment='fake')
            reject(lambda:validate_l3_replay_derivation(bad,source,derived,spec),'velocity/acceleration')
    assert (source,parent,timeline)==before
    replay_artifact_check(source,parent,timeline,spec)


def execution_artifact_check():
    from execution_timeline import build
    from physical_motiongen import freeze_physical
    from test_physical_motiongen import fake_inputs
    from integration import _digest,SIM_PLANNER_ID
    source,model,binding=fake_inputs()
    model['native_actions']=[[*[q/2 for q in row['target']['left_arm_joints']],row['target']['left_gripper'],
                             *[q/2 for q in row['target']['right_arm_joints']],row['target']['right_gripper']]
                            for row in model['ticks']]
    parent=freeze_physical(model,source,binding)
    suite={'episodes':[{**deepcopy(source['scene']['episode_scene']),
        'metadata':dict(initial_robot_state=deepcopy(source['initial_state']['model']),
                        initial_robot_state_digest=_digest(source['initial_state']['model']))}]}
    current=deepcopy(parent['ticks'][0]['target'])
    current['left_arm_joints'][0]+=.02;current['right_arm_joints'][1]-=.03
    current['left_gripper']=84.  # Preserve the physical measurement; only commands clamp.
    limits={side+'_arm_joints':[[-1.,1.]]*6 for side in ('left','right')}
    limits.update(left_gripper=[.008,83.],right_gripper=[.008,83.])
    timeline=build(parent,current_state=current,limits=limits)
    before=deepcopy((parent,source,suite))
    validate_inputs(source,binding,suite,parent,timeline)
    commands=SimpleNamespace(ArmCommand=lambda joints,opened:SimpleNamespace(joint_positions_rad=joints),
        DualArmCommand=lambda left,right:SimpleNamespace(left=left,right=right),
        ac_one_dual_arm_zero_home_action_spec=lambda:Spec())
    with patch.dict(sys.modules,{'robot_sim.specs.action':commands}):
        view=runtime_replay_plan(parent,source,timeline,Spec())
        partial=deepcopy(parent);partial['native_substeps'][-1]=4;partial['duration_s']-=4/240
        partial_timeline=build(partial,current_state=current,limits=limits)
        validate_inputs(source,binding,suite,partial,partial_timeline)
        assert runtime_replay_plan(partial,source,partial_timeline,Spec())['native_substeps'][-1]==4
    prefix=len(timeline['commands'])-len(parent['ticks'])
    assert prefix>0 and view['native_actions'][prefix:]==parent['native_actions']
    assert view['native_substeps']==[8]*prefix+parent['native_substeps']
    assert view['ticks']==timeline['commands'] and view['timeline_digest']==parent['timeline_digest']
    assert runtime_initial_model(source,timeline)['left_gripper']==82.992
    assert timeline['initialization']['actual_start']['left_gripper']==84.
    assert (parent,source,suite)==before
    geometric=deepcopy(parent);geometric['planner_id']=SIM_PLANNER_ID;geometric.pop('physical_gpu_conversion')
    geometric['planning_validation']=dict(schema_version='geometric_path_validation.v1',passed=True,
        timeline_digest=geometric['timeline_digest'],source_digest=_digest(source),
        collision_coverage=['base_link','left','right'],max_sample_step_rad=.01)
    validate_inputs(source,binding,suite,geometric,timeline)
    bad=deepcopy(timeline);bad['commands'][0]['target']['right_arm_joints'][0]+=.01
    try:validate_inputs(source,binding,suite,parent,bad)
    except ValueError:pass
    else:raise AssertionError('modified initialization accepted')
    bad_parent=deepcopy(geometric);bad_parent['planning_validation']['passed']=False
    try:validate_inputs(source,binding,suite,bad_parent,timeline)
    except ValueError:pass
    else:raise AssertionError('unverified parent accepted')
    replay_artifact_check(source,parent,timeline,Spec())


def replay_artifact_check(source,parent,timeline,spec):
    from isaac_lane import file_sha256
    from playground_pipeline import verify_l3_derivation
    commands=action_module(spec)
    with patch.dict(sys.modules,{'robot_sim.specs.action':commands}):
        view=runtime_replay_plan(parent,source,timeline,spec)
    class Tensor:
        dtype='fixture-float';device='fixture-cpu'
        def __init__(self,rows):self.rows=rows
        def cpu(self):return self
        def tolist(self):return self.rows
        def unsqueeze(self,axis):return Tensor([self.rows])
        def __getitem__(self,key):
            if isinstance(key,tuple):
                row,indices=key;return Tensor([self.rows[row][i] for i in indices])
            return Tensor(self.rows[key])
    class Robot:
        device='fixture-cpu'
        data=SimpleNamespace(joint_pos=Tensor([[0.]*16]),joint_pos_limits=Tensor([[[-3.,3.]]*16]))
        def find_joints(self,names,preserve_order):return list(range(len(names))),names
        def set_joint_position_target(self,targets,joint_ids):self.data.joint_pos=targets
    class Scene(dict):
        steps=[]
        def write_data_to_sim(self):pass
        def update(self,dt):self.steps.append(dt)
    class Process:
        def __init__(self,argv,**options):self.path=Path(argv[-1]);self.stdin=io.BytesIO();self.done=False
        def wait(self,**kw):self.path.write_bytes(b'fake-video');self.done=True;return 0
        def poll(self):return 0 if self.done else None
        def terminate(self):self.done=True
    class Frame:
        shape=(480,640,3)
        def tobytes(self):return b'fake-camera'
    scene=Scene(robot=Robot());physics=[];renders=[]
    env=SimpleNamespace(scene=scene,_sim_step_counter=0,sim=SimpleNamespace(step=lambda render:physics.append(render)),
        _m2_contact_reader=SimpleNamespace(begin_step=lambda step:None,read=lambda step:[]))
    runtime=SimpleNamespace(absolute_step=0,_render=lambda:renders.append(True))
    with TemporaryDirectory() as directory:
        output=Path(directory);(output/'camera-metadata.json').write_text('{}')
        probe={'codec_name':'h264','pix_fmt':'yuv420p','nb_read_frames':str(len(view['ticks'])),
               'duration':str(view['duration_s'])}
        with patch.dict(sys.modules,{'robot_sim.specs.action':commands,
            'torch':SimpleNamespace(tensor=lambda rows,**kw:Tensor(rows))}),\
            patch('playground_sim.camera_frame',return_value=Frame()),\
            patch('playground_sim.sensor_pose',return_value={}),\
            patch('playground_sim.shutil.which',return_value='fake-ffmpeg'),\
            patch('playground_sim.subprocess.Popen',side_effect=Process),\
            patch('playground_sim.subprocess.run',return_value=SimpleNamespace(stdout=json.dumps({'streams':[probe]}))):
            result=replay(SimpleNamespace(output=output),source,parent,env,runtime,
                          dict.fromkeys(('base_0','left_wrist_0','right_wrist_0'),object()),timeline)
        rows=[json.loads(line) for line in (output/'execution.jsonl').read_text().splitlines()]
        assert [row['target'] for row in rows]==[row['target'] for row in view['ticks']]
        assert [row['shared_command'] for row in rows]==timeline['commands']
        assert [row['l3_derived_command'] for row in rows]==view['ticks']
        assert result['execution_timeline_digest']==timeline['timeline_digest'] and result['command_count']==len(rows)
        assert result['parent_timeline_digest']==parent['timeline_digest'] and result['parent_command_count']==len(parent['ticks'])
        assert result['task_outcome']=='unverified' and result['fps']==30 and result['collision_audit_rerun'] is False
        assert len(physics)==sum(view['native_substeps']) and len(renders)==len(rows)
        assert all(dt==1/240 for dt in scene.steps)
        saved=json.loads((output/result['l3_derivation_ref']).read_text())
        assert saved==view['l3_derivation']
        assert result['l3_derivation_sha256']==file_sha256(output/result['l3_derivation_ref'])
        assert result['execution_timeline_sha256']==file_sha256(output/result['execution_timeline_ref'])
        assert json.loads((output/'manifest.json').read_text())==result
        with patch.dict(sys.modules,{'robot_sim.specs.action':commands,
                'torch':SimpleNamespace(tensor=lambda rows,**kw:Tensor(rows))}):
            verify_l3_derivation(result,saved,parent,source,timeline,spec=spec)
            reject(lambda:verify_l3_derivation(dict(result,l3_commands_digest='tampered'),saved,parent,source,timeline,spec=spec),'differs')
            with patch('playground_sim.validate_native_limits',side_effect=ValueError('hard limit sentinel')) as check:
                invalid=output/'invalid';invalid.mkdir()
                reject(lambda:replay(SimpleNamespace(output=invalid),source,parent,env,runtime,{},timeline),'hard limit sentinel')
                assert check.call_count==1


def reset_policy_check():
    """Position conversion allowance must not widen the reset readback gate."""
    from integration import _digest
    @dataclass
    class Acceptance:
        open_position_m:float=0.
    for environment,overflow,error,reason in (
        ('fake',.005,.004,None),('physical',.007,.004,None),
        ('fake',.005001,0.,'position-boundary noise allowance'),
        ('physical',.010001,0.,'position-boundary noise allowance'),
        ('physical',.007,.005001,'over 0.005 rad')):
        source,plan,timeline,spec=derivation_fixture(environment,overflow)
        suite={'episodes':[dict(metadata=dict(initial_robot_state=deepcopy(source['initial_state']['model'])))]}
        original=deepcopy((source,suite,timeline))
        def make_env(cfg):
            initial=cfg.episode['metadata']['initial_robot_state']
            observed=initial['left_arm_joints']+initial['right_arm_joints'];observed[8]+=error
            runtime=SimpleNamespace(loop=SimpleNamespace(physics_steps=0),
                read=lambda step:(SimpleNamespace(arm_positions=observed),{}),_render=lambda:None)
            return SimpleNamespace(_m2_runtime=runtime,reset=lambda **kw:(None,{}),
                action_manager=SimpleNamespace(get_term=lambda key:SimpleNamespace(gripper_limits=lambda:[])))
        modules={
            'isaaclab.app':SimpleNamespace(AppLauncher=lambda **kw:SimpleNamespace(app=object())),
            'robot_sim.specs.action':action_module(spec),
            'robot_sim.tasks.manager_based.coffee_make.env':SimpleNamespace(CoffeeMakeEnv=make_env),
            'robot_sim.tasks.manager_based.coffee_make.episode_scene':SimpleNamespace(EpisodeScene=SimpleNamespace(from_dict=lambda value:value)),
            'robot_sim.tasks.manager_based.coffee_make.generalization_env_cfg':SimpleNamespace(
                make_generalization_env_cfg=lambda **kw:SimpleNamespace(**kw,scene=SimpleNamespace(**{
                    'd405_'+name:SimpleNamespace() for name in ('base_0','left_wrist_0','right_wrist_0')}))),
            'robot_sim.tasks.manager_based.coffee_make.generalization_execution':SimpleNamespace(ControlSpec=lambda *a,**kw:SimpleNamespace()),
            'robot_sim.tasks.manager_based.coffee_make.interaction_evaluation':SimpleNamespace(EpisodeAcceptanceSpec=SimpleNamespace(development=Acceptance))}
        with TemporaryDirectory() as directory:
            args=SimpleNamespace(assets=Path(directory),output=Path(directory),gpu=0,environment='fake')
            with patch.dict(sys.modules,modules),patch('playground_sim.apply_whitebox_materials'),\
                    patch('playground_sim.bind_effective_gripper_identity'),patch('playground_sim.close_runtime'),\
                    patch('playground_sim.traceback.print_exc'):
                if reason:reject(lambda:setup(args,source,suite,plan,timeline),reason)
                else:
                    setup(args,source,suite,plan,timeline)
                    receipt=json.loads((args.output/'initial-normalization.json').read_text())
                    assert receipt['source_environment']==environment
                    assert receipt['limit_allowance_rad']==(.005 if environment=='fake' else .01)
                    assert receipt['input_initial_digest']==_digest(runtime_initial_model(source,timeline))
                    assert abs(receipt['adjustments'][0]['delta_rad']-overflow)<1e-12
        assert (source,suite,timeline)==original


def main():
    execution_artifact_check()
    l3_derivation_check()
    reset_policy_check()
    arm=SimpleNamespace(soft_lower_limits_rad=(-1.,-1.,0.,-1.,-1.,-1.),
                        soft_upper_limits_rad=(1.,1.,3.078,1.,1.,1.))
    spec=SimpleNamespace(left=SimpleNamespace(arm=arm),right=SimpleNamespace(arm=arm))
    measured=demo_state();measured['right_arm_joints']=measured['right_arm_joints'].copy()
    measured['right_arm_joints'][2]=-.0023598581578148214
    normalized,adjustments=normalize_initial_arm_limits(measured,spec)
    assert normalized['right_arm_joints'][2]==0 and measured['right_arm_joints'][2]<0
    assert len(adjustments)==1 and abs(adjustments[0]['delta_rad']-.0023598581578148214)<1e-12
    for value in (-.005001,3.083001):
        out_of_range=deepcopy(measured);out_of_range['right_arm_joints'][2]=value
        try:normalize_initial_arm_limits(out_of_range,spec)
        except ValueError:pass
        else:raise AssertionError('initial pose outside 5 mrad allowance accepted')
    within=deepcopy(measured);within['right_arm_joints'][2]=3.082
    assert normalize_initial_arm_limits(within,spec)[0]['right_arm_joints'][2]==3.078
    source,_=fixture();source['calibration_profile']=a1_profile(source['calibration_profile'])
    source['calibration_profile']['joints']['zero_offsets_rad']=[.01]*12
    model=demo_state();model.update(left_arm_joints=[.2]*6,right_arm_joints=[-.4]*6,
                                   left_gripper=70.992,right_gripper=82.992)
    target=convert_state(model,source['calibration_profile'],to_reported=True)
    raw=[.1]*6+[70.992]+[-.2]*6+[82.992]
    out=decode_target(raw,target,source,Spec())
    assert len(out)==16 and out['left_joint1']==.2 and out['right_joint11']==-.4
    assert out['left_joint7']==out['left_joint8']==.035496
    assert out['right_joint17']==out['right_joint18'] and abs(out['right_joint17']-.041496)<1e-12
    assert target['left_gripper']==71 and target['right_gripper']==83
    for slot,value in ((6,-1),(6,71),(13,83),(0,.2),(0,2),(6,float('nan'))):
        bad=raw.copy();bad[slot]=value
        try:decode_target(bad,target,source,Spec())
        except ValueError:pass
        else:raise AssertionError('invalid model/reported/native contract accepted')
    assert raw==[.1]*6+[70.992]+[-.2]*6+[82.992]
    validate_native_limits([[.041496]],['left_joint7'],[[0.,.041496]])
    for target in (.041497,-.000001,float('nan')):
        try:validate_native_limits([[target]],['left_joint7'],[[0.,.041496]])
        except ValueError:pass
        else:raise AssertionError('invalid hard-limit target accepted')
    with TemporaryDirectory() as directory:
        args=SimpleNamespace(assets=Path(directory),output=Path(directory),gpu=0)
        modules={'isaaclab.app':SimpleNamespace(AppLauncher=lambda **kw:SimpleNamespace(app=object())),
                 'robot_sim.tasks.manager_based.coffee_make.env':SimpleNamespace()}
        with patch.dict(sys.modules,modules),patch('playground_sim.close_runtime',side_effect=SystemExit(0)),patch('playground_sim.traceback.print_exc'):
            try:setup(args,source,{},None)
            except SystemExit:pass
            else:raise AssertionError('expected mocked Kit shutdown')
        receipt=json.loads((Path(directory)/'result.json').read_text())
        assert receipt['status']=='failed' and receipt['phase']=='setup' and receipt['error']
    print('Playground CPU: warm-153813 L3 derivation, source fake5/physical10 bounds, immutable suffix, saved receipt/receiver tamper guards, complete control, hard limits, reset5 and replay artifacts PASS; physics/cameras unverified')


if __name__=='__main__':main()
