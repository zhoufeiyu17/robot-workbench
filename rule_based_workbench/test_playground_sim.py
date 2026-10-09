"""Pure Python distance decoder checks. No Isaac imports, GPU, or robot access."""
from copy import deepcopy
from types import SimpleNamespace
from tempfile import TemporaryDirectory
from pathlib import Path
from unittest.mock import patch
import io
import json
import sys
from contract import demo_state
from joint_space import convert_state
from playground_sim import (decode_target, normalize_initial_arm_limits, validate_native_limits, setup,
                            validate_inputs, runtime_replay_plan, runtime_initial_model, replay)
from test_frozen_executor import fixture
from test_joint_space import a1_profile


class Spec:
    def encode(self,command):
        return [*(q/2 for q in command.left.joint_positions_rad),1.,
                *(q/2 for q in command.right.joint_positions_rad),1.]
    def decode(self,raw):
        assert raw[6]==raw[13]==1
        if any(not -1<=v<=1 for v in raw):raise ValueError('normalized limit')
        return SimpleNamespace(left=SimpleNamespace(joint_positions_rad=tuple(2*v for v in raw[:6])),
                               right=SimpleNamespace(joint_positions_rad=tuple(2*v for v in raw[7:13])))


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
        assert [row['target'] for row in rows]==[row['target'] for row in timeline['commands']]
        assert [row['shared_command'] for row in rows]==timeline['commands']
        assert result['execution_timeline_digest']==timeline['timeline_digest'] and result['command_count']==len(rows)
        assert result['parent_timeline_digest']==parent['timeline_digest'] and result['parent_command_count']==len(parent['ticks'])
        assert result['task_outcome']=='unverified' and result['fps']==30 and result['collision_audit_rerun'] is False
        assert len(physics)==sum(view['native_substeps']) and len(renders)==len(rows)
        assert all(dt==1/240 for dt in scene.steps)


def main():
    execution_artifact_check()
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
    print('Playground CPU fake: parent proofs, 30 Hz initialization/suffix, reported/model conversion, command digest and video cadence PASS; physics/cameras unverified')


if __name__=='__main__':main()
