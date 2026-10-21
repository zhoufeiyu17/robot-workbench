"""Explicit GPU qualification: compare light locked-FK updates with native full rebuild."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time


def main(args):
    from local_execution import register_auxiliary
    register_auxiliary(args,__file__)
    args.output.mkdir(parents=True,exist_ok=False)
    import torch
    from curobo.types.base import TensorDeviceType
    from curobo.types.robot import RobotConfig
    from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
    from motiongen_motion import checked_locked_transforms,locked_transform_config,sync_locked_joint_metadata
    payload=args.robot_model.read_bytes();saved=json.loads(payload)
    config=deepcopy(saved['config']);kin=config['kinematics']
    kin['link_names']=[kin['ee_link'],'right_link17','right_link18']
    tensor=TensorDeviceType(device=torch.device('cuda:0'))
    began=time.perf_counter()
    model=CudaRobotModel(RobotConfig.from_dict(config,tensor).kinematics)
    torch.cuda.synchronize();initial_seconds=time.perf_counter()-began
    current=model.kinematics_config
    addresses={name:getattr(current,name).data_ptr() for name in ('fixed_transforms','link_spheres','link_map','joint_map')}
    initial_spheres=current.link_spheres.clone()
    bounds=saved['proxy']['kinematics']['joints']
    def offset(name,value,delta):
        b=bounds[name]
        return value+delta if value+delta<=b['upper'] else max(b['lower'],value-delta)
    starts=[kin['cspace']['retract_config'],[offset(name,q,.01) for name,q in
            zip(kin['cspace']['joint_names'],kin['cspace']['retract_config'])]]
    q=tensor.to_device(starts)
    base=dict(kin['lock_joints']);changed=dict(base)
    changed['right_joint11']=offset('right_joint11',base['right_joint11'],.01)
    for a,b in (('left_joint7','left_joint8'),('right_joint17','right_joint18')):
        changed[a]=changed[b]=offset(a,base[a],.001)
    checks=[]
    for label,values in (('right_arm_and_both_jaws',changed),('reset_to_original',base)):
        section=time.perf_counter()
        full=deepcopy(config);full['kinematics']['lock_joints']=dict(values)
        reference=CudaRobotModel(RobotConfig.from_dict(full,tensor).kinematics)
        torch.cuda.synchronize();full_seconds=time.perf_counter()-section
        section=time.perf_counter()
        light=RobotConfig.from_dict(locked_transform_config(config,values),tensor).kinematics.kinematics_config
        transforms=checked_locked_transforms(current,light,torch)
        transform_error=float((transforms-reference.kinematics_config.fixed_transforms).abs().max())
        current.fixed_transforms.copy_(transforms)
        sync_locked_joint_metadata(model,values)
        torch.cuda.synchronize();light_seconds=time.perf_counter()-section
        actual=model.get_state(q);expected=reference.get_state(q)
        errors={name:float((getattr(actual,name)-getattr(expected,name)).abs().max()) for name in
                ('ee_position','ee_quaternion','links_position','links_quaternion','link_spheres_tensor')}
        preserved=(addresses=={name:getattr(current,name).data_ptr() for name in addresses}
                   and torch.equal(initial_spheres,current.link_spheres))
        passed=transform_error<=1e-7 and max(errors.values())<=5e-7 and preserved
        row=dict(case=label,passed=passed,full_native_build_seconds=full_seconds,
            transform_only_seconds=light_seconds,maximum_transform_error=transform_error,
            fk_maximum_absolute_errors=errors,collision_buffers_and_addresses_preserved=preserved,
            locked_joints=values)
        checks.append(row)
        if not passed:raise ValueError('native locked FK comparison failed: '+json.dumps(row))
        del reference,light
    result=dict(schema='motiongen_locked_native_equivalence.v1',passed=True,
        robot_model_sha256=hashlib.sha256(payload).hexdigest(),initial_build_seconds=initial_seconds,
        checks=checks,scope='native full versus transform-only FK and sphere placement; no planning, motion or task-success claim')
    (args.output/'result.json').write_text(json.dumps(result,sort_keys=True,allow_nan=False)+'\n')
    print(json.dumps(result,sort_keys=True,allow_nan=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-model',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--execution-json')
    main(parser.parse_args())
