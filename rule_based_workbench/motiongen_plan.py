"""Dual-DSW rule poses -> cuRobo MotionGen paths -> GPU acceptance."""
import argparse
from collections import Counter
from copy import deepcopy
from dataclasses import asdict
from fractions import Fraction
import hashlib
import importlib
import json
from real_contract import is_device_source
import math
from pathlib import Path
from types import SimpleNamespace
import time
from uuid import uuid4

from contract import demo_state
from gripper_distance_plan import HOLDING_STAGES, NATIVE_DISTANCE_ENCODING
from integration import CUROBO_PLANNER_ID, DualGpuPipeline, PIPELINE_LANES, SIM_POLICY_ID, _digest
from a1_asset_derivation import CERTIFICATES,PROOF,certificate_path

LEFT = tuple(f'left_joint{i}' for i in range(1, 7))
RIGHT = tuple(f'right_joint1{i}' for i in range(1, 7))
WAITS = {'preclose_settle', 'close_and_hold', 'support_settle', 'release_hold', 'home_hold'}


def write_new(path, value):
    payload=json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(',',':'))
    with Path(path).open('x') as stream:
        stream.write(payload)


def gripper_geometry(planner, urdf_path):
    """Derive metal-vs-travel offset from both URDF zero frames and metal polygons."""
    import numpy as np
    import xml.etree.ElementTree as ET
    from robot_sim.asset_cfg.robots.ac_one_finger_contact import ac_one_finger_contact_spec
    spec = ac_one_finger_contact_spec()
    if spec.rubber_thickness_m != .0015:
        raise ValueError('reviewed 1.5mm rubber pads required')
    zero = dict.fromkeys(planner.fk.movable, 0.)
    frames = planner.fk.forward(zero)
    root, measured = ET.parse(urdf_path).getroot(), []
    for side, offset in (('left', 0), ('right', 10)):
        fingers = (f'{side}_link{offset+7}', f'{side}_link{offset+8}')
        joints = (f'{side}_joint{offset+7}', f'{side}_joint{offset+8}')
        wrist = f'{side}_link{offset+6}'
        inverse = np.linalg.inv(frames[wrist])
        transformed = []
        upper = []
        for name, joint in zip(fingers, joints):
            pad = next(pad for pad in spec.pads if pad.link_name == name)
            transform = inverse@frames[name]
            transformed.append(np.asarray(pad.metal_surface_polygon_m)@transform[:3, :3].T+transform[:3, 3])
            limit = root.find(f"joint[@name='{joint}']/limit")
            if limit is None or float(limit.get('lower')) != 0.:
                raise ValueError('explicit zero-based finger limits required')
            upper.append(float(limit.get('upper')))
        moved = planner.fk.forward(dict(zero, **{joints[0]: .001}))
        axis = (inverse@moved[fingers[0]])[:3, 3]-(inverse@frames[fingers[0]])[:3, 3]
        axis /= np.linalg.norm(axis)
        moved_other = planner.fk.forward(dict(zero, **{joints[1]: .001}))
        other_axis = (inverse@moved_other[fingers[1]])[:3, 3]-(inverse@frames[fingers[1]])[:3, 3]
        other_axis /= np.linalg.norm(other_axis)
        if np.dot(axis, other_axis) > -.999999 or abs(upper[0]-upper[1]) > 1e-12:
            raise ValueError('opposed symmetric finger translations required')
        metal_offset = float((transformed[0]@axis).min()-(transformed[1]@axis).max())*1000
        measured.append(dict(side=side, native_travel_offset_mm=metal_offset, upper_per_finger_m=upper[0],
                             open_travel_mm=sum(upper)*1000))
    if abs(measured[0]['native_travel_offset_mm']-measured[1]['native_travel_offset_mm']) > 1e-6:
        raise ValueError('left/right metal travel offsets differ')
    opening = measured[0]['open_travel_mm']
    if abs(opening-measured[1]['open_travel_mm']) > 1e-9 or abs(opening-planner.open_position_m*2000) > 1e-9:
        raise ValueError('URDF and exported gripper opening differ')
    offset = sum(row['native_travel_offset_mm'] for row in measured)/2
    if not math.isfinite(offset) or not 0 <= 65.-offset <= opening:
        raise ValueError('reviewed metal target outside derived model travel')
    return dict(native_travel_offset_mm=offset, open_travel_mm=opening, max_metal_gap_mm=opening+offset,
                closed_metal_gap_mm=65., closed_travel_mm=65.-offset, rubber_gap_mm=62.,
                pad_thickness_m=spec.rubber_thickness_m, finger_contact_spec_sha256=spec.digest(), measured=measured)


def world_config(planner, cup_pose, fixed_meshes=()):
    """All source geometry is world-frame; MotionGen's world is robot base-frame."""
    from curobo.geom.types import Cuboid, Cylinder, Mesh, WorldConfig
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    base_from_world = planner.geo.mount.inverse()
    boxes, meshes, descriptor, cup_names = [], [], [], []
    for shape in (*planner.geo.scene.shapes, planner.table, *planner.obstacles):
        pose = base_from_world.compose(shape.world_from_shape)
        if shape.kind == 'box':
            boxes.append(Cuboid(name=shape.shape_path, pose=[*pose.position, *pose.orientation_wxyz], dims=list(shape.dimensions_m)))
        elif shape.kind == 'cylinder':
            axis = 'XYZ'.index(shape.axis)
            radial = next(index for index in range(3) if index != axis)
            rotation = {'X': (math.sqrt(.5), 0., math.sqrt(.5), 0.),
                        'Y': (math.sqrt(.5), math.sqrt(.5), 0., 0.), 'Z': (1., 0., 0., 0.)}[shape.axis]
            pose = pose.compose(RigidTransform(orientation_wxyz=rotation))
            meshes.append(Cylinder(name=shape.shape_path, pose=[*pose.position, *pose.orientation_wxyz],
                radius=shape.dimensions_m[radial]/2, height=shape.dimensions_m[axis]).get_mesh(process=False))
        else:
            raise ValueError('unsupported frozen world primitive: '+shape.kind)
        descriptor.append(dict(name=shape.shape_path, kind=shape.kind, dimensions_m=shape.dimensions_m, base_from_shape=asdict(pose)))
    for index, band in enumerate(planner.cup.bands):
        pose = base_from_world.compose(cup_pose).compose(RigidTransform((0., 0., (band.z_min_m+band.z_max_m)/2)))
        name = f'motiongen_cup_band_{index}'
        if getattr(band,'radius_start_m',None) is not None:
            vertices,faces=band.mesh()
            meshes.append(Mesh(name=name,pose=[*pose.position,*pose.orientation_wxyz],vertices=vertices,faces=faces))
        else:
            meshes.append(Cylinder(name=name, pose=[*pose.position, *pose.orientation_wxyz],
                radius=band.radius_m, height=band.z_max_m-band.z_min_m).get_mesh(process=False))
        cup_names.append(name)
        descriptor.append(dict(name=name, kind='cup_frustum_mesh' if getattr(band,'radius_start_m',None) is not None else 'cup_cylinder_mesh', radius_m=band.radius_m,
                               height_m=band.z_max_m-band.z_min_m, base_from_shape=asdict(pose)))
    for item in fixed_meshes:
        pose = RigidTransform(**item['base_from_shape'])
        meshes.append(Mesh(name=item['name'], pose=[*pose.position, *pose.orientation_wxyz],
                           vertices=item['vertices'], faces=item['faces']))
        descriptor.append({key: value for key, value in item.items() if key not in ('vertices', 'faces')})
    return WorldConfig(cuboid=boxes, mesh=meshes), cup_names, descriptor


def attach_cup(engine, q, cup_names):
    from curobo.geom.sphere_fit import SphereFitType
    attached = engine.generator.attach_objects_to_robot(engine._start(q), cup_names,
        surface_sphere_radius=.0001, link_name='attached_object',
        sphere_fit_type=SphereFitType.VOXEL_VOLUME_INSIDE, voxelize_method='subdivide')
    if not attached:
        raise ValueError('MotionGen cup attachment failed')
    spheres = engine.generator.robot_cfg.kinematics.kinematics_config.get_link_spheres('attached_object').detach().cpu().tolist()
    if not any(sphere[3] > 0 for sphere in spheres):
        raise ValueError('MotionGen attached cup has no active proxy spheres')
    prepared_path=certificate_path('a1_grasp_contact.json',engine.config['kinematics']['urdf_path'])
    prepared=json.loads(prepared_path.read_text()) if prepared_path.exists() else None
    contact=engine.cup_contact.certify_held(engine,q,prepared)
    return dict(claw_cup_contact=contact,method='MotionGen.attach_objects_to_robot', fit='VOXEL_VOLUME_INSIDE/subdivide',
                sphere_sha256=_digest(spheres), active_spheres=sum(s[3] > 0 for s in spheres),
                proxy_only=True, complete_cup_checked_by_native=False, requires_complete_path_validation=True)


def target(q, right, gap, right_gap):
    return dict(demo_state(), left_arm_joints=list(q), right_arm_joints=list(right),
                left_gripper=float(gap), right_gripper=float(right_gap))


def jaw_rows(q, right, start, end, right_gap, *, max_step_mm=.2):
    if not math.isfinite(max_step_mm) or max_step_mm<=0:raise ValueError('positive jaw step required')
    steps = max(1, math.ceil(abs(end-start)/max_step_mm))
    return [target(q, right, end if i == steps else start+(end-start)*i/steps, right_gap) for i in range(steps+1)]


def initial_motion_state(planner, data, source, spec, opening):
    """Measured active-arm start; inactive arm/jaws retain their actual command."""
    from isaac_lane import simulation_initial_command
    measured = dict(planner.home_state)
    if any(not math.isfinite(value) for value in measured.values()):
        raise ValueError('nonfinite simulation initial feedback')
    start, projections = dict(measured), []
    physical=is_device_source(source)
    if physical:
        from joint_space import validate_initial_state
        initial=validate_initial_state(source.get('initial_state'),source['calibration_profile'])
        frozen=dict(schema_version='physical_model_initial_command.v1',
            evidence_source=('fake_device_source' if source.get('environment')=='fake' else 'physical_source')+'.initial_state.model',source_digest=_digest(source),
            initial_state_digest=_digest(initial),target=deepcopy(initial['model']))
    else:frozen = data.get('simulation_initial_command')
    offline = frozen is None and (source.get('synthetic_fixture') is True or source.get('test_fixture') is True)
    if frozen is None and not offline:
        raise ValueError('new simulation geometry requires simulation_initial_command from runtime.loop.previous')
    if physical:
        command=frozen['target']
    elif frozen is not None:
        expected = simulation_initial_command(frozen['native_action'], spec, opening, frozen['physics_step'])
        if frozen != expected:
            raise ValueError('exported simulation reset command differs from its native decode')
        command = frozen['target']
    else:
        # Historical inputs have no command evidence. This branch is accepted
        # only by the explicitly synthetic/offline verifier, never the CLI.
        command = target([measured[name] for name in LEFT], [measured[name] for name in RIGHT],
            (measured['left_joint7']+measured['left_joint8'])*1000,
            (measured['right_joint17']+measured['right_joint18'])*1000)
    start.update(zip(RIGHT, command['right_arm_joints']))
    for names, arm in ((LEFT, spec.left.arm), (RIGHT, spec.right.arm)):
        for name, lo, hi in zip(names, arm.soft_lower_limits_rad, arm.soft_upper_limits_rad, strict=True):
            original = start[name]
            if not math.isfinite(original):
                raise ValueError('nonfinite simulation initial joint: '+name)
            projected = min(hi, max(lo, original))
            if abs(projected-original) > .005+1e-12 or (names == RIGHT and not offline and not physical and original != projected):
                raise ValueError('simulation initial joint exceeds soft-limit allowance: '+name)
            if original != projected:
                projections.append(dict(joint=name, measured_rad=original, projected_rad=projected,
                                        lower_rad=lo, upper_rad=hi))
            start[name] = projected
    maximum = max(abs(start[name]-measured[name]) for name in (*LEFT, *RIGHT))
    if maximum > .005+1e-12:
        raise ValueError('measured/planned initial arm difference exceeds 0.005 rad')
    for side, names in (('left', ('left_joint7', 'left_joint8')), ('right', ('right_joint17', 'right_joint18'))):
        gap = command[side+'_gripper']
        if not math.isfinite(gap) or not 0 <= gap <= opening:
            raise ValueError('simulation initial jaw command outside exported model opening')
        for name in names:
            start[name] = gap/2000
    if offline:
        frozen = dict(schema_version='offline_initial_command_fallback.v1',
            evidence_source='offline_fixture_measured_state_fallback', observed_reset_command=False,
            target=target([start[n] for n in LEFT], [start[n] for n in RIGHT],
                          start['left_joint7']*2000, start['right_joint17']*2000))
    return start, dict(simulation_initial_command=deepcopy(frozen), measured_joint_state=measured,
        planned_joint_state=start, soft_limit_projection=projections, max_arm_adjustment_rad=maximum,
        arm_allowance_rad=.005, inactive_arm_source=frozen['evidence_source'],
        measured_to_planned_geometry_checked_by='gpu_final_acceptance',
        max_finger_adjustment_mm=max(abs(start[n]-measured[n])*1000 for n in
                                    ('left_joint7', 'left_joint8', 'right_joint17', 'right_joint18')))


def encode_stage(stage, spec, opening, cache):
    """Freeze arms once; decoded commands are the geometry and replay truth."""
    from isaac_lane import native_target
    from robot_sim.specs.action import ArmCommand, DualArmCommand
    result, actions, error = [], [], 0.
    for row in stage['rows']:
        key = tuple(row['left_arm_joints']+row['right_arm_joints'])
        if key not in cache:
            action = list(spec.encode(DualArmCommand(ArmCommand(tuple(row['left_arm_joints']), True),
                                                     ArmCommand(tuple(row['right_arm_joints']), True))))
            action[6] = action[13] = 0.
            decoded = native_target(action, spec, opening, distance=True)
            cache[key] = cache[tuple(decoded['left_arm_joints']+decoded['right_arm_joints'])] = action
        action = cache[key].copy()
        action[6], action[13] = row['left_gripper'], row['right_gripper']
        decoded = native_target(action, spec, opening, distance=True)
        error = max(error, max(abs(a-b) for name in ('left_arm_joints', 'right_arm_joints')
                               for a, b in zip(row[name], decoded[name])))
        if error > 1e-12:
            raise ValueError('native action encoding changed model joints beyond roundoff')
        actions.append(action); result.append(decoded)
    return dict(stage, rows=result, native_actions=actions, encoding_roundoff_rad=error,
                input_joint_path_sha256=_digest([row['left_arm_joints'] for row in stage['rows']]))


def scaled_stages(stages, speed, spec, opening):
    """Uniformly resample source time; preserve stage endpoints and fixed waits."""
    if type(speed) not in (int, float) or not math.isfinite(speed) or not .1 <= speed <= 1.:
        raise ValueError('speed must be between 0.1 and 1.0')
    scale = Fraction(str(speed))
    result, mapping = [], []
    from isaac_lane import native_target
    for stage in stages:
        rows = stage['rows']
        if stage['name'] in WAITS:
            result.append(deepcopy(stage)); mapping.append(list(range(len(rows))))
            continue
        intervals = math.ceil(Fraction(len(rows)-1)/scale)
        source_times = [Fraction(i*(len(rows)-1), max(1, intervals)) for i in range(intervals+1)]
        count, derived, actions, rounding = len(source_times), [], [], 0.
        for index in range(count):
            a = min(math.floor(source_times[index]), len(rows)-1)
            b = min(a+1, len(rows)-1)
            alpha = float(source_times[index]-a) if a != b else 0.
            action = [x+(y-x)*alpha if alpha else x for x, y in zip(stage['native_actions'][a], stage['native_actions'][b])]
            row = native_target(action, spec, opening, distance=True)
            rounding = max(rounding, max(abs(value-(x+(y-x)*alpha)) for key in ('left_arm_joints', 'right_arm_joints')
                for value, x, y in zip(row[key], rows[a][key], rows[b][key])))
            if rounding > 1e-12:
                raise ValueError('resampled action decode differs from checked polyline beyond roundoff')
            derived.append(row)
            actions.append(action)
        result.append(dict(stage, rows=derived, native_actions=actions, interpolation_roundoff_rad=rounding)); mapping.append([float(t) for t in source_times])
    return result, mapping


def enforce_stage_motion_limits(stages, control, spec, opening):
    """Use existing time interpolation only for native outputs that exceed limits."""
    from motiongen_motion import discrete_motion
    result, adjustments = [], []
    velocity=control['max_target_velocity_rad_s'];acceleration=control['max_target_acceleration_rad_s2']
    for stage in stages:
        if len(stage['rows'])<2:
            result.append(stage);continue
        original=stage; speed=1.
        for _ in range(8):
            check=discrete_motion([r['left_arm_joints'] for r in stage['rows']],velocity,acceleration)
            if check['passed']:break
            if original['name'] in WAITS:raise ValueError('stationary wait has invalid arm motion')
            speed*=.9*min(1.,velocity/max(check['max_velocity_rad_s'],1e-12),
                          acceleration/max(check['max_acceleration_rad_s2'],1e-12))
            if speed<.1:raise ValueError('native stage needs more than allowed time scaling')
            scaled,times=scaled_stages([original],speed,spec,opening);stage=scaled[0]
        else:raise ValueError('native stage time interpolation did not meet control limits')
        if speed<1.:
            adjustments.append(dict(stage=original.get('substage',original['name']),speed_scale=speed,
                parent_joint_path_sha256=_digest([r['left_arm_joints'] for r in original['rows']]),
                emitted_joint_path_sha256=_digest([r['left_arm_joints'] for r in stage['rows']]),
                source_sample_times=times[0],original_ticks=len(original['rows']),ticks=len(stage['rows']),
                control_validation=check,collision_audit_rerun=False,
                method='existing uniform stage time interpolation; unchanged joint polyline'))
        result.append(stage)
    return result, adjustments


def freeze(stages, source, data, grip, spec, *, run_id):
    from isaac_lane import native_target
    ticks, actions, starts = [], [], []
    for stage in stages:
        if not stage['rows']:
            continue
        starts.append(dict(name=stage['name'], substage=stage.get('substage', stage['name']),
                           start_tick=len(ticks), start_s=len(ticks)/30,
                           motiongen_receipt_index=stage.get('receipt_index')))
        derivation=stage.get('cross_continuity') or stage.get('motion_continuity')
        if derivation:
            starts[-1]['motiongen_receipt_indices']=derivation['parent_receipt_indices']
        for row, encoded in zip(stage['rows'], stage['native_actions'], strict=True):
            decoded = native_target(encoded, spec, grip['open_travel_mm'], distance=True)
            if row != decoded:
                raise ValueError('frozen target differs from actual native replay decode')
            ticks.append(dict(tick=len(ticks), time_s=len(ticks)/30, target=decoded))
            actions.append(encoded)
    control = deepcopy(data['native_identity']['control'])
    if control['physics_hz'] != 30*control['substeps']:
        raise ValueError('native control must be 30Hz')
    plan=dict(schema_version='dual_gpu_pipeline.v1', lane='dsw-scene', **PIPELINE_LANES['dsw-scene'],
        run_id=run_id, source_run_id=source['run_id'], source_sequence=source['sequence'],
        scene_digest=source['scene']['scene_digest'], calibration_profile_digest=_digest(source['calibration_profile']),
        planner_id=CUROBO_PLANNER_ID, policy_id=SIM_POLICY_ID, ticks=ticks, native_actions=actions,
        native_action_encoding=NATIVE_DISTANCE_ENCODING, native_substeps=[control['substeps']]*len(ticks),
        duration_s=len(ticks)/30, timeline_digest=_digest(ticks), control=control, stages=starts,
        gripper_opening_mm=grip['open_travel_mm'], native_identity=deepcopy(data['native_identity']),
        motion_profile='curobo-motiongen-30hz',
        simulation=dict(validated=False, outcome='not_run'),
        gripper_distance_control=dict(schema_version='sim_gripper_distance_control.v1',
            closed_metal_gap_mm=65., rubber_gap_mm=62., pad_thickness_m=.0015,
            native_travel_offset_mm=grip['native_travel_offset_mm'], max_metal_gap_mm=grip['max_metal_gap_mm'],
            open_model_travel_mm=grip['open_travel_mm'], holding_stages=sorted(HOLDING_STAGES),
            geometry_derivation=grip, contact_detection=False, force_control=False),
        gripper_closing=dict(schema_version='commanded_gap_closing.v1',maximum_step_mm=2.,
                             initial_model_gap_mm=grip['open_travel_mm'],target_model_gap_mm=grip['closed_travel_mm'],arm_motion=False),
        gripper_release=dict(schema_version='commanded_gap_release.v2', maximum_step_mm=.2,
                             initial_model_gap_mm=grip['closed_travel_mm'], arm_motion=False))
    from control_motion import validate_control
    plan['control_validation']=validate_control(plan)
    preset=plan['native_identity'].get('physical_configuration',{}).get('cup_contact_preset')
    if preset is not None:
        from cup_presets import bind_plan_preset
        physical_sha=plan['native_identity'].get('physical_configuration_sha256')
        bind_plan_preset(plan,preset)
        if plan['native_identity']['physical_configuration_sha256']!=physical_sha:
            raise ValueError('exported cup physical configuration digest differs')
    return plan


def code_identity(urdf_path=None):
    names = ('playground_geometry','gpu_acceptance','cup_envelope','cup_contact','region_self_collision','conservative_spheres','motiongen_motion', 'motiongen_self_collision', 'motiongen_targets', 'curobo_robot_model', 'curobo_planning',
             'curobo_whole_geometry', 'curobo_relative_geometry', 'integration', 'gripper_distance_plan', 'isaac_lane',
             'control_motion','physical_motiongen','real_contract','joint_space')
    paths = [Path(__file__).resolve(),Path(__file__).with_name('a1_conservative_refinement.json'),Path(__file__).with_name('a1_self_domain_certificate.json'),Path(__file__).with_name('a1_grasp_contact.json'),Path(__file__).with_name('a1_cup_inset.json'), Path(__file__).with_name('a1_support_domain.json'), Path(__file__).with_name('a1_support_near.json'), Path(__file__).with_name('a1_fixed_domain.json'), *(Path(importlib.import_module(name).__file__).resolve() for name in names)]
    paths.append(Path(__file__).with_name('a1_asset_derivation.py'))
    if urdf_path is not None:
        paths.extend(certificate_path(name,urdf_path) for name in CERTIFICATES)
        proof=Path(urdf_path).resolve().with_name(PROOF)
        if proof.exists():paths.append(proof)
    return {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}


def prepare_model(planner, observation, data, urdf_path, opened, engine_cache=None):
    """Prepare/warm one solver; shared by environment warmup and actual requests."""
    from curobo_robot_model import build_conservative_robot_model, fixed_world_mesh_descriptors
    from motiongen_motion import MotionGenerator
    model_key=None if engine_cache is None else _digest(dict(parts=data['parts'],urdf=data['urdf_sha256'],opened=opened,
        planner_identity=planner.identity,ignores=sorted(sorted(pair) for pair in planner.geo.whole.ignores),
        rules={name:hashlib.sha256(certificate_path(name,urdf_path).read_bytes()).hexdigest()
               for name in ('a1_conservative_refinement.json','a1_self_domain_certificate.json')}))
    if engine_cache is not None and engine_cache.get('model_key')==model_key:
        config,proxy,fixed_meshes=deepcopy(engine_cache['model'])
    else:
        config, proxy = build_conservative_robot_model(data, urdf_path, planner.fk.movable, opened, ignored_link_pairs=planner.geo.whole.ignores)
        fixed_meshes = fixed_world_mesh_descriptors(data, proxy, planner.fk, opened)
        if engine_cache is not None:
            engine_cache.update(model_key=model_key,model=deepcopy((config,proxy,fixed_meshes)))
    control = data['native_identity']['control']
    velocity, acceleration = control['max_target_velocity_rad_s'], control['max_target_acceleration_rad_s2']
    config['kinematics']['cspace'].update(velocity_scale=[min(1., velocity/proxy['kinematics']['joints'][n]['velocity']) for n in LEFT],
                                         max_acceleration=float(acceleration))
    proxy['robot_config_sha256'] = _digest(config)
    proxy['motion_control_limits'] = dict(max_velocity_rad_s=velocity, max_acceleration_rad_s2=acceleration)
    world, cup_names, world_description = world_config(planner, observation.sample.world_from_cup, fixed_meshes)
    preparation_started=time.perf_counter()
    cache_key=_digest(config)
    reused=engine_cache is not None and engine_cache.get('key')==cache_key
    if reused:
        engine=engine_cache['engine']
        engine.generator.detach_object_from_robot(link_name='attached_object')
        engine.update_world(world)
        engine.update_locked_joints(config['kinematics']['lock_joints'])
        engine.receipts=[]
    else:
        engine=MotionGenerator(config,world,model_report=proxy)
        prepared_contact=json.loads(certificate_path('a1_grasp_contact.json',urdf_path).read_text())
        gap=prepared_contact['terminal_jaw_position_m']
        engine.update_locked_joints({**config['kinematics']['lock_joints'],'left_joint7':gap,'left_joint8':gap})
        engine.update_locked_joints(config['kinematics']['lock_joints'])
        engine.warmup()
        if engine_cache is not None:engine_cache.update(key=cache_key,engine=engine)
    initialization_wall=0. if reused else time.perf_counter()-preparation_started
    # Keep fresh warmup state; reused requests must not inherit prior sampling.
    if reused: engine.generator.reset_seed()
    return dict(config=config,proxy=proxy,fixed_meshes=fixed_meshes,world=world,cup_names=cup_names,
        world_description=world_description,engine=engine,reused=reused,initialization_wall=initialization_wall)


def plan_loaded(planner, observation, data, source, urdf_path, *, speed_scale=1., diagnostic=lambda name, value: None, engine_cache=None, physical_binding=None, environment='physical'):
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec
    from curobo_robot_model import build_conservative_robot_model, fixed_world_mesh_descriptors
    from gpu_acceptance import closing_geometry, jaw_sweep, SupportBounds, fixed_world_check, fixed_self_check, task_check
    from cup_envelope import prepare as prepare_cup, attach as attach_prepared_cup
    from motiongen_motion import MotionGenerator
    from motiongen_targets import generate_targets, carry_targets, straight_waypoints
    from robot_sim.tasks.manager_based.coffee_make.outlet_alignment import outlet_distance
    planning_started=time.perf_counter()
    prior_engine=(engine_cache or {}).get('engine')
    graph_start=dict(getattr(prior_engine,'cuda_graph_calls',dict(captures=0,replays=0)))
    physical=is_device_source(source)
    if physical:
        from real_contract import validate_real_source
        validate_real_source(source,physical_binding,environment=environment)
        bridge=None
    else:
        if physical_binding is not None:raise ValueError('physical binding cannot relabel a simulation source')
        bridge = DualGpuPipeline(); bridge.ingest_source(source)
    if not physical and ('initial_state' in source or source['calibration_profile']['gripper'].get('mapping') != 'sum_of_two_prismatic_displacements_mm'):
        raise ValueError('simulation model-space source required; physical binding is not supported')
    grip = gripper_geometry(planner, urdf_path)
    opening, closed = grip['open_travel_mm'], grip['closed_travel_mm']
    spec = ac_one_dual_arm_zero_home_action_spec()
    start, initial = initial_motion_state(planner, data, source, spec, opening)
    diagnostic('initial-command.json', initial)
    right, home_q = [start[name] for name in RIGHT], [start[name] for name in LEFT]
    initial_gap, right_gap = (start['left_joint7']+start['left_joint8'])*1000, (start['right_joint17']+start['right_joint18'])*1000
    action_cache = {}
    targets = generate_targets(planner, observation, data,urdf_path=urdf_path)
    diagnostic('targets.json', targets)
    if not targets['grasp_candidates'] or not targets['placement_candidates']:
        raise ValueError('no supported task-space targets')
    opened = dict(start, left_joint7=opening/2000, left_joint8=opening/2000)
    prepared=prepare_model(planner,observation,data,urdf_path,opened,engine_cache)
    config,proxy,fixed_meshes,world,cup_names,world_description,engine,reused,initialization_wall=(prepared[key] for key in
        ('config','proxy','fixed_meshes','world','cup_names','world_description','engine','reused','initialization_wall'))
    diagnostic('robot-model.json',dict(config=config,proxy=proxy))
    engine.cup_contact.allow_claw_contact(True)
    support_checker=SupportBounds(engine,planner,urdf_path)
    fixed_acceptance=dict(world=fixed_world_check(engine,planner,data,observation.sample.world_from_cup),self=fixed_self_check(engine,planner,data))
    diagnostic("fixed-gpu-acceptance.json",fixed_acceptance)
    if not all(row["passed"] for row in fixed_acceptance.values()):raise ValueError("static GPU acceptance unresolved")
    jaw_acceptance=[]
    envelope=prepare_cup(world)
    diagnostic('motiongen-initialization.json', dict(initialization_seconds=engine.initialization_seconds,
        warmup_seconds=engine.warmup_seconds, self_collision_partition=engine.self_collision_partition))
    base_from_world = planner.geo.mount.inverse()
    source_digest = _digest(source)
    stages = []

    def add(name, rows, substage=None, receipt_index=None):
        stages.append(encode_stage(dict(name=name, substage=substage or name, rows=rows, receipt_index=receipt_index),
                                   spec, opening, action_cache))

    def hold(name, q, gap, seconds):
        add(name, [target(q, right, gap, right_gap) for _ in range(round(seconds*30))])

    def move(method, q, goals, name, **options):
        previous = len(engine.receipts)
        completed = False
        try:
            path, receipt = getattr(engine, method)(q, goals, name=name, **options)
            receipt['path_sha256'] = _digest(path)
            diagnostic(f'native-stage-path-{len(engine.receipts)-1:03d}.json',dict(executable=False,path=path,receipt=deepcopy(receipt)))
            completed = True
            return path, receipt
        finally:
            for receipt in engine.receipts[previous:]:
                receipt.update(source_digest=source_digest, world_sha256=_digest(world_description),
                               robot_config_sha256=_digest(engine.config),
                               claw_cup_allowance_m=engine.cup_contact.report['current_claw_allowance_m'])
            if completed:
                # Save each complete receipt once; final collection is written
                # below. Failed native attempts retain the full diagnostic snapshot.
                for index in range(previous, len(engine.receipts)):
                    diagnostic(f'motiongen-receipt-{index:03d}.json', deepcopy(engine.receipts[index]))
            else:
                diagnostic('motiongen-receipts-latest', deepcopy(engine.receipts))

    def base_pose(pose):
        return asdict(base_from_world.compose(RigidTransform(**pose)))

    def continuous_stage(parents, name, substage, gap, *, tcp_speed=None, frontloaded=False,duration_s=None):
        from motiongen_motion import continuous_path
        joined=[r['left_arm_joints'] for p in parents for r in p['rows']]
        positions=None
        if tcp_speed is not None:
            positions=engine.generator.kinematics.get_state(engine.tensor_args.to_device(joined)).ee_position.detach().cpu().tolist()
        removed=0
        if substage=='lower':
            # Native trajectory optimization can overshoot a 5mm descent.
            # Keep its monotone vertices; the final GPU query covers new chords.
            direction=[b-a for a,b in zip(positions[0],positions[-1])]
            squared=sum(v*v for v in direction)
            if squared<=1e-16:raise ValueError('lower requires a nonzero descent')
            indices=[0];previous=0.
            for i,point in enumerate(positions[1:-1],1):
                fraction=sum((a-b)*d for a,b,d in zip(point,positions[0],direction))/squared
                if previous<fraction<1.:
                    indices.append(i);previous=fraction
            indices.append(len(joined)-1);removed=len(joined)-len(indices)
            joined=[joined[i] for i in indices];positions=[positions[i] for i in indices]
        control=data['native_identity']['control']
        path,proof=continuous_path(joined,control['max_target_velocity_rad_s'],control['max_target_acceleration_rad_s2'],
            positions=positions,tcp_speed=tcp_speed,frontloaded=frontloaded,duration_s=duration_s)
        merged=encode_stage(dict(name=name,substage=substage,rows=[target(q,right,gap,right_gap) for q in path]),spec,opening,action_cache)
        proof['parent_receipt_indices']=[p['receipt_index'] for p in parents]
        proof['parent_paths']=[dict(receipt_index=p['receipt_index'],input_joint_path_sha256=p['input_joint_path_sha256'],
            emitted_joint_path_sha256=_digest([r['left_arm_joints'] for r in p['rows']]),encoding_roundoff_rad=p['encoding_roundoff_rad']) for p in parents]
        proof['gpu']=engine.validate_continuous_path([r['left_arm_joints'] for r in merged['rows']])
        proof['removed_reverse_vertices']=removed
        merged['cross_continuity' if substage=='cross' else 'motion_continuity']=proof
        return merged

    add('initial_hold', [target(home_q, right, initial_gap, right_gap)])
    if initial_gap != opening:
        jaw_acceptance.append(jaw_sweep(engine,planner,home_q,initial_gap/2000,opening/2000))
        add('initial_open', jaw_rows(home_q, right, initial_gap, opening, right_gap))
    grasp_candidates = targets['grasp_candidates']
    path, receipt = move('plan_pose', home_q, [base_pose(row['world_from_pregrasp']) for row in grasp_candidates], 'approach_pre')
    selected_index = receipt.get('goalset_index', [0])[0]
    if not 0 <= selected_index < len(grasp_candidates):
        raise ValueError('MotionGen goalset index is invalid')
    grasp = grasp_candidates[selected_index]
    add('approach_pre', [target(q, right, opening, right_gap) for q in path], receipt_index=len(engine.receipts)-1)
    stages[-1]=continuous_stage(stages[-1:],'approach_pre','approach_pre',opening,duration_s=2.)
    path, _ = move('plan_pose', path[-1], [base_pose(grasp['world_from_tool'])], 'approach', hold_orientation=True, linear_approach=True)
    add('approach', [target(q, right, opening, right_gap) for q in path], receipt_index=len(engine.receipts)-1)
    stages[-1]=continuous_stage(stages[-1:],'approach','approach',opening,tcp_speed=.08,frontloaded=True)
    grasp_q = stages[-1]['rows'][-1]['left_arm_joints']
    hold('preclose_settle', grasp_q, opening, .1)
    grasp_state = {**opened, **dict(zip(LEFT, grasp_q))}
    closing = closing_geometry(engine, planner, grasp_state, observation.sample.world_from_cup)
    diagnostic('closing.json', closing)
    if closing.get('passed') is not True:
        raise ValueError('GPU contact-limited closing at MotionGen grasp failed: '+str(closing.get('error')))
    contact = closing['terminal_state']
    if abs(contact['left_joint7']-contact['left_joint8']) > 1e-9:
        raise ValueError('contact geometry does not preserve mechanically linked jaws')
    if closed > (contact['left_joint7']+contact['left_joint8'])*1000+1e-9:
        raise ValueError('65mm metal command is wider than bilateral contact; holding assumption is not justified')
    held_cup = RigidTransform(**closing['world_from_cup'])
    tool = planner.geo.mount.compose(planner.fk.pose(planner.fk.forward(contact)['left_link6']))
    relation = tool.inverse().compose(held_cup)
    actual_grasp = dict(grasp, cup_from_tool=asdict(relation.inverse()), tool_from_cup=asdict(relation))
    held_observation = SimpleNamespace(scene=observation.scene, sample=SimpleNamespace(world_from_cup=held_cup))
    add('close_ramp',jaw_rows(grasp_q,right,opening,closed,right_gap,max_step_mm=2.))
    hold('close_and_hold', grasp_q, closed, .6)
    failures, attachment = [], None
    for candidate in targets['placement_candidates']:
        engine.cup_contact.allow_claw_contact(True)
        trial = []
        trial_jaws=[]
        try:
            world, cup_names, world_description = world_config(planner, held_cup, fixed_meshes)
            engine.update_world(world)
            engine.update_locked_joints({name: contact[name] for name in engine.config['kinematics']['lock_joints']})
            attachment = attach_prepared_cup(engine, grasp_q, envelope)
            attachment['claw_cup_contact']=engine.cup_contact.certify_held(engine,grasp_q,json.loads(certificate_path('a1_grasp_contact.json',urdf_path).read_text()))
            route = carry_targets(planner, held_observation, actual_grasp, candidate, jaw_position_m=contact['left_joint7'])
            diagnostic('carry-route-'+candidate['candidate_id']+'.json',route)
            current = grasp_q
            for row in route['stages']:
                trial_start=len(trial)
                goal=RigidTransform(**base_pose(row['world_from_tool']))
                if row['name']=='cross':
                    state={**contact,**dict(zip(LEFT,current))}
                    begin=planner.fk.pose(planner.fk.forward(state)['left_link6'])
                    goals=straight_waypoints(begin,goal)
                else:goals=[goal]
                for part_index,goal in enumerate(goals):
                    name=candidate['candidate_id']+'/'+row['name']+(f'/{part_index}' if len(goals)>1 else '')
                    vertical=row['name'] in ('lift','raise','front_height','lower')
                    axis=2 if vertical else (1 if row['name']=='cross' else (0 if row['name']=='insert' else None))
                    path,_=move('plan_pose',current,[asdict(goal)],name,hold_orientation=True,linear_axis=axis,
                        project_to_goal_frame=not (vertical or row['name']=='insert'))
                    stage=row['name'] if row['name'] in ('lift','raise','lower') else 'transport'
                    trial.append(encode_stage(dict(name=stage,substage=row['name'],rows=[target(q,right,closed,right_gap) for q in path],
                        receipt_index=len(engine.receipts)-1),spec,opening,action_cache))
                    current=trial[-1]['rows'][-1]['left_arm_joints']
                if row['name']=='cross':
                    parents=trial[trial_start:]
                    merged=continuous_stage(parents,'transport','cross',closed)
                    trial[trial_start:]=[merged]
                    current=merged['rows'][-1]['left_arm_joints']
                elif row['name']=='raise':
                    trial[-2:]=[continuous_stage(trial[-2:],'lift','lift_raise',closed,tcp_speed=.04,duration_s=2.2)]
                elif row['name'] in ('front_height','insert','lower'):
                    trial[-1]=continuous_stage(trial[-1:],stage,row['name'],closed,tcp_speed=.01 if row['name'] in ('front_height','lower') else .06)
            place_state = {**contact, **dict(zip(LEFT, current))}
            placed_tool = planner.geo.mount.compose(planner.fk.pose(planner.fk.forward(place_state)['left_link6']))
            placed_cup = placed_tool.compose(relation)
            requested_cup = RigidTransform(**candidate['world_from_cup'])
            if math.dist(placed_cup.position, requested_cup.position) > .002+1e-7:
                raise ValueError('MotionGen placement endpoint exceeds the accepted 2mm cup target tolerance')
            receiving_distance=outlet_distance(observation.scene,planner.cup,placed_cup)
            if receiving_distance>targets['placement_goal']['tolerance_m']:
                raise ValueError('MotionGen endpoint misses receiving goal: '+str(receiving_distance))
            trial.append(encode_stage(dict(name='support_settle', rows=[target(current, right, closed, right_gap)]*3), spec, opening, action_cache))
            trial.append(encode_stage(dict(name='open', rows=jaw_rows(current, right, closed, opening, right_gap)), spec, opening, action_cache))
            trial.append(encode_stage(dict(name='release_hold', rows=[target(current, right, opening, right_gap)]*6), spec, opening, action_cache))
            engine.generator.detach_object_from_robot(link_name='attached_object')
            world, _, world_description = world_config(planner, placed_cup, fixed_meshes)
            engine.update_world(world)
            trial_jaws.append(jaw_sweep(engine,planner,current,contact['left_joint7'],opening/2000))
            engine.update_locked_joints({name: opened[name] for name in engine.config['kinematics']['lock_joints']})
            # Return through the accepted insertion lane before moving sideways;
            # a diagonal departure can sweep an open pad back into the cup.
            exit_goal=RigidTransform(**next(row['world_from_tool'] for row in route['stages'] if row['name']=='front_height'))
            path,_=move('plan_pose',current,[asdict(base_from_world.compose(exit_goal))],
                'retreat_insert',hold_orientation=True)
            trial.append(encode_stage(dict(name='retreat',substage='retreat_insert',
                rows=[target(q,right,opening,right_gap) for q in path],receipt_index=len(engine.receipts)-1),
                spec,opening,action_cache))
            engine.cup_contact.allow_claw_contact(False)
            path, _ = move('plan_joint', trial[-1]['rows'][-1]['left_arm_joints'], home_q, 'home')
            trial.append(encode_stage(dict(name='home', rows=[target(q, right, opening, right_gap) for q in path], receipt_index=len(engine.receipts)-1), spec, opening, action_cache))
            jaw_acceptance.extend(trial_jaws)
            stages.extend(trial)
            selected_placement = candidate
            break
        except ValueError as error:
            engine.generator.detach_object_from_robot(link_name='attached_object')
            failures.append(dict(candidate_id=candidate['candidate_id'], error=str(error)))
    else:
        diagnostic('placement-failures.json', failures)
        raise ValueError('MotionGen found no complete placement/retreat/home path')
    if opening != initial_gap:
        jaw_acceptance.append(jaw_sweep(engine,planner,path[-1],opening/2000,initial_gap/2000))
        add('home_gripper_restore', jaw_rows(path[-1], right, opening, initial_gap, right_gap))
    hold('home_hold', path[-1], initial_gap, .3)
    selected_receipts = []
    for stage in stages:
        index = stage.get('receipt_index')
        joints = [row['left_arm_joints'] for row in stage['rows']]
        if stage.get('cross_continuity') or stage.get('motion_continuity'):
            derived=stage.get('cross_continuity') or stage['motion_continuity']
            if (derived['gpu'].get('passed') is not True or
                    derived['gpu']['path_sha256']!=_digest(joints)):
                raise ValueError('joined cross lacks final GPU path evidence')
            for parent in derived['parent_paths']:
                receipt=engine.receipts[parent['receipt_index']]
                if receipt.get('success') is not True or receipt.get('path_sha256')!=parent['input_joint_path_sha256']:
                    raise ValueError('joined cross parent differs from MotionGen path')
                receipt.update(emitted_joint_path_sha256=parent['emitted_joint_path_sha256'],
                               encoding_roundoff_rad=parent['encoding_roundoff_rad'])
                selected_receipts.append(parent['receipt_index'])
        elif index is None:
            if any(row != joints[0] for row in joints):
                raise ValueError('arm motion lacks a MotionGen stage receipt')
        else:
            row = engine.receipts[index]
            if (row.get('success') is not True or row.get('method') not in
                    ('MotionGen.plan_single', 'MotionGen.plan_goalset', 'MotionGen.plan_single_js')
                    or row.get('path_sha256') != stage['input_joint_path_sha256']):
                raise ValueError('emitted arm path differs from its successful MotionGen receipt')
            row.update(emitted_joint_path_sha256=_digest(joints), encoding_roundoff_rad=stage['encoding_roundoff_rad'])
            selected_receipts.append(index)
    stages,control_adjustments=enforce_stage_motion_limits(stages,data['native_identity']['control'],spec,opening)
    derived_stages, source_times = scaled_stages(stages, speed_scale, spec, opening)
    diagnostic('candidate-stages.json',dict(executable=False,stages=stages,closing=closing))
    diagnostic('motiongen-receipts-latest', deepcopy(engine.receipts))
    support_acceptance=support_checker.check([row['left_arm_joints'] for stage in stages for row in stage['rows']])
    task_acceptance=task_check(engine,planner,stages,closing)
    acceptance=dict(passed=all(row['passed'] for row in [*jaw_acceptance,closing['gpu_sweep'],support_acceptance,task_acceptance]),
        backend='curobo_gpu_and_bound_offline_domains',fixed=fixed_acceptance,jaw_sweeps=jaw_acceptance,
        closing=closing['gpu_sweep'],support=support_acceptance,task_semantics=task_acceptance,
        receiving_goal=dict(passed=True,metric=targets['placement_goal']['metric'],
            distance_m=receiving_distance,tolerance_m=targets['placement_goal']['tolerance_m']),
        online_cpu_mesh_audit=False,retimed_collision_audit=False)
    diagnostic('gpu-final-acceptance.json',acceptance)
    if not acceptance['passed']:raise ValueError('GPU final acceptance failed')
    run_id = source['run_id']+'-motiongen-'+uuid4().hex[:10]
    base = freeze(stages, source, data, grip, spec, run_id=run_id+'-base')
    plan = freeze(derived_stages, source, data, grip, spec, run_id=run_id)
    backend = dict(backend='curobo_motiongen', curobo_version='0.7.8', gpu_ik=True, gpu_trajectory_search=True,
                   code_sha256=code_identity(urdf_path), robot_proxy_sha256=_digest(proxy),
                   initial_robot_config_sha256=_digest(config), native_robot_sha256=data['urdf_sha256'],
                   exported_geometry_sha256=_digest(data))
    selection = dict(grasp=grasp, placement=selected_placement, placed_cup_geometry=asdict(placed_cup),
                     placement_goal=targets['placement_goal'],carry_route=route,
                     attachment=attachment, failed_motiongen_candidates=failures)
    for value in (base, plan):
        value['simulation_initial_command'] = deepcopy(initial['simulation_initial_command'])
        value['initial_state_validation'] = deepcopy(initial)
        value['planning_backend'] = deepcopy(backend)
        value['motiongen_receipts'] = deepcopy(engine.receipts)
        value['task_targets'] = selection
        value['planning_validation'] = dict(schema_version='curobo_motiongen_path_validation.v1', passed=True,
            source_digest=source_digest, timeline_digest=value['timeline_digest'], robot_proxy_sha256=_digest(proxy),
            motiongen_receipts_sha256=_digest(engine.receipts), gpu_final_acceptance=acceptance,
            selected_motiongen_receipt_indices=selected_receipts,
            motiongen_method_counts=dict(Counter(row['method'] for row in engine.receipts)),
            selected_motiongen_method_counts=dict(Counter(engine.receipts[index]['method'] for index in selected_receipts)),
            cpu_search_fallback=False, source_robot_identity=data['planner_identity'], hardware_validated=False)
        value['planning_validation']['continuous_cross']=[stage['cross_continuity'] for stage in stages
            if stage.get('cross_continuity')]
        value['planning_validation']['continuous_motion']=[stage['motion_continuity'] for stage in stages if stage.get('motion_continuity')]
        value['planning_validation']['native_output_control_time_adjustments']=control_adjustments
    plan['planning_validation']['derivation'] = dict(method='uniform_source_time_stage_scale',
        parent_timeline_digest=base['timeline_digest'], parent_receipt_sha256=_digest(base['planning_validation']),
        speed_scale=speed_scale, parent_stage_sample_times=source_times, fixed_wait_stages=sorted(WAITS), collision_audit_rerun=False)
    plan['planning_validation']['derivation']['max_joint_interpolation_roundoff_rad'] = max(stage.get('interpolation_roundoff_rad', 0.) for stage in derived_stages)
    if physical:
        from physical_motiongen import freeze_physical
        base,plan=(freeze_physical(value,source,physical_binding,environment=environment) for value in (base,plan))
    else:
        bridge.accept_plan(base); bridge.accept_plan(plan)
    if not physical and initial['simulation_initial_command']['evidence_source'] != 'runtime.loop.previous':
        for value in (base, plan):
            value.update(offline_candidate=True, synthetic_fixture=True)
            value['planning_validation'].update(schema_version='offline_curobo_motiongen_path_validation.v1',
                offline_candidate=True, synthetic_fixture=True, dual_dsw_e2e_completed=False)
    return base, plan, dict(planning_backend=backend, gpu_final_acceptance=acceptance, grip=grip,
        motiongen_receipts=engine.receipts, selected_placement=selected_placement,
        motiongen_initialization_seconds=engine.initialization_seconds, motiongen_warmup_seconds=engine.warmup_seconds,
        engine_reused=reused,initialization_wall_seconds=initialization_wall,
        initialization_breakdown=getattr(engine,'initialization_breakdown',{}),
        cuda_graph_calls={key:value-graph_start.get(key,0) for key,value in getattr(engine,'cuda_graph_calls',{}).items()},
        cuda_graph_counter_scope='native API invocations including failed attempts; entire planning request',
        full_planning_wall_seconds=time.perf_counter()-planning_started,
        full_hot_planning_wall_seconds=time.perf_counter()-planning_started-initialization_wall)


def main(args):
    from local_execution import register_auxiliary
    register_auxiliary(args,__file__)
    args.output.mkdir(parents=True, exist_ok=False)
    started, updates = time.monotonic(), 0
    result = dict(status='failed', backend='curobo_motiongen', hardware_access=False)
    try:
        source = json.loads(args.source.read_text())
        physical=is_device_source(source)
        binding_path=getattr(args,'physical_binding',None)
        binding=json.loads(binding_path.read_text()) if binding_path is not None else None
        if physical:
            from real_contract import validate_real_source
            validate_real_source(source,binding,environment=getattr(args,'environment','physical'))
        else:
            if binding is not None:raise ValueError('physical binding cannot relabel a simulation source')
            bridge = DualGpuPipeline(); bridge.ingest_source(source)
        if source.get('synthetic_fixture') is True or source.get('test_fixture') is True:
            raise ValueError('fixture sources require the explicit offline MotionGen chain verifier')
        from playground_geometry import load
        planner, observation, data = load(args.input, source, args.native_code, args.assets)
        from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
        def diagnostic(name, value):
            nonlocal updates
            if name == 'motiongen-receipts-latest':
                name = f'motiongen-receipts-{updates:03d}.json'; updates += 1
            write_new(args.output/name, value)
        base, plan, evidence = plan_loaded(planner, observation, data, source, ac_one_asset_spec(args.assets).urdf_path,
                                           speed_scale=args.speed_scale, diagnostic=diagnostic,physical_binding=binding,
                                           environment=getattr(args,'environment','physical'))
        if getattr(args,'execution_json',None):
            from local_execution import validate,TARGET,stamp_plan
            validate({'execution':json.loads(args.execution_json)})
            stamp_plan(base,TARGET);stamp_plan(plan,TARGET)
            evidence['compute_resource']=dict(TARGET,logical_cuda_index=0)
        write_new(args.output/'base-plan.json', base)
        write_new(args.output/'plan.json', plan)
        result.update(status='motiongen-plan-ready', timeline_digest=plan['timeline_digest'],
                      input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
                      source_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(), **evidence)
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
    result['elapsed_seconds'] = time.monotonic()-started
    write_new(args.output/'result.json', result)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if result['status'] == 'motiongen-plan-ready' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('input', 'source', 'native-code', 'assets', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--speed-scale', type=float, default=1.)
    parser.add_argument('--physical-binding',type=Path,help='trusted acquisition/calibration receipt; no hardware access')
    parser.add_argument('--environment',choices=('physical','fake'),default='physical')
    parser.add_argument('--execution-json')
    raise SystemExit(main(parser.parse_args()))
