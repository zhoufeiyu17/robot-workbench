"""CPU contract fixtures only; synthetic geometry receipts never qualify physical motion."""
from copy import deepcopy
import unittest

from control_motion import validate_control
from frozen_executor import FrozenExecutor
from integration import _digest
from joint_space import convert_state
from physical_motiongen import freeze_physical
from gripper_distance_plan import release_targets,validate_release_control
from real_contract import validate_real_plan
from test_fixed_right_policy import fixture as original_fixture
import physical_cup_maintenance as maintenance


def fixture():
    parent_source,model,parent_binding=original_fixture()
    reference=deepcopy(model['ticks'][0]['target']);opening=82.992;closed=60.992
    shapes=[('initial_hold',[opening]),('initial_open',[opening]*2),('approach_pre',[opening]*2),
        ('approach',[opening]*2),('preclose_settle',[opening]*3),
        ('close_ramp',[opening-(opening-closed)*i/11 for i in range(12)]),('close_and_hold',[closed]*18),
        ('lift',[closed]*2),('transport',[closed]*2),('lower',[closed]*2),('support_settle',[closed]*3),
        ('open',release_targets(closed,opening)[0]),('release_hold',[opening]*6),('retreat',[opening]*2),
        ('home',[opening]*2),('home_gripper_restore',[opening]*2),('home_hold',[opening]*9)]
    model['ticks']=[];model['stages']=[];model['native_actions']=[]
    for name,gaps in shapes:
        model['stages'].append(dict(name=name,start_tick=len(model['ticks']),motiongen_receipt_index=0))
        for gap in gaps:
            row=dict(deepcopy(reference),left_gripper=gap)
            model['ticks'].append(dict(tick=len(model['ticks']),time_s=len(model['ticks'])/30,target=row))
            model['native_actions'].append([0.]*6+[gap]+[0.]*6+[row['right_gripper']])
    model.update(native_substeps=[8]*len(model['ticks']),duration_s=len(model['ticks'])/30,gripper_opening_mm=opening,
        native_identity={'acceptance':{}},gripper_release=release_targets(closed,opening)[1])
    model['timeline_digest']=_digest(model['ticks']);model['control_validation']=validate_control(model)
    model['planning_validation']['timeline_digest']=model['timeline_digest']
    parent=freeze_physical(model,parent_source,parent_binding)
    events=[]
    for i,tick in enumerate(parent['ticks']):
        events.append(dict(event='command',tick=i,source_tick=i,target=deepcopy(tick['target']),
            timeline_digest=parent['timeline_digest'],execution_timeline_digest='a'*64,
            send_monotonic_s=1+i/30,wall_ns=1000+i))
    events.extend([dict(event='feedback',wall_ns=10000,frozen=dict(status='completed',
        timeline_digest=parent['timeline_digest'],execution_timeline_digest='a'*64,last_tick=len(parent['ticks'])-1,
        targets_submitted=True,timeline_elapsed=True)),dict(event='motion_state',wall_ns=10001,status='READY')])
    receipt=maintenance.parent_execution_receipt(events,parent,parent_source,'b'*64)
    initial=deepcopy(parent_source['initial_state'])
    # A genuinely new current right reference in this CPU fixture, separate from the parent.
    for key in ('measured_reported','planned_reported','after_observation_reported'):initial[key]['right_arm_joints'][0]+=.002
    initial['model']=convert_state(initial['planned_reported'],parent_source['calibration_profile'],to_reported=False)
    limits=dict(left_arm_joints=[[-3.,3.]]*6,right_arm_joints=[[-3.,3.]]*6)
    initial['reported_joint_limits']=limits
    for sample in initial['owner_samples']:sample['reported_joint_limits']=deepcopy(limits)
    origin=deepcopy(parent_source['scene']['environment_from_cup'])
    context=dict(schema_version=maintenance.CONTEXT_SCHEMA,parent_timeline_digest=parent['timeline_digest'],
        parent_execution_digest=_digest(receipt),return_origin=origin,offset_world_xy_mm=[-20.,20.],
        current_scene=dict(cup_pose=deepcopy(origin),machine_pose=deepcopy(parent_source['scene']['environment_from_machine_root']),
            world_from_robot=dict(position=[0.,0.,0.],orientation_wxyz=[1.,0.,0.,0.]),
            observation=dict(environment='physical',observed_wall_ns=20000,evidence_sha256='c'*64,method='CPU contract fixture')))
    source=maintenance.build_source(parent_source,initial,context,'maintenance-fixture')
    binding=maintenance.build_binding(parent_binding,source)
    stages=maintenance.reverse_stages(parent,parent_source)
    for i,stage in enumerate(stages):
        stage.update(base_reverse_stage_index=i,right_hold_rebound=True)
        for row in stage['rows']:row['right_arm_joints']=deepcopy(initial['planned_reported']['right_arm_joints'])
    offset=dict(schema='physical_cup_return_offset.v1',stages=stages,derivation=dict(scope='CPU contract fixture only',
        parent_timeline_digest=parent['timeline_digest'],**{k:deepcopy(source['maintenance_context'][k])
            for k in ('return_origin','offset_world_xy_mm','return_goal')}),
        kinematics=dict(urdf_sha256=source['calibration_profile']['model_binding']['urdf_sha256']))
    geometry=geometry_fixture(parent,source,stages)
    return parent,parent_source,receipt,source,binding,offset,geometry,events


def geometry_fixture(parent,source,stages):
    checks={key:dict(performed=True,passed=True) for key in ('robot_self','robot_world','cup_world','grasp_contact','table_support')}
    checks['fixed_right_static']=dict(performed=False,passed=None,status='skipped_by_operator')
    return dict(schema='physical_cup_maintenance_geometry.v1',scope='complete_return_path',passed=True,
        source_digest=_digest(source),parent_timeline_digest=parent['timeline_digest'],
        urdf_sha256=source['calibration_profile']['model_binding']['urdf_sha256'],
        model_target_vectors_sha256=_digest(maintenance.model_target_vectors(stages,source)),
        native_actions_sha256=_digest([a for stage in stages for a in stage['native_actions']]),
        scene_digest=_digest(source['maintenance_context']['current_scene']),max_sample_step_rad=.005,checks=checks,
        right_static_authorization_digest=_digest(source['maintenance_context']['right_static_authorization']))


def current_fixture():
    from source_from_detection import build_assisted_source
    values=list(fixture());parent,old,_,source,binding,_,geometry,_=values
    receipt=deepcopy(old['scene']['source']['assistance_receipt']);receipt['request_id']='current-geometry-fixture'
    receipt.update(initial_state_digest=_digest(source['initial_state']),
        environment_from_cup=deepcopy(source['maintenance_context']['current_scene']['cup_pose']),
        environment_from_machine_root=deepcopy(source['maintenance_context']['current_scene']['machine_pose']))
    receipt['fixed_right_collision_policy'].update(request_id=receipt['request_id'],
        initial_state_digest=receipt['initial_state_digest'],right_arm_model_joints=deepcopy(source['initial_state']['model']['right_arm_joints']))
    receipt['cup_geometry']['profile_sha256']='f'*64
    current,_,_=build_assisted_source(receipt,source['calibration_profile'],'current-geometry-fixture',binding)
    current.update(environment='physical',simulation_only=False,training_observation=False)
    native=deepcopy(parent['native_identity']);native['physical_configuration']=dict(base=dict(cup_asset_entrypoint_sha256='d'*64))
    block=dict(schema_version=maintenance.CURRENT_GEOMETRY_SCHEMA,revision='cup-new-revision',parent_role='trajectory_seed_only',
        source_digest=_digest(current),export_sha256='a'*64,asset_pins_sha256='b'*64,asset_receipt_sha256='c'*64,
        object_mesh_sha256='d'*64,native_identity_digest=_digest(native))
    source['maintenance_context']['current_geometry']=block;binding['source_digest']=_digest(source)
    geometry.update(source_digest=_digest(source),current_geometry=deepcopy(block),current_geometry_source=current,
        current_native_identity=native,parent_source_digest=_digest(old))
    return (*values,current)


class MaintenanceTest(unittest.TestCase):
    def test_current_geometry_keeps_real_parent_and_uses_current_active_native_identity(self):
        parent,old,receipt,source,binding,offset,geometry,_,current=current_fixture()
        original=deepcopy(parent);plan=maintenance.derive_return(parent,old,receipt,source,offset,geometry)
        maintenance.validate_evidence(plan,source);validate_real_plan(plan,source,binding)
        self.assertEqual(parent,original)
        self.assertEqual(plan['native_identity'],geometry['current_native_identity'])
        self.assertNotEqual(plan['native_identity'],parent['native_identity'])
        self.assertEqual(plan['maintenance_derivation']['parent_source'],old)
        self.assertEqual(plan['maintenance_derivation']['parent_execution'],receipt)
        self.assertEqual(plan['gripper_distance_control'],parent['gripper_distance_control'])
        bad=deepcopy(plan);bad['native_identity']=deepcopy(parent['native_identity'])
        with self.assertRaises(ValueError):maintenance.validate_evidence(bad,source)

    def test_current_geometry_rejects_mixed_parent_policy_and_fresh_source_substitution(self):
        parent,old,receipt,source,_,offset,geometry,_,_=current_fixture()
        for change in (lambda g:g.update(parent_source_digest='0'*64),
                       lambda g:g['current_native_identity'].update(acceptance={'relaxed':True}),
                       lambda g:g['current_geometry'].update(export_sha256='0'*64)):
            wrong=deepcopy(geometry);change(wrong)
            with self.assertRaises(ValueError):maintenance.derive_return(parent,old,receipt,source,offset,wrong)
        for change in (lambda c:c['grasp_command'].update(metal_gap_mm=65.),
                       lambda c:c['initial_state']['planned_reported']['left_arm_joints'].__setitem__(0,.5),
                       lambda c:c['calibration_profile']['joints']['zero_offsets_rad'].__setitem__(0,.1),
                       lambda c:c.update(training_observation=True)):
            wrong_source=deepcopy(source);wrong=deepcopy(geometry);change(wrong['current_geometry_source'])
            wrong_source['maintenance_context']['current_geometry']['source_digest']=_digest(wrong['current_geometry_source'])
            wrong['current_geometry']=deepcopy(wrong_source['maintenance_context']['current_geometry']);wrong['source_digest']=_digest(wrong_source)
            with self.assertRaises(ValueError):maintenance.derive_return(parent,old,receipt,wrong_source,offset,wrong)
        legacy=deepcopy(source);legacy['maintenance_context'].pop('current_geometry')
        wrong=deepcopy(geometry);wrong['source_digest']=_digest(legacy)
        with self.assertRaises(ValueError):maintenance.derive_return(parent,old,receipt,legacy,offset,wrong)

    def test_physical_parent_current_right_61_release_and_owner_data_only_validation(self):
        parent,old,receipt,source,binding,offset,geometry,_=fixture()
        plan=maintenance.derive_return(parent,old,receipt,source,offset,geometry)
        self.assertNotIn('physical_gpu_conversion',plan)
        self.assertEqual(plan['environment'],'physical');self.assertFalse(plan['simulation_only'])
        self.assertFalse(source['training_observation']);self.assertEqual(sum(validate_release_control(plan)),53)
        self.assertEqual(plan['gripper_distance_control']['closed_metal_gap_mm'],61.)
        self.assertTrue(all(t['target']['right_arm_joints']==source['initial_state']['planned_reported']['right_arm_joints'] for t in plan['ticks']))
        self.assertNotEqual(plan['ticks'][0]['target']['right_arm_joints'],parent['ticks'][-1]['target']['right_arm_joints'])
        validate_real_plan(plan,source,binding)
        options=dict(sink=lambda _:self.fail('no control'),gate=lambda:False,stop=lambda _:self.fail('no control'),
            source_validator=lambda p,s:validate_real_plan(p,s,binding))
        with self.assertRaisesRegex(ValueError,'runtime right'):FrozenExecutor(plan,source,**options)
        follower=FrozenExecutor(plan,source,disabled_grippers=('right',),**options)
        self.assertEqual(follower.held_arms,('right',))
        with self.assertRaises(ValueError):validate_real_plan(plan,source,binding,environment='fake')

    def test_parent_log_gaps_changed_targets_and_missing_terminal_are_rejected(self):
        parent,old,_,_,_,_,_,events=fixture()
        for edit in (lambda e:e.pop(2),lambda e:e[2]['target']['left_arm_joints'].__setitem__(0,.4),
                     lambda e:e[-1].update(status='FAULT'),lambda e:e[-2]['frozen'].update(targets_submitted=False)):
            value=deepcopy(events);edit(value)
            with self.assertRaises(ValueError):maintenance.parent_execution_receipt(value,parent,old,'b'*64)

    def test_no_offset_or_geometry_can_silently_claim_acceptance(self):
        parent,old,receipt,source,_,offset,geometry,_=fixture()
        with self.assertRaises(ValueError):maintenance.derive_return(parent,old,receipt,source,None,geometry)
        with self.assertRaises(ValueError):maintenance.derive_return(parent,old,receipt,source,offset,None)
        for edit in (lambda g:g.update(source_digest='0'*64),lambda g:g.update(model_target_vectors_sha256='0'*64),
                     lambda g:g['checks']['cup_world'].update(passed=False),lambda g:g['checks']['fixed_right_static'].update(passed=True),
                     lambda g:g.update(max_sample_step_rad=.02)):
            bad=deepcopy(geometry);edit(bad)
            with self.assertRaises(ValueError):maintenance.derive_return(parent,old,receipt,source,offset,bad)
        plan=maintenance.derive_return(parent,old,receipt,source,offset,geometry)
        plan['ticks'][0]['target']['left_arm_joints'][0]+=.01
        with self.assertRaises(ValueError):maintenance.validate_evidence(plan,source)

    def test_all_requested_world_offsets_share_the_fixed_origin(self):
        _,old,_,source,_,_,_,_=fixture();origin=source['maintenance_context']['return_origin']
        for offset in ([-20.,20.],[-10.,10.],[-15.,-10.]):
            context=deepcopy(source['maintenance_context']);context['offset_world_xy_mm']=offset;context.pop('return_goal')
            current=maintenance.build_source(old,source['initial_state'],context,'offset-fixture')
            self.assertEqual(current['maintenance_context']['return_goal']['position'],
                [origin['position'][0]+offset[0]/1000,origin['position'][1]+offset[1]/1000,origin['position'][2]])


if __name__=='__main__':unittest.main()
