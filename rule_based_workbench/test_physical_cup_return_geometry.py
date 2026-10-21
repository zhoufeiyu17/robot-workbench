"""CPU contract checks; no native asset, CUDA, or physical qualification."""
from copy import deepcopy
from types import SimpleNamespace
from pathlib import Path
import hashlib,json,tempfile
import unittest
from unittest.mock import patch

from contract import demo_state
from physical_cup_return_geometry import (interpolated_rows,bind_current_right,offset_base,offset_artifact,
                                         tray_support_for_closing,current_geometry_inputs)
from test_cup_return_offset import LinearKinematics,encode
from test_cup_return_replay import fixture
from cup_return_offset import apply_offset


class GeometryTest(unittest.TestCase):
    def test_fixed_arm_jaw_composition_keeps_strict_gpu_gates_and_source_contact_bound(self):
        import numpy as np
        from physical_cup_return_geometry import fixed_arm_jaw_path_check
        from gpu_acceptance import jaw_input_digest
        engine=SimpleNamespace(tensor_args=SimpleNamespace(to_device=lambda x:np.asarray(x,dtype=np.float32)),
            torch=SimpleNamespace(equal=np.array_equal),config={'kinematics':{'lock_joints':{'right_joint11':.2}}})
        rows=[dict(left_arm_joints=[.1]*6,left_gripper=gap,right_arm_joints=[.2]*6,right_gripper=83.)for gap in (82.,78.,65.)]
        cert=dict(certificate_digest='a'*64,claw_cup_allowance_m=.003,terminal_jaw_position_m=.038,linked_jaws=True,grasp_q_model=[.1]*6)
        jaw=dict(passed=True,model_arm_q=[.1]*6,model_locked_joints={'right_joint11':.2},
            unresolved_claw_samples=0,self_max=0.,environment_max=0.,cup_wrist_max=0.,cup_world_max=0.,
            cup_claw_max=100.,source_bound_used_samples=74,contact_certificate_digest='a'*64,
            maximum_jaw_step_m=.00005,continuous_translation_padding_m=.000026,start_m=.041,end_m=.038)
        jaw['native_input_digest']=jaw_input_digest(engine,jaw['model_arm_q'],jaw['model_locked_joints'])
        proof=fixed_arm_jaw_path_check(engine,rows,jaw,cert,stage_name='close_ramp')
        self.assertTrue(proof['passed']);self.assertEqual(proof['raw_cup_claw_max'],100.)
        for kind in ('arm','right','right_command','world','wrist','unresolved','certificate','range','certificate_pose','digest','direction'):
            samples,changed,certificate=deepcopy(rows),deepcopy(jaw),deepcopy(cert)
            if kind=='arm':samples[1]['left_arm_joints'][0]+=.001
            elif kind=='right':changed['model_locked_joints']['right_joint11']+=.001
            elif kind=='world':changed['environment_max']=1.
            elif kind=='wrist':changed['cup_wrist_max']=1.
            elif kind=='unresolved':changed['unresolved_claw_samples']=1
            elif kind=='certificate':certificate['certificate_digest']='b'*64
            elif kind=='certificate_pose':certificate['grasp_q_model'][0]+=.001
            elif kind=='right_command':samples[1]['right_arm_joints'][0]+=.001
            elif kind=='digest':changed['native_input_digest']='b'*64
            elif kind=='direction':samples[1]['left_gripper']=76.;samples[2]['left_gripper']=78.
            else:samples[0]['left_gripper']=84.
            with self.subTest(kind=kind),self.assertRaises(ValueError):
                fixed_arm_jaw_path_check(engine,samples,changed,certificate,stage_name='close_ramp')
        opened=deepcopy(rows[::-1]);opening=dict(jaw,start_m=.038,end_m=.041)
        old_pose=dict(cert,grasp_q_model=[9.]*6)
        self.assertTrue(fixed_arm_jaw_path_check(engine,opened,opening,old_pose,stage_name='open')['passed'])
        with self.assertRaises(ValueError):fixed_arm_jaw_path_check(engine,rows,jaw,cert,stage_name='home_gripper_restore')

    def test_contact_stage_receipts_bind_source_and_cannot_replace_hold_checks(self):
        from integration import _digest
        from physical_cup_maintenance import validate_jaw_path_receipts
        cert=dict(certificate_digest='a'*64,source_digest='b'*64)
        stages=[]
        for name,mode in [('close_ramp','closing_sweep'),('open','opening_sweep')]:
            jaw=dict(passed=True,contact_certificate_digest='a'*64,native_input_digest='c'*64)
            gpu=dict(passed=True,geometry_backend='gpu_jaw_sweep',gpu_jaw_sweep_digest=_digest(jaw),
                contact_certificate_digest='a'*64,native_input_digest='c'*64,samples=3)
            stages.append(dict(name=name,cup_mode=mode,gpu=gpu,jaw=jaw,samples=3))
        receipt=dict(source_digest='b'*64,gripper_closing=dict(contact_certificate=cert),path_checks=stages)
        validate_jaw_path_receipts(receipt)
        for kind in ('source','digest','input','missing','hold'):
            bad=deepcopy(receipt)
            if kind=='source':bad['source_digest']='d'*64
            elif kind=='digest':bad['path_checks'][0]['jaw']['changed']=True
            elif kind=='input':bad['path_checks'][0]['gpu']['native_input_digest']='d'*64
            elif kind=='missing':bad['path_checks'].pop()
            else:bad['path_checks'][0]['name']='close_and_hold'
            with self.subTest(kind=kind),self.assertRaises(ValueError):validate_jaw_path_receipts(bad)

    def test_current_geometry_checks_export_assets_and_original_mesh_before_native_load(self):
        from test_physical_cup_maintenance import current_fixture
        from integration import _digest
        parent,old,_,source,_,_,receipt,_,current=current_fixture();block=source['maintenance_context']['current_geometry']
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);assets=root/'assets';entry=assets/'objects/anyverse_lidded_paper_cup_9cm_13cm/v1/sim/cup.usda'
            entry.parent.mkdir(parents=True);entry.write_bytes(b'CPU fixture only; not native geometry')
            sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
            alias=entry.with_name('cup-alias.usda');alias.symlink_to(entry.name)
            pins=root/'asset-pins.json';pins.write_text(json.dumps({str(entry):sha(entry),str(alias):sha(entry)}))
            mesh=root/'model.obj';mesh.write_text('CPU mesh fixture; no physical qualification')
            proof=root/'mesh-proof.json';proof.write_text(json.dumps(dict(mesh_path=str(mesh),mesh_sha256=sha(mesh),
                proof=dict(all_mesh_triangle_clips_inside_proxy=True))))
            asset=dict(status='qualified',assets=str(assets),asset_pins_sha256=sha(pins),
                cup_profile_sha256=current['cup_geometry']['profile_sha256'],bands=current['cup_geometry']['bands'],
                a1_asset_validation=dict(model_binding=source['calibration_profile']['model_binding']),
                envelope_proof=dict(continuous_core_verification=dict(passed=True)),estimate=dict(path=str(proof),sha256=sha(proof)))
            asset_path=root/'receipt.json';asset_path.write_text(json.dumps(asset))
            native=deepcopy(receipt['current_native_identity']);native['physical_configuration']['base']['cup_asset_entrypoint_sha256']=sha(entry)
            data=dict(source_digest=_digest(current),urdf_sha256=source['calibration_profile']['model_binding']['urdf_sha256'],native_identity=native)
            from maintenance_geometry import PURPOSE,STATUS
            data['export_scope']=dict(purpose=PURPOSE,session='CPU fixture',environment='physical',
                source_digest=_digest(current),parent_source_digest=source['parent_source_digest'],
                parent_timeline_digest=source['maintenance_context']['parent_timeline_digest'],
                parent_execution_digest=source['maintenance_context']['parent_execution_digest'],
                asset_pins_sha256=sha(pins),asset_receipt_sha256=sha(asset_path),object_mesh_sha256=sha(mesh),
                task_characterization='not_run',support='tray',initial_support_audit=dict(passed=True),native_runtime_sha256='a'*64)
            data['export_scope_digest']=_digest(data['export_scope'])
            (root/'result.json').write_text(json.dumps(dict(status=STATUS)))
            export=root/'planning-input.json';export.write_text(json.dumps(data))
            block.update(export_sha256=sha(export),asset_pins_sha256=sha(pins),asset_receipt_sha256=sha(asset_path),
                         object_mesh_sha256=sha(mesh),native_identity_digest=_digest(native))
            selected,checked=current_geometry_inputs(parent,old,source,export,assets,current)
            self.assertEqual(selected,current);self.assertEqual(checked,block)
            for target in (root/'outside.usda',entry.with_name('unpinned.usda')):
                target.write_bytes(entry.read_bytes());alias.unlink();alias.symlink_to(target)
                with self.assertRaisesRegex(ValueError,'current asset path or pinned bytes differ'):
                    current_geometry_inputs(parent,old,source,export,assets,current)
            for target in (entry.parent,alias,entry.with_name('missing.usda')):
                alias.unlink();alias.symlink_to(target)
                with self.assertRaisesRegex(ValueError,'current asset path or pinned bytes differ'):
                    current_geometry_inputs(parent,old,source,export,assets,current)
            alias.unlink();alias.symlink_to(entry.name)
            with self.assertRaises(ValueError):current_geometry_inputs(parent,old,source,export,assets,None)
            legacy=deepcopy(source);legacy['maintenance_context'].pop('current_geometry')
            with self.assertRaises(ValueError):current_geometry_inputs(parent,old,legacy,export,assets,current)
            for path in (entry,mesh,export,pins,asset_path):
                raw=path.read_bytes();path.write_bytes(raw+b' ')
                with self.assertRaises(ValueError):current_geometry_inputs(parent,old,source,export,assets,current)
                path.write_bytes(raw)

    def test_tray_closing_only_changes_existing_support_context_and_restores_on_failure(self):
        calls=[]
        def original(frames,cup,**options):calls.append(options);return 'original'
        geometry=SimpleNamespace(audit=original)
        with self.assertRaisesRegex(ValueError,'stop'):
            with tray_support_for_closing(geometry):
                self.assertEqual(geometry.audit(None,None,initial_table_contact=True,grasp_contact=True),'original')
                geometry.audit(None,None,initial_table_contact=False,grasp_contact=True)
                raise ValueError('stop')
        self.assertIs(geometry.audit,original)
        self.assertEqual(calls,[dict(initial_table_contact=False,grasp_contact=True,support_contact=True),
            dict(initial_table_contact=False,grasp_contact=True)])

    def test_sampling_includes_large_stage_join_and_all_original_rows(self):
        a=demo_state();b=deepcopy(a);b['left_arm_joints'][0]+=.123;b['right_arm_joints'][5]-=.046
        c=deepcopy(b);c['left_arm_joints'][2]+=.021
        values,maximum=interpolated_rows([b,c],a)
        self.assertEqual(values[0],a);self.assertEqual(values[-1],c)
        self.assertTrue(any(all(abs(x-y)<1e-12 for x,y in zip(row['left_arm_joints'],b['left_arm_joints'])) for row in values))
        self.assertLessEqual(maximum,.01+1e-12);self.assertGreater(len(values),3)
        with self.assertRaises(ValueError):interpolated_rows([b],a,maximum_step_rad=.011)
        a['left_gripper']=75.;b['left_gripper']=75.666
        values,_=interpolated_rows([b],a,maximum_gap_step_mm=.1)
        self.assertEqual(values[0],a);self.assertEqual(values[-1],b)
        self.assertLessEqual(max(abs(y['left_gripper']-x['left_gripper']) for x,y in zip(values,values[1:])),.1)

    def test_physical_offset_preserves_release_clock_and_checks_emitted_fk(self):
        original,parent,receipt,source,binding,base=fixture()
        source['replay_context']['world_from_robot']=dict(position=[0.,0.,0.],orientation_wxyz=[1.,0.,0.,0.])
        kine=LinearKinematics('fixture.urdf',source,source['replay_context']['world_from_robot'])
        names=[s['name'] for s in base['stages']];place=base['ticks'][base['stages'][names.index('support_settle')]['start_tick']]['target']
        pose=kine.pose(place);pose['position'][0]+=.02;pose['position'][1]-=.01;pose['position'][2]+=.001
        with patch('cup_return_offset.NativeReturnKinematics',LinearKinematics),patch('motiongen_plan.encode_stage',encode),\
             patch('isaac_lane.native_target',side_effect=lambda values,*_,**kw:dict(demo_state(),left_arm_joints=values[:6],left_gripper=values[6],right_arm_joints=values[7:13],right_gripper=values[13])):
            plan,proof=apply_offset(base,source,dict(effective_delta_m=[.02,-.01],placement_tool_pose=pose),kine,environment='physical')
        self.assertEqual(proof['environment'],'physical');self.assertIn('every emitted',proof['fk_error_scope'])
        self.assertLessEqual(proof['maximum_fk_position_error_m'],1e-5)
        self.assertEqual(proof['placement_tool_pose'],pose)
        for name in ('open','release_hold'):
            def segment(p):
                i=next(i for i,s in enumerate(p['stages']) if s['name']==name)
                start=p['stages'][i]['start_tick'];end=p['stages'][i+1]['start_tick'] if i+1<len(p['stages']) else len(p['ticks'])
                return [t['target']['left_gripper'] for t in p['ticks'][start:end]]
            self.assertEqual(segment(plan),segment(base))

    def test_right_rebinding_retains_left_and_records_original_projection(self):
        original,parent,receipt,source,binding,base=fixture()
        source['calibration_profile']['joints']['zero_offsets_rad']=[0.]*12
        source['maintenance_context']=dict(right_hold_reported=[.1,.2,-.0007,.4,.5,.6])
        source['initial_state']['planned_reported']=deepcopy(base['ticks'][0]['target'])
        stage=dict(name='initial_hold',substage='initial_hold',parent_stage_index=0,parent_tick_indices=[0],
                   rows=[deepcopy(base['ticks'][0]['target'])],native_actions=[deepcopy(base['native_actions'][0])])
        kine=SimpleNamespace(spec=None,limits={'right':[(-2.,2.),(-2.,2.),(0.,2.),(-2.,2.),(-2.,2.),(-2.,2.)]})
        with patch('motiongen_plan.encode_stage',encode):
            rows,proof=bind_current_right([stage],source,kine,83.)
        self.assertEqual(proof['original_reported'],[.1,.2,-.0007,.4,.5,.6])
        self.assertEqual(proof['bounded_reported'][2],0.)
        self.assertEqual(rows[0]['rows'][0]['left_arm_joints'],stage['rows'][0]['left_arm_joints'])
        self.assertEqual(rows[0]['native_actions'][0][:7],stage['native_actions'][0][:7])
        self.assertTrue(rows[0]['right_hold_rebound'])
        source['maintenance_context']['right_hold_reported'][2]=-.011
        with patch('motiongen_plan.encode_stage',encode),self.assertRaises(ValueError):
            bind_current_right([stage],source,kine,83.)

    def test_right_rebound_artifact_matches_maintenance_contract(self):
        from test_physical_cup_maintenance import fixture as physical_fixture
        from physical_cup_maintenance import reverse_stages,_stages
        parent,old,receipt,source,binding,offset,geometry,_=physical_fixture()
        kine=SimpleNamespace(spec=None,limits={'right':[(-2.,2.)]*6})
        with patch('motiongen_plan.encode_stage',encode):
            stages,projection=bind_current_right(reverse_stages(parent,old),source,kine,parent['gripper_opening_mm'])
        proof=dict(kinematics=dict(urdf_sha256=source['calibration_profile']['model_binding']['urdf_sha256']),
                   local_stage_indices=[],right_hold_projection=projection,parent_timeline_digest=parent['timeline_digest'],
                   **{k:deepcopy(source['maintenance_context'][k]) for k in ('return_origin','offset_world_xy_mm','return_goal')})
        result=offset_artifact(offset_base(parent,stages),proof,stages)
        checked=_stages(parent,old,source,result)
        self.assertEqual(len(checked),len(stages))
        self.assertTrue(all(stage['right_hold_rebound'] for stage in checked))


if __name__=='__main__':unittest.main()
