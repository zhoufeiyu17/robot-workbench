"""Completed-parent reversal contract; CPU fixtures never qualify real motion."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import physical_cup_maintenance as m
from integration import _digest
from joint_space import convert_state
from test_physical_cup_maintenance import fixture


class ParentReplayTest(unittest.TestCase):
    def setUp(self):
        self.parent,self.old,self.receipt,self.prior,self.binding,*_=fixture()
        self.initial=deepcopy(self.prior['initial_state'])

    def source(self):
        return m.build_parent_replay_source(self.parent,self.old,self.receipt,self.initial,'parent-replay-cpu')

    def test_only_reverses_completed_parent_targets_without_ik_or_collision(self):
        source=self.source()
        with patch.object(m,'_stages',side_effect=AssertionError('IK/offset route invoked')),patch.object(
                m,'_validate_geometry',side_effect=AssertionError('new collision route invoked')):
            plan=m.derive_return(self.parent,self.old,self.receipt,source)
            m.validate_evidence(plan,source)
        rows=[row for stage in m.reverse_stages(self.parent,self.old)for row in stage['rows']]
        self.assertEqual([tick['target']for tick in plan['ticks']],rows)
        self.assertNotIn('observation',source['maintenance_context']['current_scene'])
        for key in ('perception','ik','collision_check'):
            self.assertEqual(plan['planning_validation'][key],dict(performed=False))

    def test_parent_must_have_all_commands_completed_feedback_and_ready(self):
        for key in ('commands','completion','terminal'):
            with self.subTest(key=key):
                bad=deepcopy(self.receipt)
                if key=='commands':bad[key]=bad[key][:-1]
                elif key=='completion':bad[key]['status']='failed'
                else:bad[key]['status']='FAULT'
                with self.assertRaises(ValueError):
                    m.build_parent_replay_source(self.parent,self.old,bad,self.initial,'bad-parent')

    def test_current_independent_owner_continuous_events_need_actual_window_admissions(self):
        events=[]
        for command in self.receipt['commands']:
            row=deepcopy(command);stamp=row.pop('wall_ns');row.pop('event')
            row.update(execution_semantics='source_reference_knot',send_wall_ns=stamp)
            events.append(dict(sequence=len(events)+1,event='source_target',value=row,observed_wall_ns=stamp+1))
        window=dict(reference_id='c'*64,frames=12,controller_profile='cpu-fixture',
            admission=dict(reference_id='c'*64,frames=12,controller_profile='cpu-fixture',firmware_ack=False),
            command_semantics='native_future_window_admission_not_source_knot_send')
        events.append(dict(sequence=len(events)+1,event='native_reference_window',value=window,observed_wall_ns=9999))
        completion=deepcopy(self.receipt['completion']);stamp=completion.pop('wall_ns')
        events.append(dict(sequence=len(events)+1,event='feedback',value=dict(frozen=completion,raw={'sample_wall_ns':stamp}),observed_wall_ns=stamp+1))
        terminal=deepcopy(self.receipt['terminal']);stamp=terminal.pop('wall_ns');terminal.pop('event')
        events.append(dict(sequence=len(events)+1,event='motion_state',value=terminal,observed_wall_ns=stamp+1))
        receipt=m.parent_execution_receipt(events,self.parent,self.old,'d'*64)
        self.assertEqual(receipt['commands'][0]['event'],'source_target')
        self.assertEqual(receipt['terminal']['wall_ns'],stamp+1)
        source=m.build_parent_replay_source(self.parent,self.old,receipt,self.initial,'native-parent')
        self.assertFalse(m.derive_return(self.parent,self.old,receipt,source)['planning_validation']['collision_check']['performed'])
        for change in ('missing-window','wrong-window','mixed-semantics'):
            bad=deepcopy(receipt)
            if change=='missing-window':bad.pop('native_reference_windows')
            elif change=='wrong-window':bad['native_reference_windows'][0]['admission']['reference_id']='e'*64
            else:bad['commands'][0]['event']='command'
            with self.subTest(change=change),self.assertRaises(ValueError):m.validate_parent_execution(self.parent,self.old,bad)

    def test_joint_noise_is_tolerated_without_rebasing_or_new_planning(self):
        source=self.source();plan=m.derive_return(self.parent,self.old,self.receipt,source)
        self.assertEqual(plan['ticks'][0]['target']['right_arm_joints'],self.parent['ticks'][-1]['target']['right_arm_joints'])
        self.assertNotEqual(plan['ticks'][0]['target']['right_arm_joints'],self.initial['planned_reported']['right_arm_joints'])
        for key in ('measured_reported','planned_reported','after_observation_reported'):
            self.initial[key]['right_arm_joints'][0]+=.03
        self.initial['model']=convert_state(self.initial['planned_reported'],self.old['calibration_profile'],to_reported=False)
        source=self.source()
        with self.assertRaisesRegex(ValueError,'inactive right|initial|current'):
            m.derive_return(self.parent,self.old,self.receipt,source)

    def test_new_scene_offset_ik_or_collision_proofs_cannot_enter_direct_replay(self):
        source=self.source()
        for extra in ({'geometry_receipt':{}},{'offset_result':{}}):
            with self.assertRaisesRegex(ValueError,'cannot consume'):
                m.derive_return(self.parent,self.old,self.receipt,source,**extra)
        for key,value in [('offset_world_xy_mm',[1.,0.]),('pickup_policy',m.PICKUP_POLICY),
                          ('return_policy','unknown')]:
            bad=deepcopy(source);bad['maintenance_context'][key]=value
            with self.assertRaises(ValueError):m.validate_source(bad)

    def test_target_tampering_is_rejected_even_when_timeline_hash_is_updated(self):
        source=self.source();plan=m.derive_return(self.parent,self.old,self.receipt,source)
        plan['ticks'][3]['target']['left_arm_joints'][0]+=.001
        plan['timeline_digest']=_digest(plan['ticks'])
        with self.assertRaisesRegex(ValueError,'provenance|targets'):
            m.validate_evidence(plan,source)

    def test_direct_candidate_stages_through_the_actual_cpu_publication_boundary(self):
        from robot_workbench.physical_cup_return import stage_candidate
        source=self.source();plan=m.derive_return(self.parent,self.old,self.receipt,source)
        from test_fixed_right_policy import fixture as parent_fixture
        binding=m.build_binding(parent_fixture()[2],source)
        with TemporaryDirectory() as directory:
            root=Path(directory);candidate=dict(schema='physical_cup_return_candidate.v1',runtime_nonce='cpu-only',command_epoch=1)
            for name,value in (('source',source),('plan',plan),('binding',binding)):
                path=root/(name+'.json');path.write_text(json.dumps(value));candidate[name]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            path=root/'candidate.json';path.write_text(json.dumps(candidate))
            staged=stage_candidate(path,root/'stage')
            self.assertEqual(staged['metadata']['return_policy'],m.PARENT_REVERSE_POLICY)
            self.assertEqual(staged['metadata']['offset_xy_mm'],[0.,0.])
            self.assertEqual(json.loads((root/'stage/plan.json').read_text())['planning_validation']['collision_check'],dict(performed=False))

    def test_hardware_compiler_accepts_validated_parent_without_new_geometry_and_rejects_tampering(self):
        from types import SimpleNamespace
        from real_control import load_calibration,classes
        from real_contract import validate_real_plan
        source=self.source();plan=m.derive_return(self.parent,self.old,self.receipt,source)
        from test_fixed_right_policy import fixture as parent_fixture
        binding=m.build_binding(parent_fixture()[2],source)
        ref=load_calibration(Path(__file__).resolve().parents[1]/'calibration')
        owner_type,_,_=classes(ref);owner=object.__new__(owner_type)
        owner.hardware=True;owner.environment='physical';owner.backend=SimpleNamespace(passive_jaws=('right',))
        owner.source_validator=lambda p,s:validate_real_plan(p,s,binding,environment='physical')
        with patch.object(m,'_stages',side_effect=AssertionError('new IK invoked')),patch.object(
                m,'_validate_geometry',side_effect=AssertionError('new collision invoked')):
            payload,frozen,timeline=owner._compile_frozen_local(dict(action='preview_frozen',plan=plan,source=source),[(-5.,5.)]*14)
        self.assertEqual(payload['plan']['timeline_digest'],plan['timeline_digest'])
        self.assertTrue(timeline['commands']);self.assertEqual(frozen.held_arms,('right',))
        bad=deepcopy(plan);bad['maintenance_derivation']['parent_execution']['commands'].pop()
        with self.assertRaises(ValueError):owner._compile_frozen_local(dict(action='preview_frozen',plan=bad,source=source),[(-5.,5.)]*14)


if __name__=='__main__':unittest.main()
