"""Pure CPU fixture checks; official fitting and MotionGen are tested on the DSW."""
import hashlib
from pathlib import Path
import sys
import tempfile
from copy import deepcopy
from dataclasses import dataclass
from unittest.mock import patch

import numpy as np

from curobo_robot_model import (build_robot_model, fixed_world_mesh_descriptors, REQUIRED_LINKS,
                               PROXY_NUMERIC_STANDARDS, surface_net, refine_balls)


def main():
    vertices = np.array([[x, y, z] for x in (-.003, .003) for y in (-.003, .003) for z in (-.003, .003)])
    faces = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                      [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])

    def part(name, link):
        return dict(name=name, link=link, vertices=vertices.tolist(), faces=faces.tolist())

    def fake_fit(v, f, count, radius):
        return np.repeat(v.mean(0)[None], count, axis=0), np.full(count, radius), dict(method='fixture-only')

    # Pure deterministic arithmetic fixtures; neither an actual mesh fit nor a GPU test.
    triangle=np.array([[[0.,0.,0.],[.1,0.,0.],[0.,.002,0.]]])
    net, cover, normals=surface_net(np,triangle,.01)
    repeated=surface_net(np,triangle,.01)
    assert all(np.array_equal(a,b) for a,b in zip((net,cover,normals),repeated))
    assert np.all(cover<=.01) and np.allclose(normals[:,2],1.)
    # Every original triangle point has a nearby certified cell-centroid sample.
    samples=np.array([[.1*a,.002*b,0.] for a in np.linspace(0,1,21) for b in np.linspace(0,1-a,21)])
    assert np.all(np.min(np.linalg.norm(samples[:,None]-net[None],axis=2)-cover[None],axis=1)<=1e-12)
    line=np.column_stack((np.linspace(0,.2,81),np.zeros((81,2))))
    tolerance=np.full(len(line),.002)
    make=lambda i:(line[i],.006)
    empty=np.empty((0,3));empty_r=np.empty(0)
    c,r,g=refine_balls(np,line,tolerance,empty,empty_r,make,100)
    c2,r2,g2=refine_balls(np,line,tolerance,empty,empty_r,make,100)
    assert len(c)>6 and float((g-tolerance).max())<=1e-8
    assert np.array_equal(c,c2) and np.array_equal(r,r2) and np.array_equal(g,g2)
    try:refine_balls(np,line,tolerance,empty,empty_r,make,2)
    except ValueError as error:assert 'budget' in str(error) and 'residual' in str(error)
    else:raise AssertionError('unfinished adaptive proxy was silently truncated')

    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)/'a1.urdf'
        pieces = ['<robot name="a1-fixture"><link name="base_link"/>']
        state = {}
        for side, offset in (('left', 0), ('right', 10)):
            for index in range(1, 9):
                parent = 'base_link' if index == 1 else f'{side}_link{offset+min(index-1, 6)}'
                child, joint = f'{side}_link{offset+index}', f'{side}_joint{offset+index}'
                finger = index >= 7
                pieces.append(f'<link name="{child}"/><joint name="{joint}" type="{"prismatic" if finger else "revolute"}">'
                              f'<parent link="{parent}"/><child link="{child}"/><origin xyz=".03 0 0"/>'
                              '<axis xyz="0 1 0"/><limit lower="0" upper="3.5" velocity="2" effort="1"/></joint>')
                state[joint] = .025 if finger else .12
        path.write_text(''.join(pieces)+'</robot>')
        parts = [part('base_link/chassis_0/component_'+str(i), 'base_link') for i in (6, 19, 32)]
        parts += [part(link+'/body', link) for link in sorted(REQUIRED_LINKS-{'base_link'})]
        parts += [part(link+'/rubber_pad', link) for link in ('left_link7', 'left_link8', 'right_link17', 'right_link18')]
        canonical = sorted(tuple(sorted(tuple(float(v)+0. for v in point) for point in triangle)) for triangle in vertices[faces])
        mount_sha = hashlib.sha256(np.asarray(canonical, dtype='<f8').tobytes()).hexdigest()
        urdf_sha = hashlib.sha256(path.read_bytes()).hexdigest()
        data = dict(urdf_sha256=urdf_sha, parts=parts, observation=dict(scene=dict(shapes=[])), table={}, obstacles=[],
            planner_identity=dict(chassis_collision=dict(component_count=3,
                base_arm_exemptions=[['base_link/chassis_0/component_6', 'left_link1'],
                                     ['base_link/chassis_0/component_19', 'right_link11']],
                mount_component_sha256={'6': mount_sha, '19': mount_sha}),
                support_contact=dict(urdf_sha256=urdf_sha)))
        config, report = build_robot_model(data, path, set(state), state, ignored_link_pairs=[('left_link1', 'left_link2')], fit=fake_fit)
        kin = config['kinematics']
        assert len(kin['collision_link_names']) == 12 and report['allocated_spheres'] <= report['sphere_budget']
        assert report['part_coverage'] == dict(source_parts=23, sphere_parts=11, fixed_mesh_parts=12, complete=True, omitted_parts=[])
        assert len(report['part_map']) == len(parts)
        assert len(report['part_reports']) + len(report['fixed_world_geometry']['parts']) == len(parts)
        assert report['fixed_world_geometry']['cross_arm_exemptions'] == []
        assert not report['fixed_world_geometry']['cavities_filled'] and not report['fixed_world_geometry']['convex_hull']
        assert report['optimization_proxy_only'] and not report['conservative_coverage_proven']
        assert report['requires_complete_native_final_audit'] and report['max_sampled_gap_m'] > 0
        assert report['sphere_budget']==4096 and report['fixture_fitter_used'] and not report['numeric_quality_gate_passed']
        assert report['numeric_standards']==PROXY_NUMERIC_STANDARDS
        assert report['promotion_ready'] is False and not report['historical_mast_gpu_collision_verified']
        assert set(report['combined_metal_pad_sampled_coverage'])=={'left_link7','left_link8'}
        assert report['base_world_exclusion_required'] and len(report['base_collision_aliases']) == 1
        assert report['base_world_exclusion_configured'] and not report['base_world_exclusion_gpu_verified']
        for alias in report['base_collision_aliases']:
            radii = np.array([sphere['radius'] for sphere in kin['collision_spheres'][alias]])
            buffer = kin['collision_sphere_buffer'][alias]
            world_radii = (radii+buffer).astype(np.float32)
            self_radii = world_radii+np.float32(kin['self_collision_buffer'][alias]-buffer)
            assert np.all(world_radii < -1.) and np.all(self_radii.astype(float) >= radii)
            assert len(kin['collision_spheres'][alias]) > 0, 'base self spheres must remain present'
        for name, item in report['part_map'].items():
            if item['representation'] == 'fixed_world_mesh':
                assert item['alias'] not in kin['collision_spheres'] and item['alias'] not in kin['extra_links']
                continue
            assert kin['extra_links'][item['alias']]['parent_link_name'] == item['link']
            assert kin['extra_links'][item['alias']]['fixed_transform'] == [0., 0., 0., 1., 0., 0., 0.]
        exempt = {frozenset(row['parts']): row['reason'] for row in report['exemptions']}
        assert exempt[frozenset(('base_link/chassis_0/component_6', 'left_link1/body'))] == 'pinned_mount_component'
        assert frozenset(('base_link/chassis_0/component_6', 'left_link3/body')) not in exempt
        assert frozenset(('left_link3/body', 'right_link13/body')) not in exempt
        assert exempt[frozenset(('left_link2/body', 'left_link5/body'))] == 'native_local_support_delegated_to_exact_final_audit'
        assert kin['lock_joints']['right_joint11'] == .12 and kin['lock_joints']['left_joint7'] == .025
        assert kin['extra_links']['attached_object']['parent_link_name'] == 'left_link6'
        assert kin['extra_collision_spheres']['attached_object'] == 64
        assert len(report['attached_object']['proxy_ignored_pad_aliases']) == 2
        assert frozenset(('left_link6/body', 'attached_object')) not in exempt
        assert frozenset(('left_link7/body', 'attached_object')) not in exempt
        assert exempt[frozenset(('left_link7/rubber_pad', 'attached_object'))] == 'attached_cup_grasp_pad_delegated_to_exact_final_audit'
        _,contact_report=build_robot_model(data,path,set(state),state,ignored_link_pairs=[('left_link1','left_link2')],fit=fake_fit,claw_cup_allowance_m=.003)
        contact_exempt={frozenset(row['parts']):row['reason'] for row in contact_report['exemptions']}
        assert contact_exempt[frozenset(('left_link7/body','attached_object'))]=='fixed_relative_claw_cup_3mm_checked_at_attachment'
        assert contact_exempt[frozenset(('left_link6/body','attached_object'))]=='fixed_relative_wrist_cup_strict_gpu_checked_at_attachment'
        assert contact_report['attached_object']['fixed_relative_contact_certificate_required']
        assert contact_report['attached_object']['claw_cup_allowance_m']==.003


        @dataclass
        class Pose:
            position: tuple = (0., 0., 0.)
            orientation_wxyz: tuple = (1., 0., 0., 0.)
        class FK:
            def forward(self, values):
                return {name: Pose((values['right_joint11'], 0., 0.)) if name.startswith('right_') else Pose()
                        for name in REQUIRED_LINKS}
            def pose(self, value): return value
        # Geometry library winding repair is exercised by the actual-mesh CPU
        # smoke, not replaced with a guessed implementation in this fixture.
        with patch('curobo_robot_model.outward_fixed_mesh', side_effect=lambda v, f: deepcopy(f)) as repair:
            descriptors = fixed_world_mesh_descriptors(data, report, FK(), state)
            assert repair.call_count == 12
            assert len(descriptors) == report['fixed_world_geometry']['part_count']
            for item in descriptors:
                original = next(row for row in parts if row['name'] == item['source_part_name'])
                assert item['vertices'] == original['vertices'] and item['faces'] == original['faces']
                assert item['base_from_shape']['position'] == ((.12, 0., 0.) if item['link'].startswith('right_') else (0., 0., 0.))
            assert any(row['source_part_name'].endswith('component_32') for row in descriptors), 'mast omitted'
            assert not any(row['source_part_name'].endswith('component_6') for row in descriptors), 'left mount leaked into unfilterable world'
            assert any(row['source_part_name'].endswith('component_19') for row in descriptors), 'fixed right mount omitted'
            # A moving left jaw is allowed; right arm and right jaws are frozen.
            fixed_world_mesh_descriptors(data, report, FK(), dict(state, left_joint7=.03, left_joint8=.03))
            for name in ('right_joint11', 'right_joint17'):
                try:
                    fixed_world_mesh_descriptors(data, report, FK(), dict(state, **{name: state[name]+.0001}))
                except ValueError as error:
                    assert 'fixed right-arm/gripper command changed' in str(error)
                else:
                    raise AssertionError('fixed right state change accepted')
            altered = deepcopy(data); altered['parts'][-1]['vertices'][0][0] += .001
            try:
                fixed_world_mesh_descriptors(altered, report, FK(), state)
            except ValueError as error:
                assert 'source differs' in str(error)
            else:
                raise AssertionError('changed fixed geometry accepted')
        try:
            build_robot_model(data, path, set(state), state, ignored_link_pairs=[('left_link1', 'right_link11')], fit=fake_fit)
        except ValueError as error:
            assert 'cross-arm pair exemptions' in str(error)
        else:
            raise AssertionError('unmapped world-pair exemption accepted')

        def rejected(**kwargs):
            try:
                build_robot_model(data, path, set(state), state, ignored_link_pairs=[], fit=fake_fit, **kwargs)
            except ValueError:
                return
            raise AssertionError('invalid sphere budget was accepted')

        rejected(sphere_budget=1)
        rejected(counts={'pad': 0})
        # Compiler contract fixture only: not a geometric proof for a robot.
        certificate=dict(urdf_sha256=urdf_sha,source_parts_sha256=report['source_parts_sha256'],checks=[dict(
            certified=True,parts=['base_link/chassis_0/component_32','left_link1/body'],
            reason='continuous_interval_separation',minimum_separation_lower_bound_m=.001,
            joint_domain={n:[v['lower'],v['upper']] for n,v in report['kinematics']['joints'].items()})])
        _,certified=build_robot_model(data,path,set(state),state,ignored_link_pairs=[],fit=fake_fit,domain_certificate=certificate)
        assert certified['certified_domain_pair_count']==1
        assert certified['part_map']['base_link/chassis_0/component_32']['representation']=='robot_spheres'
        assert any(row['reason']=='offline_joint_domain_proof' for row in certified['exemptions'])
        wrong=deepcopy(certificate);wrong['checks'][0]['joint_domain']['left_joint1'][0]+=.01
        rejected(domain_certificate=wrong)
        upper=deepcopy(certificate);upper['checks'][0]['parts'][1]='left_link2/body'
        rejected(domain_certificate=upper)
        upper['checks'][0]['numerical_distance_reserve_m']=float('nan')
        rejected(domain_certificate=upper)
        upper['checks'][0]['numerical_distance_reserve_m']=1e-5
        _,verified=build_robot_model(data,path,set(state),state,ignored_link_pairs=[],fit=fake_fit,domain_certificate=upper)
        assert any(set(row['parts'])=={'base_link/chassis_0/component_32','left_link2/body'} and row['reason']=='offline_joint_domain_proof' for row in verified['exemptions'])
        wrong=deepcopy(certificate);wrong['source_parts_sha256']='0'*64
        rejected(domain_certificate=wrong)
        data['parts'] = [row for row in parts if row['link'] != 'right_link18']
        rejected()
        data['parts'] = parts
        data['planner_identity']['chassis_collision']['mount_component_sha256']['6'] = '0'*64
        rejected()
        assert 'torch' not in sys.modules and 'curobo' not in sys.modules
        print(dict(passed=True, fixture_parts=len(parts), spheres=report['actual_spheres'],
                   guard='part-specific mount exemption, symmetric jaws, hard budget, source identity',
                   no_curobo_or_torch_import=True))


if __name__ == '__main__':
    main()
