"""Task-space fixture checks using native pose/polygon math; no IK or GPU."""
import argparse
import math
from copy import deepcopy
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import numpy as np


def main(native_code):
    root = native_code/'source/robot_sim/robot_sim'
    for suffix in ('', 'asset_cfg', 'asset_cfg.objects', 'tasks', 'tasks.manager_based', 'tasks.manager_based.coffee_make'):
        name = 'robot_sim'+('.'+suffix if suffix else '')
        module = ModuleType(name)
        module.__path__ = [str(root.joinpath(*suffix.split('.'))) if suffix else str(root)]
        sys.modules[name] = module
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform as Pose
    from motiongen_targets import REFERENCE, generate_targets, carry_targets, retreat_target, straight_waypoints

    begin=Pose((.00236,0.,0.));end=Pose((0.,.22,0.))
    sequence=straight_waypoints(begin,end)
    assert len(sequence)==2 and sequence[-1]==end
    assert np.allclose(sequence[0].position,((np.array(begin.position)+end.position)/2))
    assert all(p.orientation_wxyz==end.orientation_wxyz for p in sequence)
    class FakeFK:
        def forward(self, state):
            result = {name: np.eye(4) for name in ('left_link6', 'left_link7', 'left_link8')}
            result['left_link7'][:3, 3] = [.1, .02+state['left_joint7'], -.1]
            result['left_link8'][:3, 3] = [.1, -.02-state['left_joint8'], -.1]
            return result

        def pose(self, matrix):
            return Pose(tuple(matrix[:3, 3]))

        def inverse(self, *args, **kwargs):
            raise AssertionError('task-space generation must not call CPU IK')

    bands = [NS(shape_path='middle', z_min_m=0., z_max_m=.13, radius_m=.0347)]
    cup = NS(profile_sha256=REFERENCE['cup_profile_sha256'], bands=bands, candidate_grasp_band_m=(.04, .065))
    vertices = [(x, y, z) for x in (-.01, .01) for y in (-.003, .003) for z in (-.01, .01)]
    parts = [NS(name=link+'/body', link=link, vertices=vertices) for link in ('left_link6', 'left_link7', 'left_link8')]
    planner = NS(fk=FakeFK(), home_state=dict(left_joint7=.02, left_joint8=.02), parts=parts, cup=cup,
                 open_position_m=.041496, GRASP_CENTER_OFFSET_M=.016, PLACEMENT_TRACKING_RESERVE_M=.002,
                 MACHINE_PLACEMENT_RESERVE_M=.007,
                 identity={key: REFERENCE[key] for key in ('native_geometry_sha256', 'envelope_sha256')})
    support = Pose((.55, -.1, .78))
    scene = NS(world_from_support=support,
               support_polygon_world_m=tuple(support.transform_point(point) for point in
                   ((-.15, -.12, 0.), (.15, -.12, 0.), (.15, .12, 0.), (-.15, .12, 0.))),
               sites=(NS(site_id='liquid_outlet', position_world_m=(.55, -.1, 1.)),),
               shapes=(NS(shape_path='machine', world_from_shape=Pose((.55, .03, .9)), dimensions_m=(.3, .2, .3)),))
    observation = NS(scene=scene, sample=NS(world_from_cup=Pose((.4, .25, .75))))
    data = dict(urdf_sha256=REFERENCE['urdf_sha256'], native_identity=dict(acceptance=dict(edge_margin_m=.002)))
    result = generate_targets(planner, observation, data)
    grasp, placement = result['grasp_candidates'][0], result['placement_candidates'][0]
    assert placement['preferred'] and placement['support_xy_m'] == (0.,.001)
    assert placement['nominal_support_xy_m']==(0.,0.) and placement['inward_bias_m']==.001
    assert data['native_identity']['acceptance']['edge_margin_m']==.002
    assert np.allclose(placement['world_from_cup']['position'],support.transform_point((0.,.001,0.)))
    assert result['placement_goal']['tolerance_m']==.01
    assert abs(result['placement_goal']['target_tolerance_m']-.009)<1e-12
    assert result['placement_goal']['tracking_reserve_m']==.001
    assert all(p['outlet_distance_m']<=.009+1e-12 for p in result['placement_candidates'])
    assert abs(placement['planning_edge_margin_m']-.009)<1e-12
    near_edge=deepcopy(observation)
    half=.1
    near_edge.scene.support_polygon_world_m=tuple(support.transform_point(point) for point in
        ((-half,-.12,0.),(half,-.12,0.),(half,.12,0.),(-half,.12,0.)))
    near_edge.scene.sites=(NS(site_id='liquid_outlet',position_world_m=support.transform_point((.06,0,.2))),)
    adjusted=generate_targets(planner,near_edge,data)['placement_candidates'][0]
    assert adjusted['preferred'] and abs(adjusted['support_xy_m'][0]-(half-.0347-.009))<1e-10
    assert adjusted['outlet_distance_m']<=.008
    assert abs(grasp['cup_from_tool']['position'][0]-REFERENCE['cup_from_tool_position_m'][0]-.006) < 1e-14
    assert abs(grasp['cup_from_tool']['position'][2]-REFERENCE['cup_from_tool_position_m'][2]) < 1e-14
    leaning=deepcopy(observation)
    leaning.sample.world_from_cup=Pose(observation.sample.world_from_cup.position,(math.cos(.01),math.sin(.01),0.,0.))
    leaning_targets=generate_targets(planner,leaning,data)
    assert leaning_targets['grasp_candidates']
    assert leaning_targets['placement_candidates'][0]['world_from_cup']['orientation_wxyz']==leaning.sample.world_from_cup.orientation_wxyz
    planner.GRASP_CENTER_OFFSET_M = .010
    before = generate_targets(planner, observation, data)
    assert before['placement_candidates'][0]['world_from_cup'] == placement['world_from_cup']
    assert before['grasp_candidates'][0]['cup_from_tool']['orientation_wxyz'] == grasp['cup_from_tool']['orientation_wxyz']
    planner.GRASP_CENTER_OFFSET_M = .016
    route = carry_targets(planner, observation, grasp, placement)
    assert route['conservative_cross_clearance_m'] >= .017-1e-12
    assert route['tray_exit']['clearance_m']>=route['solver_pose_reserve_m']-1e-12
    # A low tray must still determine withdrawal when it is below the cup.
    deeper=deepcopy(observation)
    deeper.scene.support_polygon_world_m=tuple((x,y-.08,z) for x,y,z in scene.support_polygon_world_m)
    deeper_route=carry_targets(planner,deeper,grasp,placement)
    assert deeper_route['lane_projection_m']>=route['lane_projection_m']+.079
    assert deeper_route['tray_exit']['clearance_m']>=deeper_route['solver_pose_reserve_m']-1e-12
    assert [row['name'] for row in route['stages']] == ['lift', 'raise', 'withdraw', 'cross', 'front_height', 'insert', 'lower']
    assert len({row['world_from_tool']['orientation_wxyz'] for row in route['stages']}) == 1
    assert route['stages'][-1]['world_from_cup'] == placement['world_from_cup']
    low_outlet=deepcopy(observation)
    low_outlet.scene.sites=(NS(site_id='liquid_outlet',position_world_m=(.55,-.1,.92)),)
    capped=carry_targets(planner,low_outlet,grasp,placement)
    assert capped['cross_cup_bottom_world_z_m']+.13<=.92-capped['solver_pose_reserve_m']+1e-12
    assert capped['cup_environment_allowance_m']==.003 and capped['conservative_cross_clearance_m']>=.017
    # Rigid composition and quaternion sign must not create a fake yaw change.
    for quaternion in ((1.-1e-15, 0., 0., 0.), (-1., 0., 0., 0.)):
        equivalent = deepcopy(observation)
        equivalent.sample.world_from_cup = Pose(observation.sample.world_from_cup.position, quaternion)
        carry_targets(planner, equivalent, grasp, placement)
    tilted = deepcopy(observation)
    tilted.sample.world_from_cup = Pose(observation.sample.world_from_cup.position, (.99995, 0., 0., .01))
    try:
        carry_targets(planner, tilted, grasp, placement)
    except ValueError:
        pass
    else:
        raise AssertionError('actual cup yaw change accepted')
    placed_cup=Pose(**placement['world_from_cup'])
    placed_tool=placed_cup.compose(Pose(**grasp['cup_from_tool']))
    retreat=retreat_target(planner,placed_cup,placed_tool)
    assert retreat['hand_cup_clearance_m']>=.017-1e-12 and not retreat['collision_proxy_used']
    assert Pose(**retreat['world_from_tool']).orientation_wxyz==placed_tool.orientation_wxyz
    diagonal=retreat_target(planner,placed_cup,placed_tool,lateral_world=(0.,1.,0.))
    assert diagonal['hand_cup_clearance_m']>=.017-1e-12
    assert diagonal['world_from_tool']['position'][1]>placed_tool.position[1]
    assert Pose(**diagonal['world_from_tool']).orientation_wxyz==placed_tool.orientation_wxyz
    invalid=Pose(placed_tool.position,(2**-.5,0.,2**-.5,0.))
    try:retreat_target(planner,placed_cup,invalid)
    except ValueError:pass
    else:raise AssertionError('vertical-only retreat accepted')
    # A rigid translation of the scene must translate every target by the same amount.
    translation = (.17, -.22, .09)
    transform = Pose(translation)
    moved = deepcopy(observation)
    moved.sample.world_from_cup = transform.compose(observation.sample.world_from_cup)
    moved.scene.world_from_support = transform.compose(support)
    moved.scene.support_polygon_world_m = tuple(transform.transform_point(p) for p in scene.support_polygon_world_m)
    moved.scene.sites = tuple(NS(site_id=s.site_id, position_world_m=transform.transform_point(s.position_world_m)) for s in scene.sites)
    moved.scene.shapes = tuple(NS(shape_path=s.shape_path, world_from_shape=transform.compose(s.world_from_shape), dimensions_m=s.dimensions_m) for s in scene.shapes)
    moved_targets = generate_targets(planner, moved, data)
    assert np.allclose(np.array(moved_targets['grasp_candidates'][0]['world_from_tool']['position'])-
                       grasp['world_from_tool']['position'], translation)
    assert np.allclose(np.array(moved_targets['placement_candidates'][0]['world_from_cup']['position'])-
                       placement['world_from_cup']['position'], translation)
    # An unsupported receiving goal cannot fall back to an arbitrary tray corner.
    narrow = deepcopy(observation)
    narrow.scene.support_polygon_world_m = tuple(support.transform_point(p) for p in
        ((-.04, -.04, 0.), (.04, -.04, 0.), (.04, .04, 0.), (-.04, .04, 0.)))
    rejected = generate_targets(planner, narrow, data)
    assert rejected['placement_candidates']==[]
    unreachable=deepcopy(observation)
    unreachable.scene.sites=(NS(site_id='liquid_outlet',position_world_m=support.transform_point((.3,0,.2))),)
    assert generate_targets(planner,unreachable,data)['placement_candidates']==[]
    try:
        generate_targets(planner, observation, dict(data, urdf_sha256='wrong'))
    except ValueError:
        pass
    else:
        raise AssertionError('wrong asset identity accepted')
    assert 'torch' not in sys.modules and 'scipy.optimize' not in sys.modules
    print('MotionGen task-space targets: relative-depth, fixed pitch/yaw, support, 17mm lane and identity checks passed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-code', type=Path, default=Path(__file__).resolve().parents[2]/'robot_sim-gripper-profile-20260921')
    main(parser.parse_args().native_code)
