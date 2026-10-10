"""Real native action spec: noisy reset feedback must not move the inactive arm."""
import argparse
from copy import deepcopy
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS


def main(native_code):
    root = native_code/'source/robot_sim/robot_sim'
    for suffix in ('', 'specs'):
        module = ModuleType('robot_sim'+('.'+suffix if suffix else ''))
        module.__path__ = [str(root/suffix)]
        sys.modules[module.__name__] = module
    from robot_sim.specs.action import ArmCommand, DualArmCommand, ac_one_dual_arm_zero_home_action_spec
    from integration import CUROBO_PLANNER_ID
    from isaac_lane import SimDistanceActionSpec, native_target, simulation_initial_command, distance_replay_previous
    from motiongen_plan import LEFT, RIGHT, initial_motion_state, encode_stage, target

    def rejects(call, text):
        try:
            call()
        except ValueError as error:
            assert text in str(error), error
        else:
            raise AssertionError('invalid input accepted: '+text)

    spec, opening = ac_one_dual_arm_zero_home_action_spec(), 82.992
    for environment, allowance in (('fake',.005),('physical',.01)):
        row=target([0.,-allowance,0.,0.,0.,0.],[0.]*6,opening,opening)
        original=deepcopy(row)
        encoded=encode_stage(dict(name='initial_hold',rows=[row]),spec,opening,{},environment=environment)
        assert row==original and encoded['rows'][0]['left_arm_joints'][1]==0.
        assert encoded['position_corrections'][0]['changes'][0]['original_rad']==-allowance
        decoded=native_target(encoded['native_actions'][0],spec,opening,distance=True)
        assert decoded==encoded['rows'][0] and encoded['encoding_roundoff_rad']<=1e-12
        row['left_arm_joints'][1]-=.000001
        rejects(lambda:encode_stage(dict(name='initial_hold',rows=[row]),spec,opening,{},environment=environment),'position-boundary')
    raw = list(spec.encode(DualArmCommand(ArmCommand((0.,)*6, True), ArmCommand((0.,)*6, True))))
    frozen = simulation_initial_command(raw, spec, opening, 7)
    feedback = deepcopy(frozen['target'])
    feedback['left_arm_joints'][1] = -.0002
    feedback['right_arm_joints'] = [q+.0002 for q in feedback['right_arm_joints']]
    feedback['left_gripper'] -= .008
    feedback['right_gripper'] -= .01
    measured = dict(zip((*LEFT, *RIGHT), feedback['left_arm_joints']+feedback['right_arm_joints']))
    measured.update(left_joint7=feedback['left_gripper']/2000, left_joint8=feedback['left_gripper']/2000,
                    right_joint17=feedback['right_gripper']/2000, right_joint18=feedback['right_gripper']/2000+.000001)
    planner, data = NS(home_state=measured), dict(simulation_initial_command=frozen)
    runtime = NS(loop=NS(previous=tuple(raw), physics_steps=13))
    plan = dict(planner_id=CUROBO_PLANNER_ID, simulation_initial_command=frozen, gripper_opening_mm=opening)
    previous = distance_replay_previous(runtime, spec, plan)
    adapter = SimDistanceActionSpec(spec, opening)
    old_first = list(spec.encode(DualArmCommand(ArmCommand((0.,)*6, True),
                                               ArmCommand(tuple(feedback['right_arm_joints']), True))))
    old_first[6] = old_first[13] = opening
    rejects(lambda: adapter.decode_single_arm_transition(previous, old_first, active_arm='left'), 'inactive right arm')
    fixed, evidence = initial_motion_state(planner, data, {}, spec, opening)
    stage = encode_stage(dict(name='initial_hold', rows=[target([fixed[n] for n in LEFT], [fixed[n] for n in RIGHT],
                         fixed['left_joint7']*2000, fixed['right_joint17']*2000)]), spec, opening, {})
    adapter.decode_single_arm_transition(previous, stage['native_actions'][0], active_arm='left')
    assert evidence['soft_limit_projection'] == [dict(joint='left_joint2', measured_rad=-.0002, projected_rad=0.,
                                                    lower_rad=0., upper_rad=spec.left.arm.soft_upper_limits_rad[1])]
    assert evidence['max_arm_adjustment_rad'] == .0002
    assert measured['left_joint2'] == -.0002 and measured['right_joint12'] == .0002
    assert fixed['right_joint12'] == 0. and fixed['right_joint17'] == fixed['right_joint18'] == opening/2000
    assert runtime.loop.previous == tuple(raw), 'encoding conversion mutated the actual command'
    assert previous[:6] == raw[:6] and previous[7:13] == raw[7:13]
    rejects(lambda: initial_motion_state(planner, {}, {}, spec, opening), 'requires simulation_initial_command')
    bad = deepcopy(data); bad['simulation_initial_command']['target']['right_arm_joints'][0] += .0002
    rejects(lambda: initial_motion_state(planner, bad, {}, spec, opening), 'differs from its native decode')
    bad = deepcopy(runtime); moved = list(raw); moved[7] += .0001; bad.loop.previous = tuple(moved)
    rejects(lambda: distance_replay_previous(bad, spec, plan), 'nominal reset commands differ')
    rejects(lambda: distance_replay_previous(runtime, spec, dict(plan, simulation_initial_command=None)), 'observed simulation reset')
    for value, accepted in ((-.005, True), (-.005001, False)):
        changed = NS(home_state=dict(measured, left_joint2=value))
        if accepted:
            assert initial_motion_state(changed, data, {}, spec, opening)[0]['left_joint2'] == 0.
        else:
            rejects(lambda: initial_motion_state(changed, data, {}, spec, opening), 'soft-limit allowance')
    changed = NS(home_state=dict(measured, right_joint12=.005001))
    rejects(lambda: initial_motion_state(changed, data, {}, spec, opening), 'arm difference exceeds 0.005')
    fallback, fallback_evidence = initial_motion_state(planner, {}, {'synthetic_fixture': True}, spec, opening)
    fallback_command = fallback_evidence['simulation_initial_command']
    assert not fallback_command['observed_reset_command'] and 'offline' in fallback_command['evidence_source']
    rejects(lambda: distance_replay_previous(runtime, spec, dict(plan, simulation_initial_command=fallback_command)), 'observed simulation reset')
    # Closed native reset jaws stay closed during encoding conversion too.
    closed = raw.copy(); closed[6] = closed[13] = -1.
    closed_runtime = NS(loop=NS(previous=tuple(closed), physics_steps=0))
    closed_plan = dict(plan, simulation_initial_command=simulation_initial_command(closed, spec, opening, 0))
    converted = distance_replay_previous(closed_runtime, spec, closed_plan)
    assert converted[6] == converted[13] == 0.
    assert native_target(converted, spec, opening, distance=True) == closed_plan['simulation_initial_command']['target']
    print('Actual-spec reset command/feedback split, inactive hold, 0.005rad projection, offline provenance and replay checks passed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-code', type=Path, default=Path(__file__).resolve().parents[2]/'robot_sim-gripper-profile-20260921')
    main(parser.parse_args().native_code)
