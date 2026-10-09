"""CPU-only model/reported roundtrip, freezing, provenance and legacy guards."""
from copy import deepcopy
from joint_space import convert_state, freeze_reported, replay_state, state_near, validate
from integration import _digest
from test_frozen_executor import fixture
from test_harness import rejects


def a1_profile(profile):
    profile = deepcopy(profile)
    profile['gripper'] = dict(mapping='operator-pad-basis-1p5mm-per-side-v2', max_opening_mm=83.)
    # Synthetic state test with the explicit known model identity; no calibration claim.
    profile['model_binding'] = dict(
        a1_sha256='113b975458ae273ea583e6d1b3376dc9f0df7e71bf1e3bca825985a0da650d76',
        geometry_id='44c6160a0a33a78710c228c5af2b1f9e364d8dc808f3330500830b79a2dc3b16',
        jaw_mapping_sha256='1b56775075b9fc84fe6df97b9c6eaadc826d23da8675aae5783502b9f8fd71bf')
    return profile


def physical_fixture(plan, source):
    # Synthetic native actions only exercise provenance, not the Isaac decoder.
    plan.setdefault('native_actions', [[0.] * 14 for _ in plan['ticks']])
    freeze_reported(plan, source)


def main():
    from contract import demo_state
    origin=demo_state();near=deepcopy(origin)
    near['left_arm_joints'][0]=.0049
    assert state_near(near,origin,joint_rad=.005)
    near['left_arm_joints'][0]=.0051
    assert not state_near(near,origin,joint_rad=.005)
    source, plan = fixture()
    before = deepcopy(plan)
    freeze_reported(plan, source)
    assert plan == before
    validate(plan, source)
    source['schema_version'] = 'physical_collection_source.v1'
    profile = source['calibration_profile']
    profile['joints']['zero_offsets_rad'] = [(i - 6) * .01 for i in range(12)]
    state = deepcopy(plan['ticks'][0]['target'])
    converted = convert_state(state, profile, to_reported=True)
    recovered = convert_state(converted, profile, to_reported=False)
    for side in ('left', 'right'):
        key = side + '_arm_joints'
        assert all(abs(a-b) < 1e-14 for a,b in zip(state[key], recovered[key]))
    assert converted['left_gripper'] == state['left_gripper']
    assert converted['left_arm_joints'][0] == state['left_arm_joints'][0] + .06
    assert converted['right_arm_joints'][5] == state['right_arm_joints'][5] - .05
    rejects(lambda p: validate(p, source), plan)
    physical_fixture(plan, source)
    assert plan['ticks'][0]['target'] == converted == replay_state(state, source)
    assert plan['timeline_digest'] == _digest(plan['ticks'])
    validate(plan, source)
    rejects(lambda p: freeze_reported(p, source), plan)
    changed = deepcopy(plan); changed['native_actions'][0][0] = 1
    rejects(lambda p: validate(p, source), changed)
    changed = deepcopy(source); changed['calibration_profile']['joints']['zero_offsets_rad'][0] += .1
    rejects(lambda s: validate(plan, s), changed)
    changed = deepcopy(source); changed['calibration_profile']['joints']['signs'][0] = -1
    rejects(lambda s: replay_state(state, s), changed)
    changed = deepcopy(profile); changed['joints']['zero_offsets_rad'][0] = float('nan')
    rejects(lambda p: convert_state(state, p, to_reported=True), changed)
    calibrated = a1_profile(profile)
    native = deepcopy(state)
    native['left_gripper'] = native['right_gripper'] = 82.992
    reported = convert_state(native, calibrated, to_reported=True)
    assert abs(reported['left_gripper']-83.) < 1e-12
    restored = convert_state(reported, calibrated, to_reported=False)
    assert abs(restored['left_gripper']-82.992) < 1e-12
    physical_source = deepcopy(source); physical_source['calibration_profile'] = calibrated
    physical_plan = deepcopy(before)
    for tick in physical_plan['ticks']: tick['target'] = deepcopy(native)
    physical_fixture(physical_plan, physical_source)
    validate(physical_plan, physical_source)
    assert physical_plan['joint_space']['gripper_offset_mm'] == .008
    assert physical_plan['ticks'][0]['target'] == replay_state(native, physical_source)
    changed = deepcopy(calibrated); changed['model_binding']['geometry_id'] = 'unknown'
    rejects(lambda p: convert_state(native,p,to_reported=True),changed)
    changed = deepcopy(calibrated); changed['gripper']['mapping'] = 'unknown'
    rejects(lambda p: convert_state(native,p,to_reported=True),changed)
    changed = deepcopy(profile); changed['source']['kind'] = 'installed_physical_calibration'
    rejects(lambda p: convert_state(native,p,to_reported=True),changed)
    print('Joint space CPU checks: PASS; no hardware/Isaac execution')


if __name__ == '__main__':
    main()
