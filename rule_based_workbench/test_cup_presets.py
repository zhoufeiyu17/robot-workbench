from types import SimpleNamespace as NS
from copy import deepcopy
from unittest.mock import patch
from calibrate_cup_stiffness import seed_stiffness
from cup_presets import stiffness_seeds,contact_forces,gap_summary,diagnostic_summary,bind_preset_identity,configure_preset


def main():
    assert abs(seed_stiffness(5.)-1052.6315789473683)<1e-8
    try:seed_stiffness(-1.)
    except ValueError:pass
    else:raise AssertionError('negative target force accepted')
    identity={'base_physics':'unchanged'}
    preset=dict(preset_id='firm',stiffness_n_m=12000.,damping_n_s_m=10.,diagnostic_claw_compression=True)
    sha=bind_preset_identity(identity,preset)
    assert len(sha)==64 and identity['base_physics']=='unchanged' and identity['cup_contact_preset']==preset
    assert bind_preset_identity({'base_physics':'unchanged'},{**preset,'stiffness_n_m':4000.})!=sha
    # Reuse the original material configurator and publish its identity only
    # after the actual configure call succeeds; CPU double makes no physics claim.
    soft=dict(preset_id='soft',stiffness_n_m=329.24414834950727,damping_n_s_m=10.,diagnostic_claw_compression=True)
    runtime=NS(root='/current/plant',physical_identity={'base_physics':'unchanged'},physical_sha256='before')
    stage=object();before=deepcopy(runtime.physical_identity)
    with patch('cup_presets.configure',side_effect=ValueError('missing material')):
        try:configure_preset(runtime,stage,soft)
        except ValueError:pass
        else:raise AssertionError('failed material application accepted')
    assert runtime.physical_identity==before and runtime.physical_sha256=='before'
    with patch('cup_presets.configure',return_value={'actual_configuration_fixture':True}) as apply:
        configure_preset(runtime,stage,soft)
        apply.assert_called_once_with(stage,runtime.root,soft['stiffness_n_m'],soft['damping_n_s_m'])
    assert runtime.physical_identity==dict(before,cup_contact_preset=soft)
    from motiongen_plan import freeze
    from test_fake_device_pipeline import fake_inputs
    from physical_motiongen import target_vector,vector_target
    source,model,_=fake_inputs();target=model['ticks'][0]['target']
    data={'native_identity':dict(control=deepcopy(model['control']),physical_configuration=deepcopy(runtime.physical_identity),
                                 physical_configuration_sha256=runtime.physical_sha256)}
    grip=dict(open_travel_mm=82.992,closed_travel_mm=64.992,native_travel_offset_mm=.008,max_metal_gap_mm=83.)
    stages=[dict(name='initial_hold',rows=[target,target],native_actions=[target_vector(target)]*2)]
    with patch('isaac_lane.native_target',side_effect=lambda raw,*a,**kw:vector_target(raw)):
        frozen=freeze(stages,source,data,grip,None,run_id='cpu-preset')
        bad=deepcopy(data);bad['native_identity']['physical_configuration_sha256']='changed'
        try:freeze(stages,source,bad,grip,None,run_id='bad-physical-hash')
        except ValueError:pass
        else:raise AssertionError('invalid exported physical identity was silently rewritten')
    assert frozen['cup_contact_preset']==soft
    assert frozen['native_identity']['physical_configuration']==runtime.physical_identity
    assert frozen['native_identity']['physical_configuration_sha256']==runtime.physical_sha256
    from playground_sim import requested_cup_preset
    args=NS(environment='fake',cup_contact_preset=soft)
    assert requested_cup_preset(args,None)==requested_cup_preset(args,frozen)==soft
    for invalid,plan in ((NS(environment='physical',cup_contact_preset=soft),None),
                         (NS(environment='fake',cup_contact_preset=None),frozen),
                         (args,{**frozen,'cup_contact_preset':{**soft,'damping_n_s_m':11.}}),
                         (args,{**frozen,'native_identity':{**frozen['native_identity'],'physical_configuration_sha256':'changed'}})):
        try:requested_cup_preset(invalid,plan)
        except ValueError:pass
        else:raise AssertionError('mismatched cup preset/physics identity accepted')
    try:bind_preset_identity({}, {**preset,'diagnostic_claw_compression':False})
    except ValueError:pass
    else:raise AssertionError('unapproved formal preset accepted')
    assert [r['contact_stiffness_n_m'] for r in stiffness_seeds(6.)]==[12000.,4000.,2000.]
    rows=[dict(hold_time_s=i/240,jaw_positions_m=[.035,.035]) for i in range(1,145)]
    value=gap_summary(rows)
    assert abs(value['metal_gap_mean_mm']-70.008)<1e-9 and abs(value['nominal_rubber_gap_mean_mm']-67.008)<1e-9
    assert value['gap_stable']
    for row in rows:row['jaw_positions_m']=[.035+row['hold_time_s']*.001,.035]
    assert not gap_summary(rows)['gap_stable']
    root='/World/envs/env_0';finger=root+'/Robot/left_link7';cup=root+'/Cup'
    pad=NS(actors=(cup,finger),shapes=(cup+'/Collisions/MiddleBody',finger+'/RealFingerRubberPad'),impulse=(.002,0,0),separation_m=-.001)
    metal=NS(actors=(finger,cup),shapes=(finger+'/collisions/mesh',cup+'/Collisions/MiddleBody'),impulse=(.001,0,0),separation_m=-.002)
    f=contact_forces((pad,metal),root,.004)
    assert f['left_link7']==dict(pad_n=.5,metal_n=.25,max_pad_penetration_m=.001,max_metal_penetration_m=.002,max_other_metal_penetration_m=0.)
    assert f['left_link8']['pad_n']==0.
    summary=diagnostic_summary([dict(phase='closing',cup_tilt_rad=.1,forces=f)])
    assert summary['maximum_observed_pad_compression_m']==.001
    assert summary['maximum_observed_metal_compression_m']==.002 and summary['peak_metal_force_n']==.25
    assert abs(summary['maximum_tilt_by_phase_deg']['closing']-5.729577951308233)<1e-9
    print('preset seeds, fixed-time nominal/metal jaw gap, nonconvergence and metal/pad force separation PASS')


if __name__=='__main__':main()
