from types import SimpleNamespace as NS
from calibrate_cup_stiffness import seed_stiffness
from cup_presets import stiffness_seeds,contact_forces,gap_summary,diagnostic_summary,bind_preset_identity


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
