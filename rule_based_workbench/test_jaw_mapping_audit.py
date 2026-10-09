"""Offline arithmetic/provenance check; no hardware and no implicit acceptance."""
from copy import deepcopy
from jaw_mapping_audit import audit, interpolate


def main():
    mapping=dict(COMMAND={'left':[0,.0197,.0394,.0591,.0788],'right':[0,.019325,.03865,.057975,.0773]},
                 FEEDBACK={'left':[0,.020079090438953413,.03919807246630678,.05892929753873843,.07841933595704834],
                           'right':[0,.019533262101218883,.03842884221425934,.057821904729424806,.07709738049499709]},
                 GAP_MM={'left':[0,23,43,63,83],'right':[0,22,43,63,83]})
    before=deepcopy(mapping)
    result=audit(mapping,{'bare_inner_gap_mm':83,'usable_gap_with_pads_mm':81})
    assert mapping==before and result['legacy']['mapping']==before
    assert not result['physical_revalidation_passed'] and not result['controller_mapping_changed']
    assert result['status']=='requires-four-point-ruler-basis-confirmation'
    expected={'left':.077815,'right':.07633375}
    for side, row in result['conditional_pad_basis_candidate'].items():
        assert not row['activation_allowed'] and row['metal_knots_mm'][-1]==84
        assert row['control_metal_knots_mm'][-1]==83 and row['metal_knots_mm'][0]==0
        assert abs(row['command_sdk_at_83mm']-expected[side])<1e-12
        assert row['command_sdk_at_83mm']<mapping['COMMAND'][side][-1]
        assert row['feedback_sdk_at_83mm']<mapping['FEEDBACK'][side][-1]
        assert not row['inferred_numbers_are_original_measurements']
    try: interpolate(85,[0,84],[0,1])
    except ValueError: pass
    else: raise AssertionError('extrapolation accepted')
    print('jaw audit: legacy preserved, conditional +3 mm/83 mm interpolation and provisional status PASS')


if __name__=='__main__':main()
