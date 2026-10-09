"""Offline old-reading reinterpretation and command/feedback boundary check."""
import jaw_mapping_v2 as mapping
from jaw_mapping_audit import audit


def reject(call):
    try: call()
    except ValueError: return
    raise AssertionError('invalid jaw input accepted')


def main():
    original=dict(COMMAND=mapping.COMMAND,FEEDBACK=mapping.FEEDBACK,
                  GAP_MM={'left':(0,23,43,63,83),'right':(0,22,43,63,83)})
    historical={'bare_inner_gap_mm':83,'usable_gap_with_pads_mm':81,
                'derived_if_symmetric':{'pad_intrusion_each_side_mm':1}}
    result=audit(original,historical,operator_pad_basis_confirmation='User confirmed old four-point readings were rubber-surface clearances.')
    assert result['status']=='operator-confirmed-reinterpretation'
    assert result['home_measurement']==historical and not result['physical_revalidation_passed']
    assert result['deployment_status']=='not-deployed'
    for side in ('left','right'):
        row=result['reinterpreted_mapping'][side]
        assert tuple(row['metal_knots_mm'])==mapping.GAP_MM[side]
        assert abs(mapping.command_sdk(side,83)-row['command_sdk_at_83mm']) < 1e-12
        assert mapping.command_sdk(side,83)<mapping.COMMAND[side][-1]
        assert mapping.command_sdk(side,0)==0
        assert mapping.command_sdk(side,mapping.GAP_MM[side][1]/2)==mapping.COMMAND[side][1]/2
        reject(lambda: mapping.command_sdk(side,84))
        assert mapping.hold_command_sdk(side,84)==mapping.command_sdk(side,83)
    raw=[0.]*14
    raw[6],raw[13]=mapping.FEEDBACK['left'][-1],mapping.FEEDBACK['right'][-1]
    assert mapping.feedback_gaps(raw)==[84.,84.]
    state=mapping.feedback_state(raw)
    assert state['within_measured_range']==[True,True] and state['control_range_mm']==[[0.,83.],[0.,83.]]
    for value in (float('nan'),float('inf'),-1.,83.01,True):
        reject(lambda: mapping.command_sdk('left',value))
    reject(lambda: mapping.feedback_gaps([0]*13))
    raw[6]=mapping.interpolate(86,mapping.GAP_MM['left'],mapping.FEEDBACK['left'])
    reject(lambda: mapping.feedback_gaps(raw))
    print('jaw mapping v2: confirmed +3 mm reinterpretation, 83 mm cap, 84 mm feedback, legacy history preserved PASS')


if __name__=='__main__':main()
