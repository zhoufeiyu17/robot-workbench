"""Pure scalar proof checks; native GPU boundary tests live with run evidence."""
from copy import deepcopy
import math
from cup_contact import fixed_contact_bound, fixed_wrist_clearance, permits_claw_contact,sweep_contact_bound


def main():
    assert permits_claw_contact('left_link7',[.003])
    assert not permits_claw_contact('left_link7',[.0031])
    assert not permits_claw_contact('left_link6',[.0001])
    assert not permits_claw_contact('left_link8',[])
    pose=dict(position=[0.,0.,0.],orientation_wxyz=[1.,0.,0.,0.])
    proof=dict(tool_from_held_cup=pose,terminal_jaw_position_m=.038,maximum_claw_penetration_m=.001,
               cup_radius_from_origin_m=.14,numerical_contact_reserve_m=1e-5)
    assert abs(fixed_contact_bound(proof,pose,[.038,.038])-.00101)<1e-12
    assert sweep_contact_bound(proof,pose,[.04,.04])>.003
    sweep={**proof,'jaw_sweep_source_samples':[dict(jaw_position_m=.04,tool_from_cup=pose,maximum_claw_penetration_m=.0001)]}
    assert abs(sweep_contact_bound(sweep,pose,[.04,.04])-.00011)<1e-12
    shifted=deepcopy(pose);shifted['position'][0]=.003
    assert sweep_contact_bound(sweep,shifted,[.04,.04])>.003
    moved=deepcopy(pose);moved['position'][0]=.001
    assert abs(fixed_contact_bound(proof,moved,[.0382,.038])-.00221)<1e-12
    moved['position'][0]=.002
    assert fixed_contact_bound(proof,moved,[.038,.038])>.003
    moved=deepcopy(pose);angle=math.radians(1.)
    moved['orientation_wxyz']=[math.cos(angle/2),0.,0.,math.sin(angle/2)]
    assert fixed_contact_bound(proof,moved,[.038,.038])>.003
    for key,value in [('maximum_claw_penetration_m',-.001),('cup_radius_from_origin_m',float('nan')),('numerical_contact_reserve_m',-1.)]:
        bad=deepcopy(proof);bad[key]=value
        try:fixed_contact_bound(bad,pose,[.038,.038])
        except ValueError:pass
        else:raise AssertionError('invalid prepared contact bound accepted')
    proof.update(wrist_source_separation_proven=True,minimum_wrist_clearance_m=.005)
    assert fixed_wrist_clearance(proof,pose,[.038,.038])>.0049
    moved=deepcopy(pose);moved['position'][0]=.005
    assert fixed_wrist_clearance(proof,moved,[.038,.038])<0
    for value in (0.,-1.,float('nan')):
        bad=deepcopy(proof);bad['minimum_wrist_clearance_m']=value
        try:fixed_wrist_clearance(bad,pose,[.038,.038])
        except ValueError:pass
        else:raise AssertionError('invalid wrist separation accepted')
    from motiongen_motion import MotionGenerator
    engine=MotionGenerator.__new__(MotionGenerator)
    engine.config={'kinematics':{'lock_joints':{'left_joint7':.03,'left_joint8':.03}}}
    try:engine.update_locked_joints({'left_joint7':.03,'left_joint8':.031})
    except ValueError as error:assert 'symmetric' in str(error)
    else:raise AssertionError('asymmetric linked-jaw model update accepted')
    print('fixed relative contact: translation, rotation, jaw and invalid-bound checks PASS')


if __name__=='__main__':main()
