"""Release changes existing native activation buffers, never their identities."""
from types import SimpleNamespace as NS
from cup_contact import CupContact,ALLOWANCE_M,NUMERICAL_EPS_M

class Buffer:
    def __init__(self,value):self.value=value
    def fill_(self,value):self.value=value
    def copy_(self,other):self.value=other.value

def main():
    claw,wrist=Buffer(-.003),Buffer(.005)
    hard_claw=Buffer(-.003)
    cost=NS(activation_distance=Buffer(.005),_cup_contact_children=[(None,NS(activation_distance=claw)),(None,NS(activation_distance=wrist))])
    constraint=NS(activation_distance=Buffer(0.),_cup_contact_children=[(None,NS(activation_distance=hard_claw)),(None,NS(activation_distance=Buffer(0.)))])
    rollout=NS(primitive_collision_cost=cost,primitive_collision_constraint=constraint)
    contact=CupContact.__new__(CupContact)
    contact.generator=NS(rollout_fn=rollout,get_all_rollout_instances=lambda:[rollout])
    contact.report={}
    for allowed,expected in ((True,-(ALLOWANCE_M+NUMERICAL_EPS_M)),(False,0.),(True,-(ALLOWANCE_M+NUMERICAL_EPS_M))):
        contact.allow_claw_contact(allowed)
        assert claw.value==(expected if allowed else .005) and wrist.value==.005
        assert hard_claw.value==expected, 'released-hand hard constraint must retain zero contact allowance'
        assert cost._cup_contact_children[0][1].activation_distance is claw
        assert contact.report['current_claw_allowance_m']==(.003 if allowed else 0.)
    try:contact.allow_claw_contact(1)
    except ValueError:pass
    else:raise AssertionError('non-boolean phase accepted')
    print('Native cup contact phase buffers and wrist boundary: PASS')

if __name__=='__main__':main()
