import math
from dataclasses import dataclass
from types import SimpleNamespace as NS
from unittest.mock import patch
import sys
import numpy as np
from gpu_acceptance import boxes_separated,same_support_source


def main():
    source=dict(sidecar_path='/base/policy.json',sidecar_sha256='abc',mesh_sha256={'arm':'def'},policy={'limit':.0012})
    clone={**source,'sidecar_path':'/clone/policy.json'}
    assert same_support_source(source,clone)
    assert not same_support_source(source,{**clone,'sidecar_sha256':'changed'})
    assert not same_support_source(source,{**clone,'policy':{'limit':.003}})
    a=np.array([1.,.1,.1]);b=a.copy();r=np.eye(3)
    assert not boxes_separated(r,np.zeros(3),a,b)
    assert not boxes_separated(r,np.array([2.,0.,0.]),a,b)
    assert boxes_separated(r,np.array([2.0001,0.,0.]),a,b)
    angle=math.pi/4;c,s=math.cos(angle),math.sin(angle)
    r=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
    assert not boxes_separated(r,np.array([0.,.5,0.]),a,b)
    assert boxes_separated(r,np.array([0.,1.,0.]),a,b)
    assert boxes_separated(r,np.array([0.,0.,.201]),a,b)
    # Closing must replace the prior request proof and pass the selected new
    # certificate to the GPU sweep, including an exactly centered cup.
    @dataclass
    class Pose:
        position:tuple=(0.,0.,0.)
        orientation_wxyz:tuple=(1.,0.,0.,0.)
        def compose(self,other):return Pose(tuple(a+b for a,b in zip(self.position,other.position)))
        def inverse(self):return Pose(tuple(-v for v in self.position))
        def rotate(self,values):return tuple(values)
    from gpu_acceptance import closing_geometry
    from test_grasp_contact_certificate import certificate_fixture
    proof=certificate_fixture('source','robot');engine=NS(grasp_contact_certificate={'stale':True})
    planner=NS(geo=NS(mount=Pose()));state={**dict.fromkeys([f'left_joint{i}' for i in range(1,7)],0.),'left_joint7':.041496}
    def build(*args,**kwargs):
        assert engine.grasp_contact_certificate is None
        assert kwargs['candidate_id']=='chosen' and kwargs['closed_jaw_m']==.032496
        return {'passed':True},proof
    def sweep(e,p,q,start,end,**kwargs):
        assert e.grasp_contact_certificate is proof and end==.038 and kwargs['centering'] is None
        return dict(passed=True)
    with patch.dict(sys.modules,{'robot_sim.asset_cfg.objects.coffee_machine':NS(RigidTransform=Pose)}),\
            patch('grasp_contact_certificate.build_contact_certificate',side_effect=build),\
            patch('gpu_acceptance.jaw_sweep',side_effect=sweep),\
            patch('gpu_acceptance.certificate_path',side_effect=AssertionError('old nominal proof reused')):
        result=closing_geometry(engine,planner,state,Pose(),data={},source_digest='source',candidate_id='chosen',closed_jaw_m=.032496)
    assert result['contact_certificate'] is proof
    assert result['gpu_sweep']['contact_certificate_digest']==proof['certificate_digest']
    print('oriented component bounds: contact, rotated overlap, separation and padding PASS')


if __name__=='__main__':main()
