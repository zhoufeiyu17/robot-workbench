import math
from dataclasses import dataclass
from types import SimpleNamespace as NS
from unittest.mock import patch
import sys
from copy import deepcopy
import numpy as np
from gpu_acceptance import boxes_separated,same_support_source


class Tensor(np.ndarray):
    """CPU tensor protocol for the actual SupportBounds.check control flow."""
    device='cpu-fixture'
    def __new__(cls,value,dtype=None):return np.asarray(value,dtype=dtype).view(cls)
    def cpu(self):return self
    def double(self):return Tensor(self,np.float64)
    def new_tensor(self,value):return Tensor(value)
    def new_full(self,shape,value):return Tensor(np.full(shape,value))
    def nonzero(self):return Tensor(np.argwhere(np.asarray(self)))
    def all(self,axis=None,out=None,keepdims=False,*,dim=None):return Tensor(np.asarray(self).all(axis=dim if dim is not None else axis,keepdims=keepdims))
    def any(self,axis=None,out=None,keepdims=False,*,dim=None):return Tensor(np.asarray(self).any(axis=dim if dim is not None else axis,keepdims=keepdims))
    def min(self,dim):return NS(values=Tensor(np.asarray(self).min(axis=dim)))
    def max(self,dim):return NS(values=Tensor(np.asarray(self).max(axis=dim)))


def support_fallback_check():
    from gpu_acceptance import SupportBounds
    left=tuple(f'left_joint{i}' for i in range(1,7));right=tuple(f'right_joint{i}' for i in range(11,17))
    class Source:
        def __init__(self,*,over=False,outside=False):
            self.identity={'policy':{'max_interference_m':.0012},'mesh_sha256':'frozen'}
            self.pairs=(('left_link2','left_link5'),('right_link12','right_link15'))
            self.calls=[];self.over=over;self.outside=outside
        def _pair(self,a,b,relative):
            self.calls.append((a,b,relative))
            return dict(pair=(a,b),ok=not self.over and not self.outside,contact=self.over,
                over_limit=self.over,outside_support=self.outside,limit_m=.0012,
                metric='separation_along_link2_positive_z')
    def checker(*,active12=False,clear=False,over=False,outside=False):
        value=SupportBounds.__new__(SupportBounds);value.support=Source(over=over,outside=outside)
        if not active12:value.support.pairs=value.support.pairs[:1]
        value.order=left+right if active12 else left
        value.indices={name:i for i,name in enumerate(value.order)}
        all_state=dict.fromkeys(left+right,0.019)
        all_state.update(left_joint7=.04,left_joint8=.04,right_joint17=.04,right_joint18=.04)
        value.fixed={key:v for key,v in all_state.items() if key not in value.indices}
        value._exact_identity=deepcopy(value.support.identity);value._exact_pair=value.support._pair
        source_states=[]
        def forward(state):
            assert set(state)==set(all_state)
            source_states.append(deepcopy(state));frames={}
            for side,start,a,b in (('left',0,'left_link2','left_link5'),('right',10,'right_link12','right_link15')):
                frames[a]=np.eye(4);frames[b]=np.eye(4);frames[b][0,3]=state[f'{side}_joint{start+3}']
            return frames
        value._exact_fk=NS(forward=forward)
        value.args=NS(to_device=lambda q:Tensor(q,np.float32))
        value.torch=NS(isfinite=lambda q:Tensor(np.isfinite(q)),
            einsum=lambda expr,*xs:Tensor(np.einsum(expr,*xs)),
            linalg=NS(inv=lambda matrix:Tensor(np.linalg.inv(np.asarray(matrix)))))
        matrices={}
        for a,b in value.support.pairs:
            matrices[a]=np.eye(4);matrices[b]=np.eye(4)
            if clear:matrices[b][0,3]=1.
        value.model=NS(get_state=lambda q:NS(link_poses={key:NS(get_matrix=lambda m=m:Tensor(np.repeat(m[None],len(q),axis=0)))
                                                        for key,m in matrices.items()}))
        value.boxes={key:Tensor([[0.,0.,0.,.1,.1,.1]]) for key in matrices}
        value.axes={key:Tensor([np.eye(3)]) for key in matrices};value.selected=dict.fromkeys(matrices,0)
        value.near_domains={}
        return value,source_states
    q=[.01,.02,.123456789123,.04,.05,.06]
    value,states=checker()
    # Runtime may replace check (or subsequently _pair); the captured original
    # pair method must remain the only narrow-phase entry, never recurse.
    def recurse(*args):raise AssertionError('replaced runtime checker was called recursively')
    value.support.check=recurse;value.support._pair=recurse
    result=value.check([q]);row=result['checks'][0]
    assert result['passed'] and row['broadphase_unresolved_samples']==1 and row['unresolved_samples']==0
    assert row['broadphase_unresolved_pairs']==[[0,0]] and row['unresolved_pairs']==[]
    assert row['source_exact_fallback']['sample_count']==1 and row['source_exact_fallback']['checks'][0]['result']['ok']
    assert states[0]['left_joint3']==q[2] and states[0]['right_joint13']==.019
    assert states[0]['left_joint3']!=float(np.float32(q[2])), 'exact FK used rounded GPU coordinates'
    assert value.support.calls[0][2][3]==q[2]
    value,_=checker(clear=True);assert value.check([q])['passed'] and not value.support.calls
    for options in ({'over':True},{'outside':True}):
        value,_=checker(**options);result=value.check([q]);row=result['checks'][0]
        assert not result['passed'] and row['unresolved_samples']==1
        assert row['source_exact_fallback']['checks'][0]['result']['ok'] is False
    value,_=checker();value.support.identity['mesh_sha256']='changed'
    result=value.check([q]);assert not result['passed'] and not value.support.calls
    assert 'identity' in result['checks'][0]['source_exact_fallback']['checks'][0]['error']
    value,states=checker(active12=True);q12=q+[.11,.12,.234567891234,.14,.15,.16]
    result=value.check([q12])
    assert result['passed'] and len(value.support.calls)==2
    assert all(state['right_joint13']==q12[8] for state in states)
    assert [call[:2] for call in value.support.calls]==[('left_link2','left_link5'),('right_link12','right_link15')]


def main():
    support_fallback_check()
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
