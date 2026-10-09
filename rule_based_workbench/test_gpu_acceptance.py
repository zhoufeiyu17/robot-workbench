import math
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
    print('oriented component bounds: contact, rotated overlap, separation and padding PASS')


if __name__=='__main__':main()
