"""GPU narrow phase for closed, frozen meshes; unresolved geometry never passes."""
import math


def mesh_pair_clearance(first,second,*,clearance_m=.0001,device='cuda:0'):
    import numpy as np
    import torch
    from robot_sim.tasks.manager_based.coffee_make.a1_support_triangles import closed_component_representatives
    if not math.isfinite(clearance_m) or clearance_m<.0001:
        raise ValueError('at least 0.1mm static mesh clearance required')
    representatives=[]
    for triangles in (first,second):
        triangles=np.asarray(triangles,dtype=float)
        if triangles.ndim!=3 or triangles.shape[1:]!=(3,3) or not np.isfinite(triangles).all():
            raise ValueError('finite original source triangles required')
        ids=closed_component_representatives(triangles)
        if ids is None or not len(ids):raise ValueError('closed nondegenerate source components required')
        representatives.append(ids.tolist())
    a,b=(torch.as_tensor(t,dtype=torch.float64,device=device)for t in (first,second))
    if a.device.type!='cuda':raise ValueError('GPU narrow phase required')
    margin=clearance_m+1e-9
    bmin,bmax=b.amin(1),b.amax(1);candidates=0;screened=0
    # ponytail: quadratic AABB screening, bounded 128xN blocks; use a GPU BVH if fixed meshes grow.
    for begin in range(0,len(a),128):
        block=a[begin:begin+128];lo,hi=block.amin(1),block.amax(1)
        gap=torch.maximum(lo[:,None]-bmax[None],bmin[None]-hi[:,None]).amax(-1)
        ids=(gap<=margin).nonzero();screened+=gap.numel();candidates+=len(ids)
        for offset in range(0,len(ids),2048):
            pair=ids[offset:offset+2048];x,y=block[pair[:,0]],b[pair[:,1]]
            ex,ey=x.roll(-1,1)-x,y.roll(-1,1)-y
            nx,ny=torch.linalg.cross(ex[:,0],ex[:,1]),torch.linalg.cross(ey[:,0],ey[:,1])
            axes=torch.cat((nx[:,None],ny[:,None],
                torch.linalg.cross(ex[:,:,None,:],ey[:,None,:,:]).reshape(-1,9,3),
                torch.linalg.cross(nx[:,None],ex),torch.linalg.cross(ny[:,None],ey)),1)
            length=torch.linalg.vector_norm(axes,dim=-1)
            axes=axes/length.clamp_min(1e-30)[...,None]
            px,py=torch.einsum('nvj,naj->nav',x,axes),torch.einsum('nvj,naj->nav',y,axes)
            separation=torch.maximum(px.amin(-1)-py.amax(-1),py.amin(-1)-px.amax(-1))
            separation=separation.masked_fill(length<=1e-14,-float('inf')).amax(-1)
            if not bool(torch.isfinite(separation).all()):raise ValueError('nonfinite triangle separation')
            bad=(separation<=margin).nonzero().reshape(-1)
            if len(bad):
                row=pair[int(bad[0])]
                return dict(passed=False,reason='triangle_intersection_or_clearance_unresolved',
                    triangle_pair=[begin+int(row[0]),int(row[1])],required_clearance_m=clearance_m)

    def inside(point,triangles):
        direction=triangles.new_tensor([1.,.371390676,.529173]);direction/=direction.norm()
        v0,v1,v2=triangles.unbind(1);e1,e2=v1-v0,v2-v0
        h=torch.linalg.cross(direction.expand_as(e2),e2);det=(e1*h).sum(-1)
        valid=det.abs()>1e-12;inverse=torch.where(valid,det,torch.ones_like(det)).reciprocal()
        delta=point-v0;u=inverse*(delta*h).sum(-1);q=torch.linalg.cross(delta,e1)
        v=inverse*(q*direction).sum(-1);t=inverse*(e2*q).sum(-1)
        hits=t[valid&(u>=0)&(v>=0)&(u+v<=1)&(t>1e-9)]
        return len(torch.unique(torch.round(hits*1e9)))%2==1

    for points,other,ids in ((a,b,representatives[0]),(b,a,representatives[1])):
        if any(inside(points.reshape(-1,3)[i],other)for i in ids):
            return dict(passed=False,reason='closed_component_containment',required_clearance_m=clearance_m)
    return dict(passed=True,reason='all_triangle_pairs_separated_and_no_component_containment',
        required_clearance_m=clearance_m,numerical_padding_m=1e-9,
        triangle_pairs_screened=screened,narrow_triangle_pairs=candidates,
        component_representatives=[len(v)for v in representatives],device=str(a.device),dtype=str(a.dtype))


def self_check():
    import numpy as np
    import trimesh
    mesh=trimesh.creation.box(extents=[1.,1.,1.]);a=np.asarray(mesh.triangles)
    for shift,expected in [(1.1,True),(1.0002,True),(1.00005,False),(1.,False),(.8,False)]:
        assert mesh_pair_clearance(a,a+[shift,0,0])['passed']==expected,shift
    assert not mesh_pair_clearance(a,a*.2)['passed'],'contained component passed'
    assert mesh_pair_clearance(a,a*.2+[2.,0,0])['passed']
    rotated=mesh.copy();rotated.apply_transform(trimesh.transformations.rotation_matrix(np.pi/4,[0,0,1]))
    result=mesh_pair_clearance(a,np.asarray(rotated.triangles)+[1.,1.,0.])
    assert result['passed'] and result['narrow_triangle_pairs']>0,'separated overlapping AABBs were not resolved by SAT'
    try:mesh_pair_clearance(a[:-1],a+[2.,0,0])
    except ValueError:pass
    else:raise AssertionError('open source mesh accepted')
    print('9 GPU static mesh checks passed')


if __name__=='__main__':self_check()
