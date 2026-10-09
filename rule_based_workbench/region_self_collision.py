"""GPU region rejection followed by cuRobo's official fast self-collision query.

Fixed-size top-k buffers permit CUDA graph replay. Overflow produces an invalid
positive constraint, never truncated acceptance. No custom distance/gradient CUDA.
"""
import hashlib,json


def install_cost(cost, sphere_links):
    import torch
    from curobo.cuda_robot_model.types import SelfCollisionKinematicsConfig
    from curobo.rollout.cost.self_collision_cost import SelfCollisionCost,SelfCollisionCostConfig
    from motiongen_self_collision import enabled_pairs
    if hasattr(cost,'_region_report'):return cost._region_report
    original=cost.self_collision_kin_config
    n,pairs=enabled_pairs(original)
    if len(sphere_links)!=n or not pairs:raise ValueError('matching sphere topology and nonempty pairs required')
    if original.offset.dtype!=torch.float32 or not original.offset.is_cuda:raise ValueError('CUDA float32 required')
    device=original.offset.device
    counts={};regions=[]
    for link in sphere_links:
        index=counts.get(link,0);regions.append((link,index//16));counts[link]=index+1
    keys=sorted(set(regions));mapping={v:i for i,v in enumerate(keys)}
    groups=[mapping[v] for v in regions];g=len(keys)
    coarse_pairs=sorted({tuple(sorted((groups[a],groups[b]))) for a,b in pairs})
    lookup={p:i for i,p in enumerate(coarse_pairs)}
    parent_pair_ids=[lookup[tuple(sorted((groups[a],groups[b])))] for a,b in pairs]
    group=torch.tensor(groups,device=device,dtype=torch.long)
    pair=torch.tensor(pairs,device=device,dtype=torch.long)
    coarse_pair=torch.tensor(coarse_pairs,device=device,dtype=torch.long)
    pair_group=torch.tensor(parent_pair_ids,device=device,dtype=torch.long)
    pair_capacity=18432;sphere_capacity=576
    take_pairs=min(pair_capacity,len(pairs));take_spheres=min(sphere_capacity,n)
    locations=torch.full((2*pair_capacity,),-1,device=device,dtype=torch.int16)
    offsets=torch.zeros(sphere_capacity,device=device)
    cfg=SelfCollisionKinematicsConfig(offset=offsets,collision_matrix=torch.zeros(2,device=device,dtype=torch.uint8),
        thread_location=locations,thread_max=locations.numel(),checks_per_thread=32,experimental_kernel=True)
    child=SelfCollisionCost(SelfCollisionCostConfig(weight=1.,tensor_args=cost.tensor_args,
        self_collision_kin_config=cfg,return_loss=True,classify=False))
    child.weight=cost.weight
    stats=torch.zeros(3,device=device,dtype=torch.int32)
    counters=torch.zeros(6,device=device,dtype=torch.int64)

    class IgnoreOuterMultiplier(torch.autograd.Function):
        @staticmethod
        def forward(ctx,value):return value.clone()
        @staticmethod
        def backward(ctx,gradient):return torch.ones_like(gradient)

    def forward(spheres):
        if cost.self_collision_kin_config is not original or spheres.shape[-2:]!=(n,4):
            raise ValueError('region self query requires its frozen topology')
        if spheres.dtype!=torch.float32 or spheres.device!=device:raise ValueError('CUDA float32 spheres required')
        b,h=spheres.shape[:2]
        with torch.no_grad():
            centers=spheres.detach()[...,:3];effective=spheres.detach()[...,3]+original.offset
            active_children=effective>0
            group_index=group.view(1,1,n).expand(b,h,n)
            # Axis-aligned bounds contain every active fine sphere. They avoid
            # the empty corners introduced by one enclosing sphere for a plate.
            index=group_index[...,None].expand(b,h,n,3)
            lower=torch.full((b,h,g,3),1e6,device=device)
            upper=torch.full((b,h,g,3),-1e6,device=device)
            lower.scatter_reduce_(2,index,torch.where(active_children[...,None],centers-effective[...,None]-1e-5,1e6),reduce='amin',include_self=True)
            upper.scatter_reduce_(2,index,torch.where(active_children[...,None],centers+effective[...,None]+1e-5,-1e6),reduce='amax',include_self=True)
            a,z=coarse_pair[:,0],coarse_pair[:,1]
            near=torch.ones((b,h,len(coarse_pairs)),device=device,dtype=torch.bool)
            for axis in range(3):
                near &= (lower[:,:,a,axis]<=upper[:,:,z,axis])&(lower[:,:,z,axis]<=upper[:,:,a,axis])
            risk=near.any(dim=0).any(dim=0).index_select(0,pair_group)
            selected=torch.topk(risk.to(torch.int32),take_pairs,sorted=False).indices
            chosen=pair.index_select(0,selected);valid=risk.index_select(0,selected)
            used=torch.zeros(n,device=device,dtype=torch.int32)
            used.scatter_reduce_(0,pair.flatten(),risk.to(torch.int32).repeat_interleave(2),reduce='amax',include_self=True)
            ids=torch.topk(used,take_spheres,sorted=False).indices
            active=used.index_select(0,ids)>0
            local=torch.full((n,),-1,device=device,dtype=torch.long)
            local.scatter_(0,ids,torch.arange(take_spheres,device=device))
            remapped=local[chosen]
            valid=valid&(remapped>=0).all(dim=1)
            locations.fill_(-1)
            locations[:take_pairs*2].copy_(torch.where(valid[:,None],remapped,-1).flatten().to(torch.int16))
            offsets.zero_();offsets[:take_spheres].copy_(original.offset.index_select(0,ids))
            overflow=(risk.sum()>pair_capacity)|(used.sum()>sphere_capacity)
            stats.copy_(torch.stack((risk.sum(),used.sum(),overflow.to(torch.int64))).to(torch.int32))
            counters[:4].add_(torch.stack((torch.ones_like(overflow,dtype=torch.int64),overflow.to(torch.int64),
                torch.full_like(overflow,b*h,dtype=torch.int64),overflow.to(torch.int64)*(b*h))))
            counters[4:].copy_(torch.maximum(counters[4:],stats[:2].to(torch.int64)))
        packed=spheres.index_select(2,ids)
        radius=torch.where(active.view(1,1,-1),packed[...,3],-offsets[:take_spheres].abs()-1.)
        packed=torch.cat((packed[...,:3],radius[...,None]),dim=-1)
        if take_spheres<sphere_capacity:
            padding=torch.zeros((b,h,sphere_capacity-take_spheres,4),device=device);padding[...,3]=-1.
            packed=torch.cat((packed,padding),dim=2)
        # Do not even evaluate overlapping truncated geometry on overflow.
        # Negative effective radii disable all slots and avoid coincident NaNs.
        disabled=torch.zeros_like(packed);disabled[...,3]=-offsets.abs()-1.
        value=child(torch.where(overflow,disabled,packed))
        if cost.classify:value=torch.where(value>0,value+1.,value)
        value=torch.where(overflow,torch.full_like(value,1000.),value)
        if not cost.return_loss:value=IgnoreOuterMultiplier.apply(value)
        return value

    report=dict(backend='official_curobo_region_pair_table',sphere_count=n,pair_count=len(pairs),
        region_count=g,region_pair_count=len(coarse_pairs),sphere_capacity=sphere_capacity,pair_capacity=pair_capacity,
        pair_sha256=hashlib.sha256(json.dumps(pairs).encode()).hexdigest(),
        dynamic_radius_and_attachment=True,cuda_graph_shape_static=True,overflow='positive invalid constraint, never truncate acceptance')
    cost.forward=forward;cost._region_report=report;cost._region_stats=stats;cost._region_counters=counters;cost._region_child=child
    cost._zero_home_output_refresh=True
    return report


def install_motiongen(generator):
    links=generator.kinematics.kinematics_config.link_sphere_idx_map.detach().cpu().reshape(-1).tolist()
    reports=[];seen=set()
    graph_costs={id(getattr(rollout,name)) for rollout in generator.graph_planner.get_all_rollout_instances()
        for name in ('robot_self_collision_cost','robot_self_collision_constraint') if getattr(rollout,name,None) is not None}
    from motiongen_self_collision import split_self_collision_cost
    for rollout in [generator.rollout_fn,*generator.get_all_rollout_instances()]:
        if getattr(rollout,'cuda_graph_instance',False):raise ValueError('install region query before CUDA graph capture')
        for name in ('robot_self_collision_cost','robot_self_collision_constraint'):
            cost=getattr(rollout,name,None)
            if cost is not None and id(cost) not in seen:
                seen.add(id(cost))
                if id(cost) in graph_costs:
                    reports.append(dict(role=name,backend='existing_official_full_pair_blocks_for_graph',**split_self_collision_cost(cost)))
                else:reports.append(dict(role=name,**install_cost(cost,links)))
    if not reports:raise ValueError('no MotionGen self collision cost found')
    return dict(backend='official_curobo_region_pair_table',costs=reports,before_warmup=True)
