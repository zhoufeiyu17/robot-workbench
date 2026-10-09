"""Exact pair partition for cuRobo 0.7.8 self costs, using only its fast CUDA kernel.

Every original enabled pair remains, including negative world radii and disabled
attachment slots. Each block computes native max penetration; block maxima are
reduced before the original weight/classification. Exact ties can choose another
valid max subgradient. Install on all rollouts before any CUDA graph capture.
"""
import hashlib
import json
import math


def partition_pairs(sphere_count, pairs):
    """256-index tiles bound local spheres; chunks bound checks per fast launch."""
    if type(sphere_count) is not int or sphere_count < 1:
        raise ValueError('positive sphere count required')
    ordered = sorted(tuple(pair) for pair in pairs)
    if len(set(ordered)) != len(ordered) or any(not 0 <= a < b < sphere_count for a, b in ordered):
        raise ValueError('unique canonical enabled sphere pairs required')
    tiles = {}
    for pair in ordered:
        tiles.setdefault((pair[0]//256, pair[1]//256), []).append(pair)
    groups = []
    for rows in tiles.values():
        for start in range(0, len(rows), 16384):
            subset = rows[start:start+16384]
            indices = sorted({value for pair in subset for value in pair})
            mapping = {value: i for i, value in enumerate(indices)}
            padded = 32*math.ceil(len(indices)/32)
            # kernel7 uint8 loop counters cannot safely execute ndpt=128/512.
            ndpt = next(value for value in (1, 2, 4, 8, 32) if math.ceil(len(subset)/value) <= 512)
            threads = 32*math.ceil(max(padded, math.ceil(len(subset)/ndpt), 32)/32)
            capacity = threads*ndpt
            local = [(mapping[a], mapping[b]) for a, b in subset]
            groups.append(dict(indices=indices+[indices[0]]*(padded-len(indices)),
                unique_spheres=len(indices), pairs=subset, local_pairs=local,
                checks_per_thread=ndpt, threads=threads, pair_capacity=capacity))
    recovered = sorted(pair for group in groups for pair in group['pairs'])
    if recovered != ordered:
        raise AssertionError('enabled-pair partition changed membership')
    encoded = json.dumps(ordered, separators=(',', ':')).encode()
    return groups, dict(original_spheres=sphere_count, enabled_pairs=len(ordered), blocks=len(groups),
        pair_sha256=hashlib.sha256(encoded).hexdigest(), complete_pair_coverage=True,
        max_local_spheres=max((len(g['indices']) for g in groups), default=0),
        max_enabled_pairs_per_block=max((len(g['pairs']) for g in groups), default=0),
        total_local_sphere_slots=sum(len(g['indices']) for g in groups),
        gradient_buffer_bytes_per_batch_horizon=16*sum(len(g['indices']) for g in groups),
        radius_filtering=False, kernel='cuRobo0.7.8.experimental native SelfCollisionCost',
        aggregation='max penetration, then original scalar weight and classification',
        tie_gradient='native fast-kernel winner and first block; valid max subgradient at exact ties')


def enabled_pairs(kinematics):
    """Use the complete matrix for slow configs; truncated fast locations are not a substitute."""
    n = kinematics.offset.numel()
    if kinematics.experimental_kernel:
        count = kinematics.thread_max
        if type(count) is not int or count < 0 or count % 2 or count > kinematics.thread_location.numel():
            raise ValueError('invalid native enabled-pair locations')
        rows = kinematics.thread_location[:count].detach().cpu().reshape(-1, 2).tolist()
        pairs = [tuple(sorted(row)) for row in rows if row != [-1, -1]]
    else:
        matrix = kinematics.collision_matrix.detach().cpu()
        if tuple(matrix.shape) != (n, n) or not bool((matrix == matrix.T).all()):
            raise ValueError('complete symmetric native collision matrix required')
        pairs = [(i, j) for i, row in enumerate(matrix.tolist()) for j in range(i+1, n) if row[j] == 1]
    return n, pairs


def split_self_collision_cost(cost):
    """Replace this instance's forward; preserve cost attributes and update_weight API."""
    if hasattr(cost, '_partitioned_self_collision'):
        return cost._partitioned_self_collision
    import torch
    import curobo
    from curobo.cuda_robot_model.types import SelfCollisionKinematicsConfig
    from curobo.rollout.cost.self_collision_cost import SelfCollisionCost, SelfCollisionCostConfig
    if curobo.__version__ != '0.7.8':
        raise ValueError('self-cost partition requires actual cuRobo 0.7.8')
    original = cost.self_collision_kin_config
    if (not isinstance(cost, SelfCollisionCost) or not original.offset.is_cuda
            or original.offset.dtype != torch.float32 or cost.weight.dtype != torch.float32):
        raise ValueError('native fast self kernels require CUDA float32 offsets, spheres and weights')
    n, pairs = enabled_pairs(original)
    membership = original.thread_location if original.experimental_kernel else original.collision_matrix
    membership_version = membership._version
    groups, report = partition_pairs(n, pairs)
    blocks = []
    for group in groups:
        indices = torch.tensor(group['indices'], dtype=torch.long, device=original.offset.device)
        locations = torch.full((2*group['pair_capacity'],), -1, dtype=torch.int16, device=original.offset.device)
        locations[:2*len(group['local_pairs'])] = torch.tensor(group['local_pairs'], dtype=torch.int16,
                                                             device=original.offset.device).reshape(-1)
        config = SelfCollisionKinematicsConfig(offset=original.offset.index_select(0, indices),
            thread_location=locations, thread_max=locations.numel(), checks_per_thread=group['checks_per_thread'],
            collision_matrix=torch.zeros(2, dtype=torch.uint8, device=original.offset.device), experimental_kernel=True)
        # The combination uses this native kernel's recorded gradient, never
        # a reimplementation of its collision/normalization math.
        child = SelfCollisionCost(SelfCollisionCostConfig(weight=1., tensor_args=cost.tensor_args,
                                    self_collision_kin_config=config, return_loss=True, classify=False))
        blocks.append((indices, child))

    class BlockedSelfCollision(torch.autograd.Function):
        @staticmethod
        def forward(ctx, spheres):
            needs_gradient = ctx.needs_input_grad[0]
            values = []
            for indices, child in blocks:
                child.self_collision_kin_config.offset.copy_(original.offset.index_select(0, indices))
                packed = spheres.index_select(2, indices).contiguous()
                # An outer autograd forward disables graph construction, but the
                # native kernel checks this flag to produce its gradient buffer.
                packed.requires_grad_(needs_gradient)
                values.append(child(packed))
            if values:
                penetration, winner = torch.stack(values).max(dim=0)
            else:
                penetration = torch.zeros(spheres.shape[:2], device=spheres.device, dtype=spheres.dtype)
            distance = penetration*cost.weight[0]
            if cost.classify:
                distance = torch.where(distance > 0., distance+1., distance)
            ctx.return_loss = cost.return_loss
            if needs_gradient:
                gradient = torch.zeros_like(spheres)
                for index, (indices, child) in enumerate(blocks):
                    # where is deliberate: 0 * NaN from a coincident, losing pair
                    # must not pollute the native winning-pair gradient.
                    local = torch.where((winner == index)[..., None, None], child._out_vec, 0.)
                    gradient.scatter_add_(2, indices.view(1, 1, -1, 1).expand_as(local), local)
                gradient.mul_(cost.weight[0])
                ctx.save_for_backward(gradient)
            return distance

        @staticmethod
        def backward(ctx, upstream):
            if not ctx.needs_input_grad[0]:
                return None
            gradient, = ctx.saved_tensors
            # Match native return_loss=False, which ignores external multipliers.
            return gradient*upstream[..., None, None] if ctx.return_loss else gradient

    def forward(spheres):
        current_membership = original.thread_location if original.experimental_kernel else original.collision_matrix
        if (cost.self_collision_kin_config is not original or current_membership is not membership
                or membership._version != membership_version or spheres.ndim != 4
                or tuple(spheres.shape[-2:]) != (n, 4) or spheres.dtype != torch.float32
                or spheres.device != original.offset.device):
            raise ValueError('self-cost partition requires its frozen topology and float32 [B,H,N,4] input')
        return BlockedSelfCollision.apply(spheres)

    cost.forward = forward
    cost._partitioned_self_collision = report
    cost._partitioned_blocks = torch.nn.ModuleList([child for _, child in blocks])
    # Prevent the legacy slow-output helper from wrapping this fast replacement.
    cost._zero_home_output_refresh = True
    return report


def install_motiongen_self_collision(generator):
    """Cover IK/graph/trajopt/finetune/joint-space costs AND constraints before warmup."""
    rows, seen = [], set()
    for rollout in [generator.rollout_fn, *generator.get_all_rollout_instances()]:
        if getattr(rollout, 'cuda_graph_instance', False):
            raise ValueError('self-cost partition must be installed before CUDA graph capture')
        for name in ('robot_self_collision_cost', 'robot_self_collision_constraint'):
            cost = getattr(rollout, name, None)
            if cost is None or id(cost) in seen:
                continue
            seen.add(id(cost))
            rows.append(dict(role=name, **split_self_collision_cost(cost)))
    if not rows:
        raise ValueError('MotionGen has no self costs or constraints to partition')
    return dict(installed_costs=len(rows), before_warmup=True, checks=rows,
                self_collision_enabled=True, pair_membership_and_radii_unchanged=True)
