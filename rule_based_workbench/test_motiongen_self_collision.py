"""CPU partition fixtures; optional bounded DSW GPU parity against native slow cost.

GPU invocation: --gpu --robot-config /.../robot-config.json --output /.../new-dir.
The slow-reference parity fixture must have fewer than 1024 spheres because the
upstream CUDA entry has that assertion. The partition itself supports larger N.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import time

from motiongen_self_collision import enabled_pairs, partition_pairs, split_self_collision_cost


def cpu_check():
    for n, pairs in ((4, []), (676, [(i, j) for i in range(676) for j in range(i+1, 676)]),
                     (4096, [(i, j) for i in range(4096) for j in range(i+1, min(i+17, 4096))])):
        blocks, report = partition_pairs(n, pairs)
        assert report['complete_pair_coverage'] and report['enabled_pairs'] == len(pairs)
        assert sorted(pair for block in blocks for pair in block['pairs']) == pairs
        assert report['pair_sha256'] == hashlib.sha256(json.dumps(pairs, separators=(',', ':')).encode()).hexdigest()
        for block in blocks:
            indices = block['indices']
            assert 0 < len(indices) <= 512 and len(indices) % 32 == 0
            assert len(block['pairs']) <= 16384 and block['threads'] <= 512 and block['threads'] % 32 == 0
            assert block['checks_per_thread'] in (1, 2, 4, 8, 32)
            assert block['pair_capacity'] >= len(block['pairs'])
            assert [(indices[a], indices[b]) for a, b in block['local_pairs']] == block['pairs']
    for bad in ([(0, 1), (0, 1)], [(1, 0)], [(0, 4)]):
        try: partition_pairs(4, bad)
        except ValueError: pass
        else: raise AssertionError('invalid pair set accepted')
    assert 'torch' not in sys.modules and 'curobo' not in sys.modules
    print('all enabled pairs preserved, padded fast-kernel bounds: PASS (CPU only)')


def gpu_check(args):
    from benchmark_planning import write_new
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(passed=False, hardware_access=False, executable_plan=False, scope='native fast-block versus slow self-cost parity')
    started = time.perf_counter()
    try:
        if sys.platform == 'darwin':
            raise RuntimeError('run GPU parity only on the authorized DSW')
        import torch
        import curobo
        from curobo.types.base import TensorDeviceType
        from curobo.types.robot import RobotConfig
        from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
        from curobo.cuda_robot_model.types import SelfCollisionKinematicsConfig
        from curobo.rollout.cost.self_collision_cost import SelfCollisionCost, SelfCollisionCostConfig
        if curobo.__version__ != '0.7.8': raise ValueError('cuRobo 0.7.8 required')
        raw_config = json.loads(args.robot_config.read_text())
        tensor_args = TensorDeviceType(device=torch.device('cuda:0'))
        robot = RobotConfig.from_dict(deepcopy(raw_config), tensor_args)
        model = CudaRobotModel(robot.kinematics)
        native = model.get_self_collision_config()
        n, pairs = enabled_pairs(native)
        if n >= 1024:
            raise ValueError(f'upstream slow reference asserts nspheres<1024; use the actual 676-sphere fixture for parity, got {n}')
        groups, proof = partition_pairs(n, pairs)
        result['partition'] = proof
        write_new(args.output/'partition.json', proof)
        dense = torch.zeros((n, n), device=tensor_args.device, dtype=torch.uint8)
        pair_tensor = torch.tensor(pairs, device=tensor_args.device, dtype=torch.long)
        dense[pair_tensor[:, 0], pair_tensor[:, 1]] = 1
        dense[pair_tensor[:, 1], pair_tensor[:, 0]] = 1
        reference_config = SelfCollisionKinematicsConfig(offset=native.offset.clone(), collision_matrix=dense,
            thread_location=torch.full((2,), -1, dtype=torch.int16, device=tensor_args.device),
            thread_max=0, checks_per_thread=32, experimental_kernel=False)
        slow = SelfCollisionCost(SelfCollisionCostConfig(weight=1.7, tensor_args=tensor_args,
                                                          self_collision_kin_config=reference_config))
        fast = SelfCollisionCost(SelfCollisionCostConfig(weight=1.7, tensor_args=tensor_args,
                                                          self_collision_kin_config=deepcopy(reference_config)))
        split_self_collision_cost(fast)
        radii = model.kinematics_config.link_spheres[:, 3].detach().clone()
        effective = (radii+native.offset).cpu().tolist()
        world_radii = radii.cpu().tolist()
        base_pair = next(((a, b) for a, b in pairs if world_radii[a] < -1. and effective[a] > 0 and world_radii[b] > 0), None)
        if base_pair is None:
            base_pair = next(((b, a) for a, b in pairs if world_radii[b] < -1. and effective[b] > 0 and world_radii[a] > 0), None)
        if base_pair is None: raise ValueError('actual negative-world/positive-self base pair absent')
        attached_ids = set(model.kinematics_config.get_sphere_index_from_link_name('attached_object').cpu().tolist())
        attachment_pair = next(((a, b) if a in attached_ids else (b, a) for a, b in pairs
                                if (a in attached_ids and world_radii[b] > 0) or (b in attached_ids and world_radii[a] > 0)), None)
        if attachment_pair is None: raise ValueError('enabled pair for initially disabled attached_object absent')

        far = torch.zeros((4, 3, n, 4), device=tensor_args.device)
        far[..., 0] = 100.+10.*(1.+max(effective))*torch.arange(n, device=tensor_args.device)
        far[..., 3] = radii
        def overlap(pair, attach=False):
            value = far.clone(); a, b = pair
            if attach: value[..., a, 3] = .012
            first_radius = .012+float(native.offset[a]) if attach else effective[a]
            distance = .25*(first_radius+effective[b])
            if distance <= 0: raise ValueError('positive, noncoincident overlap fixture required')
            value[..., a, :3] = 0.
            value[..., b, :3] = 0.
            value[..., b, 0] = distance
            return value
        cases = [('far', far), ('base_overlap', overlap(base_pair)), ('far_after', far),
                 ('attachment_enabled', overlap(attachment_pair, True)), ('far_after_attachment', far)]
        group_for = {pair: index for index, group in enumerate(groups) for pair in group['pairs']}
        base_group = group_for[tuple(sorted(base_pair))]
        secondary = next(((a, b) for a, b in reversed(pairs)
                          if not set((a, b)) & set(base_pair) and world_radii[a] > 0 and world_radii[b] > 0
                          and group_for[(a, b)] != base_group), None)
        if secondary is None: raise ValueError('full-model fixture needs a second enabled pair in another block')
        two = overlap(base_pair)
        a, b = secondary
        smaller = min(.25*(effective[a]+effective[b]), .25*sum(effective[i] for i in base_pair))
        two[..., a, :3] = tensor_args.to_device([10., 10., 0.])
        two[..., b, :3] = tensor_args.to_device([10.+effective[a]+effective[b]-smaller, 10., 0.])
        cases.append(('two_blocks_unique_global_max', two))
        primary_penetration = .75*sum(effective[i] for i in base_pair)
        coincident = next(((a, b) for a, b in pairs if not set((a, b)) & set(base_pair)
                           and world_radii[a] > 0 and world_radii[b] > 0
                           and effective[a]+effective[b] < primary_penetration*.5
                           and group_for[(a, b)] != base_group), None)
        if coincident is not None:
            probe = overlap(base_pair)
            probe[..., coincident[0], :3] = tensor_args.to_device([-10., 10., 0.])
            probe[..., coincident[1], :3] = tensor_args.to_device([-10., 10., 0.])
            cases.append(('nonwinning_coincident_pair', probe))
        result['nonwinning_coincident_pair_fixture'] = list(coincident) if coincident is not None else None

        def slow_value(spheres):
            slow.update_batch_size(spheres)
            slow._out_distance.zero_(); slow._out_vec.zero_(); slow._sparse_sphere_idx.zero_()
            return slow(spheres)

        def value_grad(cost, spheres, reference=False):
            query = spheres.detach().clone().requires_grad_(True)
            value = slow_value(query) if reference else cost(query)
            multiplier = torch.arange(1, value.numel()+1, device=value.device).reshape(value.shape)/3.
            grad = torch.autograd.grad((value*multiplier).sum(), query)[0]
            torch.cuda.synchronize()
            return value.detach().clone(), grad.detach().clone()

        parity = []
        for return_loss, classify, weight in ((False, False, 1.7), (True, False, 1.7), (False, True, 1.7), (True, False, -.4)):
            slow.return_loss = fast.return_loss = return_loss
            slow.classify = fast.classify = classify
            slow.update_weight(weight); fast.update_weight(weight)
            for name, spheres in cases:
                expected, expected_grad = value_grad(slow, spheres, True)
                actual, actual_grad = value_grad(fast, spheres)
                dv = float((expected-actual).abs().max()); dg = float((expected_grad-actual_grad).abs().max())
                if not torch.isfinite(actual).all() or not torch.isfinite(actual_grad).all() or dv > 2e-6 or dg > 2e-5:
                    raise AssertionError(f'{name}: value error {dv}, gradient error {dg}, return_loss={return_loss}, classify={classify}, weight={weight}')
                if name.startswith('far') and (actual.count_nonzero().item() or actual_grad.count_nonzero().item()):
                    raise AssertionError('far query retained collision cost/gradient')
                parity.append(dict(case=name, return_loss=return_loss, classify=classify, weight=weight,
                                   value_error=dv, gradient_error=dg, max_abs_cost=float(actual.abs().max())))
        result['unique_maximum_value_gradient_parity'] = parity
        result['base_pair'] = list(base_pair); result['initially_disabled_attachment_pair'] = list(attachment_pair)
        result['nonwinning_pair_in_second_block'] = list(secondary)

        # Capture forward/backward once, then change sphere/radius inputs.
        slow.return_loss = fast.return_loss = True
        slow.classify = fast.classify = False
        slow.update_weight(1.7); fast.update_weight(1.7)
        static = far[:1, :1].clone().requires_grad_(True)
        stream = torch.cuda.Stream(); stream.wait_stream(torch.cuda.current_stream())
        with torch.cuda.stream(stream):
            for _ in range(3):
                value = fast(static); torch.autograd.grad(value.sum(), static)
        torch.cuda.current_stream().wait_stream(stream)
        graph = torch.cuda.CUDAGraph()
        with torch.cuda.graph(graph):
            graph_value = fast(static)
            graph_grad = torch.autograd.grad((graph_value/3.).sum(), static)[0]
        graph_rows = []
        for name, spheres in cases:
            with torch.no_grad(): static.copy_(spheres[:1, :1])
            graph.replay(); torch.cuda.synchronize()
            expected, expected_grad = value_grad(slow, spheres[:1, :1], True)
            dv = float((graph_value-expected).abs().max()); dg = float((graph_grad-expected_grad).abs().max())
            if dv > 2e-6 or dg > 2e-5: raise AssertionError(f'captured {name} differs: {dv}, {dg}')
            graph_rows.append(dict(case=name, value_error=dv, gradient_error=dg))

        # Use actual library FK before/after its standard locked-jaw update; no synthetic joint space.
        q = tensor_args.to_device([raw_config['kinematics']['cspace']['retract_config']])
        fk_inputs = [('fk_before_locked_update', model.get_state(q).get_link_spheres().detach().clone().unsqueeze(1))]
        updated = deepcopy(raw_config)
        limits = {}
        import xml.etree.ElementTree as ET
        for joint in ET.parse(raw_config['kinematics']['urdf_path']).getroot().findall('joint'):
            limit = joint.find('limit')
            if limit is not None: limits[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))
        for names in (('left_joint7', 'left_joint8'), ('right_joint17', 'right_joint18')):
            low, high = max(limits[name][0] for name in names), min(limits[name][1] for name in names)
            previous = updated['kinematics']['lock_joints'][names[0]]
            target = previous-.002 if previous-.002 >= low else previous+.002
            if not low <= target <= high: raise ValueError('no legal locked-jaw fixture')
            for name in names: updated['kinematics']['lock_joints'][name] = target
        next_robot = RobotConfig.from_dict(deepcopy(updated), tensor_args)
        model.update_kinematics_config(next_robot.kinematics.kinematics_config)
        fk_inputs.append(('fk_after_locked_update', model.get_state(q).get_link_spheres().detach().clone().unsqueeze(1)))
        for name, spheres in fk_inputs:
            with torch.no_grad(): static.copy_(spheres)
            graph.replay(); torch.cuda.synchronize()
            expected, _ = value_grad(slow, spheres, True)
            error = float((graph_value-expected).abs().max())
            if error > 2e-6: raise AssertionError(f'{name}: captured input/FK update went stale: {error}')
            graph_rows.append(dict(case=name, value_error=error, gradient_scope='not asserted for potentially tied full-pose maxima'))
        result['captured_graph_parity'] = graph_rows

        # Representative optimizer batch/horizon, timing only the replacement.
        bench = overlap(base_pair)[:1, :1].repeat(32, 32, 1, 1).requires_grad_(True)
        for _ in range(2):
            torch.autograd.grad(fast(bench).sum(), bench)
        timings = []
        for _ in range(5):
            torch.cuda.synchronize(); tick = time.perf_counter()
            torch.autograd.grad(fast(bench).sum(), bench)
            torch.cuda.synchronize(); timings.append((time.perf_counter()-tick)*1000)
        result.update(passed=True, benchmark_shape=list(bench.shape), fast_forward_backward_ms=timings,
                      fast_median_ms=sorted(timings)[len(timings)//2], no_pairs_omitted=True,
                      gradient_scope='strict native parity for unique overlap maxima; exact ties may choose another valid subgradient')
    except Exception as error:
        import traceback
        result.update(error=f'{type(error).__name__}: {error}', traceback=traceback.format_exc())
    result['elapsed_seconds'] = time.perf_counter()-started
    write_new(args.output/'result.json', result)
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gpu', action='store_true')
    parser.add_argument('--robot-config', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.gpu:
        if args.robot_config is None or args.output is None: parser.error('--gpu requires --robot-config and --output')
        raise SystemExit(gpu_check(args))
    cpu_check()
