"""GPU sphere/box broadphase for exported Playground worlds, with object identity.

The caller must supply enclosing spheres to use a False result to prune a pair.
Surface samples are not enclosing volumes. True only requests the existing exact
mesh/contact check; it never establishes forbidden contact or its penetration.
Each instance is used by one worker at a time because query buffers are reused.
"""
from copy import deepcopy
import json
import math


class CuroboWorldBroadphase:
    """One box per cuRobo environment; output order is shapes, table, obstacles."""

    ROUNDING_PADDING_M = 1e-5

    def __init__(self, exported_data, device="cuda:0"):
        shapes = exported_data["observation"]["scene"]["shapes"]
        self.objects = tuple(deepcopy([*shapes, exported_data["table"], *exported_data["obstacles"]]))
        self.object_ids = tuple(item["shape_path"] for item in self.objects)
        if not self.object_ids or len(set(self.object_ids)) != len(self.object_ids):
            raise ValueError("world requires nonempty, unique shape_path identities")
        for item in self.objects:
            if item["kind"] != "box":
                raise ValueError(f"unsupported world kind {item['kind']!r}: {item['shape_path']}")
            pose = item["world_from_shape"]
            dims, xyz, quat = item["dimensions_m"], pose["position"], pose["orientation_wxyz"]
            if len(dims) != 3 or len(xyz) != 3 or len(quat) != 4:
                raise ValueError(f"invalid box/pose dimensions: {item['shape_path']}")
            if not all(math.isfinite(v) for v in [*dims, *xyz, *quat]) or min(dims) <= 0:
                raise ValueError(f"box dimensions/pose must be finite, dimensions positive: {item['shape_path']}")
            if not math.isclose(sum(v*v for v in quat), 1.0, abs_tol=1e-6):
                raise ValueError(f"box quaternion is not normalized: {item['shape_path']}")

        import curobo
        if curobo.__version__ != '0.7.8':
            raise ValueError('CuroboWorldBroadphase requires actual cuRobo version 0.7.8')
        import torch
        from curobo.geom.sdf.world import CollisionQueryBuffer, WorldCollisionConfig, WorldPrimitiveCollision
        from curobo.geom.types import Cuboid, WorldConfig
        from curobo.types.base import TensorDeviceType

        self.torch = torch
        self.device = torch.device(device)
        if self.device.type != "cuda":
            raise ValueError("cuRobo broadphase requires a CUDA device")
        self.tensor_args = TensorDeviceType(device=self.device, dtype=torch.float32)
        worlds = [WorldConfig(cuboid=[Cuboid(
            name=item["shape_path"], dims=list(item["dimensions_m"]),
            pose=[*item["world_from_shape"]["position"], *item["world_from_shape"]["orientation_wxyz"]],
        )]) for item in self.objects]
        self.world = WorldPrimitiveCollision(WorldCollisionConfig(
            tensor_args=self.tensor_args, world_model=worlds, cache={"obb": 1},
        ))
        self._buffer_type = CollisionQueryBuffer
        self._shape = None
        self._weight = torch.ones(1, device=self.device)
        # Clearance is added to radii. Zero activation leaves costs zero outside,
        # positive inside; costs are not advertised as exact penetration depths.
        self._activation = torch.zeros(1, device=self.device)

    def query(self, spheres, clearance=0.0):
        """Return CUDA bool [batch,horizon,sphere,object] broadphase candidates.

        Input is world-frame [x,y,z,r] in metres, shape [B,H,N,4]. The nonnegative
        clearance and 10 micrometre float32 guard inflate each radius. Output is
        newly allocated, so a subsequent query cannot overwrite earlier masks.
        """
        if not math.isfinite(clearance) or clearance < 0:
            raise ValueError("clearance must be finite and nonnegative")
        torch = self.torch
        with torch.no_grad():
            query = torch.as_tensor(spheres, dtype=torch.float32, device=self.device)
            if query.ndim != 4 or query.shape[-1] != 4 or min(query.shape[:3]) < 1:
                raise ValueError("query spheres must have nonempty shape [B,H,N,4]")
            if not bool(torch.isfinite(query).all()) or bool((query[..., 3] < 0).any()):
                raise ValueError("query spheres must be finite with nonnegative radii")
            batch, horizon, count, _ = query.shape
            n_worlds = len(self.objects)
            if tuple(query.shape) != self._shape:
                self._shape = tuple(query.shape)
                shape = (n_worlds * batch, horizon, count, 4)
                self._spheres = torch.empty(shape, dtype=torch.float32, device=self.device)
                self._env_indices = torch.arange(n_worlds, dtype=torch.int32, device=self.device).repeat_interleave(batch)
                self._buffer = self._buffer_type.initialize_from_shape(
                    shape, self.tensor_args, self.world.collision_types,
                )
            self._spheres.view(n_worlds, batch, horizon, count, 4).copy_(query.unsqueeze(0))
            self._spheres[..., 3].add_(clearance + self.ROUNDING_PADDING_M)
            costs = self.world.get_sphere_distance(
                self._spheres, self._buffer, self._weight, self._activation,
                env_query_idx=self._env_indices,
            )
            return (costs > 0).view(n_worlds, batch, horizon, count).permute(1, 2, 3, 0).contiguous()

    def warmup(self, spheres, clearance=0.0):
        """Compile/load once and allocate the same shape used by subsequent calls."""
        result = self.query(spheres, clearance=clearance)
        self.torch.cuda.synchronize(self.device)
        return result


def self_check(device="cuda:0", exported_data=None):
    """GPU/FCL comparison and fixed-batch timing; no robot or simulator needed."""
    import time
    from statistics import median
    import fcl
    import numpy as np
    import torch

    def box(name, position, dimensions, angle=0.0):
        return dict(shape_path=name, body_path=name, role="shell", kind="box", axis=None,
                    dimensions_m=dimensions, world_from_shape=dict(position=position,
                    orientation_wxyz=[math.cos(angle / 2), 0, 0, math.sin(angle / 2)]))

    data = dict(observation=dict(scene=dict(shapes=[box("rotated", [0, 0, 0], [.2, .4, .6], .5)])),
                table=box("table", [0, 0, -.4], [1, 1, .1]),
                obstacles=[box("second", [1, 0, 0], [.2, .2, .2])])
    unsupported = deepcopy(data)
    unsupported["table"]["kind"] = "mesh"
    try:
        CuroboWorldBroadphase(unsupported, device=device)
    except ValueError as error:
        assert "unsupported world kind" in str(error)
    else:
        raise AssertionError("unsupported geometry was silently converted")
    checker = CuroboWorldBroadphase(data, device=device)

    def fcl_mask(spheres, engine, clearance):
        world = [fcl.CollisionObject(fcl.Box(*p["dimensions_m"]), fcl.Transform(
            np.asarray(p["world_from_shape"]["orientation_wxyz"], dtype=np.float64),
            np.asarray(p["world_from_shape"]["position"], dtype=np.float64),
        )) for p in engine.objects]
        result = []
        for sphere in np.asarray(spheres).reshape(-1, 4):
            obj = fcl.CollisionObject(fcl.Sphere(float(sphere[3]) + clearance),
                                      fcl.Transform(np.asarray(sphere[:3], dtype=np.float64)))
            result.append([bool(fcl.collide(obj, other, fcl.CollisionRequest(), fcl.CollisionResult()))
                           for other in world])
        return np.asarray(result).reshape(*spheres.shape[:-1], len(world))

    spheres = torch.tensor([[[[4., 0., 0., .03], [4., 0., 0., .03], [4., 0., 0., .03]]]], device=device)
    first = checker.warmup(spheres)
    assert not first.any().item()
    spheres[0, 0, :, :3] = torch.tensor([[0., 0., 0.], [0., 0., -.4], [1., 0., 0.]], device=device)
    middle = checker.query(spheres)
    assert np.array_equal(middle.cpu().numpy(), np.eye(3, dtype=bool)[None, None])
    assert np.array_equal(middle.cpu().numpy(), fcl_mask(spheres.cpu().numpy(), checker, 0.0))
    spheres[..., 0] = 4.
    assert not checker.query(spheres).any().item()
    assert not first.any().item() and middle.sum().item() == 3, "prior outputs alias reusable buffers"

    # Translated box: 20 mm radius, 10 mm gap; a 11 mm clearance must turn it on.
    boundary = torch.tensor([[[[1.13, 0., 0., .02]]]], device=device)
    assert not checker.query(boundary).any().item()
    assert checker.query(boundary, clearance=.011)[0, 0, 0, 2].item()
    tangent = torch.tensor([[[[1.12, 0., 0., .02]]]], device=device)
    assert checker.query(tangent)[0, 0, 0, 2].item(), "roundoff guard lost tangent pair"

    engine = CuroboWorldBroadphase(exported_data, device=device) if exported_data is not None else checker
    rng = np.random.default_rng(37)
    samples = []
    for item in engine.objects:
        center = np.asarray(item["world_from_shape"]["position"])
        extent = max(item["dimensions_m"])
        xyz = center + rng.uniform(-1., 1., (16, 3)) * extent
        samples.extend(np.column_stack((xyz, np.full(16, .02))).tolist())
    samples = np.asarray(samples, dtype=np.float32)[None, None]
    for clearance in (0.0, .005):
        actual = engine.query(samples, clearance=clearance).cpu().numpy()
        exact = fcl_mask(samples, engine, clearance)
        guarded = fcl_mask(samples, engine, clearance + engine.ROUNDING_PADDING_M)
        assert np.all(actual[exact]), "GPU broadphase lost a FCL collision"
        assert np.array_equal(actual, guarded), "GPU object masks differ from guarded FCL spheres"

    # Fixed single batch: 128k spheres, multiplied by the actual world count.
    count = 131072
    bench = torch.tensor(samples, device=device).repeat(1, 1, math.ceil(count / samples.shape[2]), 1)[:, :, :count].contiguous()
    engine.warmup(bench, clearance=.005)
    timings = []
    for _ in range(11):
        started = time.perf_counter()
        engine.query(bench, clearance=.005)
        torch.cuda.synchronize(engine.device)
        timings.append((time.perf_counter() - started) * 1000)
    return dict(passed=True, far_collision_far_passed=True, object_ids=engine.object_ids,
                fcl_sphere_queries=samples.shape[2], benchmark_spheres=count,
                benchmark_pairs=count * len(engine.objects), warm_median_ms=median(timings),
                warm_min_ms=min(timings), timing_scope="validation, upload/copy, GPU query and synchronization",
                rounding_padding_m=engine.ROUNDING_PADDING_M, exact_robot_contact_validated=False)


if __name__ == "__main__":
    import argparse
    from pathlib import Path
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-check", action="store_true", required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--input", type=Path, help="optional exported planning-input.json for real scene boxes")
    args = parser.parse_args()
    data = json.loads(args.input.read_text()) if args.input else None
    print(json.dumps(self_check(args.device, data), indent=2))
