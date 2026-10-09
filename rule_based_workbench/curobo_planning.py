"""A1-cal cuRobo 0.7.8 FK/IK only: success is not a collision certificate.

Initialize with ``A1Kinematics(urdf, fk.movable, complete_model_state)``.
One instance owns one fixed other-arm/jaw state; construct another when it changes.
All poses use base_link coordinates and wxyz quaternions; joints use model space.
"""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

LEFT_JOINTS = tuple(f'left_joint{i}' for i in range(1, 7))


def robot_config(urdf_path, movable, positions):
    """Build only from this URDF and complete named state, with no asset defaults."""
    path = Path(urdf_path).resolve()
    raw = path.read_bytes()
    root = ET.fromstring(raw)
    joints = {}
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        name, kind = joint.get('name'), joint.get('type')
        if name in joints or kind not in ('revolute', 'prismatic'):
            raise ValueError(f'unsupported or duplicate A1 joint: {name} / {kind}')
        axis, limit = joint.find('axis'), joint.find('limit')
        xyz = tuple(map(float, axis.get('xyz', '1 0 0').split())) if axis is not None else (1., 0., 0.)
        if limit is None:
            raise ValueError(f'bounded joint limits required: {name}')
        bounds = tuple(float(limit.get(key)) for key in ('lower', 'upper', 'velocity'))
        if (len(xyz) != 3 or not all(math.isfinite(v) for v in (*xyz, *bounds))
                or sum(v*v for v in xyz) == 0 or bounds[0] >= bounds[1] or bounds[2] <= 0):
            raise ValueError(f'invalid axis or limits: {name}')
        joints[name] = dict(type=kind, axis=xyz, lower=bounds[0], upper=bounds[1], velocity=bounds[2])
    if set(joints) != set(movable) or set(positions) != set(joints):
        raise ValueError('URDF, fk.movable and model state must contain the same joints')
    if not set(LEFT_JOINTS) <= set(joints) or not all(math.isfinite(v) for v in positions.values()):
        raise ValueError('six left arm joints and complete finite model state required')
    links = [link.get('name') for link in root.findall('link')]
    if len(links) != len(set(links)) or not {'base_link', 'left_link6'} <= set(links):
        raise ValueError('unique links including base_link and left_link6 required')
    locked = {name: float(positions[name]) for name in sorted(joints) if name not in LEFT_JOINTS}
    # Like the existing numerical IK, clip only its seed; never rewrite measured state.
    retract = [max(joints[n]['lower'], min(joints[n]['upper'], positions[n])) for n in LEFT_JOINTS]
    config = dict(kinematics=dict(
        urdf_path=str(path), asset_root_path=str(path.parent), base_link='base_link', ee_link='left_link6',
        link_names=links, lock_joints=locked, load_meshes=False,
        cspace=dict(joint_names=list(LEFT_JOINTS), retract_config=retract,
                    null_space_weight=[1.]*6, cspace_distance_weight=[1.]*6, position_limit_clip=0.)))
    identity = dict(urdf_sha256=hashlib.sha256(raw).hexdigest(), joints=joints,
                    active_joints=list(LEFT_JOINTS), locked_positions=locked,
                    base_link='base_link', ee_link='left_link6', collision_checked=False)
    return config, identity


class A1Kinematics:
    """Reusable GPU solvers for one explicit frozen lock state; no Isaac imports."""

    def __init__(self, urdf_path, movable, positions, *, device='cuda:0', num_seeds=32,
                 position_threshold=4e-6, rotation_threshold=4e-6, use_cuda_graph=False):
        self.config, self.identity = robot_config(urdf_path, movable, positions)
        import torch
        import curobo
        from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
        from curobo.types.base import TensorDeviceType
        from curobo.types.robot import RobotConfig
        from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
        if curobo.__version__ != '0.7.8':
            raise RuntimeError('A1Kinematics requires cuRobo 0.7.8')
        if type(num_seeds) is not int or num_seeds < 1:
            raise ValueError('positive integer num_seeds required')
        if not all(math.isfinite(v) and v > 0 for v in (position_threshold, rotation_threshold)):
            raise ValueError('finite positive IK thresholds required')
        self.torch = torch
        self.tensor_args = TensorDeviceType(device=torch.device(device))
        self.robot = RobotConfig.from_dict(self.config, self.tensor_args)
        self.model = CudaRobotModel(self.robot.kinematics)
        if tuple(self.model.joint_names) != LEFT_JOINTS:
            raise ValueError('cuRobo active joint ordering differs from requested left arm')
        expected = self.tensor_args.to_device([[self.identity['joints'][n][key] for n in LEFT_JOINTS]
                                               for key in ('lower', 'upper')])
        actual = self.robot.kinematics.get_joint_limits().position
        if actual.shape != expected.shape or not torch.allclose(actual, expected, atol=1e-6, rtol=0):
            raise ValueError('cuRobo joint limits differ from the supplied URDF')
        self.lower, self.upper = expected[0], expected[1]
        self.ik = IKSolver(IKSolverConfig.load_from_robot_config(
            self.robot, tensor_args=self.tensor_args, num_seeds=num_seeds,
            position_threshold=position_threshold, rotation_threshold=rotation_threshold,
            self_collision_check=False, self_collision_opt=False, regularization=False,
            use_cuda_graph=use_cuda_graph))
        self.use_cuda_graph, self._ik_batch_size = use_cuda_graph, None

    def check_locked_state(self, positions):
        """Refuse reusing a solver after a jaw or the other arm has moved."""
        if set(positions) != set(self.identity['joints']) or not all(math.isfinite(v) for v in positions.values()):
            raise ValueError('complete finite model state required')
        if any(positions[name] != value for name, value in self.identity['locked_positions'].items()):
            raise ValueError('locked joint state changed; construct a solver for the new state')

    def _batch(self, values, width):
        tensor = self.tensor_args.to_device(values)
        if tensor.ndim != 2 or tensor.shape[1] != width or not len(tensor) or not self.torch.isfinite(tensor).all():
            raise ValueError(f'finite nonempty batch of shape (N, {width}) required')
        return tensor.contiguous()

    def batch_fk(self, left_positions):
        """Return CudaRobotModelState with every URDF link, for an N x 6 batch.

        Returned tensors belong to cuRobo reusable buffers; consume or clone before
        another FK call. CUDA is asynchronous; synchronize around wall benchmarks.
        """
        return self.model.get_state(self._batch(left_positions, 6))

    def batch_ik(self, positions, quaternions_wxyz, seeds=None):
        """Return IKResult (including success/errors), one solution per base-frame goal."""
        from curobo.types.math import Pose
        positions, quaternions = self._batch(positions, 3), self._batch(quaternions_wxyz, 4)
        norms = self.torch.linalg.vector_norm(quaternions, dim=-1, keepdim=True)
        if len(positions) != len(quaternions) or self.torch.any(self.torch.abs(norms-1.) > 1e-4):
            raise ValueError('matching goal batches and unit wxyz quaternions required')
        if self.use_cuda_graph and self._ik_batch_size not in (None, len(positions)):
            raise ValueError('CUDA graph IK requires the warmed batch size')
        seed_config = None
        if seeds is not None:
            seed = self._batch(seeds, 6)
            if len(seed) != len(positions):
                raise ValueError('one seed row per goal required')
            # 0.7.8 get_seed consumes (batch, seeds, dof), despite its solve_batch docstring.
            seed_config = seed.clamp(self.lower, self.upper).unsqueeze(1)
        self._ik_batch_size = len(positions)
        return self.ik.solve_batch(Pose(position=positions, quaternion=quaternions/norms),
                                   seed_config=seed_config, return_seeds=1)


def self_check():
    """Configuration check with a synthetic URDF, requiring neither torch nor CUDA."""
    import tempfile
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)/'robot.urdf'
        names = (*LEFT_JOINTS, 'right_joint11', 'left_joint7', 'left_joint8')
        body = ['<robot name="config-check"><link name="base_link"/>']
        for index, name in enumerate(names):
            child = f'left_link{index+1}' if index < 6 else f'other{index}'
            parent = 'base_link' if index in (0, 6) or index > 6 else f'left_link{index}'
            body.append(f'<link name="{child}"/><joint name="{name}" type="revolute">'
                        f'<parent link="{parent}"/><child link="{child}"/><axis xyz="0 0 -1"/>'
                        '<limit lower="-0.7" upper="1.2" velocity="2.3" effort="1"/></joint>')
        path.write_text(''.join(body)+'</robot>')
        positions = dict.fromkeys(names, .12)
        positions.update(right_joint11=.27, left_joint7=.032, left_joint8=.032)
        config, identity = robot_config(path, set(names), positions)
        assert config['kinematics']['lock_joints'] == {'right_joint11': .27, 'left_joint7': .032, 'left_joint8': .032}
        assert identity['joints']['left_joint1'] == dict(type='revolute', axis=(0., 0., -1.), lower=-.7, upper=1.2, velocity=2.3)
        assert config['kinematics']['urdf_path'] == str(path.resolve()) and not identity['collision_checked']
        instance = A1Kinematics.__new__(A1Kinematics)
        instance.identity = identity
        instance.check_locked_state(positions)
        try:
            instance.check_locked_state(dict(positions, left_joint7=.04))
        except ValueError:
            pass
        else:
            raise AssertionError('changed locked jaw must be rejected')
    print('A1 cuRobo configuration self-check passed; no GPU executed')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-check', action='store_true')
    if parser.parse_args().self_check:
        self_check()
    else:
        parser.error('use --self-check, or import A1Kinematics with frozen input paths')
