"""Offline calibrated geometry; reuse calibration FK/meshes without robot access.

Inputs are reported joints (rad) and physical metal jaw gaps (mm), never SDK
encoder gaps. No calibration is baked into the original URDF or STL files.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from arx_env.joint_head_isaac_fit import rigid
from arx_env.joint_head_workbench_geometry import Geometry


def load_snapshot(root):
    root = Path(root).resolve()
    hashes = json.loads((root / 'snapshot-sha256.json').read_text())
    for name, expected in hashes.items():
        path = (root / name).resolve()
        if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'snapshot digest mismatch: {name}')
    required = {'geometry-config.json', 'active/head.json', 'active/wrist.json', 'active/manifest.json'}
    if not required <= hashes.keys():
        raise ValueError('snapshot missing required pins')
    config = json.loads((root / 'geometry-config.json').read_text())
    for key in ('urdf', 'mesh_manifest', 'scene_seed', 'candidate', 'camera', 'chassis_mesh'):
        if config[key] not in hashes:
            raise ValueError(f'unpinned geometry input: {key}')
        config[key] = str(root / config[key])
    manifest = json.loads(Path(config['mesh_manifest']).read_text())
    for name in manifest['model']['meshes'].values():
        path = (Path(config['mesh_manifest']).parent / name).resolve()
        if not path.is_relative_to(root) or str(path.relative_to(root)) not in hashes:
            raise ValueError('unpinned link mesh')
    head, wrist, active = [json.loads((root / 'active' / name).read_text())
                           for name in ('head.json', 'wrist.json', 'manifest.json')]
    if active.get('robot_source') != 'physical' or active.get('simulation_only') is not False:
        raise ValueError('physical calibration required')
    if wrist['dependency_id'] != head['id']:
        raise ValueError('head/wrist dependency mismatch')
    for key, item in (('head', head), ('wrist', wrist)):
        if any(active[key][field] != item[field] for field in ('id', 'sha256', 'version')):
            raise ValueError('active calibration mismatch')
    return config, head, wrist, active


class CalibratedMesh:
    def __init__(self, root):
        config, head, wrist, self.active = load_snapshot(root)
        p = head['payload']['row']['parameters']
        self.offsets = np.asarray([p['joint_offsets_rad'][s] for s in ('left', 'right')], float).reshape(12)
        if not np.isfinite(self.offsets).all():
            raise ValueError('nonfinite joint offsets')
        self.head_camera = rigid(p['camera_T_world_optical'])
        self.wrist_cameras = {s: rigid(wrist['payload']['row'][s]['T_link6_camera']) for s in ('left', 'right')}
        # Keep every housing/support vertex at the existing nominal installation.
        # Fitted marker mounts are calibration fixtures, not production geometry.
        self.geometry = Geometry(config)
        self.geometry.camera = self.head_camera

    @staticmethod
    def _joints(value):
        q = np.asarray(value, float)
        if q.ndim < 1 or q.shape[-1] != 12 or not np.isfinite(q).all():
            raise ValueError('expected finite joints with final dimension 12, left then right')
        return q

    def reported_to_model(self, reported_q):
        """Feedback/command space -> model space; accepts a single pose or trajectory."""
        return self._joints(reported_q) + self.offsets

    def model_to_reported(self, model_q):
        """Model planning targets -> reported command space; never clip or retime."""
        return self._joints(model_q) - self.offsets

    def camera_poses(self, reported_q):
        q = self._joints(reported_q)
        if q.shape != (12,):
            raise ValueError('camera poses require one reported pose')
        frames = self.geometry.frames(self.reported_to_model(q))
        return {'head': self.head_camera.copy(),
                **{s: frames[s]['link6'] @ t for s, t in self.wrist_cameras.items()}}

    def calibration(self):
        """Portable inputs for a renderer/planner; no hardware authorization."""
        return dict(schema='calibrated_robot_geometry.v1', length_unit='m', angle_unit='rad',
                    joint_order=[f'{s}_J{i}' for s in ('left', 'right') for i in range(1, 7)],
                    reported_to_model='q_model = q_reported + zero_offsets_rad',
                    model_to_reported='q_reported = q_model - zero_offsets_rad',
                    zero_offsets_rad=self.offsets.tolist(),
                    T_world_head_optical=self.head_camera.tolist(),
                    T_link6_wrist_optical={s: t.tolist() for s, t in self.wrist_cameras.items()},
                    housing_policy='unchanged_from_geometry_snapshot', markers_rendered=False,
                    acceptance=self.active['acceptance'], head_id=self.active['head']['id'],
                    wrist_id=self.active['wrist']['id'], source_geometry_id=self.active['geometry_id'])

    def scene(self, reported_q, jaw_gap_mm):
        q = np.asarray(reported_q, float)
        if q.shape != (12,) or not np.isfinite(q).all():
            raise ValueError('expected 12 finite reported joints, left then right')
        geometry = self.geometry.with_gaps(*jaw_gap_mm)
        model_q = self.reported_to_model(q)
        frames = geometry.frames(model_q)
        meshes = []
        for side in ('left', 'right'):
            for name, vertices, _ in geometry.side_meshes[side]:
                transform = frames[side][name]
                # Every original triangle, not Geometry's display-only sampling.
                meshes.append((side + '_' + name, vertices @ transform[:3, :3].T + transform[:3, 3]))
        meshes.extend((name, vertices) for name, vertices, _ in geometry.static_meshes)
        cameras = {'head': self.head_camera, **{s: frames[s]['link6'] @ t for s, t in self.wrist_cameras.items()}}
        return meshes, dict(q_reported_rad=q.tolist(), q_model_rad=model_q.tolist(),
                            jaw_gap_mm=list(jaw_gap_mm), T_world_optical={k: t.tolist() for k, t in cameras.items()},
                            acceptance=self.active['acceptance'], physical_geometry_verified=False,
                            housing_policy='unchanged_from_geometry_snapshot', markers_rendered=False,
                            limitations=['wrist housing/support are proxies', 'no independent physical metrology'],
                            head_id=self.active['head']['id'], wrist_id=self.active['wrist']['id'],
                            source_geometry_id=self.active['geometry_id'])


def export_obj(meshes, path):
    with Path(path).open('x') as stream:
        stream.write('# metres; calibrated pose snapshot, not a collision certification\n')
        offset = 1
        for name, vertices in meshes:
            stream.write(f'o {name}\n')
            for vertex in vertices:
                stream.write('v %.9g %.9g %.9g\n' % tuple(vertex))
            for i in range(offset, offset + len(vertices), 3):
                stream.write(f'f {i} {i+1} {i+2}\n')
            offset += len(vertices)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--reported-q', type=json.loads, required=True)
    parser.add_argument('--jaw-gap-mm', type=json.loads, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    model = CalibratedMesh(args.snapshot)
    meshes, metadata = model.scene(args.reported_q, args.jaw_gap_mm)
    args.output.mkdir(exist_ok=False, parents=True)
    export_obj(meshes, args.output / 'robot.obj')
    metadata['mesh_parts'] = {name: len(vertices)//3 for name, vertices in meshes}
    (args.output / 'scene.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False))
    (args.output / 'calibration.json').write_text(json.dumps(model.calibration(), indent=2, ensure_ascii=False))
    print(json.dumps({'output': str(args.output), 'parts': len(meshes), 'triangles': sum(len(v)//3 for _, v in meshes)}))


if __name__ == '__main__':
    main()
