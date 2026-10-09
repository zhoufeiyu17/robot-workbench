import sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from physical_profile import build_profile, camera_offsets, configure_cameras, CAMERAS
from test_harness import PROFILE


def check(snapshot):
    registration = np.eye(4); registration[:3, 3] = [.1, -.2, .7]
    template = deepcopy(PROFILE)
    p = build_profile(snapshot, template, registration)
    assert template == PROFILE
    assert p['source']['acceptance']['status'] == 'user_accepted_provisional'
    cal = p['physical_calibration']
    np.testing.assert_allclose(p['frames']['environment_from_camera']['position'],
                               (registration @ np.array(cal['T_world_head_optical']))[:3, 3])
    assert len(set(v['serial'] for v in cal['intrinsics'].values())) == 3
    parents = {s: {'parent': s + '_parent', 'T_parent_reference': np.eye(4).tolist()} for s in CAMERAS}
    parents['head']['T_parent_reference'] = registration.tolist()
    cameras = [{'device_calibration_ref': device, 'parent': s + '_parent'} for s, device in CAMERAS.items()]
    cfg = NS(scene=NS(), d405_rig={'cameras': cameras, 'device_calibrations': {}})
    for s, device in CAMERAS.items():
        setattr(cfg.scene, 'd405_' + device, NS(prim_path=f'{{ENV_REGEX_NS}}/Robot/{s}_parent/D405RGB_{device}',
                                             offset=NS(pos=None, rot=None, convention=None)))
    offsets = configure_cameras(cfg, p, parents)
    np.testing.assert_allclose(offsets['base_0']['position'], p['frames']['environment_from_camera']['position'])
    assert all(getattr(cfg.scene,'d405_'+device).offset.convention == 'ros' for device in CAMERAS.values())
    old = deepcopy(cfg)
    wrong = deepcopy(parents); wrong['right']['parent'] = 'wrong'
    try: configure_cameras(cfg, p, wrong)
    except ValueError: pass
    else: raise AssertionError('parent mismatch accepted')
    assert cfg.d405_rig == old.d405_rig
    assert vars(cfg.scene.d405_base_0.offset) == vars(old.scene.d405_base_0.offset)
    print('physical profile: real snapshot, explicit frame registration, three cameras, atomic rejection PASS')


if __name__ == '__main__':
    check(Path(sys.argv[1]))
