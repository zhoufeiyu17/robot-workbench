"""CPU rig roundtrip and read-only base-asset overlay checks."""
import json
from pathlib import Path
import tempfile
import numpy as np
from scipy.spatial.transform import Rotation
from a1_isaac_assets import calibrated_rig, prepare, ROBOT
from a1_cal import build
from test_a1_cal import inputs
from test_harness import PROFILE
from robot_sim.asset_cfg.d405 import camera_offset


def main():
    here=Path(__file__).parent
    with tempfile.TemporaryDirectory() as directory:
        root=Path(directory); a1=here/'model_assets/ac_one_A1_pending_calibration_r2'
        active,k,_=inputs(root,a1)
        bundle=root/'cal'; build(a1,active,k,PROFILE,np.eye(4),bundle,fixture=True)
        base=root/'base'; (base/ROBOT).mkdir(parents=True)
        rig=dict(camera_proxy_size_m=[.025,.045,.045],device_calibrations={},cameras=[dict(parent=p,device_calibration_ref=d) for p,d in [('base_link','base_0'),('left_link6','left_wrist_0'),('right_link16','right_wrist_0')]])
        (base/(ROBOT+'d405_rig.json')).write_text(json.dumps(rig))
        (base/'objects').mkdir(); (base/'objects/untouched').write_text('original')
        (root/'external-mesh').write_text('mesh-bytes')
        (base/'objects/mesh').symlink_to(root/'external-mesh')
        for relative in ('source/acone/urdf/acone.urdf','sim/ac_one.usd'):
            target=base/ROBOT/relative;target.parent.mkdir(parents=True,exist_ok=True)
            target.write_text('old-base')
        old={str(p.relative_to(base)):p.read_bytes() for p in base.rglob('*') if p.is_file()}
        binding=prepare(bundle,base,root/'overlay')
        assert not binding['runtime_allowed'] and not binding['conversion_complete']
        assert {str(p.relative_to(base)):p.read_bytes() for p in base.rglob('*') if p.is_file()}==old
        assert (base/'objects/mesh').is_symlink()
        assert not (root/'overlay/objects/mesh').is_symlink()
        assert all(p.resolve().is_relative_to((root/'overlay').resolve()) for p in (root/'overlay').rglob('*'))
        (root/'overlay/objects/untouched').write_text('changed-overlay')
        assert (base/'objects/untouched').read_text()=='original'
        (root/'overlay/objects/untouched').write_text('original')
        assert (root/'overlay/objects/untouched').read_text()=='original'
        updated=json.loads((root/'overlay'/(ROBOT+'d405_rig.json')).read_text())
        profile=json.loads((bundle/'profile.json').read_text())
        for camera in updated['cameras']:
            pos,q=camera_offset(camera,updated)
            np.testing.assert_allclose(pos,[.12,.23,.91],atol=1e-12)
            np.testing.assert_allclose(Rotation.from_quat(np.array(q)[[1,2,3,0]]).as_matrix(),np.eye(3),atol=1e-12)
        for name in ('acone.urdf','acone_no_handle.urdf','acone_no_handle_d405.urdf'):
            assert (root/'overlay'/(ROBOT+'source/acone/urdf/'+name)).read_bytes()==(bundle/'robot.urdf').read_bytes()
    print('A1 Isaac assets: optical roundtrip, common URDF aliases, isolated overlay PASS (CPU only)')


if __name__=='__main__':main()
