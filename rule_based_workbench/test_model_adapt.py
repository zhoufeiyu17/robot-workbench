"""Check frozen A1 optical frames, joint-space convention and baseline isolation."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from model_adapt import verify
from arx_env.urdf_mesh import origin


def main():
    assets=Path(__file__).parent/'model_assets'
    native=assets/'ac_one_baseline_v1';pending=assets/'ac_one_A1_pending_calibration'
    verify(native);verify(pending)
    original_tree=ET.parse(native/'robot.urdf').getroot();pending_tree=ET.parse(pending/'robot.urdf').getroot()
    assert [ET.tostring(j) for j in original_tree.findall('joint')]==[ET.tostring(j) for j in pending_tree.findall('joint')]
    assert not (pending/'adaptation.json').exists()
    spec=json.loads((pending/'calibration-input.json').read_text())
    assert spec['zero_offsets_rad'] is None and spec['hardware_access_authorized'] is False
    pending_manifest=json.loads((pending/'manifest.json').read_text())
    assert pending_manifest['old_x5_calibration_applied'] is False
    assert pending_manifest['calibration_status']=='required_head_then_wrists'
    for root,count in ((original_tree,4),(pending_tree,0)):
        assert sum(e.get('name') in ('left_handle','right_handle') for e in root.iter() if e.tag in ('visual','collision'))==count
    print('PASS: current AC-One seals, unmodified joints/cameras, two handles removed, no X5 calibration, hardware gate')
    base,a1=assets/'ac_one_x5_baseline_v1',assets/'ac_one_x5_A1'
    verify(base);verify(a1)
    roots=[ET.parse(p/'robot.urdf').getroot() for p in (base,a1)]
    before,after=[{j.get('name'):j for j in root.findall('joint')} for root in roots]
    adaptation=json.loads((a1/'adaptation.json').read_text())
    assert adaptation['joint_zero_offsets_baked_into_urdf'] is False
    offsets=np.asarray(adaptation['zero_offsets_rad']);assert offsets.shape==(12,)
    for name,joint in before.items():
        if '_rgb_optical_frame' not in name:
            assert ET.tostring(joint)==ET.tostring(after[name]),name
    for name,target in [('top',adaptation['T_world_head_optical']),
                        ('left_wrist',adaptation['T_link6_wrist_optical']['left']),
                        ('right_wrist',adaptation['T_link6_wrist_optical']['right'])]:
        composed=origin(after[name+'_camera_link_joint'].find('origin'))@origin(after[name+'_camera_rgb_optical_frame_joint'].find('origin'))
        assert np.allclose(composed,target,atol=1e-12,rtol=0),name
    for root,count in zip(roots,(4,0)):
        assert sum(e.get('name') in ('left_handle','right_handle') for e in root.iter() if e.tag in ('visual','collision'))==count
        assert not any('sticker' in e.get('name','').lower() for e in root.iter())
    q=np.random.default_rng(5).uniform(-1,1,(100,12))
    assert np.allclose((q+offsets)-offsets,q,atol=1e-15)
    for p in (base,a1):assert json.loads((p/'manifest.json').read_text())['runtime_allowed'] is False
    print('PASS: seals, housing invariance, three optical frames, single offset, handles, no stickers, runtime gate')


if __name__=='__main__':main()
