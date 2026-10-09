"""CPU check: reject stale assets even when planner and replay would agree."""
import json
from pathlib import Path
import tempfile

from a1_asset_validation import ROBOT, sha256, validate_assets


def main():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        urdf = root/(ROBOT+'source/acone/urdf/a1-cal.urdf')
        usd = root/(ROBOT+'sim/a1-cal.usd')
        for path in (urdf, usd):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('calibrated '+path.suffix)
        for name in ('acone', 'acone_no_handle', 'acone_no_handle_d405'):
            urdf.with_name(name+'.urdf').symlink_to(urdf.name)
        for name in ('ac_one', 'ac_one_no_handle', 'ac_one_no_handle_d405'):
            usd.with_name(name+'.usd').symlink_to(usd.name)
        model = {'urdf_sha256': sha256(urdf), 'geometry_id': 'calibrated'}
        profile = dict(profile_id='a1-cal-current', model_binding=model)
        binding = dict(schema='a1_isaac_assets.v1', conversion_complete=True,
                       profile_id=profile['profile_id'], model_binding=model,
                       urdf=str(urdf), usd=str(usd), usd_sha256=sha256(usd))
        binding_path = root/'a1-binding.json'
        rig_path = root/(ROBOT+'d405_rig.json')
        binding_path.write_text(json.dumps(binding))
        rig_path.write_text(json.dumps(dict(id=profile['profile_id'], calibration_binding=model)))
        pins = {str(p): sha256(p) for p in (urdf, usd, binding_path, rig_path)}
        assert validate_assets(root, profile, pins)['model_binding'] == model

        def rejected(candidate=profile, candidate_pins=pins):
            try:
                validate_assets(root, candidate, candidate_pins)
            except (ValueError, FileNotFoundError):
                return
            raise AssertionError('stale or unpinned asset accepted')

        rejected({**profile, 'model_binding': {**model, 'geometry_id': 'other'}})
        rejected(candidate_pins={})
        usd.write_text('legacy model shared by planner and replay')
        rejected()
        usd.write_text('calibrated .usd')
        alias = usd.with_name('ac_one_no_handle_d405.usd')
        alias.unlink()
        alias.write_text('legacy alias')
        rejected()
    print('A1 asset identity: valid binding accepted; stale model, missing pins, changed USD and legacy alias rejected')


if __name__ == '__main__':
    main()
