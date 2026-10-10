"""Verify the actual native asset inputs against a pinned physical A1-cal profile."""
import hashlib
import json
from pathlib import Path


ROBOT = 'robots/arx_ac_one/v1/'


def sha256(path):
    with Path(path).open('rb') as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
        return digest.hexdigest()


def validate_assets(asset_root, profile, pins):
    """Run on the asset host before geometry/place; this does not authorize motion."""
    root = Path(asset_root)
    if not root.is_absolute():
        raise ValueError('A1 asset root must be absolute')
    if not isinstance(profile, dict):
        profile = json.loads(Path(profile).read_text())
    paths = {'binding': root/'a1-binding.json',
             'urdf': root/(ROBOT+'source/acone/urdf/a1-cal.urdf'),
             'usd': root/(ROBOT+'sim/a1-cal.usd'),
             'rig': root/(ROBOT+'d405_rig.json')}
    hashes = {}
    for name, path in paths.items():
        expected = pins.get(str(path))
        if not isinstance(expected, str) or len(expected) != 64:
            raise ValueError('missing SHA pin for A1 asset: '+str(path))
        hashes[name] = sha256(path)
        if hashes[name] != expected:
            raise ValueError('A1 asset SHA mismatch: '+str(path))
    binding = json.loads(paths['binding'].read_text())
    model = profile.get('model_binding')
    if (not isinstance(model, dict) or not model.get('urdf_sha256')
            or binding.get('schema') != 'a1_isaac_assets.v1'
            or binding.get('conversion_complete') is not True
            or not profile.get('profile_id')
            or binding.get('profile_id') != profile['profile_id']
            or binding.get('model_binding') != model
            or hashes['urdf'] != model['urdf_sha256']
            or hashes['usd'] != binding.get('usd_sha256')):
        raise ValueError('A1 asset conversion/profile/model binding mismatch')
    for key in ('urdf', 'usd'):
        if Path(binding.get(key, '')).resolve() != paths[key].resolve():
            raise ValueError('A1 binding points outside selected model: '+key)
    for stem in ('acone', 'acone_no_handle', 'acone_no_handle_d405'):
        if sha256(paths['urdf'].with_name(stem+'.urdf')) != hashes['urdf']:
            raise ValueError('native URDF alias differs from A1-cal: '+stem)
    for stem in ('ac_one', 'ac_one_no_handle', 'ac_one_no_handle_d405'):
        if sha256(paths['usd'].with_name(stem+'.usd')) != hashes['usd']:
            raise ValueError('native USD alias differs from A1-cal: '+stem)
    rig = json.loads(paths['rig'].read_text())
    if rig.get('id') != profile['profile_id'] or rig.get('calibration_binding') != model:
        raise ValueError('camera rig differs from A1-cal profile')
    from a1_asset_derivation import CERTIFICATES,PROOF,certificate_path,mechanical_identity
    sidecar=paths['urdf'].with_name('a1_support_contact.json');proof_path=paths['urdf'].with_name(PROOF)
    derivation=binding.get('support_derivation')
    # Unmarked historical assets retain their original native sidecar checks.
    # Every newly derived binding explicitly requires all of its proof files.
    if proof_path.exists() or derivation is not None:
        if not isinstance(derivation,dict):raise ValueError('derived A1 support binding required')
        extras=[sidecar,proof_path,*[paths['urdf'].parent/'derived-certificates'/name for name in CERTIFICATES]]
        for path in extras:
            if sha256(path)!=pins.get(str(path)):raise ValueError('missing or mismatched derived A1 proof pin: '+str(path))
        if (sha256(proof_path)!=derivation.get('proof_sha256') or sha256(sidecar)!=derivation.get('sidecar_sha256')
                or mechanical_identity(paths['urdf'])!=derivation.get('mechanical_identity_sha256')):
            raise ValueError('derived support mechanical/sidecar binding differs')
        for name in CERTIFICATES:certificate_path(name,paths['urdf'])
        hashes.update(support_sidecar=sha256(sidecar),support_derivation=sha256(proof_path))
    return dict(profile_id=profile['profile_id'], model_binding=model, asset_sha256=hashes)
