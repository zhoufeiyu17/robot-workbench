"""Frozen object selection shared by detection, native geometry and episode data.

The prepared asset root owns scene-assets.json. Loading it never starts a model,
simulator or device. A missing file is supported only for explicit legacy runs.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re

SCHEMA = 'workbench.scene_asset_set.v1'
LEGACY_MACHINE = 'objects/owner_coffee_machine_00003/v1'
LEGACY_CUP = 'objects/anyverse_lidded_paper_cup_9cm_13cm/v1'


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def _sha(value):
    if not isinstance(value, str) or not re.fullmatch(r'(?:sha256:)?[0-9a-f]{64}', value):
        raise ValueError('scene asset requires a SHA256 digest')
    return value.removeprefix('sha256:')


def asset_reference(value):
    path = PurePosixPath(value) if isinstance(value, str) else PurePosixPath()
    if (path.is_absolute() or len(path.parts) != 3 or path.parts[0] != 'objects'
            or any(p in ('.', '..') for p in path.parts) or str(path) != value):
        raise ValueError('scene asset must name a versioned objects/<id>/<version> directory')
    return value


def validate(value):
    if (not isinstance(value, dict) or set(value) != {'schema', 'machine', 'cup', 'perception',
            'input_sha256', 'asset_set_digest'} or value.get('schema') != SCHEMA):
        raise ValueError('explicit versioned scene asset set required')
    for kind in ('machine', 'cup'):
        item = value[kind]
        fields={'asset_id', 'revision', 'native_asset', 'interaction_profile_sha256'}
        if (not isinstance(item, dict) or not fields<=set(item)
                or set(item)-fields-({'grasp_command','inward_envelope','pose_refinement'} if kind=='cup' else set())):
            raise ValueError('scene asset '+kind+' identity/profile required')
        for key in ('asset_id', 'revision'):
            if not isinstance(item[key], str) or not item[key].strip():
                raise ValueError('scene asset '+kind+'.'+key+' required')
        asset_reference(item['native_asset'])
        _sha(item['interaction_profile_sha256'])
        if 'grasp_command' in item:
            validate_grasp_command(item['grasp_command'],item['interaction_profile_sha256'])
    perception = value['perception']
    if not isinstance(perception, dict) or set(perception) != {'release_manifest_digest', 'runtime_config_id', 'node_config_sha256'}:
        raise ValueError('scene assets require one frozen two-object detection runtime')
    for item in perception.values():
        _sha(item)
    pins = value['input_sha256']
    if not isinstance(pins, dict) or not pins:
        raise ValueError('scene assets require original publication and derived file pins')
    for raw, expected in pins.items():
        path = Path(raw)
        if not path.is_absolute() or '..' in path.parts:
            raise ValueError('scene asset pin must be an absolute contained file path')
        _sha(expected)
    envelope=value['cup'].get('inward_envelope')
    if envelope is not None:
        if (not isinstance(envelope,dict) or set(envelope)!={'path','sha256'}
                or envelope.get('path') not in pins
                or _sha(envelope.get('sha256'))!=_sha(pins[envelope['path']])):
            raise ValueError('selected cup inward envelope requires an immutable file pin')
    refinement=value['cup'].get('pose_refinement')
    if refinement is not None:
        from cup_pose_refinement import validate_spec
        validate_spec(refinement,value['cup']['interaction_profile_sha256'])
        if _sha(refinement['mesh_sha256']) not in {_sha(v) for v in pins.values()}:
            raise ValueError('cup pose refinement requires its visual mesh file pin')
    if value['asset_set_digest'] != digest({k: v for k, v in value.items() if k != 'asset_set_digest'}):
        raise ValueError('scene asset set digest differs from content')
    return value


def validate_grasp_command(command,profile_sha256):
    if (not isinstance(command,dict) or set(command)!={'units','controlled_gripper','metal_gap_mm','derivation'}
            or command.get('units')!='metal_inner_gap_mm' or command.get('controlled_gripper')!='left_gripper'):
        raise ValueError('selected cup requires an explicit metal-gap grasp seed')
    rule=command['derivation']
    if (not isinstance(rule,dict) or set(rule)!={'schema','cup_profile_sha256','reference_height_m','required_diameter_m','pad_thickness_m','qualified'}
            or rule.get('schema')!='cup_grasp_seed.v1' or rule.get('qualified') is not False
            or _sha(rule.get('cup_profile_sha256'))!=_sha(profile_sha256)
            or any(type(rule.get(k)) not in (int,float) or not math.isfinite(rule[k]) or rule[k]<=0
                   for k in ('reference_height_m','required_diameter_m','pad_thickness_m'))
            or rule['pad_thickness_m']!=.0015):
        raise ValueError('grasp seed must bind the selected cup and existing pad geometry')
    desired=1000*(rule['required_diameter_m']+2*rule['pad_thickness_m'])
    expected=min((65.,71.),key=lambda gap:abs(gap-desired))
    if type(command['metal_gap_mm']) not in (int,float) or command['metal_gap_mm']!=expected:
        raise ValueError('grasp seed differs from the nearest existing 65/71mm candidate')
    return command


def derive_grasp_command(cup,*,pad_thickness_m=.0015):
    """Select a search seed from existing commands; final contact proof is separate."""
    low,high=cup.candidate_grasp_band_m;height=(low+high)/2
    diameter=cup.required_opening_m(height-1e-6,height+1e-6)
    desired=1000*(diameter+2*pad_thickness_m)
    command=dict(units='metal_inner_gap_mm',controlled_gripper='left_gripper',
        metal_gap_mm=min((65.,71.),key=lambda gap:abs(gap-desired)),
        derivation=dict(schema='cup_grasp_seed.v1',cup_profile_sha256=cup.profile_sha256,
            reference_height_m=height,required_diameter_m=diameter,pad_thickness_m=pad_thickness_m,qualified=False))
    return validate_grasp_command(command,cup.profile_sha256)


def load(asset_root, *, expected_digest=None):
    path = Path(asset_root)/'scene-assets.json'
    if not path.exists() and not path.is_symlink():
        if expected_digest is not None:
            raise ValueError('selected scene-assets.json is missing; legacy fallback is forbidden')
        return None
    if path.is_symlink() or not path.is_file() or path.stat().st_size > 1024*1024:
        raise ValueError('bounded scene asset manifest must be a regular file')
    value = validate(json.loads(path.read_bytes()))
    if expected_digest is not None and value['asset_set_digest'] != expected_digest:
        raise ValueError('prepared scene asset set differs from the selected combination')
    for raw, expected in value['input_sha256'].items():
        file = Path(raw)
        if file.is_symlink() or not file.is_file() or hashlib.sha256(file.read_bytes()).hexdigest() != _sha(expected):
            raise ValueError('scene asset dependency changed: '+raw)
    envelope=value['cup'].get('inward_envelope')
    if envelope is not None:
        proof=json.loads(Path(envelope['path']).read_bytes())
        if _sha(proof.get('cup_profile_sha256'))!=_sha(value['cup']['interaction_profile_sha256']):
            raise ValueError('selected cup inward envelope belongs to another profile')
    refinement=value['cup'].get('pose_refinement')
    if refinement is not None:
        path=next(p for p,h in value['input_sha256'].items() if _sha(h)==_sha(refinement['mesh_sha256']))
        _check_rim_feature(path,refinement)
    return value


def _check_rim_feature(path,spec):
    """Check the declared circle without importing the offline numerical fitter.

    Scene manifests are also read by stdlib-only bridge/supervisor processes.
    Numerical fitting stays in the acquisition worker, outside these readers.
    """
    vertices=[tuple(float(v)for v in line.split()[1:4])for line in Path(path).read_text().splitlines()
              if line.startswith('v ')]
    error='cup pose refinement differs from the pinned visual mesh feature'
    if not vertices or any(len(p)!=3 or not all(math.isfinite(v)for v in p)for p in vertices):
        raise ValueError(error)
    top=max(p[2]for p in vertices);height=spec['rim_height_m'];radius=spec['radius_m']
    upper=[p for p in vertices if p[2]>top*.9]
    maximum=max(math.hypot(p[0],p[1])for p in upper)
    points=[p for p in upper if abs(p[2]-height)<1e-6 and abs(math.hypot(p[0],p[1])-radius)<1e-5]
    if not top*.9<height<=top+1e-6 or abs(maximum-radius)>1e-5 or len(points)<16:
        raise ValueError(error)
    angles=sorted({round(math.atan2(p[1],p[0]),6)for p in points})
    if len(angles)<16 or max(b-a for a,b in zip(angles,angles[1:]+[angles[0]+2*math.pi]))>=.3:
        raise ValueError(error)


def episode_selection(episode):
    metadata = getattr(episode, 'metadata', None) or {}
    value = metadata.get('scene_asset_set')
    return validate(value) if value is not None else None


def selected_cup_asset(asset_root, episode):
    """Native runtime and planner must select the same profile and USD."""
    selected = load(asset_root)
    declared = episode_selection(episode)
    if selected != declared:
        raise ValueError('episode and prepared native scene asset sets differ')
    if selected is None:
        return LEGACY_CUP
    if getattr(episode, 'machine_asset', None) != selected['machine']['native_asset']:
        raise ValueError('episode machine differs from selected scene asset set')
    return selected['cup']['native_asset']


def validate_episode_machine(machine_asset, metadata):
    selection = (metadata or {}).get('scene_asset_set')
    if selection is None:
        return False
    return validate(selection)['machine']['native_asset'] == machine_asset


def check_geometry(selection, machine_profile_sha256, cup_profile_sha256):
    validate(selection)
    for kind, actual in (('machine', machine_profile_sha256), ('cup', cup_profile_sha256)):
        if _sha(actual) != _sha(selection[kind]['interaction_profile_sha256']):
            raise ValueError('native '+kind+' geometry differs from selected interaction profile')


def check_detection(selection, summary, tracks=None):
    validate(selection)
    if any(_sha(summary.get(k)) != _sha(v) for k, v in selection['perception'].items()):
        raise ValueError('detection runtime differs from selected scene asset set')
    for track in tracks or []:
        kind = {'coffee_machine': 'machine', 'cup': 'cup'}.get(track.get('object_type'))
        if kind is None:
            continue
        actual = (track.get('provenance') or {}).get('asset') or {}
        expected = selection[kind]
        if any(actual.get(k) != expected[k] for k in ('asset_id', 'revision')):
            raise ValueError('detected '+kind+' model differs from selected scene asset set')


def episode_metadata(selection, initial=None):
    metadata = deepcopy(initial or {})
    if selection is not None:
        metadata['scene_asset_set'] = deepcopy(validate(selection))
    return metadata
