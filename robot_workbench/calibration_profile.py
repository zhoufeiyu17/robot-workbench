"""Offline C-installed fake calibration -> D observation and planning inputs.

Use the current run's nominal A1 bundle, never the resident plant's fixed USD.
The result is an isolated URDF/profile candidate; USD conversion and planning
qualification remain separate work on the authorized simulator host.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _json(path):
    return json.loads(Path(path).read_bytes())


def _inside(path, root):
    path=Path(path)
    if not path.is_absolute():path=root/path
    if path.is_symlink() or not path.resolve().is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError('current run input missing, symlinked or outside its root: '+str(path))
    return path.resolve()


def build_fake_profile(config_path, capture_root, template, registration, output, *, expected_install):
    """Create new files only; all arguments identify caller-owned current-run inputs.

    config_path is the resident run's calibration-config.json; its parent is the
    run root. capture_root is this run's Workbench output below that root.
    expected_install has head_id, wrist_id and the actual manifest.json byte SHA.
    registration must explicitly contain length_unit='m' and
    T_environment_calibration_world (the plant base_link registration).
    """
    from a1_cal import build, canonical, read_active
    from integration import _digest
    from model_adapt import digest, verify
    from physical_profile import camera_offsets, rigid
    from robot_workbench.sample_clock import clock_for_source

    config_path=Path(config_path).resolve();run_root=config_path.parent
    capture_root=Path(capture_root).resolve();output=Path(output).resolve()
    config=_json(config_path)
    if (config.get('environment')!='fake' or config.get('simulation_only') is not True
            or config.get('robot_source')!='simulated' or not str(config.get('robot_id','')).startswith('fake-')):
        raise ValueError('explicit current fake calibration configuration required')
    capture_clock=clock_for_source(config,'fake')
    if not capture_root.is_dir() or not capture_root.is_relative_to(run_root):
        raise ValueError('capture_root must be the current resident run Workbench output')
    if (not isinstance(expected_install,dict) or set(expected_install)!={'head_id','wrist_id','manifest_sha256'}
            or any(not isinstance(v,str) or not v for v in expected_install.values())):
        raise ValueError('current installed head/wrist identities and manifest SHA required')
    if (not isinstance(registration,dict) or registration.get('length_unit')!='m'
            or 'T_environment_calibration_world' not in registration):
        raise ValueError('explicit T_environment_calibration_world with metre units required')
    transform=rigid(registration['T_environment_calibration_world'])
    root=Path(config['calibration_root'])
    if (not root.is_absolute() or not root.resolve().is_relative_to(run_root)
            or root.resolve().is_relative_to(Path('/var/lib/anyverse/calibration').resolve())):
        raise ValueError('fake installation must be isolated below this current run')
    active=root/'hardware'/'active'  # Legacy C business namespace, never physical authorization.
    active_paths={name:_inside(active/(name+'.json'),run_root) for name in ('head','wrist','manifest')}
    active_sha={name+'.json':_sha(path) for name,path in active_paths.items()}
    if active_sha['manifest.json']!=expected_install['manifest_sha256']:
        raise ValueError('active manifest differs from the current installation receipt')
    nominal=Path(config['urdf']).resolve().parent
    verify(nominal)
    contract=_json(nominal/'calibration-input.json');model=_json(nominal/'manifest.json')
    if (contract.get('schema')!='robot_calibration_input.v1' or contract.get('zero_offsets_rad') is not None
            or contract.get('units')!={'length':'m','joint_angle':'rad','public_jaw_gap':'mm'}
            or model.get('calibration_status')!='required_head_then_wrists'
            or config.get('asset_geometry_id')!=contract.get('geometry_id')
            or config.get('asset_seal_sha256')!=digest(nominal/'SHA256SUMS.json')):
        raise ValueError('current C geometry must be the sealed nominal A1, with no baked calibration')
    if Path(config['urdf']).resolve()!=nominal/'robot.urdf':
        raise ValueError('nominal A1 robot.urdf is required')
    records=read_active(active,contract['geometry_id'],asset_seal=config['asset_seal_sha256'],environment='fake')
    if records['manifest']['robot_id']!=config['robot_id']:
        raise ValueError('installed group belongs to another fake robot')
    pins={str(config_path):_sha(config_path),str(nominal/'SHA256SUMS.json'):digest(nominal/'SHA256SUMS.json')}
    batches={}
    for kind in ('head','wrist'):
        record=records[kind]
        if record['id']!=expected_install[kind+'_id']:
            raise ValueError(kind+' result differs from current installation receipt')
        batch=Path(record['source_batch'])
        if not batch.is_absolute():batch=capture_root/batch
        path=_inside(batch/'dataset.json',capture_root);completion=_inside(batch/'completion.json',capture_root)
        data=_json(path)
        if (data.get('runtime_environment')!='fake' or data.get('simulation_only') is not True
                or data.get('robot_source')!='simulated' or data.get('robot_id')!=config['robot_id']
                or data.get('asset_geometry_id')!=contract['geometry_id']
                or data.get('asset_seal_sha256')!=config['asset_seal_sha256']
                or data.get('clock')!=capture_clock
                or not data.get('frames') or _json(completion).get('passed') is not True):
            raise ValueError(kind+' must come from a completed current-run fake capture')
        batches[kind]=data;pins.update({str(path):_sha(path),str(completion):_sha(completion)})
    wrist_contract=records['wrist']['payload']['contract']
    if (wrist_contract.get('head_snapshot',{}).get('id')!=records['head']['id']
            or batches['wrist'].get('wrist_contract')!=wrist_contract
            or records['wrist']['payload']['row'].get('pair_status')!='complete'):
        raise ValueError('installed wrists must retain this head and current capture contract')
    camera_path=_inside(config['camera'],run_root);head_camera=_json(camera_path)
    if head_camera!=batches['head'].get('camera'):
        raise ValueError('head intrinsics differ from the actual current-run capture')
    cameras={'head':head_camera,**wrist_contract['cameras']}
    if {key:value.get('serial') for key,value in cameras.items()}!=config.get('hardware_camera_serials'):
        raise ValueError('installed camera identities differ from the current fake producer')
    mapping=Path(config['jaw_mapping_file']).resolve()
    if _sha(mapping)!=digest(nominal/'jaw-mapping.json'):
        raise ValueError('runtime jaw mapping differs from nominal A1 evidence')
    pins.update({str(camera_path):_sha(camera_path),str(mapping):_sha(mapping)})
    pins.update({str(path):active_sha[name+'.json'] for name,path in active_paths.items()})
    if output.exists():raise FileExistsError(output)
    if any(output.is_relative_to(path.resolve()) for path in (nominal,capture_root,active)):
        raise ValueError('output must be separate from nominal model, captures and active installation')
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.fake-calibration-profile-',dir=output.parent) as temporary:
        stage=Path(temporary)/'bridge';stage.mkdir()
        build(nominal,active,head_camera,template,transform,stage/'model',environment='fake')
        # Verify that a concurrent install did not change the group consumed by build().
        for name,record in records.items():
            if _json(stage/'model'/'calibration_sources'/(name+'.json'))!=record:
                raise ValueError('installation changed while building the planning model')
        original=ET.parse(nominal/'robot.urdf').getroot();adapted=ET.parse(stage/'model'/'robot.urdf').getroot()
        for name in contract['joint_order']:
            old,new=(tree.find("joint[@name='"+name+"']") for tree in (original,adapted))
            if old is None or new is None or ET.canonicalize(ET.tostring(old).decode(),strip_text=True)!=ET.canonicalize(ET.tostring(new).decode(),strip_text=True):
                raise ValueError('joint calibration was baked into the planning URDF')
        profile=_json(stage/'model'/'profile.json')
        parents={role:dict(parent=parent,T_parent_reference=[[float(i==j) for j in range(4)] for i in range(4)])
                 for role,parent in contract['camera_parents'].items()}
        offsets=camera_offsets(profile,parents)
        binding=dict(environment='fake',runtime_environment='fake',simulation_only=True,robot_source='simulated',
            evidence_source='fake_device_rgbd',robot_id=config['robot_id'],camera_serials=config['hardware_camera_serials'],
            calibration_accepted=True,calibration_profile_digest=_digest(profile),
            calibration_snapshot_id=active_sha['manifest.json'],active_sha256=active_sha,
            head_id=records['head']['id'],wrist_id=records['wrist']['id'],model_binding=deepcopy(profile['model_binding']))
        planner=dict(schema='fake_calibration_planning_model.v1',environment='fake',simulation_only=True,
            urdf=str(output/'model'/'robot.urdf'),urdf_sha256=_sha(stage/'model'/'robot.urdf'),
            bundle=str(output/'model'),bundle_seal_sha256=_sha(stage/'model'/'SHA256SUMS.json'),
            reported_to_model='q_model = q_reported + zero_offsets_rad',joint_zero_offsets_baked_into_urdf=False,
            zero_offsets_rad=profile['joints']['zero_offsets_rad'],camera_offsets=offsets,registration=registration,
            asset_status='URDF_ready_USD_conversion_and_native_asset_binding_required',runtime_allowed=False,
            target_only=True,actual_plant_unchanged=True)
        configuration=dict(environment='fake',acquisition=dict(profile_path=str(output/'profile.json'),
            acquisition_binding_path=str(output/'acquisition-binding.json'),measured_initial_state=True),
            planning_model_path=str(output/'planning-model.json'))
        receipt=dict(schema='installed_fake_calibration_bridge.v1',runtime_environment='fake',simulation_only=True,
            robot_source='simulated',calibration_snapshot_id=binding['calibration_snapshot_id'],
            calibration_capture_clock=capture_clock,
            head_id=records['head']['id'],wrist_id=records['wrist']['id'],active_sha256=active_sha,
            input_sha256=pins,registration=registration,profile_digest=_digest(profile),
            planning_bundle_seal_sha256=planner['bundle_seal_sha256'],runtime_allowed=False,
            qualification='offline configuration only; no calibration accuracy or GPU execution claim')
        for name,value in [('profile',profile),('acquisition-binding',binding),('planning-model',planner),
                           ('configuration',configuration),('bridge',receipt)]:
            (stage/(name+'.json')).write_bytes(canonical(value)+b'\n')
        if any(_sha(path)!=expected for path,expected in pins.items()):
            raise ValueError('a pinned current-run input changed while freezing the bridge')
        verify(nominal);verify(stage/'model');stage.rename(output)
    return dict(output=str(output),profile_path=str(output/'profile.json'),
        acquisition_binding_path=str(output/'acquisition-binding.json'),planning_model_path=str(output/'planning-model.json'),
        calibration_snapshot_id=binding['calibration_snapshot_id'],simulation_only=True,runtime_allowed=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('config','capture-root','template','registration','expected-install','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(build_fake_profile(args.config,args.capture_root,_json(args.template),_json(args.registration),
        args.output,expected_install=_json(args.expected_install)),indent=2))


if __name__=='__main__':main()
