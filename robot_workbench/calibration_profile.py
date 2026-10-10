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


def _capture_evidence(config,capture_root,records,expected_install):
    """Same origin checks for fresh C and a pinned diagnostic C source."""
    batches={};pins={}
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
                or data.get('asset_geometry_id')!=config['asset_geometry_id']
                or data.get('asset_seal_sha256')!=config['asset_seal_sha256']
                or data.get('clock')!=config['clock']
                or not data.get('frames') or _json(completion).get('passed') is not True):
            raise ValueError(kind+' must come from a completed current-run fake capture')
        batches[kind]=data;pins.update({str(path):_sha(path),str(completion):_sha(completion)})
    contract=records['wrist']['payload']['contract']
    if (contract.get('head_snapshot',{}).get('id')!=records['head']['id']
            or batches['wrist'].get('wrist_contract')!=contract
            or records['wrist']['payload']['row'].get('pair_status')!='complete'):
        raise ValueError('installed wrists must retain this head and current capture contract')
    return batches,pins


def build_fake_profile(config_path, capture_root, template, registration, output, *, expected_install,diagnostic_source=None):
    """Create new files only; all arguments identify caller-owned current-run inputs.

    config_path is the resident run's calibration-config.json; its parent is the
    run root. capture_root is this run's Workbench output below that root.
    expected_install has head_id, wrist_id and the actual manifest.json byte SHA.
    registration must explicitly contain length_unit='m' and
    T_environment_calibration_world (the plant base_link registration).
    """
    if diagnostic_source is not None:
        return _build_diagnostic_profile(config_path,template,registration,output,expected_install,diagnostic_source)
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
    batches,capture_pins=_capture_evidence(config,capture_root,records,expected_install);pins.update(capture_pins)
    wrist_contract=records['wrist']['payload']['contract']
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


def _fixture_identity(base):
    """Only the fixed observation plant; D code, worker paths and fits are excluded."""
    value=dict(assets=base['pipeline_config']['assets'],native_code=base['pipeline_config']['native_code'],
        native_files={name:sha for name,sha in base['lanes']['source']['raw_provenance']['source_files'].items()
                      if name.startswith('source/')},
        physical=base['native_identity']['physical_configuration'],initial_suite=base['initial_suite'],
        camera_configuration=base['lanes']['source']['configuration']['cameras'],
        cup_contact_preset=base['pipeline_config'].get('cup_contact_preset'),
        joint_offsets_rad=base.get('plant_joint_offsets_rad',[0.]*12),metal_gap_offset_mm=.008)
    if not value['native_files'] or not value['physical']:raise ValueError('explicit fixed plant identity required')
    return value


def check_diagnostic_pins(source):
    for name,digest in source['input_sha256'].items():
        path=Path(name)
        if (not path.is_absolute() or path.is_symlink() or not path.is_file()
                or _sha(path)!=digest):raise ValueError('diagnostic source pin changed: '+name)


def diagnostic_metadata(source):
    return dict(mode='warm_diagnostic',fresh_C_completed=False,source_run=source['source_run'],
        origin_robot_id=source['origin_robot_id'],origin_clock=deepcopy(source['origin_clock']),
        expected_install=deepcopy(source['expected_install']),qualification='reused fake calibration diagnostic; not an empty-store case')


def validate_diagnostic_source(spec,base,nominal,markers,output):
    """Explicit fake-only diagnostic import; immutable origin is never relabelled."""
    from a1_cal import read_active
    from integration import _digest
    from model_adapt import verify
    from robot_workbench.sample_clock import clock_for_source
    if (not isinstance(spec,dict) or set(spec)!={'mode','source_run','source_base_manifest','expected_install','input_sha256'}
            or spec['mode']!='warm_diagnostic' or nominal.get('simulation_only') is not True):
        raise ValueError('explicit warm_diagnostic fake source contract required')
    root=Path(spec['source_run']);destination=Path(output).resolve()
    if (not root.is_absolute() or root.is_symlink() or not root.is_dir()
            or destination==root.resolve() or destination.is_relative_to(root.resolve())):
        raise ValueError('diagnostic output must be separate from its existing source run')
    if not isinstance(spec['input_sha256'],dict) or not spec['input_sha256']:
        raise ValueError('diagnostic source file pins required')
    check_diagnostic_pins(spec)
    config_path=_inside(root/'calibration-config.json',root);config=_json(config_path)
    if (config.get('environment')!='fake' or config.get('simulation_only') is not True
            or config.get('robot_source')!='simulated' or not str(config.get('robot_id','')).startswith('fake-')):
        raise ValueError('diagnostic calibration source must be an actual fake capture')
    clock_for_source(config,'fake')
    base_path=Path(spec['source_base_manifest'])
    if not base_path.is_absolute() or base_path.is_symlink():raise ValueError('pinned source base manifest required')
    origin_base=_json(base_path);run=_json(root/'run.json')
    if run.get('base_sha256')!=_sha(base_path) or _fixture_identity(origin_base)!=_fixture_identity(base):
        raise ValueError('diagnostic source uses a different fixed PhysX fixture')
    source_nominal_path=base_path.with_name('nominal-config.json');source_nominal=_json(source_nominal_path)
    if run.get('nominal_config_sha256')!=_sha(source_nominal_path):
        raise ValueError('source nominal configuration differs from its recorded run')
    nominal_keys=('urdf','mesh_manifest','scene_seed','candidate','camera_seed','jaw_mapping_file','model_family',
        'asset_geometry_id','asset_seal_sha256','parking_support_contact','wrist_nominal_mounts',
        'wrist_camera_housings','wrist_support_proxy','margin_px')
    if any(config.get(key)!=nominal.get(key) or source_nominal.get(key)!=nominal.get(key) for key in nominal_keys):
        raise ValueError('diagnostic nominal estimator inputs differ')
    fixture=_json(root/'plant-fixture.json');printed=_json(root/'truth-fixture.json')
    if (fixture.get('source_clock_id')!=config['clock']['source_clock_id']
            or fixture.get('joint_offsets_rad')!=base.get('plant_joint_offsets_rad',[0.]*12)
            or fixture.get('metal_gap_offset_mm')!=.008 or printed.get('markers')!=markers):
        raise ValueError('diagnostic fixed device/printed-marker fixture differs')
    if config.get('candidate') and printed.get('nominal_candidate_sha256')!=_sha(config['candidate']):
        raise ValueError('source printed marker candidate differs from its recorded fixture')
    nominal_root=Path(config['urdf']).parent;seal_path=nominal_root/'SHA256SUMS.json'
    if _sha(seal_path)!=config['asset_seal_sha256']:raise ValueError('diagnostic nominal asset seal differs')
    verify(nominal_root)
    # Truth fixtures are used only for equality admission, never sent to C or a solver.
    active=Path(config['calibration_root'])/'hardware/active'
    paths={name:_inside(active/(name+'.json'),root) for name in ('head','wrist','manifest')}
    records=read_active(active,config['asset_geometry_id'],asset_seal=config['asset_seal_sha256'],environment='fake')
    expected=dict(head_id=records['head']['id'],wrist_id=records['wrist']['id'],manifest_sha256=_sha(paths['manifest']))
    if spec['expected_install']!=expected or records['manifest']['robot_id']!=config['robot_id']:
        raise ValueError('diagnostic source group differs from the declared group')
    _capture_evidence(config,root/'calibration-workbench',records,expected)
    required=[config_path,base_path,source_nominal_path,seal_path,root/'run.json',root/'plant-fixture.json',root/'truth-fixture.json',
              root/'registration.json',*paths.values(),Path(config['camera'])]
    for key in ('urdf','mesh_manifest','scene_seed','candidate','camera_seed','jaw_mapping_file','head_motion_seed'):
        if config.get(key):required.append(Path(config[key]))
    if config.get('chassis_mesh'):required.append(Path(config['mesh_manifest']).parent/config['chassis_mesh'])
    for kind in ('head','wrist'):
        batch=Path(records[kind]['source_batch'])
        if not batch.is_absolute():batch=root/'calibration-workbench'/batch
        required.extend([_inside(batch/'dataset.json',root/'calibration-workbench'),_inside(batch/'completion.json',root/'calibration-workbench')])
    if any(str(path) not in spec['input_sha256'] for path in required):
        raise ValueError('diagnostic source pin set omits required C/fixture evidence')
    check_diagnostic_pins(spec)
    return dict(deepcopy(spec),origin_robot_id=config['robot_id'],origin_clock=deepcopy(config['clock']),
        records=records,fixture_identity_sha256=_digest(_fixture_identity(base)))


def diagnostic_snapshot(source):
    check_diagnostic_pins(source)
    return dict(verified=True,**deepcopy(source['expected_install']),diagnostic_origin=diagnostic_metadata(source),fresh_C_completed=False)


def _build_diagnostic_profile(config_path,template,registration,output,expected_install,source):
    from a1_cal import canonical,read_active
    from integration import _digest
    from robot_workbench.sample_clock import clock_for_source
    current=_json(config_path);root=Path(config_path).resolve().parent;output=Path(output).resolve()
    check_diagnostic_pins(source)
    if (source.get('mode')!='warm_diagnostic' or expected_install!=source['expected_install']
            or current.get('environment')!='fake' or current.get('simulation_only') is not True
            or current.get('robot_source')!='simulated' or current.get('robot_id')==source['origin_robot_id']
            or not str(current.get('robot_id','')).startswith('fake-')
            or clock_for_source(current,'fake')==source['origin_clock']
            or not output.is_relative_to(root) or root.is_relative_to(Path(source['source_run']).resolve())
            or output.is_relative_to(Path(source['source_run']).resolve())):
        raise ValueError('warm diagnostic requires a new fake robot, clock and output scope')
    origin=Path(source['source_run']);old=_json(origin/'calibration-config.json')
    records=read_active(Path(old['calibration_root'])/'hardware/active',old['asset_geometry_id'],
                        asset_seal=old['asset_seal_sha256'],environment='fake')
    if (source['origin_robot_id']!=old['robot_id'] or source['origin_clock']!=old['clock']
            or source['records']!=records):raise ValueError('diagnostic origin context changed after validation')
    if (current['hardware_camera_serials']!=old['hardware_camera_serials']
            or _json(current['camera'])!=_json(old['camera'])
            or registration!=_json(origin/'registration.json')):
        raise ValueError('new diagnostic sensors or base registration differ from the source fixture')
    for key in ('urdf','jaw_mapping_file','asset_geometry_id','asset_seal_sha256'):
        if current[key]!=old[key]:raise ValueError('new diagnostic nominal geometry differs')
    for side in ('left','right'):
        camera=_json(current['hardware_wrist_camera_files'][side])
        if camera!=source['records']['wrist']['payload']['contract']['cameras'][side]:
            raise ValueError('new diagnostic wrist intrinsics differ')
    if output.exists():raise FileExistsError(output)
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.warm-diagnostic-',dir=output.parent) as temporary:
        stage=Path(temporary)/'profile'
        build_fake_profile(origin/'calibration-config.json',origin/'calibration-workbench',template,registration,stage,
                           expected_install=source['expected_install'])
        metadata=diagnostic_metadata(source)
        binding=_json(stage/'acquisition-binding.json')
        binding.update(robot_id=current['robot_id'],diagnostic_origin=metadata,fresh_C_completed=False)
        bridge=_json(stage/'bridge.json');bridge.update(diagnostic_origin=metadata,fresh_C_completed=False,
            observation_robot_id=current['robot_id'],observation_clock=deepcopy(current['clock']),
            qualification='warm fake diagnostic only; original C identities retained; no empty-store qualification')
        # Admission may read fixed plant truth, but workers receive only its digest.
        # Keep the origin builder's actual calibration/model inputs as the D graph.
        admission=dict(schema='warm_fake_diagnostic_admission.v1',mode='warm_diagnostic',fresh_C_completed=False,
            fixture_identity_sha256=source['fixture_identity_sha256'],
            admission_evidence_set_sha256=_digest(source['input_sha256']),admission_evidence_files=len(source['input_sha256']),
            expected_install=deepcopy(expected_install),observation_robot_id=current['robot_id'],
            observation_clock=deepcopy(current['clock']),qualification='fixed-fixture equality admission only; not calibration or task accuracy')
        (stage/'diagnostic-admission.json').write_bytes(canonical(admission)+b'\n')
        bridge['diagnostic_admission_sha256']=_sha(stage/'diagnostic-admission.json')
        bridge['input_sha256'][str(output/'diagnostic-admission.json')]=bridge['diagnostic_admission_sha256']
        bridge['input_sha256'][str(Path(config_path).resolve())]=_sha(config_path)
        for path in (current['camera'],*current['hardware_wrist_camera_files'].values()):
            bridge['input_sha256'][str(Path(path).resolve())]=_sha(path)
        planner=_json(stage/'planning-model.json')
        planner.update(urdf=str(output/'model/robot.urdf'),bundle=str(output/'model'),fresh_C_completed=False)
        configuration=_json(stage/'configuration.json')
        configuration['acquisition'].update(profile_path=str(output/'profile.json'),acquisition_binding_path=str(output/'acquisition-binding.json'))
        configuration['planning_model_path']=str(output/'planning-model.json')
        for name,value in [('acquisition-binding',binding),('bridge',bridge),('planning-model',planner),('configuration',configuration)]:
            (stage/(name+'.json')).write_bytes(canonical(value)+b'\n')
        check_diagnostic_pins(source);stage.rename(output)
    return dict(output=str(output),profile_path=str(output/'profile.json'),acquisition_binding_path=str(output/'acquisition-binding.json'),
        planning_model_path=str(output/'planning-model.json'),calibration_snapshot_id=expected_install['manifest_sha256'],
        simulation_only=True,runtime_allowed=False,fresh_C_completed=False,diagnostic_origin=metadata)


def diagnostic_mesh_view(workbench,source):
    """Read-only origin candidates over the current nominal geometry, no store import."""
    from types import SimpleNamespace
    check_diagnostic_pins(source)
    rows={kind:deepcopy(source['records'][kind]['payload']['row']) for kind in ('head','wrist')}
    for kind,row in rows.items():row.update(id=source['records'][kind]['id'],kind=kind)
    contract=deepcopy(source['records']['wrist']['payload']['contract'])
    if contract['geometry_id']!=workbench.geometry_id:raise ValueError('diagnostic C geometry fingerprint differs')
    view=SimpleNamespace(geometry=workbench.geometry,geometry_id=workbench.geometry_id,
        calibration_snapshot=workbench.calibration_snapshot,
        calibration_results=lambda:[deepcopy(rows['head'])],wrist_results=lambda:[deepcopy(rows['wrist'])],
        wrist_contract=lambda row:deepcopy(contract))
    view.wrist_overlay_error=lambda row:type(workbench).wrist_overlay_error(view,row)
    candidates=[dict(id=row['id'],kind=kind,label='复用标定诊断 · '+kind,status='diagnostic_readonly',
        details={'diagnostic_origin':diagnostic_metadata(source)}) for kind,row in rows.items()]
    return view,candidates


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('config','capture-root','template','registration','expected-install','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(build_fake_profile(args.config,args.capture_root,_json(args.template),_json(args.registration),
        args.output,expected_install=_json(args.expected_install)),indent=2))


if __name__=='__main__':main()
