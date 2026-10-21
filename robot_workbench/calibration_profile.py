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
import shutil
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


def _capture_evidence(config,capture_root,records,expected_install,*,environment='fake'):
    """Same origin checks for fresh C and a pinned diagnostic C source."""
    from robot_workbench.sample_clock import clock_for_source
    simulated=environment=='fake'
    batches={};pins={};resume=_resume_receipt(config,capture_root.parent) if simulated else None
    expected_clock=clock_for_source(config,environment)
    if resume:pins[config['calibration_resume']['path']]=config['calibration_resume']['sha256']
    for kind in ('head','wrist'):
        record=records[kind]
        if record['id']!=expected_install[kind+'_id']:
            raise ValueError(kind+' result differs from current installation receipt')
        batch=Path(record['source_batch'])
        if not batch.is_absolute():batch=capture_root/batch
        path=_inside(batch/'dataset.json',capture_root);completion=_inside(batch/'completion.json',capture_root)
        data=_json(path)
        origin=(resume or {}).get('capture_origins',{}).get(str(path.parent.relative_to(capture_root)))
        if origin and (_sha(path)!=origin['dataset_sha256'] or _sha(completion)!=origin['completion_sha256']):
            raise ValueError('imported capture differs from its pinned original batch')
        runtime_environment=data.get('runtime_environment')
        if (environment=='physical' and 'runtime_environment' not in data
                and data.get('environment')=='hardware' and data.get('robot_source')=='physical'
                and data.get('simulation_only') is False and data.get('robot_id')==config['robot_id']):
            runtime_environment='physical'  # Legacy schema interpretation; retain original bytes and clocks.
        if (runtime_environment!=environment or data.get('simulation_only') is not simulated
                or data.get('robot_source')!=('simulated' if simulated else 'physical') or data.get('robot_id')!=config['robot_id']
                or data.get('asset_geometry_id')!=config['asset_geometry_id']
                or data.get('asset_seal_sha256')!=config['asset_seal_sha256']
                or clock_for_source(data,environment)!=(origin['clock'] if origin else expected_clock)
                or not data.get('frames') or _json(completion).get('passed') is not True):
            raise ValueError(kind+(' must come from a completed current-run fake capture' if simulated
                                  else ' must come from a completed physical capture of this device'))
        batches[kind]=data;pins.update({str(path):_sha(path),str(completion):_sha(completion)})
    contract=records['wrist']['payload']['contract']
    if (contract.get('head_snapshot',{}).get('id')!=records['head']['id']
            or batches['wrist'].get('wrist_contract')!=contract
            or records['wrist']['payload']['row'].get('pair_status')!='complete'):
        raise ValueError('installed wrists must retain this head and current capture contract')
    return batches,pins


def build_fake_profile(config_path, capture_root, template, registration, output, *, expected_install,diagnostic_source=None):
    """Compatibility entry retaining all fake provenance and isolation checks."""
    return build_device_profile(config_path,capture_root,template,registration,output,
        expected_install=expected_install,diagnostic_source=diagnostic_source,environment='fake')


def build_device_profile(config_path, capture_root, template, registration, output, *, expected_install,
                         diagnostic_source=None,environment=None):
    """Create new files only; all arguments identify caller-owned current-run inputs.

    config_path is the resident run's calibration-config.json; its parent is the
    run root. capture_root is this run's Workbench output below that root.
    expected_install has head_id, wrist_id and the actual manifest.json byte SHA.
    registration must explicitly contain length_unit='m' and
    T_environment_calibration_world (the plant base_link registration).
    """
    if diagnostic_source is not None:
        if environment not in (None,'fake'):
            raise ValueError('fake diagnostic provenance cannot authorize physical calibration')
        return _build_diagnostic_profile(config_path,template,registration,output,expected_install,diagnostic_source)
    from a1_cal import build, canonical, read_active
    from integration import _digest
    from model_adapt import digest, verify
    from physical_profile import camera_offsets, rigid
    from robot_workbench.sample_clock import clock_for_source

    config_path=Path(config_path).resolve();run_root=config_path.parent
    capture_root=Path(capture_root).resolve();output=Path(output).resolve()
    config=_json(config_path)
    environment=environment or config.get('environment')
    simulated=environment=='fake'
    if (environment not in ('fake','physical') or config.get('environment')!=environment
            or config.get('simulation_only') is not simulated
            or config.get('robot_source')!=('simulated' if simulated else 'physical')
            or bool(str(config.get('robot_id','')).startswith('fake-'))!=simulated
            or not str(config.get('robot_id','')).strip()):
        raise ValueError('explicit current '+str(environment)+' calibration configuration required')
    if not simulated and config.get('calibration_resume'):
        raise ValueError('fake resume provenance cannot authorize physical calibration')
    capture_clock=clock_for_source(config,environment)
    if not capture_root.is_dir() or simulated and not capture_root.is_relative_to(run_root):
        raise ValueError('capture_root must be the current resident run Workbench output')
    if (not isinstance(expected_install,dict) or set(expected_install)!={'head_id','wrist_id','manifest_sha256'}
            or any(not isinstance(v,str) or not v for v in expected_install.values())):
        raise ValueError('current installed head/wrist identities and manifest SHA required')
    if (not isinstance(registration,dict) or registration.get('length_unit')!='m'
            or 'T_environment_calibration_world' not in registration):
        raise ValueError('explicit T_environment_calibration_world with metre units required')
    transform=rigid(registration['T_environment_calibration_world'])
    root=Path(config['calibration_root'])
    if (not root.is_absolute() or root.is_symlink() or simulated and (not root.resolve().is_relative_to(run_root)
            or root.resolve().is_relative_to(Path('/var/lib/anyverse/calibration').resolve()))):
        raise ValueError('fake installation must be isolated below this current run' if simulated else 'explicit physical calibration root required')
    active=root/'hardware'/'active'  # Legacy C business namespace, never physical authorization.
    active_paths={name:_inside(active/(name+'.json'),run_root if simulated else root) for name in ('head','wrist','manifest')}
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
    records=read_active(active,contract['geometry_id'],asset_seal=config['asset_seal_sha256'],environment=environment)
    if records['manifest']['robot_id']!=config['robot_id']:
        raise ValueError('installed group belongs to another '+environment+' robot')
    pins={str(config_path):_sha(config_path),str(nominal/'SHA256SUMS.json'):digest(nominal/'SHA256SUMS.json')}
    batches,capture_pins=_capture_evidence(config,capture_root,records,expected_install,environment=environment);pins.update(capture_pins)
    wrist_contract=records['wrist']['payload']['contract']
    camera_path=_inside(config['camera'],run_root if simulated else Path(config['camera']).resolve().parent);head_camera=_json(camera_path)
    if head_camera!=batches['head'].get('camera'):
        raise ValueError('head intrinsics differ from the actual current-run capture')
    cameras={'head':head_camera,**wrist_contract['cameras']}
    if {key:value.get('serial') for key,value in cameras.items()}!=config.get('hardware_camera_serials'):
        raise ValueError('installed camera identities differ from the current '+environment+' producer')
    mapping=Path(config['jaw_mapping_file']).resolve()
    if _sha(mapping)!=digest(nominal/'jaw-mapping.json'):
        raise ValueError('runtime jaw mapping differs from nominal A1 evidence')
    pins.update({str(camera_path):_sha(camera_path),str(mapping):_sha(mapping)})
    pins.update({str(path):active_sha[name+'.json'] for name,path in active_paths.items()})
    if output.exists():raise FileExistsError(output)
    if any(output.is_relative_to(path.resolve()) for path in (nominal,capture_root,active)):
        raise ValueError('output must be separate from nominal model, captures and active installation')
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.'+environment+'-calibration-profile-',dir=output.parent) as temporary:
        stage=Path(temporary)/'bridge';stage.mkdir()
        build(nominal,active,head_camera,template,transform,stage/'model',environment=environment)
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
        binding=dict(environment=environment,runtime_environment=environment,simulation_only=simulated,robot_source='simulated' if simulated else 'physical',
            evidence_source='fake_device_rgbd' if simulated else 'physical_rgbd',robot_id=config['robot_id'],camera_serials=config['hardware_camera_serials'],
            calibration_accepted=True,calibration_profile_digest=_digest(profile),
            calibration_snapshot_id=active_sha['manifest.json'],active_sha256=active_sha,
            head_id=records['head']['id'],wrist_id=records['wrist']['id'],model_binding=deepcopy(profile['model_binding']))
        planner=dict(schema=environment+'_calibration_planning_model.v1',environment=environment,simulation_only=simulated,
            urdf=str(output/'model'/'robot.urdf'),urdf_sha256=_sha(stage/'model'/'robot.urdf'),
            bundle=str(output/'model'),bundle_seal_sha256=_sha(stage/'model'/'SHA256SUMS.json'),
            reported_to_model='q_model = q_reported + zero_offsets_rad',joint_zero_offsets_baked_into_urdf=False,
            zero_offsets_rad=profile['joints']['zero_offsets_rad'],camera_offsets=offsets,registration=registration,
            asset_status='URDF_ready_USD_conversion_and_native_asset_binding_required',runtime_allowed=False,
            target_only=True,actual_device_configuration_unchanged=True)
        if simulated:planner['actual_plant_unchanged']=True
        configuration=dict(environment=environment,acquisition=dict(profile_path=str(output/'profile.json'),
            acquisition_binding_path=str(output/'acquisition-binding.json'),measured_initial_state=True),
            planning_model_path=str(output/'planning-model.json'))
        receipt=dict(schema='installed_'+environment+'_calibration_bridge.v1',runtime_environment=environment,simulation_only=simulated,
            robot_source='simulated' if simulated else 'physical',calibration_snapshot_id=binding['calibration_snapshot_id'],
            calibration_capture_clock=capture_clock,
            head_id=records['head']['id'],wrist_id=records['wrist']['id'],active_sha256=active_sha,
            input_sha256=pins,registration=registration,profile_digest=_digest(profile),
            planning_bundle_seal_sha256=planner['bundle_seal_sha256'],runtime_allowed=False,
            qualification='offline configuration only; no calibration accuracy or GPU execution claim')
        legacy={kind:dict(source_batch=records[kind]['source_batch'],environment=data['environment'],
                robot_source=data['robot_source'],simulation_only=data['simulation_only'],robot_id=data['robot_id'],
                runtime_environment_field_present=False,clock_field_present='clock' in data)
                for kind,data in batches.items() if environment=='physical' and 'runtime_environment' not in data}
        if legacy:
            receipt['legacy_capture_schema']=dict(schema='hardware_physical_without_runtime_environment',captures=legacy,
                interpretation='physical from explicit hardware/physical/false provenance and matching robot identity',
                original_datasets_modified=False,original_timestamps_modified=False)
        resume=_resume_receipt(config,run_root) if simulated else None
        if resume:
            origins={kind:deepcopy(resume['capture_origins'].get(str(
                _inside(Path(record['source_batch'])/'dataset.json',capture_root).parent.relative_to(capture_root))))
                for kind,record in records.items() if kind in ('head','wrist')}
            clocks={kind:deepcopy(data['clock']) for kind,data in batches.items()}
            receipt.update(calibration_capture_clock=clocks['head'] if clocks['head']==clocks['wrist'] else None,
                calibration_capture_clocks=clocks,calibration_origins=origins,observation_clock=deepcopy(config['clock']),
                fresh_C_completed=not any(origins.values()),empty_store_qualification=False)
            binding.update(fresh_C_completed=receipt['fresh_C_completed'],calibration_origins=origins)
        for name,value in [('profile',profile),('acquisition-binding',binding),('planning-model',planner),
                           ('configuration',configuration),('bridge',receipt)]:
            (stage/(name+'.json')).write_bytes(canonical(value)+b'\n')
        if any(_sha(path)!=expected for path,expected in pins.items()):
            raise ValueError('a pinned current-run input changed while freezing the bridge')
        verify(nominal);verify(stage/'model');stage.rename(output)
    return dict(output=str(output),profile_path=str(output/'profile.json'),
        acquisition_binding_path=str(output/'acquisition-binding.json'),planning_model_path=str(output/'planning-model.json'),
        calibration_snapshot_id=binding['calibration_snapshot_id'],simulation_only=simulated,runtime_allowed=False)


def load_device_profile(profile_path, *, environment, robot_id=None):
    """CPU-only frozen profile/binding lookup for infer; no USD conversion or warmup."""
    from integration import _digest
    from real_observation import validate_binding
    path=Path(profile_path)
    if path.name!='profile.json' or path.is_symlink():raise ValueError('canonical frozen profile.json required')
    root=path.resolve().parent
    profile=_json(root/'profile.json');binding=_json(root/'acquisition-binding.json');receipt=_json(root/'bridge.json')
    if (receipt.get('runtime_environment')!=environment or receipt.get('simulation_only') is not (environment=='fake')
            or binding.get('calibration_profile_digest')!=_digest(profile)
            or receipt.get('profile_digest')!=_digest(profile)
            or robot_id is not None and binding.get('robot_id')!=robot_id):
        raise ValueError('frozen device profile identity differs')
    for path,expected in receipt['input_sha256'].items():
        if _sha(path)!=expected:raise ValueError('installed profile input changed: '+path)
    validate_binding(profile,binding,environment=environment)
    return profile,binding


def installed_profile_provider(*,config_path,capture_root,registration,profile_template,output,
                               installed_snapshot,environment,diagnostic_source=None):
    """Shared CPU-only C -> Infer profile cache, independent of D/GPU preparation."""
    import secrets
    import threading
    if environment not in ('fake','physical') or not callable(installed_snapshot):
        raise ValueError('explicit device environment and current C installation reader required')
    cached={};lock=threading.Lock()
    def current():
        snapshot=installed_snapshot()
        if not snapshot or snapshot.get('verified') is not True:
            raise ValueError('推理需要正式安装的头部与双腕标定')
        config=_json(config_path)
        if config.get('environment')!=environment:raise ValueError('calibration environment changed')
        manifest=Path(config['calibration_root'])/'hardware/active/manifest.json'
        return config,dict(head_id=snapshot['head_id'],wrist_id=snapshot['wrist_id'],manifest_sha256=_sha(manifest))
    def profile():
        with lock:
            config,expected=current()
            if expected!=cached.get('installation'):
                value=build_device_profile(config_path,capture_root,_json(profile_template),_json(registration),
                    Path(output)/('inference-profile-'+secrets.token_hex(8)),expected_install=expected,
                    environment=environment,diagnostic_source=diagnostic_source)
                if current()[1]!=expected:raise ValueError('calibration changed while building the inference profile')
                cached.update(installation=expected,path=value['profile_path'])
            result=load_device_profile(cached['path'],environment=environment,robot_id=config['robot_id'])
            if current()[1]!=expected:raise ValueError('calibration changed while reading the inference profile')
            return result
    return profile


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


def diagnostic_wrist_cameras(config,records,root):
    """Raw device configs and rectified C contracts have distinct image domains."""
    paths=config.get('hardware_wrist_camera_files')
    if not isinstance(paths,dict) or set(paths)!={'left','right'}:
        raise ValueError('diagnostic source requires both raw wrist camera configuration files')
    cameras={side:_json(_inside(path,root)) for side,path in paths.items()}
    for side,raw in cameras.items():
        # Workbench.plan_wrist and capture_sample retain K/size/serial while
        # rectifying into the same virtual pinhole image domain (capture.py:214).
        rectified={**raw,'rectified':True,'coefficients':[0.]*5,'distortion_model':'distortion.inverse_brown_conrady'}
        if rectified!=records['wrist']['payload']['contract']['cameras'][side]:
            raise ValueError('diagnostic source wrist capture contract differs from normalized raw intrinsics')
    return cameras


def validate_diagnostic_source(spec,base,nominal,markers,output):
    """Explicit fake-only diagnostic import; immutable origin is never relabelled."""
    return _validate_reuse_source(spec,base,nominal,markers,output,mode='warm_diagnostic')


def _validate_reuse_source(spec,base,nominal,markers,output,*,mode):
    from a1_cal import read_active
    from integration import _digest
    from model_adapt import verify
    from robot_workbench.sample_clock import clock_for_source
    if (not isinstance(spec,dict) or set(spec)!={'mode','source_run','source_base_manifest','expected_install','input_sha256'}
            or spec['mode']!=mode or nominal.get('simulation_only') is not True):
        raise ValueError('explicit '+mode+' fake source contract required')
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
    diagnostic_wrist_cameras(config,records,root)
    required=[config_path,base_path,source_nominal_path,seal_path,root/'run.json',root/'plant-fixture.json',root/'truth-fixture.json',
              root/'registration.json',*paths.values(),Path(config['camera'])]
    required.extend(_inside(path,root) for path in config['hardware_wrist_camera_files'].values())
    if config.get('calibration_resume'):required.append(_inside(config['calibration_resume']['path'],root))
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


def _resume_receipt(config,root):
    binding=config.get('calibration_resume')
    if binding is None:return None
    path=_inside(binding['path'],Path(root));value=_json(path)
    if (_sha(path)!=binding['sha256'] or value.get('schema')!='fake_calibration_resume.v1'
            or value.get('mode')!='dev_resume' or value.get('robot_id')!=config['robot_id']
            or value.get('observation_clock')!=config['clock']):
        raise ValueError('calibration resume provenance differs from this fake device')
    return value


def _resume_files(root):
    """Copy only durable C results/batches and device statistics, never execution state."""
    root=Path(root);paths=[]
    for environment in ('hardware','simulation'):
        library=root/'calibrations'/environment
        if library.is_symlink():raise ValueError('resume library must not be symlinked')
        paths.extend(library.glob('v*.json'))
    directories=[root/'calibrations/hardware/active',
        root/'calibrations/diagnostics/arrival-errors/simulated',
        *(root/'calibration-workbench').glob('batch-*')]
    for directory in directories:
        if directory.is_symlink():raise ValueError('resume input directory must not be symlinked')
        if directory.exists():paths.extend(directory.rglob('*'))
    files={}
    for path in sorted(set(paths)):
        if any(parent.is_symlink() for parent in (path,*path.parents)):
            raise ValueError('resume input must not be symlinked')
        if path.is_dir() or path.name=='.lock' or path.suffix=='.tmp':continue
        checked=_inside(path,root)
        files[str(checked.relative_to(root))]=_sha(checked)
    return files


def validate_resume_source(spec,base,nominal,markers,output):
    """Admit one fixed virtual device's history without writing to its source run."""
    source=_validate_reuse_source(spec,base,nominal,markers,output,mode='dev_resume')
    root=Path(source['source_run']);config=_json(root/'calibration-config.json')
    if Path(config['calibration_root'])!=root/'calibrations':
        raise ValueError('resume requires the resident run calibration library')
    files=_resume_files(root)
    if any(source['input_sha256'].get(str(root/path))!=digest for path,digest in files.items()):
        raise ValueError('resume source pin set omits a result, image or arrival statistic')
    previous=_resume_receipt(config,root);origins={}
    for relative in files:
        path=Path(relative)
        if path.parts[0]!='calibration-workbench' or path.name!='dataset.json':continue
        data=_json(root/path);completion=path.with_name('completion.json')
        if str(completion) not in files or _json(root/completion).get('passed') is not True:continue
        batch=str(path.parent.relative_to('calibration-workbench'))
        origin=(previous or {}).get('capture_origins',{}).get(batch)
        if (data.get('robot_id')!=config['robot_id'] or data.get('robot_source')!='simulated'
                or data.get('runtime_environment')!='fake' or data.get('simulation_only') is not True
                or data.get('asset_geometry_id')!=config['asset_geometry_id']
                or data.get('asset_seal_sha256')!=config['asset_seal_sha256']
                or data.get('clock')!=(origin['clock'] if origin else config['clock'])
                or origin and (files[relative]!=origin['dataset_sha256'] or files[str(completion)]!=origin['completion_sha256'])):
            raise ValueError('resume batch has a different device, geometry or capture provenance')
        origins[batch]=dict(source_run=origin['source_run'] if origin else str(root),clock=deepcopy(data['clock']),
            dataset_sha256=files[relative],completion_sha256=files[str(completion)])
    source.update(copy_files=files,capture_origins=origins)
    return source


def restore_calibration(source,config,output):
    """Independent writable copies, unchanged version/capture identities, fresh runtime."""
    from a1_cal import canonical
    from arx_env.joint_head_calibration_store import CalibrationStore
    from arx_env.joint_head_arrival_stats import ArrivalStats
    from integration import _digest
    from robot_workbench.sample_clock import clock_for_source
    root=Path(output).resolve();origin=Path(source['source_run']);old=_json(origin/'calibration-config.json')
    check_diagnostic_pins(source)
    if (source.get('mode')!='dev_resume' or config.get('environment')!='fake' or config.get('simulation_only') is not True
            or config.get('robot_source')!='simulated' or config['robot_id']!=old['robot_id']
            or source['origin_robot_id']!=old['robot_id'] or source['origin_clock']!=old['clock']
            or clock_for_source(config,'fake') in [old['clock'],*[entry['clock'] for entry in source['capture_origins'].values()]]
            or root==origin.resolve() or root.is_relative_to(origin.resolve())
            or Path(config['calibration_root'])!=root/'calibrations'):
        raise ValueError('resume requires the same fake robot, a new clock and an independent output')
    for key in ('urdf','jaw_mapping_file','asset_geometry_id','asset_seal_sha256','hardware_camera_serials'):
        if config[key]!=old[key]:raise ValueError('resume nominal geometry or camera identities differ')
    for current,previous in [(config['camera'],old['camera']),*[(config['hardware_wrist_camera_files'][side],
            old['hardware_wrist_camera_files'][side]) for side in ('left','right')]]:
        if _json(current)!=_json(previous):raise ValueError('resume raw camera intrinsics differ')
    if _json(root/'registration.json')!=_json(origin/'registration.json'):
        raise ValueError('resume base registration differs')
    if _resume_files(origin)!=source['copy_files']:raise ValueError('resume file inventory changed after validation')
    for relative,digest in source['copy_files'].items():
        path=_inside(origin/relative,origin);target=root/relative
        if any(parent.is_symlink() for parent in target.parents):
            raise ValueError('resume destination must not be symlinked')
        if target.exists() or target.is_symlink():raise FileExistsError(target)
        target.parent.mkdir(parents=True,exist_ok=True)
        with path.open('rb') as src,target.open('xb') as dst:shutil.copyfileobj(src,dst)
        if _sha(target)!=digest:raise ValueError('resume copy differs: '+relative)
    for environment in ('hardware','simulation'):
        store=CalibrationStore(root/'calibrations'/environment,config['robot_id'],environment)
        store.list()
        if environment=='hardware' and not (store.active() or {}).get('verified'):
            raise ValueError('resumed installation does not match its copied library')
    stats=root/'calibrations/diagnostics/arrival-errors/simulated'
    # Validate/rebuild only the copy. Original events, identities and clocks remain unchanged.
    if stats.exists():ArrivalStats(stats,config['robot_id'],'simulated')
    check_diagnostic_pins(source)
    receipt=dict(schema='fake_calibration_resume.v1',mode='dev_resume',robot_id=config['robot_id'],
        source_run=str(origin),origin_clock=deepcopy(old['clock']),observation_clock=deepcopy(config['clock']),
        expected_install=deepcopy(source['expected_install']),capture_origins=deepcopy(source['capture_origins']),
        imported_files=deepcopy(source['copy_files']),fixture_identity_sha256=source['fixture_identity_sha256'],
        admission_evidence_set_sha256=_digest(source['input_sha256']),fresh_C_completed=False,
        arrival_statistics='same virtual device cumulative simulated events; original event metadata retained',
        qualification='reused fake calibration for editable development; not an empty-store qualification')
    path=root/'calibration-resume.json'
    with path.open('xb') as stream:stream.write(canonical(receipt)+b'\n')
    return dict(path=str(path),sha256=_sha(path))


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
    origin_cameras=diagnostic_wrist_cameras(old,records,origin)
    for side in ('left','right'):
        camera=_json(current['hardware_wrist_camera_files'][side])
        if camera!=origin_cameras[side]:
            raise ValueError('new diagnostic raw wrist intrinsics differ from the pinned source device')
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
        for path in old['hardware_wrist_camera_files'].values():
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
        details={'diagnostic_origin':diagnostic_metadata(source),
                 **({'head_result_id':contract['head_snapshot']['id']} if kind=='wrist' else {})}) for kind,row in rows.items()]
    return view,candidates


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('config','capture-root','template','registration','expected-install','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(build_fake_profile(args.config,args.capture_root,_json(args.template),_json(args.registration),
        args.output,expected_install=_json(args.expected_install)),indent=2))


if __name__=='__main__':main()
