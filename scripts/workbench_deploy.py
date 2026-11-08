#!/usr/bin/env python3
"""Seal an installed physical workbench, then start that exact release without SDK acquisition.

Only user systemd units declared by the release are operated. No rebuild, pip,
CAN reset, SDK acquisition, reboot or forced process kill. Explicit collection
handoff uses its normal stop path, which may return the robot home.
"""
import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import secrets
import shlex
import socket
import stat
import subprocess
import sys
import time
from types import SimpleNamespace
from urllib.parse import urlsplit
from urllib.request import build_opener, ProxyHandler, HTTPRedirectHandler, Request

SCHEMA = 'workbench.startup-release.v1'
ORDER = ('control_owner', 'cameras', 'workbench', 'manager')
PROPERTIES = 'LoadState,ActiveState,SubState,MainPID,ControlPID,ControlGroup,FragmentPath,DropInPaths,NeedDaemonReload,SendSIGKILL'


class Blocked(RuntimeError):
    pass


def service_order(manifest):
    return (('recording',) if 'recording' in manifest['services'] else ()) + ORDER[:2] + (('pedal',) if 'pedal' in manifest['services'] else ()) + (('model',) if 'model' in manifest['services']and manifest.get('model_lifecycle','managed')!='external' else ()) + ORDER[2:-1] + (('triton_control',) if 'triton_control' in manifest['services'] else ()) + ORDER[-1:]


def require(ok, message):
    if not ok:
        raise Blocked(message)


def read_json(path):
    path = Path(path)
    require(path.is_file() and path.stat().st_size < 8*1024**2, 'missing or oversized JSON: '+str(path))
    return json.loads(path.read_text())


def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def private_write(path, value):
    path = Path(path)
    with os.fdopen(os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'w') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def inside(path, root):
    p = Path(path)
    require(p.is_absolute() and not p.is_symlink() and p.resolve().is_relative_to(Path(root).resolve()), 'path outside release root: '+str(p))
    return p


def unit_state(name):
    require(re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*\.service', name), 'invalid unit name')
    p = subprocess.run(['systemctl', '--user', 'show', name, '--property='+PROPERTIES],
                       capture_output=True, text=True, timeout=10)
    require(p.returncode == 0, 'cannot inspect unit: '+name)
    row = dict(line.split('=', 1) for line in p.stdout.splitlines() if '=' in line)
    require(row.get('LoadState') == 'loaded' and row.get('MainPID', '').isdigit(), 'unit unavailable: '+name)
    return row


def unit_pin(name):
    row = unit_state(name)
    paths = [row['FragmentPath'], *shlex.split(row.get('DropInPaths', ''))]
    require(paths[0] and all(Path(p).is_absolute() and Path(p).is_file() for p in paths), 'unit files unavailable: '+name)
    return dict(unit=name, files={p: sha(p) for p in paths})


def verify_unit(pin):
    current = unit_pin(pin['unit'])
    require(current == pin, 'unit or drop-in changed; reseal the qualified release: '+pin['unit'])
    row = unit_state(pin['unit'])
    require(row.get('NeedDaemonReload') != 'yes', 'unit is not loaded from current disk configuration: '+pin['unit'])
    return row


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args):
        return None


def http_json(url, body=None, headers=None):
    require(urlsplit(url).scheme == 'http', 'only the registered HTTP endpoint is supported')
    request=url if body is None else Request(url,data=json.dumps(body).encode(),headers={'Content-Type':'application/json',**(headers or {})})
    with build_opener(ProxyHandler({}), NoRedirect()).open(request, timeout=4) as response:
        raw = response.read(8*1024**2+1)
    require(len(raw) <= 8*1024**2, 'HTTP response exceeds bound')
    return json.loads(raw)


def handoff_collection(pin, device_id, *, stop_active=False, progress=lambda *a,**k:None, timeout=240):
    """Use the registered collection shutdown; never kill a PID or edit its environment."""
    require(isinstance(pin,dict) and set(pin)=={'unit','files','status_url'},'registered collection unit/files/status URL required')
    url=urlsplit(pin['status_url'])
    require(url.scheme=='http' and url.hostname=='127.0.0.1' and url.port and url.path=='/api/status'
            and not url.query and not url.fragment and not url.username,'collection status must be the registered loopback API')
    before=verify_unit({k:pin[k]for k in ('unit','files')})
    if before['ActiveState']=='inactive' and before['MainPID']=='0':
        return dict(unit=pin['unit'],status='already_stopped',exit_verified=True,stop_requested=False)
    require(before['ActiveState']=='active' and before['ControlPID']=='0','collection service state is uncertain')
    require(before.get('SendSIGKILL')=='no','collection Web stop must not force-kill surviving hardware children')
    def observe():
        value=http_json(pin['status_url'])
        require(value.get('robot_vin')==device_id,'collection API belongs to another robot')
        require(value.get('orphan_alive') is False and value.get('teleop') is None,'collection orphan/teleoperation must be resolved by its owner')
        require(type(value.get('manager_alive'))is bool,'collection manager state is unknown')
        return value
    value=observe();session=(value.get('collection')or{}).get('session_id');requested=False
    if value['manager_alive']:
        require(stop_active,'collection is active; normal stop is required before startup')
        progress('stopping_collection',detail=dict(session_id=session,may_return_home=True))
        http_json(pin['status_url'].removesuffix('/api/status')+'/api/collect/stop',{})
        requested=True;deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            progress('waiting_collection_exit')
            value=observe()
            require((value.get('collection')or{}).get('session_id')in(None,session),'collection session changed during handoff')
            settled=(not value['manager_alive'] and value.get('collect_status')in('idle','stopped')
                and (not value.get('collection') or value['collection'].get('phase')=='stopped' and value['collection'].get('safe_ok')is True))
            if settled:break
            require(value.get('collect_status')not in('stop_timeout','failed','error'),'collection reported failed shutdown; no SDK acquire')
            time.sleep(.25)
        else:raise Blocked('collection stop outcome unconfirmed; no Web stop or SDK acquire')
    require(value.get('collect_status')in('idle','stopped'),'collection has not completed normal shutdown')
    if value.get('collection'):
        require(value['collection'].get('safe_ok') is True and value['collection'].get('phase')=='stopped',
                'collection hardware shutdown not verified; no SDK acquire')
    current=verify_unit({k:pin[k]for k in ('unit','files')})
    require(current['MainPID']==before['MainPID'] and current['ActiveState']=='active','collection Web identity changed')
    progress('stopping_collection_web')
    subprocess.run(['systemctl','--user','stop','--no-block',pin['unit']],check=True,timeout=10)
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        progress('waiting_collection_web_exit')
        stopped=unit_state(pin['unit'])
        if stopped['MainPID']=='0' and stopped['ActiveState']=='inactive':break
        time.sleep(.25)
    else:raise Blocked('collection Web exit unconfirmed; no force kill or SDK acquire')
    group=before.get('ControlGroup')
    require(isinstance(group,str)and group.startswith('/')and'..'not in Path(group).parts,'collection cgroup identity missing')
    processes=Path('/sys/fs/cgroup')/group.lstrip('/')/'cgroup.procs'
    require(not processes.exists()or not processes.read_text().strip(),'collection children remain; no SDK acquire')
    return dict(unit=pin['unit'],status='stopped',exit_verified=True,stop_requested=requested,
                collection_session_id=session,normal_hardware_stop=(value.get('collection')or{}).get('safe_ok'),
                save_ok=(value.get('collection')or{}).get('save_ok'),delete_api_called=False,
                data_policy='existing collection stop/finalize; inspect its own save receipt',may_have_returned_home=requested)


def restore_collection_web(pin,device_id,*,progress=lambda *a,**k:None,timeout=40):
    """Restore the exact previously registered webpage; never start a collection."""
    row=verify_unit({k:pin[k]for k in ('unit','files')})
    require(row['ControlPID']=='0'and row['ActiveState']in ('active','inactive')and row['MainPID'].isdigit(),
            'collection Web state is unknown; not restarted')
    started=row['ActiveState']=='inactive'
    if started:
        require(row['MainPID']=='0','collection Web PID remains')
        group=row.get('ControlGroup')
        if group:
            p=Path('/sys/fs/cgroup')/group.lstrip('/')/'cgroup.procs'
            require(not p.exists()or not p.read_text().strip(),'collection children remain; Web not started')
        progress('restoring_collection_web')
        subprocess.run(['systemctl','--user','start','--no-block',pin['unit']],check=True,timeout=10)
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        progress('waiting_collection_web')
        try:
            row=verify_unit({k:pin[k]for k in ('unit','files')});value=http_json(pin['status_url'])
            if row['ActiveState']=='active'and int(row['MainPID'])>0:
                require(value.get('robot_vin')==device_id,'restored collection Web belongs to another robot')
                require(value.get('orphan_alive')is False,'collection has an unresolved orphan; inspect its own interface')
                return dict(unit=pin['unit'],started=started,web_pid=int(row['MainPID']),ready=True,
                    collection_started=False,manager_alive=value.get('manager_alive'),collect_status=value.get('collect_status'))
        except OSError:pass
        time.sleep(.25)
    raise Blocked('collection Web readiness unconfirmed; no automatic retry or collection start')


def api(manifest, path):
    return http_json(manifest['url']+path)


def dependencies(path, root, files):
    """Pin declared static file inputs; never traverse output roots or credentials."""
    root=Path(root).resolve()
    path=Path(path)
    if path.is_symlink():
        require(path.resolve().is_relative_to(root),'dependency symlink escapes release root: '+str(path))
        files[str(path)]=sha(path)
        dependencies(path.resolve(),root,files)
        return
    path = inside(path, root)
    if str(path) in files:
        return
    require(path.is_file(), 'missing deployment input: '+str(path))
    files[str(path)] = sha(path)
    if path.suffix != '.json':
        return
    value = read_json(path)
    def walk(value, key=''):
        if key == 'backend_attempt_pointer':
            return  # Launcher-owned current-attempt receipt changes after every restart.
        if any(word in key.lower() for word in ('token', 'secret', 'password', 'descriptor', 'state_root', 'output', 'identity_path')):
            return
        if isinstance(value, dict):
            for k, v in value.items():
                walk(v, k)
        elif isinstance(value, list):
            for v in value:
                walk(v, key)
        elif isinstance(value, str) and value.startswith('/'):
            p = Path(value)
            # Directories are intentionally not a recursive search surface.
            if p.is_file() and p.resolve().is_relative_to(root):
                dependencies(p, root, files)
    walk(value)


def command_inputs(argv):
    """ExecStart --output names a generated receipt, not a release input."""
    skip=False
    live_owner=any(value.endswith('/serve_prometheus_rgb.py')for value in argv)
    live_state=any(value.endswith('/camera_owner.py')for value in argv)
    for value in argv:
        if skip:skip=False;continue
        if value=='--output' or live_owner and value=='--owner' or live_state and value=='--state':skip=True;continue
        if value.startswith('--output='):continue
        if live_owner and value.startswith('--owner='):continue
        if live_state and value.startswith('--state='):continue
        if value.startswith('/') and Path(value).is_file() and Path(value).suffix in ('.py','.json'):
            yield value


def seal_model_service(manager, business):
    """Seal either existing managed model adapter with the same endpoint binding."""
    from robot_workbench.docker_model_service import DockerModelSpec,option
    spec=manager['services']['model']
    require(manager.get('model_port') and business.get('inference'),'model port and inference business must be registered together')
    require(urlsplit(business['inference']['model_endpoint']).port==manager['model_port'],'model endpoint/manager port differ')
    if spec.get('kind')=='docker_model':
        DockerModelSpec(name='model',**spec);sealed=dict(spec);command=spec['command']
    else:
        from robot_workbench.managed_services import ServiceSpec
        ServiceSpec(name='model',**spec)
        require(spec['kind']=='user_unit','model must use a registered managed adapter')
        sealed=dict(spec,**unit_pin(spec['unit']))
        unit_text='\n'.join(Path(p).read_text() for p in sealed['files'])
        commands=[shlex.split(line.split('=',1)[1]) for line in unit_text.splitlines()if line.startswith('ExecStart=')]
        require(len(commands)==1 and spec['expected_script']in commands[0],'model unit entrypoint differs')
        command=commands[0]
        require(option(command,'--port')==[str(manager['model_port'])],'model unit port differs')
    require(option(command,'--token-file')==[business['inference']['model_token_file']], 'model authorization binding differs from inference business')
    return sealed


def camera_descriptor(manager, business):
    descriptor=business.get('camera_service_descriptor') or manager.get('camera_descriptor')
    if descriptor:return descriptor
    native=manager.get('native_rgb') or (business.get('inference')or{}).get('native_rgb')
    return str(Path(native['directory'])/'recording-camera-service.json') if native else None


def seal(args):
    root = Path(args.root).resolve()
    manager_path = inside(args.manager_profile, root)
    manager = read_json(manager_path)
    require(manager.get('environment') == 'physical', 'physical manager profile required')
    address = urlsplit(args.url)
    require(address.scheme == 'http' and address.hostname == manager['robot_address'] and address.port
            and not address.path.strip('/') and not address.query and not address.username and not address.fragment,
            'URL must match the declared machine address')
    require(ipaddress.ip_address(address.hostname).is_private, 'private machine address required')
    pins = manager['publication_files']
    physical = read_json(pins['control_owner_profile']); business = read_json(pins['physical_profile'])
    require(physical['robot_id'] == business['robot_id'] == manager['device_id'], 'device IDs disagree')
    for key in ('can_interfaces','cameras','jaw_mapping_file','sdk_quiesce','native_gripper_calibration',
                'dm_joint_velocity_contract','dm_torque_contract','motion_gain_profile','target_interpreter_profile','native_reference_api','pedal_stop_key_code'):
        require(physical.get(key) == business.get(key), 'Owner/business binding differs: '+key)
    require(business.get('control_owner_descriptor') == manager.get('control_owner_descriptor'), 'Owner descriptor mismatch')
    require(business.get('calibration_config') and manager.get('target_model_manifest'), 'manual planning and target display must be deployed before sealing')
    require(not physical.get('acquisition_blocked_reason'), 'device commissioning is still blocked')
    require(physical.get('motion_gain_profile') == 'normal_X5', 'declare qualified baseline motion gains explicitly')
    services = {}
    for role in tuple(name for name in ('recording','pedal')if name in manager['services'])+ORDER[:-1]:
        spec = manager['services'][role]
        require(spec['kind'] == 'user_unit', 'startup supports installed user units only')
        services[role] = dict(spec, **unit_pin(spec['unit']))
    manager_pin = unit_pin(args.manager_unit)
    text = Path(next(iter(manager_pin['files']))).read_text()
    lines = [shlex.split(line.removeprefix('ExecStart=')) for line in text.splitlines() if line.startswith('ExecStart=')]
    require(len(lines) == 1 and str(manager_path) in lines[0], 'manager unit does not use selected manager profile')
    argv = lines[0]
    script = next((p for p in argv if p.endswith('/serve_workbench_manager.py')), None)
    require(script is not None, 'unexpected manager entrypoint')
    source = inside(Path(script).parent.parent, root)
    sys.path.insert(0, str(source))
    from robot_workbench.ros_data import plane_config
    try:
        require(plane_config(physical.get('data_plane'))==plane_config(business.get('data_plane')),
                'Owner/business ROS data_plane binding differs')
    except ValueError as error:raise Blocked('explicit matching ROS data_plane required before sealing')from error
    services['manager'] = dict(kind='user_unit', expected_script=script,
                               identity_path=str(Path(args.output).with_suffix('.manager-identity.json')), **manager_pin)
    triton = manager.get('triton_preview')
    if triton:
        from robot_workbench.triton_preview import validate_config
        validate_config(triton)
        require(getattr(args,'triton_unit',None), 'configured Triton CPU service needs --triton-unit in startup manifest')
        pin=unit_pin(args.triton_unit)
        unit_text='\n'.join(Path(p).read_text() for p in pin['files'])
        starts=[shlex.split(line.split('=',1)[1]) for line in unit_text.splitlines() if line.startswith('ExecStart=')]
        require(len(starts)==1, 'one explicit Triton CPU entrypoint required')
        argv=starts[0];entry=next((p for p in argv if p.endswith('/serve_triton_control.py')),None)
        require(entry and '--config' in argv and '--port' in argv, 'unexpected Triton CPU entrypoint')
        require(int(argv[argv.index('--port')+1])==urlsplit(triton['endpoint']).port, 'Triton port differs from manager')
        config=read_json(inside(argv[argv.index('--config')+1],root))
        require(str(Path(config['workspace'])/'triton-control'/'token')==triton['token_file'], 'Triton credential binding differs from manager')
        services['triton_control']=dict(kind='user_unit',expected_script=entry,
            identity_path=str(Path(args.output).with_suffix('.triton-identity.json')),**pin)
    else:
        require(not getattr(args,'triton_unit',None), 'Triton unit supplied without manager binding')
    if 'model' in manager['services']:
        services['model']=seal_model_service(manager,business)
    require(len({s['unit'] for s in services.values()if 'unit'in s}) == len([s for s in services.values()if 'unit'in s]), 'distinct component units required')
    for role, spec in services.items():
        inside(spec['expected_script'], root)
        if spec['kind']=='docker_model':continue  # Its adapter validates immutable image/CID and isolation.
        unit_text = '\n'.join(Path(p).read_text() for p in spec['files'])
        isolation=('NoNewPrivileges=yes','SendSIGKILL=no','Restart=no')
        isolation+=('Environment=CUDA_VISIBLE_DEVICES=',) if role=='triton_control' else ('ProtectSystem=strict','ProtectHome=read-only','ReadWritePaths='+str(root))
        for required in isolation:
            require(required in unit_text, 'isolation setting missing for '+role+': '+required)
        starts=[shlex.split(line.split('=',1)[1]) for line in unit_text.splitlines() if line.startswith('ExecStart=')]
        require(len(starts)==1 and spec['expected_script'] in starts[0], 'unit entrypoint does not match registry: '+role)
    owner_text = '\n'.join(Path(p).read_text() for p in services['control_owner']['files'])
    require('AMENT_PREFIX_PATH=' in owner_text and 'ExecStartPre=' in owner_text, 'Owner needs CPU solver preflight and AMENT environment')
    from robot_workbench.machine_profile import Deployment
    wrapped = {k: SimpleNamespace(spec=SimpleNamespace(expected_script=v['expected_script'])) for k, v in services.items()}
    deployment = Deployment(source, wrapped); deployment.bind_configuration(manager)
    files = {str(p): digest for p, digest in deployment.files.values()}
    require(all(files.values()), 'source files unavailable')
    # Rewalk JSON pins even if Deployment already captured their direct digest.
    static = {}
    dependencies(manager_path, root, static)
    dependencies(pins['control_owner_profile'], root, static)
    dependencies(pins['physical_profile'], root, static)
    for spec in services.values():
        if spec['kind']=='docker_model':
            for v in command_inputs(spec['command']):
                if Path(v).resolve().is_relative_to(root):dependencies(v,root,static)
        for p in spec.get('files',{}):
            for line in Path(p).read_text().splitlines():
                if line.startswith(('ExecStart=', 'ExecStartPre=')):
                    for v in command_inputs(shlex.split(line.split('=',1)[1])):
                        dependencies(v, root, static)
    # Solver resources are loaded indirectly through ament and native factories.
    for line in owner_text.splitlines():
        if line.startswith('Environment='):
            for item in shlex.split(line.split('=',1)[1]):
                if item.startswith('AMENT_PREFIX_PATH='):
                    for prefix in item.split('=',1)[1].split(':'):
                        for p in inside(prefix,root).joinpath('share').rglob('*'):
                            if p.is_file():dependencies(p,root,static)
    sdk_models=Path(physical['sdk_quiesce']['module']['path']).parent.parent/'models'
    for p in sdk_models.rglob('*'):
        if p.is_file():dependencies(p,root,static)
    model = Path(manager['target_model_manifest']).parent
    for name in ('model.json', 'model.bin.gz'):
        dependencies(model/name, root, static)
    import xml.etree.ElementTree as ET
    # Solver-only URDFs need link/inertial data, not their unused visual meshes.
    # The display/path-check URDF, however, must have every referenced mesh.
    geometry=read_json(business['calibration_config'])
    urdf=geometry['urdf']
    for mesh in ET.parse(urdf).iter('mesh'):
        name = Path(mesh.get('filename'))
        require(not name.is_absolute() and '..' not in name.parts, 'URDF mesh escapes asset root')
        dependencies(Path(urdf).parent/name, root, static)
    files.update(static)
    tool=inside(Path(__file__).resolve(),root)
    files[str(tool)]=sha(tool)
    launcher=inside(root/'start-workbench.sh',root)
    require(launcher.is_file(),'install start-workbench.sh before sealing')
    files[str(launcher)]=sha(launcher)
    conflicts = []
    if args.collection_unit:
        require(args.collection_status_url and urlsplit(args.collection_status_url).hostname == '127.0.0.1', 'loopback collection status endpoint required')
        conflicts.append(dict(**unit_pin(args.collection_unit), status_url=args.collection_status_url))
    require(physical.get('exclusive_idle_units',[]) == [c['unit'] for c in conflicts], 'collection handoff registry does not match Owner')
    manifest = dict(schema=SCHEMA, device_id=manager['device_id'], uid=os.getuid(), machine_id=Path('/etc/machine-id').read_text().strip(),
        root=str(root), source=str(source), url=args.url.rstrip('/'), manager_profile=str(manager_path),
        owner_descriptor=manager['control_owner_descriptor'], camera_descriptor=camera_descriptor(manager,business),
        can_interfaces=physical['can_interfaces'], cameras=physical['cameras'], services=services, conflicts=conflicts,
        ports={'manager':[address.hostname,address.port], 'workbench':['127.0.0.1',manager['backend_port']]},
        files=files, created_wall_ns=time.time_ns(), startup_acquires_sdk=False,
        scope='service startup only; current robot fault/gains/calibration are not certified by sealing')
    if triton:
        manifest['triton_preview']=triton
        manifest['ports']['triton_control']=['127.0.0.1',urlsplit(triton['endpoint']).port]
    if 'model'in services:
        manifest['model_port']=manager['model_port']
        require(manager.get('model_lifecycle','managed')in ('managed','external'),'known model lifecycle required')
        manifest['model_lifecycle']=manager.get('model_lifecycle','managed')
        manifest['ports']['model']=['127.0.0.1',manager['model_port']]
    output = inside(args.output, root); output.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    private_write(output, manifest)
    return dict(sealed=True, manifest=str(output), pinned_files=len(files), units=[s['unit'] for s in services.values()if 'unit'in s],model_container=(services.get('model')or{}).get('container_name'),robot_commands=0)


def load(path):
    p=Path(path);info=p.lstat()
    require(stat.S_ISREG(info.st_mode) and info.st_uid==os.getuid() and not info.st_mode&0o077, 'private owned startup manifest required')
    m=read_json(p)
    require(read_json(m['manager_profile']).get('control_lifecycle','managed')!='external',
        'external control consumers use the registered software launcher, not device-administrator deployment startup')
    require(m.get('schema')==SCHEMA and m.get('uid')==os.getuid() and m.get('startup_acquires_sdk') is False, 'startup contract differs')
    require(m['machine_id']==Path('/etc/machine-id').read_text().strip(), 'release belongs to a different machine')
    require(set(m['services'])==set(ORDER)|({name for name in ('recording','pedal')if name in read_json(m['manager_profile'])['services']})|({'triton_control'} if m.get('triton_preview') else set())|({'model'}if m.get('model_port')else set()), 'complete declared service startup required')
    require(m.get('model_lifecycle','managed')in ('managed','external')and
        m.get('model_lifecycle','managed')==read_json(m['manager_profile']).get('model_lifecycle','managed'),
        'startup model lifecycle differs from the manager profile; reseal the deployment')
    return m


def services(m):
    sys.path.insert(0,m['source'])
    from robot_workbench.managed_services import ManagedService, ServiceSpec
    def make(role,spec):
        if spec['kind']=='docker_model':
            from robot_workbench.docker_model_service import DockerModelSpec,DockerModelService
            return DockerModelService(DockerModelSpec(name=role,**spec))
        return ManagedService(ServiceSpec(name=role,kind='user_unit',unit=spec['unit'],expected_script=spec['expected_script'],identity_path=spec['identity_path'],start_timeout_s=30))
    return {role:make(role,spec)for role,spec in m['services'].items()}


def preflight(m):
    changed=[p for p,digest in m['files'].items() if not Path(p).is_file()
             or not Path(p).resolve().is_relative_to(Path(m['root']).resolve()) or sha(p)!=digest]
    require(not changed, 'sealed files changed/missing; no start: '+', '.join(changed[:5]))
    for pin in [*m['services'].values(),*m['conflicts']]:
        if pin.get('kind')!='docker_model':verify_unit({k:pin[k] for k in ('unit','files')})
    for name in m['can_interfaces'].values():
        p=Path('/sys/class/net')/name
        require(p.is_dir() and (p/'type').read_text().strip()=='280', 'missing/non-CAN interface: '+name)
        require(int((p/'flags').read_text().strip(),16)&1, 'CAN interface is down: '+name)
    available=os.statvfs(m['root']);require(available.f_bavail*available.f_frsize>=512*1024**2, 'less than 512 MiB available for service startup; capture has separate space checks')
    return services(m)


def runtime_summary(s):
    r=s['runtime']
    return {k:r.get(k) for k in ('connected','state','runtime_nonce','holder','command_epoch','automatic_enabled','feedback_healthy','stationary')}


def observe(m):
    s=api(m,'/api/state?environment=physical&business=research')
    require(s.get('device',{}).get('id')==m['device_id'], 'HTTP device identity differs')
    e=api(m,'/api/environment?environment=physical&view=summary')
    require(e.get('deployment',{}).get('files_unchanged') is True, 'manager publication drift')
    media={c['id']:c for c in s.get('media',[])}
    cameras=all(media.get(k,{}).get('ready') is True and
        0 <= (s['server_wall_ns']-media[k].get('source_wall_ns',0))/1e9 < 2 for k in ('head','left','right'))
    from robot_workbench.state_contract import control_healthy
    triton_ready=True
    if m.get('triton_preview'):
        from robot_workbench.triton_preview import TritonPreviewClient
        triton_ready=TritonPreviewClient(m['triton_preview']).catalog().get('available') is True
    return dict(runtime=runtime_summary(s), control_ready=control_healthy(s['runtime']),backend_instance=s['server_instance_id'], cameras_ready=cameras,
        triton_ready=triton_ready,
        model_configured=bool(m.get('model_port')),model_ready=(e.get('model')or{}).get('ready') is True,
        target_model_ready=bool(s.get('preview',{}).get('webgl_model')),
        manual_ready=len(s.get('manual',{}).get('limits',[]))==12,
        active_job=bool(e.get('active_job')),maintenance=bool(e.get('maintenance')),
        business_busy=bool(e.get('activity',{}).get('busy')))


def old_owner_files(m, apply=False):
    """Only prior-boot socket/token remnants are quarantined, never same-boot faults."""
    descriptor=Path(m['owner_descriptor']);root=descriptor.parent
    leftovers=[p for p in (root/'control.sock',root/'control-token') if p.exists() or p.is_symlink()]
    if not leftovers:return []
    d=read_json(descriptor);identity=d.get('identity',{})
    require(d.get('device_id')==m['device_id'] and identity.get('uid')==os.getuid()
            and identity.get('boot_id') and identity['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'same-boot or unknown Owner remnants: use explicit maintenance, not startup cleanup')
    require(d.get('socket_path')==str(root/'control.sock') and d.get('token_file')==str(root/'control-token'), 'Owner descriptor paths differ')
    for p in [descriptor,*leftovers]:
        st=p.lstat();require(not p.is_symlink() and st.st_uid==os.getuid() and not st.st_mode&0o077,'unsafe stale Owner file')
    if apply:
        require(unit_state(m['services']['control_owner']['unit'])['MainPID']=='0','Owner started during cleanup')
        for p in Path('/proc').iterdir():
            if not p.name.isdigit():continue
            try:argv=(p/'cmdline').read_bytes().split(b'\0')
            except FileNotFoundError:continue
            except PermissionError:continue
            require(not (os.fsencode(m['services']['control_owner']['expected_script']) in argv and os.fsencode(root) in argv), 'unmanaged Owner already uses this root')
        dest=root/('prior-boot-'+str(time.time_ns()));dest.mkdir(mode=0o700)
        for p in [*leftovers,descriptor]:p.rename(dest/p.name)
    return [p.name for p in leftovers]


@contextmanager
def startup_lock(path):
    fd=os.open(path,os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW,0o600)
    try:
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        yield
    finally:os.close(fd)


def warm_model(m, timeout=360, expected_model=None, expected_backend=None):
    """Use the normal reviewed manager operation; never bind or execute robot actions."""
    deadline=time.monotonic()+min(30,timeout)
    while time.monotonic()<deadline:
        current=api(m,'/api/environment?environment=physical&view=summary')
        observed_model=current.get('target',{}).get('services',{}).get('model')
        if (current.get('health',{}).get('backend_reachable')is True
                and (expected_model is None or observed_model==expected_model)
                and (expected_backend is None or current.get('target',{}).get('backend_instance')==expected_backend)):break
        time.sleep(.25)
    else:raise Blocked('manager has not observed the current backend/model identity; no model preparation submitted')
    if (current.get('model')or{}).get('ready')is True:return dict(reused=True)
    operation=current.get('operations',{}).get('prepare_model')or{}
    require(operation.get('allowed')is True,operation.get('disabled_reason')or'model prepare is unavailable')
    result=http_json(m['url']+'/api/operations/action',dict(action='prepare_model',request_id='startup-model-'+secrets.token_hex(12),
        expected_target=current['target'],arms_supported=False,parameters={}),headers={
        'Origin':m['url'],'X-CSRF-Token':current['csrf'],'X-Workbench-Deployment':current['deployment']['sha256']})
    job_id=result['job_id'];deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        job=api(m,'/api/operations/jobs/'+job_id+'?environment=physical')
        if job['status']not in ('queued','running','cancel_requested'):
            require(job['status']=='succeeded'and(job.get('verification')or{}).get('ok')is True,
                    'model preparation failed/unconfirmed: '+json.dumps(job.get('error')or{}))
            return dict(job_id=job_id,reused=False)
        time.sleep(.25)
    raise Blocked('model preparation deadline exceeded; inspect job '+job_id+'; not resubmitted')


def start(m, managed, *, handoff=False, timeout=40):
    actions=[];paused=[];handoffs=[]
    before={k:v.snapshot() for k,v in managed.items()}
    if m.get('model_lifecycle')=='external':
        require(before.get('model',{}).get('state')=='running'and before['model'].get('owned')is True,
                'external model must be started by its service administrator; no consumer lifecycle action performed')
    require(all(v['state'] in ('running','stopped') for v in before.values()), 'unknown/transitioning service identity; inspect before retry')
    for role,(host,port) in m.get('ports',{}).items():
        if before[role]['state']=='stopped':
            with socket.socket() as probe:
                probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                probe.bind((host,port))  # Fail before changing anything if another service owns it.
    live=observe(m) if all(before[k]['state']=='running' for k in ORDER) else None
    owner_before=None
    if live is None and before['control_owner']['state']=='running':
        from robot_workbench.control_owner import ControlChannel
        channel=ControlChannel(m['owner_descriptor'],timeout=2)
        owner_before=channel.call('inspect')
        if owner_before.get('connected'):
            owner_before=channel.call('snapshot',expected_nonce=owner_before['runtime_nonce'],business='research')
            require(owner_before.get('feedback_healthy')is True and owner_before.get('stationary')is True
                    and not owner_before.get('automatic_enabled'),'existing Owner is not healthy idle; startup will not interfere')
    if live:
        require(not live['active_job'] and not live['maintenance'] and not live['business_busy'], 'business/maintenance active; startup will not interfere')
    if before['control_owner']['state']=='stopped':old_owner_files(m)  # Validate before any collection handoff.
    for conflict in m['conflicts']:
        row=unit_state(conflict['unit'])
        if row['ActiveState']=='inactive' and row['MainPID']=='0':continue
        require(row['ActiveState']=='active' and row['ControlPID']=='0', 'collection service state is uncertain')
        require(handoff, 'collection webpage active; stop collection then explicitly use --handoff-collection')
        require(not live or live['runtime']['connected'] is False and live['runtime']['state']=='DISCONNECTED', 'control is connected/faulted; do not change collection ownership')
        handoffs.append(handoff_collection(conflict,m['device_id'],stop_active=True))
        paused.append(conflict['unit']);actions.append('paused '+conflict['unit'])
    if before['control_owner']['state']=='stopped':old_owner_files(m,apply=True)
    for role in service_order(m):
        current=managed[role].snapshot()
        if current['state']=='running':continue
        require(current['state']=='stopped','service state changed: '+role)
        result=managed[role].start()
        actions.append('started '+role)
        require(result['state']=='running' and result.get('owned'), 'service startup not confirmed: '+role)
        if role=='recording':
            from robot_workbench.recording_process import recording_call
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                try:
                    recorded=recording_call(os.environ['WORKBENCH_RECORDING_SERVICE'],'inspect',timeout=2)
                    require(recorded['ready'] and recorded['pid']==result['identity']['pid'],'old recording descriptor')
                    break
                except (KeyError,OSError,ValueError,Blocked):time.sleep(.1)
            else:raise Blocked('resident recording readiness not confirmed')
        if role=='control_owner':
            # The only IPC here is a read, never acquire/recover/stop/submit.
            from robot_workbench.control_owner import ControlChannel
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                try:
                    descriptor=read_json(m['owner_descriptor'])
                    require(descriptor.get('identity',{}).get('pid')==result['identity']['pid'],'old Owner descriptor')
                    r=ControlChannel(m['owner_descriptor'],timeout=2).call('inspect')
                    break
                except (OSError,ValueError,Blocked):time.sleep(.2)
            else:raise Blocked('Owner IPC readiness not confirmed')
            require(r.get('connected') is False, 'new Owner unexpectedly acquired control')
        if role=='cameras' and m.get('camera_descriptor'):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                try:
                    d=read_json(m['camera_descriptor']);require(d.get('parent_pid')==result['identity']['pid'],'old camera descriptor')
                    break
                except (OSError,ValueError,Blocked):time.sleep(.2)
            else:raise Blocked('camera source has not published its own descriptor')
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        try:
            after=observe(m)
            if after['cameras_ready'] and after['target_model_ready'] and after['manual_ready'] and after.get('triton_ready',True):break
        except (OSError,ValueError,Blocked):pass
        time.sleep(.25)
    else:raise Blocked('UI/cameras/model/manual/Triton tools did not become ready; no automatic restart or rollback')
    model_preparation=None
    if m.get('model_port'):
        from robot_workbench.management import stable_identity
        model_preparation=warm_model(m,expected_model=stable_identity(managed['model'].snapshot().get('identity')),expected_backend=after['backend_instance'])
        after=observe(m)
        require(after['model_ready'],'registered model has not become ready')
    baseline=live['runtime']if live else owner_before
    if baseline and baseline.get('connected'):
        keys=('connected','runtime_nonce','holder','automatic_enabled')if model_preparation and not model_preparation['reused']else('connected','runtime_nonce','command_epoch','holder')
        require(all(after['runtime'][k]==baseline.get(k) for k in keys), 'control changed during startup; not resubmitted')
    else:
        require(after['runtime']['connected'] is False and not after['runtime']['automatic_enabled'], 'startup must not acquire/execute control')
    ready=after.get('control_ready') is True
    return dict(ok=True,services_ready=True,control_ready=ready,model_configured=bool(m.get('model_port')),model_ready=after.get('model_ready',False),model_preparation=model_preparation,url=m['url'],actions=actions,paused_collection_units=paused,
        collection_handoffs=handoffs,collection_stop_may_home=any(x.get('may_have_returned_home')for x in handoffs),
        control=after['runtime'],checks=after,robot_commands=0,sdk_acquisitions=0,
        note='explicit control recovery required' if not ready else 'services ready; SDK acquisition remains explicit')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    p=sub.add_parser('seal',help='freeze an installed, qualified deployment; does not start it')
    for flag in ('root','manager-profile','manager-unit','url','output'):p.add_argument('--'+flag,required=True)
    p.add_argument('--collection-unit');p.add_argument('--collection-status-url')
    p.add_argument('--triton-unit',help='registered CPU-only Triton tool service; never starts a GPU model job')
    for name in ('check','start'):
        p=sub.add_parser(name);p.add_argument('--manifest',required=True)
        if name=='start':p.add_argument('--handoff-collection',action='store_true',help='normally stop the registered collection and its Web; may return home; never force-kill')
    args=parser.parse_args();report=None
    try:
        require(sys.platform=='linux','run on the Linux robot host')
        if args.action=='seal':result=seal(args)
        else:
            m=load(args.manifest)
            with startup_lock(str(Path(args.manifest).parent/'startup.lock')):
                managed=preflight(m)
                if args.action=='check':
                    states={k:v.snapshot() for k,v in managed.items()}
                    conflicts=[{'unit':c['unit'],'state':unit_state(c['unit'])['ActiveState']} for c in m['conflicts']]
                    result=dict(ok=True,integrity_passed=True,services=states,conflicts=conflicts,robot_commands=0)
                    if states['control_owner']['state']=='stopped':
                        try:result['previous_boot_remnants']=old_owner_files(m)
                        except (OSError,ValueError,Blocked) as error:result['owner_blocker']=str(error)
                    try:result['live']=observe(m)
                    except (OSError,ValueError,Blocked):result['live']=None
                    result['startup_ready']=not result.get('owner_blocker') and all(v['state']in('running','stopped')for v in states.values()) and all(c['state']=='inactive' for c in conflicts)
                else:
                    report=Path(args.manifest).parent/('start-'+str(time.time_ns())+'.json')
                    # Intent survives partial startup/timeout; never stores credentials.
                    private_write(report,dict(status='started',wall_ns=time.time_ns(),automatic_retry=False))
                    result=start(m,managed,handoff=args.handoff_collection)
        if report:private_write(report.with_suffix('.result.json'),result)
        print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result.get('ok',True) else 3
    except (OSError,ValueError,KeyError,Blocked,subprocess.SubprocessError) as error:
        result=dict(ok=False,error=str(error),automatic_retry=False,robot_commands=0,
            recovery='inspect current service state; no rollback/SDK release is attempted')
        if report:private_write(report.with_suffix('.result.json'),result)
        print(json.dumps(result,ensure_ascii=False,indent=2));return 2


if __name__=='__main__':raise SystemExit(main())
