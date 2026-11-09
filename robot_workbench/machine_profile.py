"""Declared machine metadata and the running manager's bounded source manifest."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time


def digest(value):return hashlib.sha256(value).hexdigest()


def _text(value):
    return isinstance(value,str) and 0<len(value)<=2048 and '\0' not in value


class MachineProfile:
    def __init__(self,config):
        value=deepcopy(config.get('machine_profile',{}))
        if not isinstance(value,dict) or set(value)-{'label','host_name','hardware','calibration','known_degradations'}:
            raise ValueError('known trusted machine profile fields required')
        for key in ('label','host_name'):
            if key in value and not _text(value[key]):raise ValueError('declared machine labels must be bounded strings')
        hardware=value.setdefault('hardware',{})
        if not isinstance(hardware,dict) or set(hardware)-{'robot_model','robot_serial','can_interfaces','cameras','disabled_grippers'}:
            raise ValueError('known hardware index fields required')
        for key in ('robot_model','robot_serial'):
            if key in hardware and not _text(hardware[key]):raise ValueError('declared hardware identity must be text')
        interfaces=hardware.get('can_interfaces',{})
        if not isinstance(interfaces,dict) or set(interfaces)-{'left','right'} or any(not _text(v) for v in interfaces.values()):
            raise ValueError('declared left/right CAN names required')
        cameras=hardware.get('cameras',{})
        if (not isinstance(cameras,dict) or set(cameras)-{'head','left','right'}
                or any(not isinstance(row,dict) or set(row)-{'serial','intrinsics_index'}
                       or not _text(row.get('serial')) for row in cameras.values())):
            raise ValueError('declared camera roles and serials required')
        disabled=hardware.get('disabled_grippers',[])
        if not isinstance(disabled,list) or any(not isinstance(v,str) for v in disabled) or len(set(disabled))!=len(disabled) or set(disabled)-{'left','right'}:
            raise ValueError('declared disabled grippers must be distinct left/right names')
        calibration=value.setdefault('calibration',{})
        if not isinstance(calibration,dict) or set(calibration)-{'configuration_index','library_index','active_manifest_index'}:
            raise ValueError('known calibration index fields required')
        indexes=[*calibration.values(),*[row['intrinsics_index'] for row in cameras.values() if 'intrinsics_index' in row]]
        if any(not _text(p) or not Path(p).is_absolute() or '..' in Path(p).parts for p in indexes):
            raise ValueError('machine indexes must be trusted absolute paths')
        degraded=value.setdefault('known_degradations',[])
        if (not isinstance(degraded,list) or len(degraded)>32 or any(not isinstance(row,dict)
                or set(row)!={'capability','reason'} or any(not _text(v) for v in row.values()) for row in degraded)):
            raise ValueError('bounded declared capability limitations required')
        self.declared=value;self.identity=dict(device_id=config['device_id'],environment=config['environment'])
        indexes={}
        for name,raw in self.declared['calibration'].items():
            path=Path(raw);row=dict(path=raw,status='unavailable',sha256=None)
            try:
                if path.is_dir():row['status']='directory_present'
                elif path.is_file():
                    if path.stat().st_size>2*1024*1024:row['status']='present_not_hashed_size_limit'
                    else:row.update(status='file_present',sha256=digest(path.read_bytes()))
            except OSError:row['status']='unreadable'
            indexes[name]=row
        self.indexes=indexes;self.captured_wall_ns=time.time_ns()

    def snapshot(self):
        return dict(schema='robot_workbench.machine_profile.v1',identity=deepcopy(self.identity),
            declared=deepcopy(self.declared),calibration_indexes=deepcopy(self.indexes),checked_wall_ns=self.captured_wall_ns,
            evidence_scope='trusted_deployment_declarations_and_indexes_captured_at_startup',hardware_verified=False,
            hardware_verification='not_performed_by_management',configured=bool(self.declared.get('hardware') or self.declared.get('label')))


class Deployment:
    """Capture source identity once at startup; never poll or gate on file drift."""
    def __init__(self,root,services):
        root=Path(root);paths={}
        roots={'manager':root}
        for name,service in services.items():
            spec=getattr(service,'spec',None)
            if spec is not None:
                script=Path(spec.expected_script)
                if script.parent.name=='scripts':roots[name]=script.parent.parent
        self.components={}
        seen={}
        for component,source in roots.items():
            if source in seen:
                self.components[component]=dict(self.components[seen[source]],source_component=seen[source]);continue
            entries={}
            for folder in ('robot_workbench','rule_based_workbench','scripts','web','calibration','detector_adapter'):
                for path in (source/folder).rglob('*'):
                    if path.is_file() and not path.is_symlink() and path.suffix in ('.py','.js','.mjs','.html','.css','.sh'):
                        key=component+':'+str(path.relative_to(source));paths[key]=path;entries[key]=str(path)
            self.components[component]=dict(root=str(source),source_files=len(entries))
            seen[source]=component
        for name,service in services.items():
            spec=getattr(service,'spec',None)
            if spec is not None:paths['service:'+name]=Path(spec.expected_script)
        self.files={}
        for name,path in sorted(paths.items()):
            try:raw=path.read_bytes()
            except OSError:raw=None
            self.files[name]=(path,digest(raw) if raw is not None else None)
        for name,component in self.components.items():
            prefix=component.get('source_component',name)+':'
            component['source_sha256']=digest(json.dumps({key:sha for key,(_,sha) in self.files.items()if key.startswith(prefix)},sort_keys=True).encode())
        self.sha256=digest(json.dumps({name:sha for name,(_,sha) in self.files.items()},sort_keys=True).encode())
        self.built_wall_ns=time.time_ns()

    def bind_configuration(self,config):
        """Hash trusted configuration and explicitly pinned external SDK/profile files."""
        pins=config.get('publication_files',{})
        if (not isinstance(pins,dict) or any(not isinstance(name,str) or not isinstance(raw,str)
                or not Path(raw).is_absolute() or '..' in Path(raw).parts for name,raw in pins.items())):
            raise ValueError('publication_files requires named absolute trusted files')
        for name,raw in pins.items():
            path=Path(raw)
            if path.is_symlink() or not path.is_file() or path.stat().st_size>32*1024**2:
                raise ValueError('publication pin must be a bounded regular file: '+name)
            self.files['configuration:'+name]=(path,digest(path.read_bytes()))
        self.configuration_sha256=digest(json.dumps(config,sort_keys=True).encode())
        self.sha256=digest(json.dumps(dict(files={name:sha for name,(_,sha) in self.files.items()},
            configuration=self.configuration_sha256),sort_keys=True).encode())

    def snapshot(self):
        return dict(sha256=self.sha256,source_scope='composite_deployed_sources_entrypoints_and_trusted_configuration',
            source_files=len(self.files),captured_wall_ns=self.built_wall_ns,
            source_verification='captured_at_startup',components=deepcopy(self.components),
            configuration_sha256=getattr(self,'configuration_sha256',None),
            entrypoints={name.removeprefix('service:'):sha for name,(_,sha) in self.files.items() if name.startswith('service:')})
