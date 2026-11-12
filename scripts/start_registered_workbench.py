#!/usr/bin/env python3
"""Start registered software services and return only after selected-model readiness."""
import argparse,json,re,subprocess,time
from pathlib import Path
import sys
from urllib.request import build_opener,ProxyHandler
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.workbench_deploy import http_json,Blocked,require,handoff_collection,ensure_camera_inputs,startup_operation,old_owner_files,unit_state,startup_lock


def acquire_control(config):
    """The explicitly configured desktop launch is the operator's takeover confirmation."""
    from robot_workbench.state_contract import control_healthy
    url=config['url']+'/api/state?environment=physical&business=inference'
    before=http_json(url)['runtime']
    if control_healthy(before):return dict(ready=True,reused=True,sdk_acquisitions=0,runtime_nonce=before['runtime_nonce'])
    result=startup_operation(config,'reclaim_control',arms_supported=True,timeout=300)
    deadline=time.monotonic()+10
    while True:
        after=http_json(url)['runtime']
        if control_healthy(after):break
        require(time.monotonic()<deadline,'control readiness unconfirmed after takeover; inspect job '+result['job_id'])
        time.sleep(.1)
    return dict(ready=True,reused=False,job_id=result['job_id'],runtime_nonce=after['runtime_nonce'],
        sdk_acquisitions=int(not before.get('connected')or before.get('runtime_nonce')!=after['runtime_nonce']),
        confirmation_basis='operator-approved desktop launch')


def open_page(config):
    """Open the manager independently of model/control preparation."""
    unit=config.get('ui_unit')
    require(unit in config['units'] and re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*\.service',str(unit)),
        'registered ui_unit required for early browser opening')
    started=time.monotonic()
    subprocess.run(['systemctl','--user','start',unit],check=True,timeout=4)
    opener=build_opener(ProxyHandler({}));url=config['url']+'/?environment=physical&business=inference'
    while True:
        try:
            with opener.open(url,timeout=.5)as response:
                if response.status==200:break
        except OSError:pass
        require(time.monotonic()-started<4,'management webpage did not start within the desktop opening budget')
        time.sleep(.05)
    from rule_based_workbench.launch_environment import clean_environment
    subprocess.run(['xdg-open',url],check=True,capture_output=True,
        env=clean_environment(),timeout=max(.1,5-(time.monotonic()-started)))
    timing=dict(open_completed_wall_ns=time.time_ns(),open_completed_after_s=time.monotonic()-started)
    print('WORKBENCH_PAGE_OPEN '+json.dumps(timing),file=sys.stderr,flush=True)
    return timing


def start(config):
    # Existing consumers remain software-only; takeover must be explicitly configured.
    takeover=config.get('connect_control',False)
    require(type(takeover)is bool,'explicit boolean connect_control required')
    require(not takeover or config.get('manager_profile'),'takeover requires a registered manager profile')
    external_units=set();recording_unit=None;handoff=None;control=None
    if config.get('manager_profile'):
        profile=Path(config['manager_profile'])
        require(profile.is_absolute(),'absolute manager profile required')
        manager=json.loads(profile.read_text())
        services=manager.get('services')
        require(isinstance(services,dict),'explicit supported user_unit service mappings required')
        require(manager.get('model_lifecycle','managed')in ('managed','external'),'known model lifecycle required')
        require(manager.get('control_lifecycle','managed')in ('managed','external'),'known control lifecycle required')
        require(not takeover or manager.get('control_lifecycle','managed')=='managed'
            and 'control_owner'in services and manager.get('collection_handoff'),
            'takeover requires managed Owner and registered collection handoff')
        roles=['model']if manager.get('model_lifecycle')=='external'else[]
        if manager.get('control_lifecycle')=='external':
            require(manager.get('model_lifecycle')=='external'and manager.get('control_consumer_id')
                and manager.get('control_owner_descriptor'),'external control consumer configuration required')
            roles+=['control_owner','cameras']
            if 'recording' in services:roles.append('recording')
        elif 'recording' in services:
            recording=services['recording']
            require(isinstance(recording,dict)and recording.get('kind')=='user_unit'and re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*\.service',str(recording.get('unit'))),
                'recording requires a registered user unit')
            recording_unit=recording['unit']
        for role in roles:
            model=services.get(role)
            require(isinstance(model,dict)and model.get('kind')=='user_unit'
                and isinstance(model.get('unit'),str)
                and re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*\.service',model['unit']),
                f'external {role} requires an explicit supported user_unit mapping before any service startup')
            external_units.add(model['unit'])
        if manager.get('control_lifecycle','managed')=='managed' and manager.get('control_owner_descriptor'):
            if unit_state(services['control_owner']['unit'])['MainPID']=='0':
                old_owner_files(dict(owner_descriptor=manager['control_owner_descriptor'],
                    device_id=manager['device_id'],services=services),apply=True)
        if not takeover and manager.get('control_lifecycle','managed')=='managed' and manager.get('collection_handoff'):
            handoff=handoff_collection(manager['collection_handoff'],manager['device_id'],stop_active=False)
    units=([recording_unit]if recording_unit else [])+[unit for unit in config['units']if unit!=recording_unit]
    for unit in units:
        require(re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*\.service',str(unit)),'registered workbench unit required')
        if unit in external_units:continue
        # Takeover owns the collector handoff before opening shared USB devices.
        if takeover and unit==(manager['services'].get('cameras')or{}).get('unit'):continue
        subprocess.run(['systemctl','--user','start',unit],check=True,timeout=30)
    url=config['url'];deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        try:
            current=http_json(url+'/api/environment?environment=physical&view=summary')
            if (current.get('health',{}).get('backend_reachable')and current.get('target',{}).get('services',{}).get('model')
                    and (not takeover or current.get('health',{}).get('control_owner_reachable')is True)):break
        except (OSError,ValueError,Blocked):pass
        time.sleep(.2)
    else:raise Blocked('UI, model HTTP or control Owner unavailable; inspect services')
    if recording_unit:
        require(current.get('health',{}).get('recording_service',{}).get('ready')is True,
            'resident recording service readiness unconfirmed; inspect its registered service')
    if takeover:
        control=acquire_control(config)
        current=http_json(url+'/api/environment?environment=physical&view=summary')
    calibration=http_json(url+'/api/state?environment=physical&business=calibration')
    manual_ready=(calibration.get('workflow',{}).get('manual_queue')or{}).get('available')is True
    require(manual_ready,'manual planning workflow unavailable; configure calibration_config in the business profile')
    if config.get('manager_profile') and manager.get('control_lifecycle','managed')=='managed' and manager.get('target_model_manifest'):
        require(bool(calibration.get('preview',{}).get('webgl_model')),'registered target model unavailable')
        ensure_camera_inputs(config,state=calibration)
        current=http_json(url+'/api/environment?environment=physical&view=summary')
    if config.get('manager_profile')and 'pedal'in manager['services']:
        deadline=time.monotonic()+10
        while True:
            state=http_json(url+'/api/state?environment=physical&business=inference')
            pedal=(state.get('workflow',{}).get('inference')or{}).get('operator_input')or{}
            if pedal.get('state')=='ready':break
            require(time.monotonic()<deadline,'pedal input not ready: '+str(pedal.get('error')or pedal.get('state')))
            time.sleep(.1)
    parameters={}
    saved_path=Path(manager['state_root'])/'inference-startup.json' if config.get('manager_profile') and manager.get('state_root') else None
    if saved_path is not None and saved_path.is_file():
        saved=json.loads(saved_path.read_text())
        require(saved.get('schema')=='workbench.inference-startup.v1','unsupported saved inference startup configuration')
        parameters={key:saved[key]for key in ('model_id','expected_bundle_digest')}
    if current.get('model',{}).get('ready'):
        return dict(ui_ready=True,manual_planning_ready=True,model_status='ready',reused=True,url=url,
            sdk_acquisitions=(control or{}).get('sdk_acquisitions',0),control=control,collection_handoff=handoff)
    result=startup_operation(config,'prepare_model',timeout=360,parameters=parameters)
    current=http_json(url+'/api/environment?environment=physical&view=summary')
    require(current.get('model',{}).get('ready')is True,'selected model readiness changed after preparation')
    return dict(ui_ready=True,manual_planning_ready=True,model_status='ready',job_id=result['job_id'],reused=False,url=url,
        sdk_acquisitions=(control or{}).get('sdk_acquisitions',0),control=control,collection_handoff=handoff)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--open-page-only',action='store_true')
    a=p.parse_args();config=json.loads(a.config.read_text())
    if a.open_page_only:open_page(config)
    else:
        with startup_lock(a.config.absolute().with_suffix('.startup.lock')):
            print(json.dumps(start(config),ensure_ascii=False))
