#!/usr/bin/env python3
"""Start registered software services and return only after selected-model readiness."""
import argparse,json,re,secrets,subprocess,time
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.workbench_deploy import http_json,Blocked,require,handoff_collection


def start(config):
    # Only explicitly registered software; never stop active collection or acquire SDK.
    external_units=set();recording_unit=None;handoff=None
    if config.get('manager_profile'):
        profile=Path(config['manager_profile'])
        require(profile.is_absolute(),'absolute manager profile required')
        manager=json.loads(profile.read_text())
        services=manager.get('services')
        require(isinstance(services,dict),'explicit supported user_unit service mappings required')
        require(manager.get('model_lifecycle','managed')in ('managed','external'),'known model lifecycle required')
        require(manager.get('control_lifecycle','managed')in ('managed','external'),'known control lifecycle required')
        roles=['model']if manager.get('model_lifecycle')=='external'else[]
        if manager.get('control_lifecycle')=='external':
            require(manager.get('model_lifecycle')=='external'and manager.get('control_consumer_id')
                and manager.get('control_owner_descriptor'),'external control consumer configuration required')
            roles+=['control_owner','cameras']
            if 'recording' in services:roles.append('recording')
        elif 'recording' in services:
            recording=services['recording']
            require(isinstance(recording,dict)and recording.get('kind')=='user_unit'and re.fullmatch(r'robot-workbench-[A-Za-z0-9_.@-]+\.service',str(recording.get('unit'))),
                'recording requires a registered user unit')
            recording_unit=recording['unit']
        for role in roles:
            model=services.get(role)
            require(isinstance(model,dict)and model.get('kind')=='user_unit'
                and isinstance(model.get('unit'),str)
                and re.fullmatch(r'robot-workbench-[A-Za-z0-9_.@-]+\.service',model['unit']),
                f'external {role} requires an explicit supported user_unit mapping before any service startup')
            external_units.add(model['unit'])
        if manager.get('control_lifecycle','managed')=='managed' and manager.get('collection_handoff'):
            handoff=handoff_collection(manager['collection_handoff'],manager['device_id'],stop_active=False)
    units=([recording_unit]if recording_unit else [])+[unit for unit in config['units']if unit!=recording_unit]
    for unit in units:
        require(unit.startswith('robot-workbench-')and unit.endswith('.service'),'registered workbench unit required')
        if unit in external_units:continue
        subprocess.run(['systemctl','--user','start',unit],check=True,timeout=30)
    url=config['url'];deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        try:
            current=http_json(url+'/api/environment?environment=physical&view=summary')
            if current.get('health',{}).get('backend_reachable')and current.get('target',{}).get('services',{}).get('model'):break
        except (OSError,ValueError,Blocked):pass
        time.sleep(.2)
    else:raise Blocked('UI or model HTTP endpoint unavailable; inspect services')
    if recording_unit:
        require(current.get('health',{}).get('recording_service',{}).get('ready')is True,
            'resident recording service readiness unconfirmed; inspect its registered service')
    calibration=http_json(url+'/api/state?environment=physical&business=calibration')
    manual_ready=(calibration.get('workflow',{}).get('manual_queue')or{}).get('available')is True
    require(manual_ready,'manual planning workflow unavailable; configure calibration_config in the business profile')
    if config.get('manager_profile') and manager.get('control_lifecycle','managed')=='managed' and manager.get('target_model_manifest'):
        require(bool(calibration.get('preview',{}).get('webgl_model')),'registered target model unavailable')
        deadline=time.monotonic()+30
        while True:
            media={row['id']:row for row in calibration.get('media',[])}
            if all(media.get(role,{}).get('ready')is True for role in ('head','left','right')):break
            require(time.monotonic()<deadline,'registered raw camera inputs did not become ready')
            time.sleep(.2)
            calibration=http_json(url+'/api/state?environment=physical&business=calibration')
    if current.get('model',{}).get('ready'):return dict(ui_ready=True,manual_planning_ready=True,model_status='ready',reused=True,url=url,sdk_acquisitions=0,collection_handoff=handoff)
    active=current.get('active_job');jobs=current.get('jobs',{})
    if isinstance(active,str):active=jobs.get(active)
    if active:
        require(active.get('request',{}).get('action')=='prepare_model','another management operation is active')
        job_id=active['job_id']
    else:
        op=current.get('operations',{}).get('prepare_model',{})
        require(op.get('allowed')is True,op.get('disabled_reason')or'model preparation unavailable')
        result=http_json(url+'/api/operations/action',dict(action='prepare_model',request_id='startup-'+secrets.token_hex(12),expected_target=current['target'],arms_supported=False,parameters={}),headers={'Origin':url,'X-CSRF-Token':current['csrf'],'X-Workbench-Deployment':current['deployment']['sha256']})
        job_id=result['job_id']
    deadline=time.monotonic()+360
    while time.monotonic()<deadline:
        job=http_json(url+'/api/operations/jobs/'+job_id+'?environment=physical')
        if job['status']not in('queued','running','cancel_requested'):
            require(job['status']=='succeeded'and(job.get('verification')or{}).get('ok')is True,
                'model preparation failed/unconfirmed; inspect job '+job_id)
            current=http_json(url+'/api/environment?environment=physical&view=summary')
            require(current.get('model',{}).get('ready')is True,'selected model readiness changed after preparation')
            return dict(ui_ready=True,manual_planning_ready=True,model_status='ready',job_id=job_id,reused=bool(active),url=url,sdk_acquisitions=0,collection_handoff=handoff)
        time.sleep(.2)
    raise Blocked('model preparation deadline exceeded; inspect job '+job_id+'; not resubmitted')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True);a=p.parse_args()
    print(json.dumps(start(json.loads(a.config.read_text())),ensure_ascii=False))
