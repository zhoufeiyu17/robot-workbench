#!/usr/bin/env python3
"""Loopback X5 workbench: kinematic planning plus configured rendered capture. No robot IO."""
from collections import deque
from functools import cached_property
import argparse
import hashlib
import json
import os
import secrets
import shutil
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, parse_qs
from urllib.request import Request, build_opener, ProxyHandler, HTTPRedirectHandler
from urllib.error import HTTPError, URLError
import numpy as np
from arx_env.joint_head_workbench_geometry import Geometry, jaw_gaps
from arx_env.joint_head_jaw_mapping import command_sdk, feedback_gaps, feedback_state, mapping_identity
from arx_env.joint_head_isaac_fit import rigid


def read(path):
    return json.loads(Path(path).read_text())


def transport_environment(config):
    environment=config.get('environment','physical')
    if environment not in ('physical','fake'):raise ValueError('明确指定 physical/fake 设备环境')
    if environment=='fake' and (config.get('simulation_only') is not True or config.get('robot_source','simulated')!='simulated'):
        raise ValueError('Fake 接入必须明确 simulation_only=true 且 robot_source 为 simulated')
    return environment


def load_wrist_selection(path):
    """Load the persisted wrist selection.

    The selection is one paired record (S3): id / pair_status / left / right. Nothing
    here interprets it per side. A legacy per-side mapping, a partial write or
    a wrong type is discarded rather than half-understood - reading a paired file as a
    per-side mapping crashed startup with "'str' object has no attribute 'get'".
    Whether the record is still valid for the current environment and still exists in
    the library is decided by restore_wrist_display, which runs on every remote sync.
    """
    try:
        raw=read(path)
    except (OSError,ValueError):
        return {}
    if not isinstance(raw,dict) or not isinstance(raw.get('id'),str) or not raw.get('id') or not all(side in raw for side in ('left','right')):
        return {}
    return raw


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self,req,fp,code,msg,headers,newurl):
        return None


class PipelineStopped(Exception):
    """Raised inside the calibration pipeline when the operator asked it to stop.

    Swallowed by the pipeline job so the run ends as STOPPED with a report of what was
    done, rather than as a failure that looks like a defect in the calibration.
    """


class Workbench:
    STANDARD_MOTION_SEED = 'x5-standard-head-motion-seed-v1'
    PREFLIGHT_PATHS=frozenset(('/api/hardware-trajectory-preview','/api/hardware-jaw-preview','/api/queue','/api/calibration-preview',
        *('/api/control/'+action for action in ('trajectory-preview','jaw-preview','motion-preview','home-preview','both-home-preview','arm-home-preview'))))
    def __init__(self, config, output):
        environment=transport_environment(config)
        if environment=='fake':
            root=Path(config.get('calibration_root',''))
            if (not str(config.get('robot_id','')).startswith('fake-') or not root.is_absolute()
                    or root.resolve().is_relative_to(Path('/var/lib/anyverse/calibration').resolve())):
                raise ValueError('Fake 必须使用独立 fake- robot_id 和绝对 calibration_root，禁止物理标定目录')
        if config.get('jaw_mapping_file'):
            from arx_env.joint_head_jaw_mapping import configure_mapping
            configure_mapping(config['jaw_mapping_file'])
        if config.get('mode','simulation')!='simulation':raise ValueError('Hardware adapter is not verified; use the existing real owner, not this workbench')
        self.live=config.get('live_backend');self.remote=None;self.remote_received=0.;self.remote_error='DSW connecting';self.imported=set();self.pending_capture=None;self.importing=False;self.import_error=None;self.last_remote_timestamp=None;self.remote_advanced=0.;self.preview_revision=0;self.calibration=None
        if self.live:
            endpoint=urlsplit(self.live['url'])
            if endpoint.scheme!='http' or endpoint.hostname not in ('127.0.0.1','localhost') or endpoint.username or endpoint.path not in ('','/'):raise ValueError('live backend must use a loopback HTTP tunnel')
            token_path=Path(self.live['token_file'])
            if not token_path.is_absolute():raise ValueError('absolute token_file required')
            self.live_token=token_path.read_text().strip()
            if not self.live_token:raise ValueError('empty backend token')
            self.http=build_opener(ProxyHandler({}),NoRedirect())
        self.hardware_history=deque(maxlen=180);self.hardware_history_lock=threading.Lock()
        self.hardware_camera=None
        self.hardware_feedback=None;self.hardware_received=0.;self.hardware_error=None;self.mirror_error=None
        self.control_status=None;self.control_received=0.;self.control_error=None;self.control_last_disconnect=None;self.hardware_last_disconnect=None;self.calibration_previews={}
        self.fake_restart_lock=threading.Lock();self.fake_restart_pending={};self.fake_restart_in_progress=False
        self.control_mode="hardware_sync" if config.get("hardware_control") else "simulation";self.mode_lock=threading.RLock();self.hardware_tokens=set()
        self.review_lock=threading.RLock();self.review_generation=0;self.review_stops_pending=0
        self._mesh_view_mode=None
        # Each operating environment keeps its own queue, plan and overlay
        # selection, so switching modes swaps working sets instead of wiping them.
        self.environments={'simulation':{},'hardware':{}}
        self.live_frames={};self.live_frame_errors={};self.preview_pending=None;self.preview_condition=threading.Condition();self.preview_result=None
        self.control=config.get('hardware_control')
        if self.control:
            port=self.control['port']
            if type(port) is not int or not 1024<=port<=65535:raise ValueError('invalid hardware gateway port')
            self.control_token=Path(self.control['token_file']).read_text().strip()
            if len(self.control_token)<32:raise ValueError('invalid hardware gateway token')
            self.control_http=build_opener(ProxyHandler({}),NoRedirect())
        if config.get('hardware_camera_port') is not None:
            from arx_env.joint_head_workbench_hardware import HardwareCameraAdapter
            self.hardware_camera=HardwareCameraAdapter(config['hardware_camera_port'],environment=environment)
        from arx_env.joint_head_workbench_hardware import HardwareCameraAdapter
        self.wrist_cameras={side:HardwareCameraAdapter(port,environment=environment) for side,port in config.get('hardware_wrist_camera_ports',{}).items() if side in ('left','right')}
        self.hardware_reader=None
        if config.get('hardware_status_owner_port') is not None:
            from arx_env.joint_head_workbench_hardware import HardwareReadAdapter
            self.hardware_reader=HardwareReadAdapter(config['hardware_status_owner_port'],environment=environment)
        self.config=config
        if environment=='fake' and config.get('resident_deadline_wall_ns') is not None:
            self.resident_capture_budget()
        self.geometry=Geometry(config); self.output=Path(output).resolve()
        self.output.mkdir(parents=True,exist_ok=True)
        from arx_env.joint_head_calibration_store import CalibrationStore
        self.calibration_root=Path(config.get('calibration_root',self.output/'calibrations')).absolute()
        CalibrationStore(self.calibration_root,config.get('robot_id','unassigned-simulation')).split_legacy()
        self.store=CalibrationStore(self.calibration_root/'simulation',config.get('robot_id','unassigned-simulation'),'simulation')
        self.hardware_store=CalibrationStore(self.calibration_root/'hardware',config.get('robot_id','unassigned-simulation'),'hardware')
        self.workflow={'kind':'head','head_result_id':None}
        self.mesh_rendering_enabled=False
        self.marker_prior_source=None
        self.delete_tokens={}
        self.display_meshes=[]
        for side in (() if self.live else ('left','right')):
            for name,v,_ in self.geometry.side_meshes[side]:
                # ponytail: 4 mm vertex clustering for UI only; collision/render use original mesh.
                keys,inverse=np.unique(np.round(v/.004).astype(int),axis=0,return_inverse=True)
                vertices=np.zeros((len(keys),3));np.add.at(vertices,inverse,v)
                vertices/=np.bincount(inverse)[:,None]
                faces=inverse.reshape(-1,3);faces=faces[(faces[:,0]!=faces[:,1])&(faces[:,1]!=faces[:,2])&(faces[:,0]!=faces[:,2])]
                signs=np.zeros(len(keys))
                if name=='link6' and self.geometry.hand is not None:np.add.at(signs,inverse,self.geometry.hand['signs']);signs/=np.bincount(inverse)
                self.display_meshes.append((side,name,vertices,np.unique(faces,axis=0),signs))
        self.jaw_mm=jaw_gaps([self.geometry.gaps[s]*1000 for s in ('left','right')])
        self.q=np.array(config.get('initial_q',[0.]*12),float)
        self.velocity_limits=np.minimum(self.geometry.velocity_limits,config.get('simulation_speed_limit_rad_s',1.0))
        self.csrf=secrets.token_urlsafe(24); self.lock=threading.RLock()
        self.status='READY'; self.reason=None; self.tokens={}; self.queue=[]; self.frames=[]
        self.fit_result=None; self.evaluation=None; self.batch=None; self.stop=threading.Event(); self.process=None
        self.actual={'parts':[],'camera':None} if self.live else self.display(self.q)
        self.plan=None; self.selected_batch=None
        fingerprint=hashlib.sha256()
        fingerprint.update(Path(sys.modules[Geometry.__module__].__file__).read_bytes())
        for key in ('urdf','scene_seed','candidate','camera','mesh_manifest'):
            fingerprint.update(Path(config[key]).read_bytes())
        if config.get('head_motion_seed'):
            fingerprint.update(Path(config['head_motion_seed']).read_bytes())
        fingerprint.update(json.dumps({key:config.get(key) for key in ('model_family','asset_geometry_id','asset_seal_sha256','parking_support_contact','gap_m','marker_size_m','margin_px','wrist_support_proxy','table_z_m','table_clearance_m','chassis_scale','chassis_T_world_mesh','static_boxes','wrist_camera_housings')},sort_keys=True).encode())
        if config.get('chassis_mesh'):fingerprint.update(Path(config['chassis_mesh']).read_bytes())
        for side,meshes in sorted(self.geometry.side_meshes.items()):
            fingerprint.update(side.encode())
            for name,vertices,faces in meshes:
                fingerprint.update(name.encode());fingerprint.update(vertices.tobytes());fingerprint.update(faces.tobytes())
        self.geometry_id=fingerprint.hexdigest()
        self.plan_geometry_id=self.geometry_id
        saved=self.output/'workbench-state.json'
        if saved.exists():
            restored=read(saved)
            self.queue=restored.get('queue',[]);self.plan=restored.get('plan')
            self.selected_batch=restored.get('selected_batch')
            self.plan_geometry_id=restored.get('geometry_id')
            self.control_mode=restored.get('control_mode','hardware_sync')
            self.workflow=restored.get('workflow',self.workflow)
            self.mesh_rendering_enabled=restored.get('mesh_rendering_enabled') is True
            if self.control_mode not in ('simulation','hardware_sync'):self.control_mode='simulation'
            saved_mode=restored.get('mesh_view_mode')
            if saved_mode in ('simulation','hardware'):self._mesh_view_mode=saved_mode
            saved=restored.get('marker_prior_source')
            if isinstance(saved,str) and saved:self.marker_prior_source=saved
            saved_report=restored.get('pipeline_report')
            if isinstance(saved_report,dict) and saved_report:self.pipeline_report=saved_report
            saved_environments=restored.get('environments')
            if isinstance(saved_environments,dict):
                self.environments={'simulation':saved_environments.get('simulation') or {},'hardware':saved_environments.get('hardware') or {}}
        elif (self.output/'plan.json').exists():
            self.plan_geometry_id=None
            self.plan=read(self.output/'plan.json')
            self.queue=[dict(w,name=f"{w.get('arm','')} {w.get('kind','')} {w.get('cell','')}",speeds=[.2]*12) for w in self.plan['waypoints']]
        self.batch=self.plan
        if self.selected_batch:
            self.load_batch(self.selected_batch)
        elif config.get('review_dataset'):
            dataset=Path(config['review_dataset']).resolve()
            if not dataset.is_relative_to(self.output):raise ValueError('review dataset must be under output root')
            self.load_batch(str(dataset.relative_to(self.output)))

        self.restore_calibration()
        self.register_existing_calibrations()
        # Paired or nothing: see load_wrist_selection for why this must never be
        # iterated as a per-side mapping.
        self.wrist_selected=load_wrist_selection(self.output/'wrist-selected.json')
        if self.workflow.get('head_result_id') not in {r['id'] for r in self.environment_results(self.calibration_results())}:
            self.workflow['head_result_id']=None
        self.wrist_plan_progress=None
        self.monitor_stop=threading.Event();self.camera_processes={}
        threading.Thread(target=self.monitor_camera_processes,daemon=True).start()
        for camera in [self.hardware_camera,*self.wrist_cameras.values()]:
            if camera:threading.Thread(target=camera.monitor,args=(self.monitor_stop,),daemon=True).start()
        if self.control:threading.Thread(target=self.monitor_control,daemon=True).start()
        if self.live:
            threading.Thread(target=self.monitor_remote,daemon=True).start()
            threading.Thread(target=self.monitor_display_restore,daemon=True).start()
            for view in ('preview','actual','head'):
                threading.Thread(target=self.monitor_frames,args=(view,),daemon=True).start()
            # Bound in-flight display requests to cover the DSW round trip; mailbox stays latest-only.
            for _ in range(3):threading.Thread(target=self.send_previews,daemon=True).start()
        if self.hardware_reader:
            threading.Thread(target=self.monitor_hardware,daemon=True).start()
            if self.live:threading.Thread(target=self.mirror_hardware,daemon=True).start()

    @cached_property
    def review_lock(self):
        # Also supports the small, constructor-free offline protocol fixtures.
        return threading.RLock()

    def invalidate_reviews(self):
        # Never acquire page/mode locks here: stop must preempt their work.
        with self.review_lock:
            self.review_generation=getattr(self,'review_generation',0)+1
            for name in ('tokens','hardware_tokens','calibration_previews'):
                getattr(self,name,{}).clear()
            progress=getattr(self,'motion_preflight',{}) or {}
            if progress.get('status')=='checking':
                self.motion_preflight={**progress,'status':'rejected','stage':'预检已撤销','error':'停止或控制上下文已变化，请重新预览'}

    def begin_preflight(self,stage):
        progress={'request_id':secrets.token_hex(8),'status':'checking','stage':stage,'started_at_s':time.time()}
        self.motion_preflight=progress
        return progress

    def preflight_progress(self,progress,stage,*,error=None,done=False):
        with self.review_lock:
            if (getattr(self,'motion_preflight',{}) or {}).get('request_id')!=progress['request_id']:return
            if self.motion_preflight.get('status')=='rejected':return
            self.motion_preflight={**progress,'stage':stage,'status':'rejected' if error else 'ready' if done else 'checking',
                                   'elapsed_s':max(0.,time.time()-progress['started_at_s']),**({'error':str(error)} if error else {})}

    def review_snapshot(self):
        with self.review_lock:
            if getattr(self,'review_stops_pending',0):raise ValueError('停止请求正在处理，请完成后重新预览')
            if getattr(self,'fake_restart_in_progress',False):raise ValueError('Fake 控制器正在恢复，请完成后重新预览')
            return (getattr(self,'review_generation',0),self.control_mode,getattr(self,'geometry_id',None),
                    json.dumps(getattr(self,'workflow',None),sort_keys=True),getattr(self,'pipeline_run_id',None))

    def require_review(self,review):
        if review!=self.review_snapshot():raise ValueError('预览已失效：停止、控制模式或计划依赖已变化，请重新审阅')

    def request_stop(self,body=None,*,hardware=False):
        self.stop.set()
        self.invalidate_reviews()
        if hardware or self.control_mode=='hardware_sync':return self.control_request('stop',body or {})
        if self.live:return self.remote_request('/stop',{})
        return {'message':'停止请求已提交'}

    def control_request(self,action,body=None,*,calibration=False,review=None):
        if action=='stop':
            with self.review_lock:
                self.review_stops_pending=getattr(self,'review_stops_pending',0)+1
                self.invalidate_reviews()
            try:return self._control_request(action,body)
            finally:
                with self.review_lock:
                    self.invalidate_reviews()
                    self.review_stops_pending-=1
        motion_actions={'trajectory-preview','jaw-preview','motion-preview','home-preview','both-home-preview','arm-home-preview','motion-confirm'}
        if action not in motion_actions:return self._control_request(action,body)
        with self.mode_lock:
            if self.control_mode!='hardware_sync':raise ValueError('纯仿真模式禁止下发真机动作；请切换到真机同步模式')
            if action=='motion-confirm':
                # Confirmation remains serialized with mode changes, but stop never
                # waits for this lock. The owner's epoch rejects a late confirmation.
                with self.review_lock:
                    review=self.review_snapshot()
                    if body.get('token') not in self.hardware_tokens:raise ValueError('控制模式已变化或预览过期，请重新审阅真机动作')
                    self.hardware_tokens.discard(body['token'])
                self.check_owner_mapping()
                self.require_review(review)
                return self._control_request(action,body)
            review=self.review_snapshot() if review is None else review
            self.require_review(review)
            body=json.loads(json.dumps(body,allow_nan=False))
            geometry=self.geometry
        preview_started=time.monotonic()
        self.check_owner_mapping()
        self.require_review(review)
        result=self._control_request(action,body)
        if not calibration and getattr(self,'config',{}).get('model_family')=='ac_one':
            plans=result.get('plans')
            if not isinstance(plans,list) or not plans:raise ValueError('A1 动作预览缺少冻结路径，无法检查支撑干涉')
            checks=[]
            for index,plan in enumerate(plans):
                self.require_review(review)
                endpoints=[]
                for key in ('q_start','q_target'):
                    values=plan.get(key) if isinstance(plan,dict) else None
                    if (not isinstance(values,(list,tuple)) or len(values)!=14
                            or any(type(v) not in (int,float) or not np.isfinite(v) for v in values)):
                        raise ValueError('A1 动作预览包含无效冻结关节位置')
                    endpoints.append(values[:6]+values[7:13])
                check=geometry.support_path_check(*endpoints)
                if not check['ok']:
                    raise ValueError(f'第 {index+1} 段动作超过支撑干涉上限或无法确认：'+json.dumps(check,ensure_ascii=False))
                checks.append(check)
            result['support_checks']=checks
            result['expires_in_s']=max(0.,float(result.get('expires_in_s',30))-(time.monotonic()-preview_started))
            if result['expires_in_s']<=0:raise ValueError('支撑路径检查已完成，但动作预览已过期；请重新预览以使用已缓存的检查结果')
        with self.review_lock:
            self.require_review(review)
            if result.get('token'):self.hardware_tokens.add(result['token'])
            result['review_generation']=review[0]
        return result

    def check_owner_mapping(self):
        if getattr(self,'config',{}).get('jaw_mapping_file'):
            owner=self._control_request('state').get('owner',{})
            if owner.get('jaw_mapping')!=mapping_identity():raise ValueError('控制器尚未加载当前 A1 夹爪映射，请在可靠支撑双臂后重新连接')

    def _control_request(self,action,body=None):
        allowed={'state','feedback','takeover-preview','takeover-confirm','trajectory-preview','capture-ack','acquire','stop','recover','release-preview','release-confirm','jaw-preview','motion-preview','home-preview','both-home-preview','arm-home-preview','motion-confirm'}
        if action not in allowed or (action in ('state','feedback'))!=(body is None):raise ValueError('unsupported hardware operation')
        if not self.control:raise ValueError('真机控制网关未配置')
        request=Request(f"http://127.0.0.1:{self.control['port']}/api/{action}",data=None if body is None else json.dumps(body,allow_nan=False).encode(),headers={'Authorization':'Bearer '+self.control_token,'Content-Type':'application/json'})
        try:
            with self.control_http.open(request,timeout=60 if action in ('acquire','takeover-confirm') else 3) as response:raw=response.read(4*1024*1024+1)
            if len(raw)>4*1024*1024:raise ValueError('hardware reply too large')
            return json.loads(raw)
        except HTTPError as exc:
            raise ValueError('真机：'+exc.read(65536).decode(errors='replace')) from None
        except (OSError,URLError) as exc:raise ValueError('真机网关不可用；操作结果未知，不自动重试：'+str(exc)) from None

    def monitor_camera_processes(self):
        from arx_env.joint_head_workbench_hardware import camera_process_inventory
        serials=list(self.config.get('hardware_camera_serials',{}).values())
        while not self.monitor_stop.is_set():
            try:self.camera_processes=camera_process_inventory(serials)
            except Exception as exc:self.camera_processes={'error':str(exc),'process_alive':None}
            self.monitor_stop.wait(2)

    def monitor_control(self):
        while True:
            try:self.control_status=self.control_request('state');self.control_received=time.monotonic();self.control_error=None
            except ValueError as exc:
                reason=str(exc)
                if reason!=self.control_error:self.control_last_disconnect={'reason':reason,'at_s':time.time()}
                self.control_error=reason;self.control_status=None
            time.sleep(.5)

    def control_available(self):
        return bool(self.control_status and time.monotonic()-self.control_received<2 and self.control_status.get('motion_enabled') and self.hardware_connected() and self.control_status.get('owner',{}).get('pid')==self.hardware_feedback['owner'].get('pid'))

    def monitor_hardware(self):
        last_source=None;last_sdk=None;advanced=[0.,0.]
        while True:
            began=time.monotonic()
            try:
                if self.hardware_reader is not None:
                    view=self.hardware_reader.read_state()
                    if not view.get('reachable') or not view.get('feedback',{}).get('valid'):
                        raise ValueError(view.get('error') or '; '.join(view.get('feedback',{}).get('reasons',[])) or '真机状态不可用')
                    owner=view['raw_owner']; feedback=owner['calibration_feedback']
                    value={'owner':owner,'source_age_s':view['feedback']['host_sample_age_s'],'reachable':True,'state_view':view}
                else:raise ValueError('独立状态网关未配置')
                q=np.asarray(feedback['qpos'],float);sdk=feedback['sdk_timestamps'];source=(owner['pid'],feedback.get('source_clock_id'))
                if q.shape!=(14,) or not np.isfinite(q).all() or not owner.get('feedback_healthy') or not 0<=value['source_age_s']<=.5:raise ValueError('真机反馈无效或过期')
                if source!=last_source:last_sdk=None;advanced=[0.,0.];last_source=source
                if len(sdk)!=2 or not all(type(v) in (int,float) and np.isfinite(v) for v in sdk):raise ValueError('SDK clocks invalid')
                if last_sdk:
                    for i in range(2):
                        if sdk[i]>last_sdk[i]:advanced[i]=began
                        elif sdk[i]<last_sdk[i]:advanced[i]=0.
                last_sdk=sdk
                if any(began-t>.5 for t in advanced):raise ValueError('等待两臂 SDK 反馈推进')
                self.hardware_feedback=value;self.hardware_received=time.monotonic();self.hardware_error=None
                with self.hardware_history_lock:self.hardware_history.append(value)
            except Exception as exc:
                reason=str(exc)
                if reason!=self.hardware_error:self.hardware_last_disconnect={'reason':reason,'at_s':time.time()}
                self.hardware_error=reason
            time.sleep(max(.001,1/30-(time.monotonic()-began)))

    def feedback_history_snapshot(self):
        # Never wait for page rendering or filesystem scans to publish feedback.
        with self.hardware_history_lock:return list(self.hardware_history)

    def feedback_diagnostics(self):
        received=getattr(self,'hardware_received',0.)
        return {'read_error':getattr(self,'hardware_error',None),
                'last_read_error':getattr(self,'hardware_last_disconnect',None),
                'last_received_age_s':time.monotonic()-received if received else None}

    def hardware_connected(self):
        return bool(getattr(self,'hardware_feedback',None) and not self.hardware_error and time.monotonic()-self.hardware_received<.5)

    def hardware_sync_active(self):
        return getattr(self,'control_mode','hardware_sync')=='hardware_sync' and self.hardware_connected()

    # --- Six independent state dimensions -------------------------------------
    # Every control's availability derives from these, never from ad hoc flags.
    # Tracking the robot, seeing the robot and being allowed to move it are three
    # different things: operating_mode=hardware_sync does NOT require owning control.
    def dsw_connection(self):
        if not self.live:return 'disconnected'
        return 'stale' if self.remote_error or time.monotonic()-self.remote_received>3 else 'connected'

    def robot_feedback(self):
        if not getattr(self,'hardware_feedback',None):return 'unavailable'
        return 'stale' if self.hardware_error or time.monotonic()-self.hardware_received>=.5 else 'fresh'

    def robot_control(self):
        # A connected owner can still forbid motion; that does not transfer ownership.
        status=getattr(self,'control_status',None)
        if not status or time.monotonic()-self.control_received>=2:return 'unavailable'
        owner=status.get('owner') or {}
        if not owner.get('connected') or type(owner.get('pid')) is not int:return 'unavailable'
        measured=(getattr(self,'hardware_feedback',None) or {}).get('owner') or {}
        if measured.get('pid') is None:return 'unavailable'
        return 'owned_by_workbench' if owner['pid']==measured['pid'] else 'owned_by_other'

    def execution_blocker(self):
        """One explanation for the status bar and every disabled execute button."""
        def result(code,title,reason='',action=None,detail='',since_ns=None):
            error=getattr(self,'fake_restart_error',None)
            if error and code!='ready':detail=(detail+'\n上次恢复失败：'+error).strip()
            return dict(code=code,title=title,reason=reason,action=action,detail=detail,since_ns=since_ns)
        if getattr(self,'fake_restart_in_progress',False):
            return result('owner_restarting','Fake 控制器恢复中','正在重启并核验新控制器；旧动作不会继续。')
        if getattr(self,'control_mode','simulation')=='simulation':
            return (result('dsw_unavailable','DSW 画面不可用','DSW 渲染未连接，请使用画面恢复入口。','reconnect_dsw')
                    if self.dsw_connection()!='connected' else result('ready','可执行'))
        status=getattr(self,'control_status',None)
        if not status or time.monotonic()-getattr(self,'control_received',0.)>=2:
            return result('owner_unavailable','控制网关不可用',getattr(self,'control_error',None) or '无法核对控制器身份与状态。')
        owner=status.get('owner') or {}
        if not isinstance(owner,dict) or not owner:
            return result('owner_unavailable','控制器未接入','网关在线，但未找到可核对的控制器。')
        state=owner.get('state')
        fake=(getattr(self,'config',{}).get('simulation_only') is True
              and owner.get('simulation_only') is True and owner.get('physical_hardware_connected') is False)
        if state in ('FAULT','FAULT_STOP_UNCONFIRMED','EMERGENCY'):
            diagnostic=owner.get('last_fault_diagnostic') or {}
            gap=diagnostic.get('tick_gap_s')
            raw=str(owner.get('reason') or state)
            reason=(f'控制循环停顿 {gap*1000:.1f} ms，超过 100 ms 保护线；执行已锁定。'
                    if 'control tick gap' in raw and type(gap) in (int,float) else '控制器报告：'+raw+'；执行已锁定。')
            since=diagnostic.get('wall_ns') if type(diagnostic.get('wall_ns')) is int else None
            if owner.get('recoverable_fault'):
                return result('owner_fault_recoverable','控制器故障',reason,'recover','确认故障后只重新开放预览，不恢复旧轨迹。',since)
            if fake:
                dsw_ready=self.dsw_connection()=='connected'
                return result('owner_fault_fake_restart','Fake 控制器故障',reason,
                              'restart_fake' if dsw_ready else 'reconnect_dsw',
                              ('可仅重启 Fake 控制器；已保存图像和标定保留，旧轨迹与预览作废。' if dsw_ready
                               else '请先恢复 Fake DSW 画面，再重启控制器。'),since)
            return result('owner_fault_physical','物理控制器故障',reason,None,
                          '请现场确认双臂已可靠支撑，再按现有断开与接入流程处理；网页不会自动重启物理控制器。',since)
        if not self.hardware_connected():
            return result('feedback_stale','关节反馈不可用',getattr(self,'hardware_error',None) or '未收到新鲜的关节反馈。')
        measured=(self.hardware_feedback or {}).get('owner') or {}
        if owner.get('pid')!=measured.get('pid'):
            return result('control_other_owner','控制器身份不一致','网关与反馈指向不同的 owner，不能确认本工作台持有控制权。')
        if not status.get('motion_enabled'):
            return result('owner_unavailable','控制器尚不能执行',status.get('motion_disabled_reason') or '网关未开放动作。')
        if fake and self.dsw_connection()!='connected':
            return result('dsw_unavailable','DSW 画面不可用','Fake 图像渲染未连接，请先恢复画面。','reconnect_dsw')
        return result('ready','可执行')

    def robot_source(self):
        # What is actually behind the hardware link. Distinct from the operating
        # mode: a simulated robot speaks the real protocol but is not metal, and
        # conflating the two is what made fake-device results read as physical.
        if self.control_mode!='hardware_sync':return 'simulated'
        owner=(getattr(self,'hardware_feedback',None) or {}).get('owner') or {}
        if owner.get('simulation_only') is True:return 'simulated'
        return 'physical' if owner.get('simulation_only') is False else 'unknown'

    def arrival_store(self, source):
        from arx_env.joint_head_arrival_stats import ArrivalStats
        if source not in ('physical','simulated'):raise ValueError('到位误差需要明确的真机或模拟来源')
        with self.lock:
            if not hasattr(self,'arrival_stores'):self.arrival_stores={}
            if source not in self.arrival_stores:
                self.arrival_stores[source]=ArrivalStats(self.calibration_root/'diagnostics'/'arrival-errors'/source,
                    self.config.get('robot_id','unassigned-simulation'),source)
            return self.arrival_stores[source]

    def arrival_state(self):
        source=self.robot_source()
        if not hasattr(self,'calibration_root'):return {'error':'到位统计目录尚未配置','robot_source':source}
        if source=='unknown':source='simulated' if getattr(self,'config',{}).get('simulation_only') is True else 'physical' if getattr(self,'config',{}).get('simulation_only') is False else 'unknown'
        try:
            result=self.arrival_store(source).summary()
            feedback=getattr(self,'hardware_feedback',None) or {}
            owner=feedback.get('raw_owner') or feedback.get('owner') or {}
            result['recording_supported']='arrival_events' in owner
            return result
        except (ValueError,OSError) as exc:return {'error':str(exc),'robot_source':source}

    def record_arrivals(self, owner, pending, dataset, source):
        preview=pending['preview']
        hashes={plan['sha256'] for plan in preview.get('plans',[])}
        recorded=pending.setdefault('arrival_recorded',set())
        for event in owner.get('arrival_events',[]):
            if event.get('status')!='success':continue
            if (event.get('owner_pid')!=preview['owner_pid'] or event.get('command_epoch')!=preview['command_epoch']
                    or event.get('plan_sha256') not in hashes or event.get('event_id') in recorded):continue
            self.arrival_store(source).record({**event,'batch_id':str(dataset.relative_to(self.output)),
                                               'workflow':pending['workflow']['kind']})
            recorded.add(event['event_id'])

    def task_state(self):
        if getattr(self,'pipeline_running',False):return 'reviewing' if getattr(self,'pipeline_review',None) else 'pipeline'
        status=str(getattr(self,'status','READY') or 'READY').upper()
        mapping={'READY':'idle','PLANNING':'planning','CAPTURING':'capturing','FITTING':'fitting','STOPPED':'stopped','REJECTED':'fault'}
        if status not in mapping:return 'fault'
        state=mapping[status]
        if state=='idle' and self.live and self.control_mode=='simulation':
            remote=str((self.remote or {}).get('status',(self.remote or {}).get('state',''))).upper()
            if remote in ('VALIDATING','EXECUTING','RUNNING','SETTLING','AWAITING_CAPTURE'):return 'executing'
        return state

    def mesh_view_mode(self):
        # Explicit overlay environment. The fallback only supplies the initial
        # value; once chosen it stays put regardless of control_mode (G5).
        # Read control_mode directly - going through active_environment() would
        # recurse, since that one now defers to this method.
        return getattr(self,'_mesh_view_mode',None) or ('hardware' if self.control_mode=='hardware_sync' else 'simulation')

    def control_state(self):
        blocker=self.execution_blocker()
        owner=((getattr(self,'control_status',None) or {}).get('owner') or {})
        return {'operating_mode':self.control_mode,'dsw_connection':self.dsw_connection(),'robot_feedback':self.robot_feedback(),
                'robot_control':self.robot_control(),'controller_state':owner.get('state'),
                'mesh_rendering_enabled':getattr(self,'mesh_rendering_enabled',False),
                'mesh_rendering_applied':(self.remote or {}).get('mesh_rendering_enabled')==getattr(self,'mesh_rendering_enabled',False) if self.live else True,
                'execution_blocker':blocker,'task_state':self.task_state(),'mesh_view_mode':self.mesh_view_mode()}

    def fake_restart_owner(self):
        """Inspect the exact simulated service before offering a restart."""
        if self.config.get('simulation_only') is not True or self.control_mode!='hardware_sync':
            raise ValueError('此操作仅限 Fake 真机')
        self.idle()
        if self.dsw_connection()!='connected':raise ValueError('请先恢复 Fake DSW 画面，再重启控制器')
        if getattr(self,'pipeline_running',False) or self.pending_capture or self.importing:
            raise ValueError('Fake 任务或采集仍在进行，不能重启控制器')
        status=self.control_request('state')
        owner=status.get('owner') or {}
        if (owner.get('simulation_only') is not True or owner.get('physical_hardware_connected') is not False
                or owner.get('state') not in ('FAULT','FAULT_STOP_UNCONFIRMED')
                or owner.get('recoverable_fault') or owner.get('automatic_enabled')
                or (owner.get('motion') or {}).get('status') in ('VERIFYING','RUNNING','SETTLING','AWAITING_CAPTURE')
                or type(owner.get('pid')) is not int or type(owner.get('command_epoch')) is not int):
            raise ValueError('只有空闲且确认为模拟设备的故障 owner 才能自助重启')
        try:
            unit=subprocess.run(['/usr/bin/systemctl','--user','show','-p','MainPID','--value','joint-head-fake-device.service'],
                                capture_output=True,text=True,timeout=4,check=True)
            if int(unit.stdout.strip())!=owner['pid']:raise ValueError('Fake 服务进程与控制网关 owner 不一致')
        except (OSError,subprocess.SubprocessError,ValueError) as exc:
            raise ValueError('无法核对 Fake 服务身份：'+str(exc)) from None
        return owner

    def fake_controller_restart_preview(self,body):
        if body:raise ValueError('Fake 重启预览不接受参数')
        with self.fake_restart_lock:
            if self.fake_restart_in_progress:raise ValueError('Fake 控制器正在恢复')
            owner=self.fake_restart_owner()
            token=secrets.token_urlsafe(24)
            self.fake_restart_pending={token:(time.monotonic()+30,owner['pid'],owner['command_epoch'])}
        return dict(token=token,owner_pid=owner['pid'],command_epoch=owner['command_epoch'],expires_in_s=30,
                    effect='只重启 simulation_only 的 Fake 控制器；保留已保存数据、标定与队列，旧预览和未完成动作作废。不会连接物理硬件。')

    def fake_controller_restart_confirm(self,body):
        if set(body)!={'token','owner_pid','command_epoch'} or not isinstance(body.get('token'),str):
            raise ValueError('需要本次 Fake 重启预览 token、owner PID 和动作版本')
        with self.fake_restart_lock:
            if self.fake_restart_in_progress:raise ValueError('Fake 控制器正在恢复')
            expected=self.fake_restart_pending.pop(body['token'],None)
            if (expected is None or time.monotonic()>expected[0]
                    or type(body['owner_pid']) is not int or type(body['command_epoch']) is not int
                    or (body['owner_pid'],body['command_epoch'])!=expected[1:]):
                raise ValueError('Fake 重启预览已过期或 owner 已变化')
            owner=self.fake_restart_owner()
            if (owner['pid'],owner['command_epoch'])!=expected[1:]:
                raise ValueError('Fake owner 已变化，请重新预览')
            self.fake_restart_in_progress=True
            self.fake_restart_error=None
            self.invalidate_reviews()
        try:
            unit='joint-head-fake-device.service'
            for action in ('stop','start'):
                if action=='start':
                    deadline=time.monotonic()+7
                    while time.monotonic()<deadline:
                        remote=self.remote_request('/state')
                        if remote.get('status')=='READY' and not remote.get('mirror'):break
                        time.sleep(.2)
                    else:raise ValueError('Fake DSW 未退出旧镜像，未启动新控制器')
                try:
                    subprocess.run(['/usr/bin/systemctl','--user',action,unit],
                                   capture_output=True,text=True,timeout=15,check=True)
                except (OSError,subprocess.SubprocessError):
                    raise ValueError('Fake 控制器服务'+('停止' if action=='stop' else '启动')+'失败；请检查服务状态') from None
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                try:
                    status=self.control_request('state');new=status.get('owner') or {}
                    if (status.get('motion_enabled') and new.get('pid')!=owner['pid']
                            and new.get('simulation_only') is True and new.get('physical_hardware_connected') is False
                            and new.get('state')=='READY' and new.get('connected')
                            and new.get('feedback_healthy') and not new.get('automatic_enabled')
                            and status.get('host_sample_recent')):
                        self.invalidate_reviews()
                        return dict(message='Fake 控制器已恢复；旧动作不会继续，请重新预览',old_owner_pid=owner['pid'],owner_pid=new['pid'])
                except ValueError:pass
                time.sleep(.2)
            raise ValueError('Fake 服务已启动但控制器未就绪；请查看当前故障状态')
        except Exception as exc:
            self.fake_restart_error=str(exc)
            raise
        finally:
            with self.fake_restart_lock:self.fake_restart_in_progress=False

    def environment_key(self):
        return 'hardware' if self.control_mode=='hardware_sync' else 'simulation'

    def environment_snapshot(self):
        return {'queue':self.queue,'plan':self.plan,'plan_geometry_id':self.plan_geometry_id,
                'selected_batch':self.selected_batch,'workflow':getattr(self,'workflow',{'kind':'head','head_result_id':None}),
                'wrist_selected':dict(getattr(self,'wrist_selected',{})),'calibration':self.calibration}

    def select_environment(self, saved):
        # An environment that has never been used comes back empty, which is a
        # valid state: switching must never discard the other environment's work.
        state=saved or {}
        self.queue=state.get('queue') or []
        self.plan=state.get('plan')
        self.plan_geometry_id=state.get('plan_geometry_id') or getattr(self,'geometry_id',None)
        self.selected_batch=state.get('selected_batch')
        self.workflow=state.get('workflow') or {'kind':'head','head_result_id':None}
        self.wrist_selected=dict(state.get('wrist_selected') or {})
        self.calibration=state.get('calibration')
        if self.calibration and not self.calibration_asset_matches(self.calibration):
            self.calibration=None
        (self.output/'realtime-calibration.json').write_text(json.dumps(self.calibration or {}))
        (self.output/'wrist-selected.json').write_text(json.dumps(self.wrist_selected))

    def set_control_mode(self, mode):
        if mode not in ('simulation','hardware_sync'):raise ValueError('invalid control mode')
        with self.mode_lock, self.lock:
            if mode==self.control_mode:return {'control_mode':mode,'hardware_sync_active':self.hardware_sync_active()}
            self.idle()
            owner=(getattr(self,'control_status',None) or {}).get('owner',{})
            if owner.get('automatic_enabled') or owner.get('automatic') or (owner.get('motion') or {}).get('status') in ('VERIFYING','RUNNING','SETTLING','AWAITING_CAPTURE'):
                raise ValueError('真机轨迹仍在执行，请先停止或等待完成再切换模式')
            # Switching never sends a target or changes the robot's gain/pedal state.
            # Stash the outgoing environment and load the incoming one: each keeps
            # its own queue, plan, batch and overlay selection.
            environments=getattr(self,'environments',None)
            if not isinstance(environments,dict):environments=self.environments={'simulation':{},'hardware':{}}
            environments[self.environment_key()]=self.environment_snapshot()
            self.invalidate_reviews()
            self.control_mode=mode
            self.select_environment(environments.get(self.environment_key()))
            self.tokens.clear();self.hardware_tokens.clear()
            getattr(self,'calibration_previews',{}).clear()
            self.persist()
            if self.live:
                try:
                    # Push the incoming environment's overlay choice, or clear it;
                    # clearing the mirror stays last as the marker that pure
                    # simulation took over.
                    if self.calibration:self.remote_request('/calibration',self.calibration.get('parameters') or {})
                    else:self.remote_request('/calibration',{'clear':True})
                    if mode=='simulation':self.remote_request('/mirror',{'clear':True})
                    self.mirror_error=None
                except ValueError as exc:self.mirror_error=str(exc)
            return {'control_mode':mode,'hardware_sync_active':self.hardware_sync_active(),'queue_length':len(self.queue)}

    def mirror_hardware(self):
        sequence=0;mirroring=False;session=secrets.token_hex(8)
        while True:
            began=time.monotonic()
            try:
                with self.mode_lock:
                    if self.hardware_sync_active():
                        value=self.hardware_feedback;owner=value['owner'];q=owner['calibration_feedback']['qpos'];sequence+=1
                        self.remote_request('/mirror',{'source_id':session+':'+str(owner['pid']),'sequence':sequence,'source_age_s':value['source_age_s']+time.monotonic()-self.hardware_received,'q':q[:6]+q[7:13],'jaw_gap_mm':np.clip(feedback_gaps(q),0,83).tolist()})
                        mirroring=True;self.mirror_error=None
                    elif mirroring:
                        self.remote_request('/mirror',{'clear':True});mirroring=False
            except Exception as exc:self.mirror_error=str(exc)
            # DSW failures only affect this read-only mirror, never robot control.
            time.sleep(max(.001,1/30-(time.monotonic()-began)))

    def current_pose(self):
        """The 12 joint angles the next motion starts from.

        Measured owner feedback whenever it is available, otherwise the angle set this
        workbench believes it is at. A hardware execution with no fresh feedback is
        refused before it reaches here."""
        feedback=(getattr(self,'hardware_feedback',None) or {}).get('owner',{}).get('calibration_feedback',{})
        q=feedback.get('qpos')
        if isinstance(q,list) and len(q)==14 and self.hardware_sync_active():
            return np.asarray(q[:6]+q[7:13],float)
        return np.asarray(self.q,float)

    def queue_is_plan_checked(self,rows):
        """Match an unchanged generated queue to the current model.

        Frozen calibration seeds are replayed without collision screening;
        edited/manual queues still require their own checks.
        """
        plan=getattr(self,'plan',None) or {}
        if not plan or plan.get('plan_complete') is False:return False
        if getattr(self,'plan_geometry_id',None)!=self.geometry_id:return False
        if len(rows)!=len(self.queue):return False
        return all(np.allclose(self.geometry._q(a['q']),self.geometry._q(b['q']),atol=1e-9) for a,b in zip(rows,self.queue))

    def bounded_joint_pose(self,value):
        from robot_workbench.joint_bounds import project_joint_positions
        raw=self.geometry._q(value)
        bounded,corrections=project_joint_positions(raw.tolist(),np.asarray(self.geometry.limits,float).tolist(),
            environment=transport_environment(self.config))
        return np.asarray(bounded),dict(requested_q=raw.tolist(),command_q=bounded,corrections=corrections)

    def require_checked_path(self,rows,start):
        """Refuse motion whose swept path this workbench has not geometrically checked.

        Uses the planner's own `path_check`: 0.04 rad steps, full-mesh convex hulls
        with the jaws held open, chassis and support-plane screening. The operator's
        clearance tick describes the physical scene; it cannot tell whether the model
        says this trajectory runs the left arm through the right one. Until 2026-09-22
        only the planner and the no-renderer manual queue ever checked, so a hand-made
        target could self-collide and still be executed on hardware."""
        geometry=self.geometry.with_gaps(83.,83.)
        previous=np.asarray(start,float)
        for index,row in enumerate(rows):
            target=self.geometry._q(row['q'])
            check=geometry.path_check(previous,target)
            if not check['ok']:
                pairs=check.get('collisions') or []
                detail=('超出关节限位' if not pairs else '模型判定干涉/自碰撞：'+'；'.join(
                    '/'.join(str(part) for part in pair)+(('（求解器不确定，按保守处理）' if len(pair)>2 else ''))
                    for pair in pairs))
                raise ValueError(f'第 {index+1} 个路径点未通过路径检查（路径位置 {check["fraction"]:.2f}）：{detail}；请改用无自碰撞的目标')
            previous=target
        return len(rows)

    def hardware_trajectory_preview(self,body,*,review=None):
        with self.mode_lock:
            review=self.review_snapshot() if review is None else review
            self.require_review(review)
            if not self.hardware_connected():raise ValueError('真机反馈已断开；请重新检查接入状态')
            rows=json.loads(json.dumps(body.get('waypoints'),allow_nan=False))
            if not isinstance(rows,list) or not rows:raise ValueError('轨迹须包含路径点')
            waypoints=[];checked_waypoints=[];boundary_rows=[]
            for row in rows:
                q=self.geometry._q(row['q']);speeds=np.asarray(row.get('speeds',[.2]*12),float)
                bounded,boundary=self.bounded_joint_pose(q);boundary_rows.append(boundary)
                if speeds.shape!=(12,) or not np.isfinite(speeds).all() or np.any(speeds<=0):raise ValueError('真机各关节速度须为有限正数')
                waypoint={'q':q.tolist(),'speeds':speeds.tolist()}
                if 'jaw_gap_mm' in row:waypoint['jaw_gap_mm']=jaw_gaps(row['jaw_gap_mm']).tolist()
                if 'jaw_speed_mm_s' in row:waypoint['jaw_speed_mm_s']=row['jaw_speed_mm_s']
                waypoints.append(waypoint)
                checked_waypoints.append(dict(waypoint,q=bounded.tolist()))
            checked=self.queue_is_plan_checked(rows);start,start_boundary=self.bounded_joint_pose(self.current_pose())
        progress=self.begin_preflight('正在检查路径')
        try:
            if not checked:self.require_checked_path(checked_waypoints,start)
            self.require_review(review)
            self.preflight_progress(progress,'正在生成执行预览并核验冻结路径')
            result=self.control_request('trajectory-preview',{'waypoints':waypoints},review=review)
            if start_boundary['corrections'] or any(row['corrections'] for row in boundary_rows):
                plans=result.get('plans') or []
                if len(plans)!=len(boundary_rows) or any(not np.allclose(
                        [plan['q_target'][i] for i in (*range(6),*range(7,13))],row['command_q'],atol=1e-12,rtol=0)
                        for plan,row in zip(plans,boundary_rows)):
                    self.hardware_tokens.discard(result.get('token'))
                    raise ValueError('控制器合法目标与已检查的边界投影不同，请重新生成执行预览')
                result=dict(result,position_boundary=dict(environment=transport_environment(self.config),
                    start=start_boundary,waypoints=boundary_rows))
            self.preflight_progress(progress,'执行预览已就绪，等待审阅',done=True)
            return result
        except Exception as exc:
            self.preflight_progress(progress,'执行预览未通过',error=exc)
            raise

    def queue_pose(self,body,*,review=None):
        with self.mode_lock,self.lock:
            self.idle();review=self.review_snapshot() if review is None else review
            self.require_review(review)
            q=self.geometry._q(body['q']);gaps=jaw_gaps(body.get('jaw_gap_mm',self.jaw_mm))
            bounded,boundary=self.bounded_joint_pose(q)
            speeds=np.asarray(body.get('speeds',[.2]*12),float);jaw_speed=body.get('jaw_speed_mm_s',10.)
            if speeds.shape!=(12,) or not np.isfinite(speeds).all() or np.any(speeds<=0):raise ValueError('invalid queued speeds')
            if type(jaw_speed) not in (int,float) or not np.isfinite(jaw_speed) or jaw_speed<=0:raise ValueError('positive jaw speed required')
            hardware=self.control_mode=='hardware_sync'
            if hardware and not self.hardware_connected():raise ValueError('真机反馈已断开；请重新检查接入状态')
            start,start_boundary=self.bounded_joint_pose(self.current_pose() if hardware else self.q)
            queue=json.loads(json.dumps(self.queue));geometry=self.geometry
        progress=self.begin_preflight('正在检查待加入队列的路径')
        try:
            if hardware:self.require_checked_path([{'q':bounded.tolist()}],start)
            elif not geometry.with_gaps(83.,83.).path_check(start,bounded)['ok']:raise ValueError('目标未通过路径检查')
            with self.mode_lock,self.lock:
                with self.review_lock:
                    self.require_review(review);self.idle()
                    if self.queue!=queue:raise ValueError('队列已变化，请重新加入姿态')
                    self.preflight_progress(progress,'已加入队列',done=True)
                    self.invalidate_reviews()
                    if not self.queue:self.plan_geometry_id=self.geometry_id
                    self.queue.append({'name':f'手动姿态 {len(self.queue)+1}','q':q.tolist(),'jaw_gap_mm':gaps.tolist(),'capture':False,'jaw_speed_mm_s':jaw_speed,'speeds':speeds.tolist()})
                    if boundary['corrections'] or start_boundary['corrections']:
                        self.queue[-1]['position_boundary']=dict(start=start_boundary,target=boundary,environment=transport_environment(self.config))
                    self.plan=None
                self.persist()
            return {'message':'已加入队列'}
        except Exception as exc:
            self.preflight_progress(progress,'未加入队列',error=exc)
            raise

    def standard_motion_seed_active(self):
        return (self.plan or {}).get('motion_seed') == self.STANDARD_MOTION_SEED

    def hardware_calibration_cameras(self):
        cameras={'head':self.hardware_camera}
        if self.workflow['kind']=='wrist':cameras.update(self.wrist_cameras)
        required=('head','left','right') if self.workflow['kind']=='wrist' else ('head',)
        if set(cameras)!=set(required) or any(c is None for c in cameras.values()):raise ValueError('标定所需相机尚未配置')
        intrinsics={}
        for name,camera in cameras.items():
            _,meta=camera.read_calibration_frame();intrinsics[name]=meta['intrinsics']
            if meta.get('serial')!=intrinsics[name].get('serial'):raise ValueError('实际相机序列号与内参不匹配：'+name)
            expected=self.config.get('hardware_camera_serials',{}).get(name)
            if expected is not None and meta.get('serial')!=expected:raise ValueError('实际相机序列号与配置角色不匹配：'+name)
        return cameras,intrinsics

    @staticmethod
    def calibration_waypoints(rows):
        return [dict(row,jaw_gap_mm=[80.,80.],jaw_speed_mm_s=10.) for row in rows]

    def preview_calibration(self,*,review=None):
        with self.mode_lock:
            review=self.review_snapshot() if review is None else review
            self.require_review(review)
            workflow=dict(self.workflow)
            self.idle()
            if self.control_mode!='hardware_sync' or not self.hardware_connected() or not self.control_available():raise ValueError('真机标定需要有效反馈与控制权；不会回退到仿真')
            owner=self.control_status['owner']
            if 'calibration_capture_ack_v1' not in owner.get('capabilities',[]):raise ValueError('当前控制器尚未支持到位采样确认；请在设备可用时更新控制器')
            if not self.queue:raise ValueError('请先生成标定计划')
            # The frozen standard seed was already reviewed once.  Do not rerun
            # planner coverage/geometry gates on every capture; the controller still
            # owns live feedback, limits, stop and protocol checks.
            if not self.standard_motion_seed_active() and self.plan_geometry_id!=self.geometry_id:raise ValueError('请先按当前模型生成标定计划')
            if not self.standard_motion_seed_active() and (self.plan or {}).get('plan_complete') is False:raise ValueError('当前标定计划未通过采样检查，请重新规划')
            if any(row.get('jaw_gap_mm')!=[80.,80.] for row in self.queue):raise ValueError('标定计划要求左右夹爪均为80 mm（含过渡点）；请重新生成计划并审阅')
            frozen=json.loads(json.dumps(self.queue));contract=(self.plan or {}).get('wrist_contract')
            if self.workflow['kind']=='wrist':
                if not contract or contract.get('environment')!='hardware' or contract['head_snapshot']['id']!=self.workflow['head_result_id']:raise ValueError('请重新生成当前头标定依赖下的真机双腕计划')
                self.require_joint_head_dependency({'parameters':contract['head_snapshot']})
            elif contract:raise ValueError('计划类型与头部标定不符')
        cameras,intrinsics=self.hardware_calibration_cameras()
        for name,camera in cameras.items():
            _,meta=camera.read_calibration_frame()
            if meta.get('serial')!=intrinsics[name].get('serial'):raise ValueError('相机序列号与内参不匹配：'+name)
        rows=[];sample=0;capture_id=secrets.token_hex(12)
        for i,w in enumerate(frozen):
            capture=w.get('capture') is not False
            w['sample_id']=capture_id+'-'+str(i)
            if capture:
                w.setdefault('split','holdout' if sample%5==4 else 'fit');sample+=1
            rows.append({**{k:w[k] for k in ('q','speeds','jaw_gap_mm','jaw_speed_mm_s') if k in w},'capture':capture,**({'sample_id':w['sample_id']} if capture else {})})
        if not sample:raise ValueError('当前队列没有采样点')
        preview=self.control_request('trajectory-preview',{'waypoints':rows},calibration=True,review=review)
        preview['collision_checked']=False
        with self.review_lock:
            self.require_review(review)
            self.calibration_previews={preview['token']:{'preview':preview,'queue':frozen,'contract':contract,'workflow':workflow,'intrinsics':intrinsics,'pipeline_run_id':review[4],'expires':time.monotonic()+preview.get('expires_in_s',30)}}
        return {**preview,'environment':'hardware','runtime_environment':transport_environment(self.config),'samples':sample,'kind':workflow['kind']}

    def resident_capture_budget(self,*,check=True):
        """One immutable wall/monotonic anchor for this mounted fake resident.

        Independent fake entries without a resident contract keep their existing
        finite per-batch wall fallback. Physical behavior is unchanged.
        """
        if transport_environment(self.config)!='fake' or self.config.get('resident_deadline_wall_ns') is None:return None
        resident=self.config['resident_deadline_wall_ns']
        if type(resident) is not int or resident<=0:raise ValueError('Fake resident_deadline_wall_ns 必须是有效的固定墙钟期限')
        budget=getattr(self,'_resident_capture_budget',None)
        if budget is None:
            now=time.monotonic();wall=time.time_ns();remaining=max(0.,(resident-wall)/1e9)
            budget=dict(resident_deadline_wall_ns=resident,resident_deadline_monotonic_s=now+remaining,
                wall_deadline_ns=resident-45_000_000_000,deadline_monotonic_s=now+remaining-45.,
                anchored_wall_ns=wall,anchored_monotonic_s=now,cleanup_reserve_s=45.)
            self._resident_capture_budget=budget
        if resident!=budget['resident_deadline_wall_ns']:raise ValueError('Fake 驻留期限已变化，不能延长当前采集预算')
        if check and (time.monotonic()>=budget['deadline_monotonic_s'] or time.time_ns()>=budget['wall_deadline_ns']):
            raise ValueError('Fake 采集已达到固定驻留墙钟期限（保留45秒清退时间）')
        return dict(budget)

    def confirm_calibration(self,body):
        self.idle()
        if getattr(self,'pipeline_running',False):self.pipeline_stop()
        if set(body)!={'token','confirmation','physical_clearance'} or body['confirmation']!='START_REVIEWED_SINGLE_ARM' or body['physical_clearance'] is not True:raise ValueError('请明确确认完整轨迹与现场净空')
        pending=self.calibration_previews.pop(body['token'],None)
        if not pending or pending['expires']<time.monotonic() or pending['workflow']!=self.workflow or pending.get('pipeline_run_id')!=getattr(self,'pipeline_run_id',None):raise ValueError('标定预览已过期或依赖变化')
        self.resident_capture_budget()  # Reject an exhausted fake contract before confirming any motion.
        if self.control_mode!='hardware_sync' or not self.control_available():raise ValueError('真机控制或反馈不可用')
        root=self.output/('batch-hardware-'+secrets.token_hex(5));dataset=root/'capture';dataset.mkdir(parents=True)
        manifest={'environment':'hardware','runtime_environment':transport_environment(self.config),'robot_source':self.robot_source(),'simulation_only':self.robot_source()!='physical','robot_id':self.store.robot_id,'frames':[],'planned_frames':pending['queue'],'camera':pending['intrinsics']['head']}
        manifest.update(geometry_id=self.geometry_id,asset_geometry_id=self.config.get('asset_geometry_id'),
                        asset_seal_sha256=self.config.get('asset_seal_sha256'),jaw_mapping=mapping_identity())
        if pending['contract']:manifest['wrist_contract']=pending['contract']
        (dataset/'dataset.json').write_text(json.dumps(manifest,allow_nan=False))
        self.active_capture={'batch_id':str(dataset.relative_to(self.output)),'captured':0,
                             'pipeline_run_id':pending.get('pipeline_run_id'),'kind':pending['workflow']['kind'],
                             'expected_count':sum(row.get('capture') is not False for row in pending['queue'])}
        self.status='CAPTURING';self.reason=None
        if not getattr(self,'pipeline_running',False):self.stop.clear()
        self.tokens.clear()
        def run():
            try:
                self.collect_hardware(pending,dataset,manifest)
                with self.lock:self.status='STOPPED' if self.stop.is_set() else 'READY'
            except Exception as exc:
                with self.lock:self.status='REJECTED';self.reason=str(exc)
        worker=threading.Thread(target=run,daemon=True)
        try:
            self.control_request('motion-confirm',body)
            worker.start()
        except Exception as exc:
            self.status='REJECTED';self.reason=str(exc)
            try:
                self.control_request('stop',{'owner_pid':pending['preview']['owner_pid'],'command_epoch':pending['preview']['command_epoch']})
            except Exception:pass
            (dataset/'completion.json').write_text(json.dumps({'passed':False,'captured':0,'reason':str(exc),
                'environment':'hardware','runtime_environment':transport_environment(self.config)}));raise
        label='Fake' if transport_environment(self.config)=='fake' else '真机'
        return {'message':'已启动'+label+'标定：每个采样点保存成功后才继续','environment':'hardware',
                'runtime_environment':transport_environment(self.config),'batch_id':str(dataset.relative_to(self.output))}

    def collect_hardware(self,pending,dataset,manifest):
        from arx_env.joint_head_workbench_capture import capture_sample
        from robot_workbench.sample_clock import assert_progress, clock_spec, sample_stamp
        environment=transport_environment(self.config)
        label='Fake' if environment=='fake' else '真机'
        source_clock=None;first_source_stamp=None;last_source_stamp=None;ready_source=None;ready_step=None
        preview=pending['preview'];pid=preview['owner_pid'];epoch=preview['command_epoch']
        window_v2=None
        tentative=None;created=[]
        seen=set();expected={w['sample_id']:w for w in pending['queue'] if w.get('capture') is not False}
        started=time.monotonic();started_wall_ns=time.time_ns()
        legacy_budget=float(preview['total_motion_duration_s'])+len(pending['queue'])*20+30
        budget=None;deadline=started+legacy_budget
        last_motion={}
        def check_budget():
            if budget:self.resident_capture_budget()
            elif time.monotonic()>deadline:raise ValueError(label+'采集超时')
        def timing():
            return dict(runtime_environment=environment,started_wall_ns=started_wall_ns,observed_wall_ns=time.time_ns(),
                elapsed_wall_s=max(0.,time.monotonic()-started),deadline_monotonic_s=deadline,
                budget_origin='fixed_resident_wall' if budget else 'independent_fake_finite_wall_fallback' if environment=='fake' else 'physical_legacy_wall',
                resident_budget=budget,legacy_wall_budget_s=legacy_budget,
                planned_motion_duration_s=float(preview['total_motion_duration_s']),clock=source_clock,
                first_source_stamp=first_source_stamp,last_source_stamp=last_source_stamp,
                source_span_s=(last_source_stamp[0]-first_source_stamp[0])/1e9 if first_source_stamp is not None else None,
                last_motion=last_motion,planned_samples=len(expected),accepted_samples=len(seen),owner_pid=pid,command_epoch=epoch)
        def save():
            tmp=dataset/'dataset.json.tmp';tmp.write_text(json.dumps(manifest,allow_nan=False));tmp.replace(dataset/'dataset.json')
        def checked_owner(owner=None):
            nonlocal window_v2,source_clock,first_source_stamp,last_source_stamp,last_motion
            if owner is None:owner=self.control_request('state').get('owner') or {}
            if owner.get('pid')!=pid or owner.get('command_epoch')!=epoch:
                raise ValueError('控制器身份或动作版本已变化，采集中止'+('：'+str(owner['reason']) if owner.get('reason') else ''))
            if not owner.get('feedback_healthy') or owner.get('state') in ('FAULT','FAULT_STOP_UNCONFIRMED','EMERGENCY','STOPPED','DISCONNECTED'):
                raise ValueError(owner.get('reason') or label+'反馈失效或已停止')
            if environment=='fake':
                if owner.get('environment')!='fake' or owner.get('mode')!='simulation' or owner.get('simulation_only') is not True:
                    raise ValueError('Fake 采集必须绑定明确的模拟 Owner，不能跨设备环境')
                feedback=owner.get('calibration_feedback') or {}
                source_clock=source_clock or clock_spec('fake',clock_domain=feedback.get('clock_domain'),source_clock_id=feedback.get('source_clock_id'))
                point=sample_stamp(feedback,source_clock,wall_key='sample_wall_ns')
                last_source_stamp=assert_progress(last_source_stamp,point,allow_same=True)
                if first_source_stamp is None:first_source_stamp=point
                manifest['clock']=source_clock
            elif owner.get('environment') not in (None,'physical') or owner.get('simulation_only') is True:
                raise ValueError('物理采集拒绝 Fake Owner')
            if window_v2 is None:window_v2='calibration_capture_window_v2' in (owner.get('capabilities') or [])
            last_motion={key:(owner.get('motion') or {}).get(key) for key in ('status','sample_id','waypoint_index','capture_ready_source_ns','capture_ready_physics_step')}
            return owner
        def same_window(owner,identity,ready_ns):
            motion=owner.get('motion') or {}
            return (motion.get('status')=='AWAITING_CAPTURE' and motion.get('sample_id')==identity and motion.get('capture_ready_wall_ns')==ready_ns
                    and (environment!='fake' or (motion.get('capture_ready_source_ns')==ready_source
                         and motion.get('capture_ready_physics_step')==ready_step and motion.get('capture_clock_domain')=='simulation'
                         and motion.get('capture_source_clock_id')==source_clock['source_clock_id'])))
        def reject_window(identity,ready_ns,reason):
            nonlocal tentative
            if tentative is not None:
                manifest['frames'][:]=[r for r in manifest['frames'] if r is not tentative]
            for path in created:path.unlink(missing_ok=True)
            created.clear();tentative=None
            manifest.setdefault('capture_rejections',[]).append({'sample_id':identity,'capture_ready_wall_ns':ready_ns,'reason':str(reason),'at_s':time.time()})
            save()
        def write_sample(path,raw):
            # Exclusive creation keeps rollback from deleting pre-existing batch files.
            with path.open('xb') as stream:
                created.append(path);stream.write(raw)
        try:
            budget=self.resident_capture_budget(check=False)
            if budget:deadline=budget['deadline_monotonic_s']
            check_budget()
            cameras,intrinsics=self.hardware_calibration_cameras()
            if intrinsics!=pending['intrinsics']:raise ValueError('预览后实际相机内参已变化')
            while True:
                if self.stop.is_set() or self.control_mode!='hardware_sync':raise ValueError(label+'采集已停止')
                check_budget()
                owner=checked_owner();motion=owner.get('motion') or {}
                self.record_arrivals(owner,pending,dataset,manifest['robot_source'])
                if motion.get('status')=='AWAITING_CAPTURE':
                    identity=motion.get('sample_id');w=expected.get(identity)
                    if w is None:raise ValueError('采样身份不匹配')
                    if identity in seen:
                        self.stop.wait(.03);continue  # ACK is queued; wait for the owner tick without acknowledging twice.
                    ready_ns=motion.get('capture_ready_wall_ns')
                    if type(ready_ns) is not int or ready_ns<=0:raise ValueError('控制器缺少采样就绪时间，拒绝使用缓存帧')
                    capture_time={'not_before_ns':ready_ns}
                    if environment=='fake':
                        ready_source,ready_step=sample_stamp(dict(host_receive_ns=ready_ns,
                            clock_domain=motion.get('capture_clock_domain'),source_clock_id=motion.get('capture_source_clock_id'),
                            source_time_ns=motion.get('capture_ready_source_ns'),physics_step=motion.get('capture_ready_physics_step')),source_clock)
                        if ready_source>last_source_stamp[0] or ready_step>last_source_stamp[1]:raise ValueError('Fake 就绪边界晚于实际反馈')
                        capture_time=dict(environment='fake',source_clock_id=source_clock['source_clock_id'],not_before_source_ns=ready_source)
                    required={w['observer']:[30 if w['observer']=='left' else 24]} if pending['workflow']['kind']=='wrist' else {'head':[w.get('expected_marker_id') or (24 if w.get('arm')=='left' else 30)]}
                    limit=time.monotonic()+2;sample=None
                    while True:
                        if self.stop.is_set() or self.control_mode!='hardware_sync':raise ValueError(label+'采样已停止')
                        if budget:check_budget()
                        try:sample=capture_sample(cameras,intrinsics,self.feedback_history_snapshot,feedback_diagnostics=self.feedback_diagnostics,owner_pid=pid,command_epoch=epoch,required_markers=required,**capture_time);break
                        except Exception as exc:
                            if self.stop.is_set() or self.control_mode!='hardware_sync':raise ValueError(label+'采样已停止') from exc
                            if window_v2 and not same_window(checked_owner(),identity,ready_ns):
                                reject_window(identity,ready_ns,exc);break
                            if not isinstance(exc,ValueError) or time.monotonic()>limit:raise
                            self.stop.wait(.05)
                    if sample is None:continue
                    if window_v2 and not same_window(checked_owner(),identity,ready_ns):
                        reject_window(identity,ready_ns,'采样过程中窗口撤销');continue
                    observer=w.get('observer','head');paired=sample['rows'][observer]
                    row={**sample['row'],'qpos':paired['qpos'],'q_reported_rad':paired['q_reported12'],'bracket_gap_s':paired['bracket_gap_s'],'pose_span_rad':paired['pose_span_rad'],'camera_metadata':{name:value['metadata'] for name,value in sample['rows'].items()},'jaw_gap_mm':feedback_gaps(paired['qpos']),'id':identity,'planned_id':w.get('id',identity),'split':w['split'],'valid_for_fit':True,'environment':'hardware'}
                    row.update({k:w[k] for k in ('observer','target') if k in w});row['camera_images']={};row['capture_ready_wall_ns']=ready_ns
                    if environment=='fake':row.update(capture_ready_source_ns=ready_source,clock=source_clock,runtime_environment='fake')
                    for name,raw in sample['images'].items():
                        filename=identity+'-'+name+'.png';write_sample(dataset/filename,raw);row['camera_images'][name]=filename
                    for name,raw in sample.get('raw_images',{}).items():write_sample(dataset/(identity+'-'+name+'-raw.jpg'),raw)
                    row['image']=row['camera_images']['head'];row['detections']=row['camera_detections'].get('head',{})
                    write_sample(dataset/(identity+'.json'),json.dumps(row,allow_nan=False).encode())
                    if pending['contract'] and any(sample['cameras'][side]!=pending['contract']['cameras'][side] for side in ('left','right')):raise ValueError('采样时相机内参已变化，必须重新规划')
                    tentative=row;manifest['frames'].append(row);manifest['cameras']=sample['cameras'];save()
                    (dataset/'camera.json').write_text(json.dumps(sample['cameras']['head']))
                    # Publish and sync every sample artifact before allowing the next movement.
                    for artifact in dataset.iterdir():
                        if artifact.is_file():
                            with artifact.open('rb') as stream:os.fsync(stream.fileno())
                    fd=os.open(dataset,os.O_RDONLY|os.O_DIRECTORY)
                    try:os.fsync(fd)
                    finally:os.close(fd)
                    if self.stop.is_set() or self.control_mode!='hardware_sync':raise ValueError('采样期间已停止，不确认继续动作')
                    if budget:check_budget()
                    ack={'owner_pid':pid,'command_epoch':epoch,'sample_id':identity}
                    if window_v2:ack['capture_ready_wall_ns']=ready_ns
                    try:self.control_request('capture-ack',ack)
                    except Exception:
                        if not window_v2:raise
                        # A rejected or transport-uncertain ACK cannot commit a frame; require the owner's receipt.
                    if window_v2:
                        while True:
                            if self.stop.is_set() or self.control_mode!='hardware_sync':raise ValueError('采样期间已停止，不确认继续动作')
                            owner=self.control_request('state').get('owner') or {};receipt=owner.get('last_capture_ack') or {}
                            accepted=receipt.get('sample_id')==identity and receipt.get('capture_ready_wall_ns')==ready_ns
                            if environment=='fake':accepted=accepted and receipt.get('command_epoch')==epoch and receipt.get('source_clock_id')==source_clock['source_clock_id'] and receipt.get('capture_ready_source_ns')==ready_source
                            if accepted and owner.get('pid')==pid:break  # Keep a confirmed frame if a later fault changes the epoch.
                            checked_owner(owner)
                            if not same_window(owner,identity,ready_ns):
                                reject_window(identity,ready_ns,'窗口撤销或旧窗口 ACK 未确认');break
                            if budget:check_budget()
                            elif time.monotonic()>deadline:raise ValueError('等待采样 ACK 确认超时')
                            self.stop.wait(.03)
                        if tentative is None:continue
                    seen.add(identity);tentative=None;created.clear()
                    with self.lock:self.active_capture={'batch_id':str(dataset.relative_to(self.output)),'captured':len(seen),
                                                       'pipeline_run_id':pending.get('pipeline_run_id'),'kind':pending['workflow']['kind'],
                                                       'expected_count':len(expected)}
                elif len(seen)==len(expected) and owner.get('state')=='READY' and not owner.get('automatic_enabled'):
                    break
                self.stop.wait(.03)
            (dataset/'completion.json').write_text(json.dumps({'passed':True,'captured':len(seen),'accepted':len(seen),'failed':0,
                'environment':'hardware','runtime_environment':environment,'timing':timing()}))
        except Exception as exc:
            if tentative is not None or created:
                try:reject_window(identity,ready_ns,exc)
                except OSError:pass
            try:
                self.control_request('stop',{'owner_pid':pid,'command_epoch':epoch})
            except Exception:pass
            try:
                manifest['failure']={'reason':str(exc),'at_s':time.time(),'runtime_environment':environment,'timing':timing()}
                evidence=getattr(exc,'evidence',{})
                for name,raw in evidence.get('raw_images',{}).items():(dataset/('failed-'+name+'-raw.jpg')).write_bytes(raw)
                manifest['failure']['evidence']={k:v for k,v in evidence.items() if k not in ('images','raw_images')}
                save()
                (dataset/'completion.json').write_text(json.dumps({'passed':False,'captured':len(manifest['frames']),'reason':str(exc),
                    'environment':'hardware','runtime_environment':environment,'timing':timing()}))
            except OSError:pass
            raise
        finally:
            with self.lock:self.load_batch(str(dataset.relative_to(self.output)));self.persist()

    def remote_request(self,path,body=None,binary=False):
        request=Request(self.live['url'].rstrip('/')+path,data=None if body is None else json.dumps(body,allow_nan=False).encode(),headers={'Host':'127.0.0.1:'+str(self.live.get('remote_port',8118)),'Authorization':'Bearer '+self.live_token,'Content-Type':'application/json'})
        try:
            with self.http.open(request,timeout=3) as response:raw=response.read()
        except HTTPError as exc:
            try:reason=json.loads(exc.read()).get('error',str(exc))
            except Exception:reason=str(exc)
            raise ValueError('DSW: '+reason) from None
        except (OSError,URLError) as exc:raise ValueError('DSW unavailable: '+str(exc)) from None
        return raw if binary else json.loads(raw)

    @staticmethod
    def read_stream_frame(response):
        line=response.readline(4096)
        while line in (b'\r\n',b'\n'):line=response.readline(4096)
        if line.strip()!=b'--frame':raise ValueError('invalid frame boundary')
        headers={}
        for _ in range(16):
            line=response.readline(4096)
            if line==b'\r\n':break
            key,value=line.decode('ascii').split(':',1);headers[key.lower()]=value.strip()
        size=int(headers['content-length'])
        if not 0<size<=2*1024*1024:raise ValueError('invalid frame size')
        raw=response.read(size)
        if len(raw)!=size:raise ValueError('truncated frame')
        return raw,float(headers['x-frame-timestamp'])

    def monitor_frames(self,view):
        request=Request(self.live['url'].rstrip('/')+'/stream.mjpg?view='+view,
                        headers={'Authorization':'Bearer '+self.live_token,'Host':'127.0.0.1:'+str(self.live.get('remote_port',8118))})
        while True:
            try:
                with self.http.open(request,timeout=5) as response:
                    while True:
                        raw,stamp=self.read_stream_frame(response)
                        self.live_frames[view]=(raw,stamp,time.monotonic())
            except (OSError,ValueError,KeyError,URLError) as exc:
                self.live_frame_errors[view]={'reason':str(exc),'at_s':time.time()}
                time.sleep(.2)

    def live_frame(self,view):
        self.require_remote()
        frame=self.live_frames.get(view)
        if frame is None:raise ValueError('DSW frame stream connecting')
        raw,stamp,received=frame
        # An idle preview is still current; active/head frames must keep arriving.
        if time.monotonic()-received>3 and (view!='preview' or stamp!=(self.remote or {}).get('frame_timestamp_s',{}).get(view)):
            raise ValueError('DSW frame stream stale')
        return raw

    def require_remote(self):
        remote=self.remote
        if not remote or time.monotonic()-self.remote_received>3 or self.remote_error:raise ValueError('DSW feedback unavailable or stale; execution disabled')
        if remote.get('mode')!='simulation':raise ValueError('Unverified hardware backend refused')
        return remote

    def monitor_remote(self):
        while True:
            try:
                remote=self.remote_request('/state')
                q=np.asarray(remote.get('q',remote.get('qpos')),float)
                if remote.get('mode')!='simulation' or q.shape!=(12,) or not np.isfinite(q).all():raise ValueError('invalid simulation feedback')
                stamp=float(remote['timestamp_s'])
                if not np.isfinite(stamp):raise ValueError('invalid DSW timestamp')
                if stamp!=self.last_remote_timestamp:self.last_remote_timestamp=stamp;self.remote_advanced=time.monotonic()
                if time.monotonic()-self.remote_advanced>3:raise ValueError('stale DSW render feedback')
                # Publish health without waiting on catalog reads or preview network I/O.
                self.jaw_mm=jaw_gaps(remote.get('jaw_gap_mm',self.jaw_mm))
                self.remote=remote;self.remote_received=time.monotonic();self.remote_error=None;self.q=q
                capture_id=remote.get('capture_id')
                if remote.get('dataset_ready') and capture_id and capture_id==self.pending_capture and capture_id not in self.imported and not self.importing and not self.import_error:
                    self.importing=True
                    threading.Thread(target=self.import_capture,args=(capture_id,),daemon=True).start()
            except Exception as exc:
                self.remote_error=str(exc);self.remote_last_error={'reason':str(exc),'at_s':time.time()}
                self.tokens.clear()
            time.sleep(.4)

    def restore_display_once(self):
        remote=self.require_remote()
        enabled=getattr(self,'mesh_rendering_enabled',True)
        if 'mesh_rendering_enabled' in remote and remote['mesh_rendering_enabled']!=enabled:
            self.remote_request('/overlay-rendering',{'enabled':enabled})
        if not enabled:return
        if self.calibration and (remote.get('calibration') or {}).get('id')!=self.calibration['id']:
            self.remote_request('/calibration',self.calibration.get('parameters') or self.calibration_candidate(self.calibration['candidate'],self.calibration['batch']))
        self.restore_wrist_display(remote)

    def monitor_display_restore(self):
        # Optional overlays must never make a healthy renderer appear disconnected.
        while True:
            try:
                self.restore_display_once();self.display_restore_error=None
            except Exception as exc:self.display_restore_error=str(exc)
            time.sleep(1.)

    def import_capture(self,capture_id):
        try:self.download_capture(capture_id)
        except Exception as exc:
            with self.lock:self.import_error=str(exc)
        finally:self.importing=False

    def download_capture(self,capture_id):
        dataset=self.remote_request('/dataset')
        if dataset.get('capture_id')!=capture_id:raise ValueError('capture identity changed')
        root=self.output/('batch-live-'+secrets.token_hex(5));target=root/'capture';target.mkdir(parents=True)
        names={'camera.json','robot.urdf'}
        for row in dataset.get('frames',[]):
            names.add(row['image']);names.add(row.get('id',Path(row['image']).stem)+'.json')
            names.update(row.get('camera_images',{}).values())
        for name in names:
            if not isinstance(name,str) or Path(name).name!=name or name in ('.','..'):raise ValueError('invalid remote artifact name')
            (target/name).write_bytes(self.remote_request('/artifacts/'+name,binary=True))
        (target/'dataset.json').write_text(json.dumps(dataset,allow_nan=False))
        completion=self.remote_request('/artifacts/completion.json',binary=True)
        with self.lock:
            (target/'completion.json').write_bytes(completion)
            self.imported.add(capture_id);self.pending_capture=None
            self.load_batch(str(target.relative_to(self.output)));self.persist()

    def persist(self):
        # Save plans and review selection only. Restart never resumes a job or restores execution tokens.
        target=self.output/'workbench-state.json';temporary=target.with_suffix('.tmp')
        with temporary.open('w') as stream:
            json.dump({'queue':self.queue,'plan':self.plan,'selected_batch':self.selected_batch,'geometry_id':self.plan_geometry_id,'control_mode':self.control_mode,'mesh_view_mode':getattr(self,'_mesh_view_mode',None),'mesh_rendering_enabled':getattr(self,'mesh_rendering_enabled',False),'environments':getattr(self,'environments',{'simulation':{},'hardware':{}}),'workflow':getattr(self,'workflow',{'kind':'head','head_result_id':None}),'marker_prior_source':getattr(self,'marker_prior_source',None),'pipeline_report':self.stored_pipeline_report()},stream,allow_nan=False)
            stream.flush();os.fsync(stream.fileno())
        temporary.replace(target)

    def stored_pipeline_report(self):
        """The last pipeline report as JSON-safe data, or None when it cannot be stored.

        Kept so the panel can still say what the last run did after a restart: an operator
        who restarts the workbench needs to see whether the button finished, and which step
        it stopped on. A report that will not serialise is dropped rather than taking the
        whole state write down with it.
        """
        report=getattr(self,'pipeline_report',None)
        if not isinstance(report,dict) or not report:return None
        try:
            return json.loads(json.dumps(report,allow_nan=False,default=float))
        except (TypeError,ValueError):
            return None

    def history(self):
        rows=[]
        candidates=set(self.output.rglob('completion.json'))
        candidates|={p.parent/'capture'/'completion.json' for p in self.output.rglob('batch-*/request.json')}
        for completion in candidates:
            dataset=completion.parent.resolve()
            if not dataset.is_relative_to(self.output):continue
            row={'id':str(dataset.relative_to(self.output)),'completion':read(completion) if completion.exists() else None}
            request=dataset.parent/'request.json'
            row['status']='completed' if row['completion'] else 'incomplete'
            manifest=dataset/'dataset.json'
            try:meta=read(manifest) if manifest.exists() else {}
            except (ValueError,OSError):meta={}
            row['kind']='wrist' if meta.get('wrist_contract') else 'head'
            row['environment']=meta.get('environment','simulation')
            row['cameras']=sorted({'head'}|{side for f in meta.get('frames',[]) for side in f.get('camera_images',{})})
            row['updated_at']=max((p.stat().st_mtime for p in (completion,request) if p.exists()),default=0)
            rows.append(row)
        return sorted(rows,key=lambda row:row['updated_at'],reverse=True)

    def load_batch(self,batch_id):
        if not isinstance(batch_id,str) or batch_id not in {row['id'] for row in self.history()}:raise ValueError('unknown historical batch')
        dataset=(self.output/batch_id).resolve();rows=[]
        manifest=dataset/'dataset.json'
        source_rows=read(manifest).get('frames',[]) if manifest.exists() else [read(path) for path in sorted(dataset.glob('*.json'))]
        for row in source_rows:
            if not isinstance(row,dict) or 'image' not in row or 'q_reported_rad' not in row:continue
            image=(dataset/row['image']).resolve()
            if not image.is_relative_to(dataset):raise ValueError('invalid frame image path')
            rows.append({**row,'image_url':'/artifacts/'+str(image.relative_to(self.output))})
        fit=None;report=None
        for path in sorted(dataset.glob('fit*.json'),key=lambda path:path.stat().st_mtime,reverse=True):
            candidate=read(path)
            if isinstance(candidate,dict) and 'metrics_rectified_px' in candidate:
                fit=path;report=candidate;break
        if manifest.exists() and read(manifest).get('wrist_contract'):
            reports=sorted(dataset.glob('wrist-fit-*.json'),key=lambda path:path.stat().st_mtime,reverse=True)
            if reports:report=read(reports[0])
        if fit:
            for row in rows:
                overlay=dataset/(fit.stem+'-'+Path(row['image']).stem+'-overlay.png')
                if overlay.exists():row['overlay_url']='/artifacts/'+str(overlay.relative_to(self.output))
        evaluation_path=dataset/'frozen-evaluation.json'
        evaluation=read(evaluation_path) if evaluation_path.exists() else None
        if evaluation:
            overlays={Path(path).name for path in evaluation.get('overlays',[])}
            for row in rows:
                overlay=dataset/('frozen-evaluation-'+Path(row['image']).stem+'-overlay.png')
                if overlay.name in overlays and overlay.is_file():row['overlay_url']='/artifacts/'+str(overlay.relative_to(self.output))
        self.evaluation=evaluation
        self.dataset=dataset;self.frames=rows;self.fit_result=report;self.selected_batch=batch_id
        completion=dataset/'completion.json'
        self.batch={'completion':read(completion) if completion.exists() else None}

    def review_batch(self,batch_id):
        """Replay is a response value, never the collection/solver selection."""
        import copy
        view=copy.copy(self)
        view.load_batch(batch_id)
        manifest=view.dataset/'dataset.json'
        dataset=read(manifest) if manifest.exists() else {}
        for row in view.frames:
            images={'head':row['image'],**row.get('camera_images',{})};row['image_urls']={}
            for camera,name in images.items():
                image=(view.dataset/name).resolve()
                if camera not in ('head','left','right') or not image.is_relative_to(view.dataset):raise ValueError('invalid camera image')
                if image.is_file():row['image_urls'][camera]='/artifacts/'+str(image.relative_to(self.output))
            row['detections_by_camera']={'head':row.get('detections',{}),**row.get('camera_detections',{})}
        dependency=dataset.get('wrist_contract',{}).get('head_snapshot',{}).get('id')
        results=[dict(r,kind='head') for r in self.calibration_results() if r['batch']==batch_id or r['id']==dependency]
        results += [dict(r) for r in self.wrist_results() if str(Path(r['file']).parent)==batch_id]
        return {'id':batch_id,'kind':'wrist' if dataset.get('wrist_contract') else 'head','frames':view.frames,
                'results':[{k:v for k,v in r.items() if k not in ('parameters','result')} for r in results],
                'fit_result':view.fit_result,'evaluation':view.evaluation,'completion':view.batch['completion']}

    def historical_overlay(self,body):
        import copy
        import cv2
        from arx_env.joint_head_workbench_overlay import mesh_overlay
        batch=body.get('batch_id');review=self.review_batch(batch)
        index=body.get('frame_index');camera=body.get('camera','head')
        if type(index) is not int or not 0<=index<len(review['frames']) or camera not in ('head','left','right'):raise ValueError('invalid historical frame/camera')
        row=review['frames'][index];dataset=(self.output/batch).resolve()
        name=({'head':row['image'],**row.get('camera_images',{})}).get(camera)
        if name is None:raise ValueError('该批次未采集此相机')
        image=(dataset/name).resolve()
        if not image.is_relative_to(dataset):raise ValueError('invalid historical image')
        result_id=body.get('result_id')
        q=np.asarray(row.get('camera_feedback',{}).get(camera,{}).get('q_reported12',row['q_reported_rad']),float)
        if q.shape!=(12,) or not np.isfinite(q).all():raise ValueError('invalid historical joints')
        if camera=='head':
            allowed={r['id'] for r in review['results'] if r['kind']=='head'}
            result=next((r for r in self.calibration_results() if r['id']==result_id and result_id in allowed),None)
            if not result:raise ValueError('请选择此批次的头标定结果')
            if not self.calibration_asset_matches(result):raise ValueError('标定结果不属于当前 A1 几何')
            candidate=result['parameters'];k=read(dataset/'camera.json');T=rigid(candidate['camera_T_world_optical'])
        else:
            result=next((r for r in self.wrist_results() if r['id']==result_id and not self.wrist_overlay_error(r) and str(Path(r['file']).parent)==batch),None)
            if not result:raise ValueError('请选择此批次参数有效的双腕标定结果')
            if not self.calibration_asset_matches(result):raise ValueError('标定结果不属于当前 A1 几何')
            contract=self.wrist_contract(result);candidate=contract['head_snapshot'];k=contract['cameras'][camera]
            offsets=np.concatenate([candidate['joint_offsets_rad'][s] for s in ('left','right')])
            T=self.geometry.frames(q+offsets)[camera]['link6']@rigid(result[camera]['T_link6_camera'])
        if not k.get('rectified') and any(k.get('coefficients',[])):raise ValueError('历史畸变图需先提供绑定原帧的去畸变图像')
        pixels=cv2.imread(str(image))
        if pixels is None or pixels.shape[:2]!=(k['height'],k['width']):raise ValueError('historical image dimensions disagree')
        view=copy.copy(self.geometry)
        if candidate.get('snapshot_complete'):
            view=view.with_marker_mounts({s:candidate['T_link6_marker'][str(i)] for s,i in [('left',24),('right',30)]})
        view.k=k
        offsets=np.concatenate([candidate['joint_offsets_rad'][s] for s in ('left','right')])
        rendered=mesh_overlay(view,cv2.cvtColor(pixels,cv2.COLOR_BGR2RGB),q,{'camera_T_world_optical':T,'offsets':offsets,'jaw_gap_mm':row.get('jaw_gap_mm',[83,83])})
        target=dataset/('mesh-'+camera+'-'+result_id[:16]+'-'+image.stem+'.png')
        if not cv2.imwrite(str(target),cv2.cvtColor(rendered,cv2.COLOR_RGB2BGR)):raise ValueError('overlay write failed')
        return {'batch':batch,'camera':camera,'candidate_id':result_id,'frame_index':index,'image_url':'/artifacts/'+str(target.relative_to(self.output))}

    def deletion_inventory(self,batch_id):
        if batch_id not in {r['id'] for r in self.history()}:raise ValueError('unknown historical batch')
        path=self.output/batch_id
        if not path.is_dir() or path.is_symlink() or path.resolve()==self.output or not path.resolve().is_relative_to(self.output):raise ValueError('invalid deletion path')
        if any(p.is_symlink() for p in (path,*path.parents)) or any(p.is_symlink() for p in path.rglob('*')):raise ValueError('symlink deletion refused')
        for store in (getattr(self,'store',None),getattr(self,'hardware_store',None)):
            if store and (store.root.is_relative_to(path) or path.is_relative_to(store.root)):raise ValueError('cannot delete calibration library')
        entries=sorted((str(p.relative_to(path)),p.stat().st_size,p.stat().st_mtime_ns) for p in path.rglob('*') if p.is_file())
        return path,entries

    def deletion_scope(self,body):
        """Which datasets a deletion covers: one batch, or every batch at once.

        A full wipe still runs the same per-dataset inventory, so the path identity,
        symlink and library-overlap refusals apply to every batch unchanged."""
        scope=body.get('scope') or 'batch'
        if scope!='all':return scope,[body.get('id')],body.get('id')
        ids=[row['id'] for row in self.history()]
        if not ids:raise ValueError('当前没有可删除的批次')
        return scope,ids,None

    def prune_empty_batch(self,path):
        """Drop a batch directory once every dataset in it is gone.

        Only `batch-*` directories that are genuinely empty are removed; anything
        still holding a file (a renamed request record, another dataset) stays, so
        this can never discard evidence, and it never reaches above self.output."""
        for candidate in (path,path.parent):
            if candidate==self.output or not candidate.is_dir():return
            if not candidate.name.startswith('batch-') or candidate.is_symlink():return
            try:
                if any(candidate.iterdir()):return
                candidate.rmdir()
            except OSError:return

    def deleted_batch(self,batch_id):
        """True when a record's source batch no longer exists on disk.

        Registered versions survive a wipe, but their frames are gone, so anything
        still pointing at them is stale rather than broken - callers must say so
        instead of dropping the record."""
        if not batch_id:return False
        try:return not (self.output/batch_id).exists()
        except OSError:return False

    def clear_remote_overlay(self):
        """Best-effort renderer cleanup after a full wipe.

        Raw files are already gone by the time this runs, so a renderer failure is
        reported rather than raised: the deletion is not rolled back."""
        if not self.live or self.remote_error:return self.remote_error or None
        errors=[]
        try:self.remote_request('/calibration',{'clear':True})
        except ValueError as exc:errors.append('头标定叠加：'+str(exc))
        for side in ('left','right'):
            try:self.remote_request('/wrist-calibration',{'observer':side,'clear':True})
            except ValueError as exc:errors.append(side+'腕叠加：'+str(exc))
        return '；'.join(errors) or None

    def delete_history(self,body,preview=False):
        # Serialize with fit/import transitions, but never touch a live plan or calibration selection.
        if self.importing or self.pending_capture or self.status in ('RUNNING','PLANNING','CAPTURING','FITTING') or (self.remote or {}).get('status') in ('EXECUTING','VALIDATING','RUNNING','SETTLING','AWAITING_CAPTURE'):
            raise ValueError('采集、导入或求解进行中，禁止删除')
        scope,ids,batch_id=self.deletion_scope(body)
        if scope!='all' and batch_id==self.selected_batch:raise ValueError('此批次正在采集与求解栏目中使用；请先在该栏目清空或切换求解批次')
        inventories=[]
        for one in ids:
            path,entries=self.deletion_inventory(one)
            inventories.append((one,path,entries))
        digest=hashlib.sha256(json.dumps([[one,entries] for one,_,entries in inventories]).encode()).hexdigest()
        # A persistence failure must abort before any raw file is removed.
        self.register_existing_calibrations()
        wanted=set(ids)
        preserved=[r['version'] for r in self.library_state()['versions'] if r.get('source_batch') in wanted]
        files=sum(len(entries) for _,_,entries in inventories)
        total=sum(entry[1] for _,_,entries in inventories for entry in entries)
        if preview:
            token=secrets.token_urlsafe(24)
            self.delete_tokens={token:(batch_id,digest,time.monotonic()+120,scope)}
            return {'scope':scope,'token':token,'id':batch_id,
                    'batches':[one for one,_,_ in inventories],'files':files,'bytes':total,
                    'preserved_versions':preserved,
                    'detail':'当前机器人工作台本地批次；DSW原始运行和其他副本不删除',
                    'clears':(['所选求解批次','实时叠加选择'] if scope=='all' else []),
                    'stale':(['当前求解结果（原始批次删除后标记为 stale，版本仍可下载）'] if scope=='all' else [])}
        value=self.delete_tokens.pop(body.get('token'),None)
        if not value or value[0]!=batch_id or value[1]!=digest or value[3]!=scope or value[2]<time.monotonic():
            raise ValueError('删除预览已过期或文件变化，请重新确认')
        removed=[]
        for one,path,_ in inventories:
            shutil.rmtree(path)
            request=path.parent/'request.json'
            if path.name=='capture' and path.parent.name.startswith('batch-') and request.is_file():request.rename(path.parent/'request-deleted.json')
            self.prune_empty_batch(path.parent)
            removed.append(one)
        overlay_error=None
        if scope=='all':
            # The selected batch and the live overlay both pointed at frames that no
            # longer exist. Registered versions keep their frozen snapshots.
            self.selected_batch=None;self.calibration=None;self.wrist_selected={}
            (self.output/'realtime-calibration.json').write_text('{}')
            (self.output/'wrist-selected.json').write_text('{}')
            self.persist()
            overlay_error=self.clear_remote_overlay()
            self.overlay_cleared_at=time.time()
        return {'id':batch_id,'scope':scope,'removed':removed,'files':files,'bytes':total,
                'overlay_error':overlay_error,
                'message':('已永久删除本地原始采集；标定版本与依赖快照保留' if scope!='all'
                           else '已永久删除 '+str(len(removed))+' 个本地批次；标定版本保留，所选求解批次与实时叠加已清空，求解结果标记为 stale')}

    @staticmethod
    def calibration_snapshot(parameters):
        """Freeze measured head/joint/sticker parameters without changing the legacy ID."""
        result=json.loads(json.dumps(parameters,allow_nan=False))
        result['marker_ids']={'left':24,'right':30}
        mounts=parameters.get('T_link6_marker')
        result['T_link6_marker']={}
        errors=[]
        for marker in ('24','30'):
            try:result['T_link6_marker'][marker]=rigid(mounts[marker]).tolist()
            except (ValueError,TypeError,KeyError):errors.append('missing or invalid T_link6_marker['+marker+']')
        try:
            rigid(result['camera_T_world_optical'])
            for side in ('left','right'):
                offsets=np.asarray(result['joint_offsets_rad'][side],float)
                if offsets.shape!=(6,) or not np.isfinite(offsets).all():raise ValueError('invalid offsets')
        except (ValueError,TypeError,KeyError):errors.append('invalid head camera or joint offsets')
        digest=None
        if not errors:
            payload={key:result[key] for key in ('id','camera_T_world_optical','joint_offsets_rad','marker_ids','T_link6_marker')}
            digest=hashlib.sha256(json.dumps(payload,sort_keys=True,allow_nan=False).encode()).hexdigest()
            if parameters.get('snapshot_sha256') not in (None,digest):
                errors.append('frozen snapshot digest mismatch');digest=None
        result.update(snapshot_complete=not errors,snapshot_error='; '.join(errors) or None,snapshot_sha256=digest)
        return result

    def restore_calibration(self):
        saved=self.output/'realtime-calibration.json'
        if not saved.exists():return
        choice=read(saved)
        if not choice.get('batch'):return
        if not self.calibration_asset_matches(choice):return
        if choice.get('environment','simulation') != self.active_environment():
            return
        # Existing frozen parameters are authoritative; never fill missing mounts
        # from a newer fit or nominal geometry merely because the legacy ID matches.
        parameters=choice.get('parameters')
        candidate=self.calibration_snapshot(parameters) if parameters else self.calibration_candidate(choice['candidate'],choice['batch'])
        if candidate['id']==choice.get('id'):
            self.calibration={**choice,'parameters':candidate}
            # Keep a mismatching stored digest as evidence; rewriting it as None
            # would make the same altered payload appear valid on the next restart.
            if self.calibration!=choice and not (parameters and parameters.get('snapshot_sha256') and not candidate['snapshot_complete']):
                saved.write_text(json.dumps(self.calibration,allow_nan=False))

    def calibration_candidate(self,name,batch_id=None,report=None):
        batch_id=batch_id or self.selected_batch
        report=self.fit_result if report is None and batch_id==self.selected_batch else report
        if report is None and batch_id!=self.selected_batch:
            dataset=(self.output/batch_id).resolve()
            if not dataset.is_relative_to(self.output.resolve()):raise ValueError('invalid calibration source batch')
            report=None
            for path in sorted(dataset.glob('fit*.json'),key=lambda path:path.stat().st_mtime,reverse=True):
                value=read(path)
                if isinstance(value,dict) and 'metrics_rectified_px' in value:
                    report=value;break
        if name not in ('joint_candidate','fixed_joint_baseline') or not report:raise ValueError('所选标定来源批次尚无可用拟合')
        candidate=report if name=='joint_candidate' else report.get(name,{})
        camera=np.asarray(candidate.get('camera_T_world_optical'),float)
        offsets=candidate.get('joint_offsets_rad',{})
        if camera.shape!=(4,4) or not np.isfinite(camera).all():raise ValueError('candidate camera transform unavailable')
        for side in ('left','right'):
            value=np.asarray(offsets.get(side),float)
            if value.shape!=(6,) or not np.isfinite(value).all():raise ValueError('candidate joint offsets unavailable')
        result={'camera_T_world_optical':camera.tolist(),'joint_offsets_rad':offsets,
                'robot_source':report.get('robot_source','unknown'),'simulation_only':report.get('simulation_only',True)}
        identity=[batch_id,name,{k:result[k] for k in ('camera_T_world_optical','joint_offsets_rad')}]
        if report.get('objective')=='sticker_corners_plus_mesh':
            identity += [report.get('solver'),candidate.get('T_link6_marker'),report.get('source'),report.get('acceptance')]
            result.update(solver=report.get('solver'),objective=report['objective'],acceptance=report.get('acceptance',{}),
                          optimizer=report.get('optimizer',{}),metrics_rectified_px=report.get('metrics_rectified_px',{}),
                          mesh_metrics=report.get('mesh_metrics',{}))
        if report.get('asset_geometry_id'):
            result.update(asset_geometry_id=report['asset_geometry_id'],asset_seal_sha256=report.get('asset_seal_sha256'))
            identity.append(result['asset_geometry_id'])
        result['id']=hashlib.sha256(json.dumps(identity,sort_keys=True,allow_nan=False).encode()).hexdigest()
        return self.calibration_snapshot({**result,'T_link6_marker':candidate.get('T_link6_marker')})

    def stored_results(self,kind):
        if not hasattr(self,'store'):return []
        results=[]
        stores=[]
        for store in (self.store,getattr(self,'hardware_store',None)):
            if store is not None and all(store.root != other.root for other in stores):stores.append(store)
        for store in stores:
            for record in store.list():
                if record['kind'] not in kind:continue
                row=dict(record['payload']['row'])
                if record['kind'] in ('wrist','left','right') and record['payload'].get('contract',{}).get('geometry_id')!=self.geometry_id:continue
                row.update(version=record['version'],kind=record['kind'],environment=record['environment'])
                results.append(row)
        return results

    def store_for(self, environment):
        if environment not in ('simulation','hardware'):raise ValueError('invalid calibration environment')
        return self.hardware_store if environment=='hardware' else self.store

    def wrist_contract(self,row):
        for store in (getattr(self,'store',None),getattr(self,'hardware_store',None)):
            if store is None:continue
            try:return store.get(row['id'])['payload']['contract']
            except ValueError:pass
        return read(self.output/Path(row['file']).parent/'dataset.json')['wrist_contract']

    def wrist_head_id(self,row):
        """The head result a wrist record was solved against (its frozen dependency)."""
        if row.get('kind') not in ('wrist','left','right'):return None
        try:return self.wrist_contract(row)['head_snapshot']['id']
        except (KeyError,TypeError,ValueError,OSError):return None

    def registered_records(self):
        """Registered versions *with* their payloads.

        `library_state` drops payloads because the panel only shows metadata, but
        deciding whether a head can be saved needs the wrist record's contract and its
        dependency, so pairing reads them back.
        """
        records=[]
        for store in (getattr(self,'store',None),getattr(self,'hardware_store',None)):
            if store is None:continue
            try:records.extend(store.list())
            except (ValueError,OSError):continue
        return records

    def calibration_groups(self):
        """The saveable unit: one head plus the wrist result solved against it.

        A head on its own is not a calibration the robot can use: the wrist extrinsics
        and the joint offsets are solved under a frozen head snapshot, so the two only
        mean something together. Every head therefore appears as one group, carrying its
        matching paired wrist result or, when there is none, the reason - "why can this
        not be saved" is the question the list has to answer.
        """
        if not hasattr(self,'store'):return []
        heads=self.calibration_results()
        bound={}
        for row in self.wrist_results():
            head_id=self.wrist_head_id(row)
            if head_id is not None:bound.setdefault(head_id,[]).append(row)
        registered={}
        for record in self.registered_records():
            if record.get('kind')=='head' or not record.get('dependency_id'):continue
            registered.setdefault(record['dependency_id'],[]).append(record)

        def newest(rows):
            def stamp(row):
                try:return (self.output/Path(row['file'])).stat().st_mtime
                except (KeyError,TypeError,OSError):return 0.
            return sorted(rows,key=stamp,reverse=True)

        groups=[]
        installed=self.installed_pair() or {}
        for head in heads:
            environment=head.get('environment','simulation')
            eligible=self.head_dependency_eligible(head)
            matched=[row for row in bound.get(head['id'],[])
                     if row.get('environment','simulation')==environment and row.get('pair_status')=='complete']
            unsaved=newest([row for row in matched if not row.get('version')])
            saved=newest([row for row in matched if row.get('version')])
            wrist=unsaved[0] if unsaved else (saved[0] if saved else None)
            # "Saving" is what the library already does on its own (a solved head is
            # published as a dependency record); what an operator decides is which group
            # is *in force*. So a group is installable whenever it is a complete pair
            # under the current geometry, whether or not it has been published before.
            is_installed=bool(wrist and installed.get('head_id')==head['id']
                              and installed.get('wrist_id')==wrist['id'])
            installable=bool(wrist) and eligible and not is_installed
            # A matching wrist result already proves the head was good enough to be a
            # dependency when it was solved, so eligibility is only the blocker worth
            # reporting when there is no wrist to pair with. `reason` is the sentence a
            # tooltip or a refusal uses; `reason_short` is what fits in a list row.
            reason=short=None
            if is_installed:
                reason='这一组是当前有效标定（安装于 %s）'%(installed.get('installed_at') or '未知时间')
                short='当前有效'
            elif installable:
                pass
            elif saved:
                reason='这一组已登记（头部 %s ＋ 双腕 %s），但没有被安装为当前有效标定'%(head.get('version') or '未登记',saved[0].get('version'))
                short='已登记未安装'
            elif not eligible:
                reason='头部未通过 mesh／贴纸独立留出门槛，不能作为双腕依赖，也不能单独成为标定版本'
                short='头部未通过留出门槛'
            else:
                recent=newest([record for record in registered.get(head['id'],[]) if record.get('environment')==environment])
                record=recent[0] if recent else None
                payload=(record.get('payload') or {}) if record else {}
                geometry=(payload.get('contract') or {}).get('geometry_id')
                status=(payload.get('row') or {}).get('pair_status')
                if record and geometry not in (None,self.geometry_id):
                    reason='绑定的双腕 %s 与当前几何不符（几何指纹已变更），需按当前几何重跑腕半场'%record.get('version')
                    short='绑定的双腕 %s 几何不符'%record.get('version')
                elif record and status and status!='complete':
                    reason='绑定的双腕 %s 未通过留出检查（%s），不能成为标定版本'%(record.get('version'),status)
                    short='绑定的双腕 %s 未通过留出检查'%record.get('version')
                elif record:
                    reason='绑定的双腕 %s 在当前环境下不可用，需重新求解'%record.get('version')
                    short='绑定的双腕 %s 不可用'%record.get('version')
                else:
                    reason='尚无与该头部匹配的双腕结果：在“采集与求解”里以它为依赖完成双腕采集与求解'
                    short='缺匹配双腕'
            groups.append({'id':head['id'],'kind':'group','environment':environment,
                           'head':{**{k:v for k,v in head.items() if k!='parameters'},'eligible':eligible},
                           'wrist':({k:v for k,v in self.wrist_result_summary(wrist).items()
                                     if k in ('id','kind','version','batch','pair_status','environment','accepted')} if wrist else None),
                           'installable':installable,'installed':is_installed,'saveable':installable,
                           'registered':bool(saved),'reason':reason,'reason_short':short})
        return groups

    def install_provisional_pair(self, body):
        """An explicit, exact-pair user exception; never changes solver acceptance."""
        self.idle()
        if body.get('confirmation')!='INSTALL_USER_ACCEPTED_PROVISIONAL':
            raise ValueError('explicit provisional installation confirmation required')
        head=next((r for r in self.calibration_results() if r['id']==body.get('head_result_id')),None)
        wrist=next((r for r in self.wrist_results() if r['id']==body.get('wrist_result_id')),None)
        if not head or not wrist:raise ValueError('exact head and paired wrist results required')
        self.require_calibration_environment(head);self.require_calibration_environment(wrist)
        self.require_joint_head_dependency(head)
        contract=self.wrist_contract(wrist)
        if contract['head_snapshot']['id']!=head['id']:raise ValueError('wrist head dependency mismatch')
        path=(self.output/wrist['file']).resolve()
        if not path.is_relative_to(self.output.resolve()):raise ValueError('invalid result path')
        raw=path.read_bytes()
        if hashlib.sha256(b'paired'+raw).hexdigest()!=wrist['id']:raise ValueError('wrist result changed')
        report=json.loads(raw)
        row=json.loads(json.dumps(wrist))
        metrics={}
        for side in ('left','right'):
            result=report[side]
            if (result.get('numerically_valid') is not True or result.get('optimizer_success') is not True
                    or result.get('jacobian_rank')!=6
                    or not all(result.get(split,{}).get('positive_depth') is True for split in ('fit','holdout'))):
                raise ValueError('provisional result still requires observable converged positive-depth geometry')
            row[side]['T_link6_camera']=rigid(result['T_link6_camera']).tolist()
            metrics[side]={'holdout':result['holdout'],'thresholds':result['acceptance'],
                           'accepted':result['accepted'],'jacobian_rank':result['jacobian_rank']}
        exception={'status':'user_accepted_provisional','normal_acceptance_passed':False,
                   'head_result_id':head['id'],'wrist_result_id':wrist['id'],
                   'authorization':'explicit operator acceptance of this result error',
                   'metrics':metrics,'capture_coverage':wrist.get('capture_coverage')}
        row['deployment_status']='user_accepted_provisional'
        payload={'row':{k:v for k,v in row.items() if k not in ('version','kind')},
                 'contract':contract,'acceptance_exception':exception}
        store=self.store_for(wrist['environment'])
        head_record=self.register_result(head)
        wrist_record=store.register('wrist',wrist['id'],payload,str(Path(wrist['file']).parent),
            environment=wrist['environment'],dependency_id=head['id'],
            validation={'pair_status':wrist['pair_status'],'accepted':False,'acceptance_exception':exception})
        installed=store.install(head_record,wrist_record,geometry_id=self.geometry_id,
            robot_source=self.robot_source(),acceptance=exception,
            notes='用户明确接受本版误差的临时标定；原数值验收失败保持不变，后续应替换为通过正常验收的版本。')
        return {'message':'已安装用户接受误差的临时标定','installed':self.notify_installation(),'path':installed['path']}

    def notify_installation(self):
        """Notify the embedding owner before this install is published as success."""
        installed=self.installed_pair()
        callback=getattr(self,'on_calibration_installed',None)
        if callback is not None:callback(installed)
        return installed

    def installed_pair(self):
        """The group currently installed for this workbench's environment, if any.

        Read from the library's `active/` directory, which is the only path a consumer
        needs: one head plus the wrist solved against it, replaced atomically on install.
        A tampered or half-present install is reported as unusable rather than used.
        """
        environment=self.active_environment()
        store=getattr(self,'hardware_store',None) if environment=='hardware' else getattr(self,'store',None)
        if store is None or not hasattr(store,'active'):
            return None
        try:
            manifest=store.active()
        except (ValueError,OSError) as exc:
            return {'environment':environment,'error':str(exc),'verified':False,'problems':[str(exc)]}
        if not manifest:
            return None
        if getattr(self,'config',{}).get('asset_geometry_id') and manifest.get('geometry_id')!=self.geometry_id:
            manifest={**manifest,'verified':False,'problems':[*(manifest.get('problems') or []),'已安装标定属于其他几何资产；当前 A1 尚未安装匹配结果']}
        return {'environment':manifest.get('environment'),'head_id':(manifest.get('head') or {}).get('id'),
                'wrist_id':(manifest.get('wrist') or {}).get('id'),
                'head_version':(manifest.get('head') or {}).get('version'),
                'wrist_version':(manifest.get('wrist') or {}).get('version'),
                'installed_at':manifest.get('installed_at'),'geometry_id':manifest.get('geometry_id'),
                'acceptance':manifest.get('acceptance') or {},'notes':manifest.get('notes'),
                'path':manifest.get('path'),'verified':manifest.get('verified'),
                'problems':manifest.get('problems') or []}

    def wrist_acceptance_px(self):
        """The declared holdout gate for the wrist half.

        3.0 px unless the run declares otherwise (`wrist_acceptance.max_holdout_rms_px`).
        Raising it is a calibration-quality decision, so the value is written into the
        plan's contract and from there into the installed manifest - a pair is never
        judged by a gate nobody declared.
        """
        declared=((getattr(self,'config',None) or {}).get('wrist_acceptance') or {}).get('max_holdout_rms_px',3.)
        value=float(declared)
        if not 0.5<=value<=5.:
            raise ValueError('wrist_acceptance.max_holdout_rms_px must be between 0.5 and 5 px')
        return value

    def wrist_acceptance_note(self):
        declared=((getattr(self,'config',None) or {}).get('wrist_acceptance') or {})
        return declared.get('reason')

    def install_acceptance(self, wrist):
        """The numbers the pair was accepted on, carried next to the installed files.

        The contract may leave the gate implicit; the installed manifest should still say
        which threshold applied and whether the contract declared it or the tool defaulted.
        """
        declared=(wrist.get('acceptance') or {}).get('max_holdout_rms_px')
        holdout=wrist.get('holdout') or {}
        return {'objective':wrist.get('objective'),'pair_status':wrist.get('pair_status'),
                'max_holdout_rms_px':float(declared) if declared is not None else 2.0,
                'max_holdout_rms_px_source':'contract' if declared is not None else 'default',
                'left_holdout_rms_px':(holdout.get('left') or {}).get('rms_px'),
                'right_holdout_rms_px':(holdout.get('right') or {}).get('rms_px')}

    def install_notes(self, head_record, wrist_record):
        """Provenance of the installed pair, so the directory explains itself later."""
        note=('头部 %s（批次 %s）＋ 双腕 %s（批次 %s）；腕契约几何 %s'
              %(head_record.get('version'),head_record.get('source_batch'),
                wrist_record.get('version'),wrist_record.get('source_batch'),self.geometry_id))
        reason=self.wrist_acceptance_note()
        if reason:
            note+='；留出门槛 %.2f px（%s）'%(self.wrist_acceptance_px(),reason)
        return note

    def wrist_overlay_error(self,row):
        """Display needs coherent finite parameters, not successful acceptance."""
        try:
            if row.get('kind')!='wrist':raise ValueError('需要成对腕参数')
            contract=self.wrist_contract(row)
            if contract.get('geometry_id')!=self.geometry_id:raise ValueError('几何指纹不一致')
            snapshot=self.calibration_snapshot(contract['head_snapshot'])
            if not snapshot['snapshot_complete']:raise ValueError('头标定快照不完整')
            digest=hashlib.sha256(json.dumps(contract,sort_keys=True,allow_nan=False).encode()).hexdigest()
            if row['result']['contract_sha256']!=digest or row['result']['head_snapshot_sha256']!=snapshot['snapshot_sha256']:
                raise ValueError('标定依赖摘要不一致')
            if row.get('environment','simulation')!=contract.get('environment','simulation'):raise ValueError('环境不一致')
            for side in ('left','right'):
                if row[side].get('observer')!=side or row[side].get('display_identity_valid') is False:raise ValueError('相机侧别或采集依赖不一致')
                rigid(row[side]['T_link6_camera'])
            return None
        except (ValueError,KeyError,TypeError,OSError) as exc:
            return '参数不可绘制：'+str(exc)

    def wrist_overlay_options(self):
        """Wrist-side choices for the overlay, *including* the ones that cannot be drawn.

        `wrist_results` drops every wrist record whose contract geometry no longer
        matches, so after a geometry change the overlay selector would simply look empty
        even though the operator has a paired version to point at. The reason travels
        with each option instead, and every option names the head version it was solved
        against, so the pairing is visible without opening the library.
        """
        if not hasattr(self,'store'):return []
        head_versions={}
        for record in self.registered_records():
            if record.get('kind')=='head':head_versions.setdefault(record['id'],record.get('version'))
        options={}
        for row in self.wrist_results():
            head_id=self.wrist_head_id(row)
            options[row['id']]={'id':row['id'],'version':row.get('version'),'kind':'wrist',
                                'label':row.get('label'),'head_id':head_id,'head_version':head_versions.get(head_id),
                                'environment':row.get('environment','simulation'),
                                'pair_status':row.get('pair_status'),
                                'accepted':row.get('accepted'),'deployment_status':row.get('deployment_status'),
                                'selectable':not self.wrist_overlay_error(row),'reason':self.wrist_overlay_error(row)}
        for record in self.registered_records():
            if record.get('kind')=='head' or record['id'] in options:continue
            payload=record.get('payload') or {}
            geometry=(payload.get('contract') or {}).get('geometry_id')
            status=(payload.get('row') or {}).get('pair_status')
            head_id=record.get('dependency_id')
            if record.get('kind')!='wrist':
                reason='历史单侧记录（不是成对腕标定），不能作为叠加来源'
            elif geometry not in (None,self.geometry_id):
                reason='与当前几何不符（几何指纹已变更），需按当前几何重跑腕半场'
            elif status and status!='complete':
                reason='未通过留出检查（%s），不能作为叠加来源'%status
            else:
                reason='当前环境下不可用，需重新求解'
            options[record['id']]={'id':record['id'],'version':record.get('version'),'kind':record.get('kind'),
                                   'head_id':head_id,'head_version':head_versions.get(head_id),
                                   'environment':record.get('environment'),'pair_status':status,
                                   'selectable':False,'reason':reason}
        return list(options.values())

    def active_environment(self):
        # The overlay / candidate environment is explicit (mesh_view_mode) and is
        # decoupled from the operating mode: hardware candidates can be inspected
        # while running pure simulation and vice versa (G5). Moving the robot stays
        # gated by operating_mode + robot_control, never by this.
        return self.mesh_view_mode()

    def calibration_asset_matches(self, row):
        asset=getattr(self,'config',{}).get('asset_geometry_id')
        return not asset or (row.get('parameters') or row).get('asset_geometry_id',row.get('asset_geometry_id'))==asset

    def require_calibration_environment(self, row):
        source=row.get('environment','simulation')
        if source != self.active_environment():
            raise ValueError(('真机' if self.active_environment()=='hardware' else '仿真')+'模式不能使用'+('仿真' if self.active_environment()=='hardware' else '真机')+'标定文件')
        if not self.calibration_asset_matches(row):
            raise ValueError('标定结果不属于当前 A1 几何，需重新头部与双腕标定')
        expected=self.robot_source()
        if expected=='physical' and (row.get('robot_source')!='physical' or row.get('simulation_only') is not False):
            raise ValueError('物理真机仅允许明确来自 physical 的标定产物')
        if expected=='simulated' and row.get('robot_source')=='physical':
            raise ValueError('假真机不能使用物理真机标定产物')
        return row

    def environment_results(self, rows):
        environment=self.active_environment()
        return [row for row in rows if row.get('environment','simulation')==environment
                and self.calibration_asset_matches(row)]

    @staticmethod
    def head_dependency_eligible(row):
        parameters=row.get('parameters',row)
        if (row.get('optimizer') or parameters.get('optimizer') or {}).get('success') is False:return False
        if parameters.get('objective')!='sticker_corners_plus_mesh' or parameters.get('snapshot_complete') is not True:return False
        acceptance=parameters.get('acceptance',{})
        checks=acceptance.get('image_checks',{})
        if parameters.get('optimizer',{}).get('success') is not True or checks.get('positive_depth') is not True:return False
        mesh=parameters.get('mesh_metrics',{}).get('holdout',{})
        try:
            coverage=float(mesh['macro_coverage']);error=float(mesh['macro_mean_penalized_px'])
            marker=float(parameters['metrics_rectified_px']['holdout']['rms_px'])
        except (KeyError,TypeError,ValueError):return False
        from arx_env.joint_head_x5_hybrid import head_mesh_thresholds
        try:thresholds=head_mesh_thresholds({**parameters,**row},acceptance.get('thresholds'))
        except (TypeError,ValueError):return False
        return bool(np.isfinite([coverage,error,marker]).all()
                    and thresholds['holdout_mesh_min_coverage'] <= coverage <= 1.
                    and 0 <= error <= thresholds['holdout_mesh_max_px'] and 0 <= marker <= 2.)

    @classmethod
    def require_joint_head_dependency(cls,row):
        if not cls.head_dependency_eligible(row):
            parameters=row.get('parameters',row)
            marker=parameters.get('metrics_rectified_px',{}).get('holdout',{}).get('rms_px','缺失')
            mesh=parameters.get('mesh_metrics',{}).get('holdout',{})
            from arx_env.joint_head_x5_hybrid import head_mesh_thresholds
            try:
                thresholds=head_mesh_thresholds({**parameters,**row},parameters.get('acceptance',{}).get('thresholds'))
                limits=f"mesh误差≤{thresholds['holdout_mesh_max_px']:g}px、覆盖率≥{thresholds['holdout_mesh_min_coverage']:.0%}"
            except (TypeError,ValueError):limits='mesh阈值记录无效'
            raise ValueError('腕标定需要同来源、完整快照且通过 mesh/贴纸独立留出门槛的头部联合结果；'
                f"当前留出：贴纸RMS={marker}px（≤2px），mesh误差={mesh.get('macro_mean_penalized_px','缺失')}px、"
                f"覆盖率={mesh.get('macro_coverage','缺失')}（0～1）；要求{limits}。请查看该头部结果的质量详情。")
        return row

    def register_result(self,row):
        if not hasattr(self,'store'):return None
        kind=row.get('kind') or row.get('observer') or 'head'
        dependency=None
        environment=row.get('result',{}).get('environment',row.get('environment','simulation'))
        if environment not in ('simulation','hardware'):raise ValueError('未知标定来源，不能登记')
        payload={'row':{k:v for k,v in row.items() if k not in ('version','kind') and (k!='environment' or v=='hardware')}}
        if kind=='wrist':
            # Paired versions only: both sides must pass. A single-side draft stays
            # browsable and diagnosable but must never register as a usable version.
            if row.get('pair_status')!='complete':raise ValueError('腕标定需要左右两侧都通过留出检查；当前状态 '+str(row.get('pair_status')))
            contract=self.wrist_contract(row);payload['contract']=contract
            dependency=contract['head_snapshot']['id']
            head=next((r for r in self.calibration_results() if r['id']==dependency),None)
            if head and head.get('environment','simulation')!=environment:raise ValueError('腕标定与头标定依赖来源不一致')
            if head:self.register_result(head)
            if not head:
                record=self.store_for(environment).get(dependency)
                if record['kind']!='head' or record['environment']!=environment:raise ValueError('腕标定缺少同来源头标定')
            validation={'holdout':row['holdout'],'pair_status':row.get('pair_status'),
                        'pair':{'left':bool((row.get('left') or {}).get('accepted')),'right':bool((row.get('right') or {}).get('accepted'))},
                        'physical_validated':False}
            batch=str(Path(row['file']).parent)
        elif kind!='head':
            contract=self.wrist_contract(row);payload['contract']=contract
            dependency=contract['head_snapshot']['id']
            head=next((r for r in self.calibration_results() if r['id']==dependency),None)
            if head and head.get('environment','simulation')!=environment:raise ValueError('腕标定与头标定依赖来源不一致')
            if head:self.register_result(head)
            if not head:
                record=self.store_for(environment).get(dependency)
                if record['kind']!='head' or record['environment']!=environment:raise ValueError('腕标定缺少同来源头标定')
            validation={'holdout':row['holdout'],'physical_validated':False}
            batch=str(Path(row['file']).parent)
        else:
            batch=row['batch'];validation={'snapshot_complete':row['parameters']['snapshot_complete'],'physical_validated':False,
                'objective':row['parameters'].get('objective','sticker_corners'),'acceptance':row['parameters'].get('acceptance',{})}
        return self.store_for(environment).register(kind,row['id'],payload,batch,environment=environment,dependency_id=dependency,validation=validation)

    @staticmethod
    def registrable(row):
        """Whether a record on disk can be published as a version.

        The library holds heads and *complete* wrist pairs. A single-sided record or a
        rejected draft is not a version - it stays visible in the catalog with its reason,
        which is where an operator looks for "why is this not usable". This is the same
        rule `register_result` enforces, stated up front so the backfill can skip the
        expected refusals deliberately rather than by catching an exception.
        """
        if row.get('kind') in ('wrist','left','right'):return row.get('pair_status')=='complete'
        return True

    def register_existing_calibrations(self):
        """Publish what is on disk that can be a version; keep the rest as visible drafts.

        Called from `__init__` and again after a wrist solve - both places where an escape
        is not acceptable: at startup it means the workbench cannot start at all, and at the
        end of a solve it reports a *successful* solve as a rejection (an earlier draft is
        still on disk). Expected refusals are skipped by `registrable`; anything else is a
        real inconsistency, so it is recorded in `registration_skips` and reported through
        `library_state` rather than swallowed.
        """
        self.registration_skips=[]
        for row in self.calibration_results()+self.wrist_results():
            if not self.registrable(row):continue
            try:self.register_result(row)
            except ValueError as exc:
                self.registration_skips.append({'id':row.get('id'),'kind':row.get('kind'),'reason':str(exc)})

    def library_state(self):
        if not hasattr(self,'store'):return {'root':None,'robot_id':None,'versions':[]}
        stores=[self.store,getattr(self,'hardware_store',None)]
        versions=[]
        for store in stores:
            if store is None:continue
            versions.extend({k:v for k,v in r.items() if k!='payload'} for r in store.list())
        # A wipe removes the frames but keeps the versions. Mark them so the library
        # can say "原始批次已删除" instead of implying the source is still there.
        for row in versions:row['stale']=self.deleted_batch(row.get('source_batch'))
        return {'root':str(getattr(self,'calibration_root',self.store.root.parent)),'robot_id':self.store.robot_id,
                'versions':versions,'skipped':list(getattr(self,'registration_skips',[])),
                'by_environment':{'simulation':str(self.store.root),'hardware':str(self.hardware_store.root) if hasattr(self,'hardware_store') else None}}

    def calibration_store_record(self, calibration_id):
        for store in (self.store, getattr(self,'hardware_store',None)):
            if store is None:continue
            try:return store.get(calibration_id)
            except ValueError:pass
        raise ValueError('unknown calibration identifier')

    def calibration_results(self):
        results=[]
        for row in self.history():
            dataset=self.output/row['id']
            for path in sorted(dataset.glob('fit*.json'),key=lambda p:p.stat().st_mtime,reverse=True):
                try:
                    report=read(path)
                    for name in ('joint_candidate','fixed_joint_baseline'):
                        try:candidate=self.calibration_candidate(name,row['id'],report)
                        except (ValueError,TypeError):continue
                        camera=np.asarray(candidate['camera_T_world_optical'])
                        if not np.allclose(camera[3],[0,0,0,1]) or not np.allclose(camera[:3,:3].T@camera[:3,:3],np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(camera[:3,:3]),1,atol=1e-5):continue
                        result={'id':candidate['id'],'candidate':name,'batch':row['id'],'file':path.name,'parameters':candidate}
                        result.update(objective=candidate.get('objective','sticker_corners'),
                            solver=candidate.get('solver','legacy_corners'),acceptance=candidate.get('acceptance',{}),
                            optimizer=candidate.get('optimizer',{}),metrics_rectified_px=candidate.get('metrics_rectified_px',{}),
                            mesh_metrics=candidate.get('mesh_metrics',{}))
                        result['environment']=report.get('environment','simulation')
                        result.update(robot_source=candidate['robot_source'],simulation_only=candidate['simulation_only'])
                        results.append(result)
                except (ValueError,TypeError,OSError):continue
        merged=results+self.stored_results(('head',))
        return list({row['id']:row for row in merged}.values())

    def calibration_state(self):
        selected=self.calibration
        remote=(self.remote or {}).get('calibration') or {}
        rows=self.environment_results(self.calibration_results())
        # Eligibility is current policy, not part of an immutable calibration snapshot.
        rows=[{**row,'head_dependency_eligible':self.head_dependency_eligible(row),'stale':self.deleted_batch(row.get('batch'))} for row in rows]
        return {'selected':selected,'ready':bool(selected and remote.get('id')==selected['id'] and remote.get('ready') is True and not self.remote_error),'frame_id':remote.get('calibration_frame_id'),'frame_sequence':remote.get('frame_sequence'),'source_timestamp_s':remote.get('timestamp_s'),'rendered_at_s':remote.get('rendered_at_s'),'error':remote.get('error'),'install':self.installed_pair(),'groups':self.calibration_groups(),'wrist_options':self.wrist_overlay_options(),'results':[{**{k:v for k,v in row.items() if k!='parameters'},'snapshot_complete':row['parameters']['snapshot_complete']} for row in rows],'available':[name for name in ('joint_candidate','fixed_joint_baseline') if self.fit_result and (self.fit_result if name=='joint_candidate' else self.fit_result.get(name,{})).get('camera_T_world_optical')]}

    def wrist_side_status(self, result, side, digest, snapshot, contract):
        """Per-side eligibility, kept next to the paired record.

        The thresholds are unchanged; what changed is that a rejected side now
        reports why instead of being dropped, so a single-side result can be shown
        as an incomplete draft rather than disappearing."""
        reasons=[]
        if not result:reasons.append('该侧没有求解结果')
        else:
            if result.get('accepted') is not True:reasons.append('未通过验收')
            if result.get('coverage_complete') is not True and result.get('partial_capture_authorized') is not True:reasons.append('观察覆盖不完整')
            if result.get('observer')!=side:reasons.append('observer 与侧别不匹配')
            if result.get('contract_sha256')!=digest:reasons.append('采集契约摘要不一致')
            if result.get('head_snapshot_sha256')!=snapshot['snapshot_sha256']:reasons.append('头标定快照不一致')
            if result.get('environment','simulation')!=contract.get('environment','simulation'):reasons.append('环境标注不一致')
            if result.get('physical_validated') is not False:reasons.append('物理验证标志异常')
            if result.get('jacobian_rank')!=6:reasons.append('雅可比秩不足 6')
            if result.get('optimizer_success') is not True:reasons.append('优化未收敛')
            if not all((result.get(k) or {}).get('positive_depth') is True for k in ('fit','holdout')):reasons.append('深度不全为正')
            threshold=float(contract.get('acceptance',{}).get('max_holdout_rms_px',2.))
            rms=float((result.get('holdout') or {}).get('rms_px',float('nan')))
            if not np.isfinite(rms) or not 0<=rms<=threshold:reasons.append('留出 rms 超出阈值 '+str(threshold)+' px')
            try:rigid(result.get('T_link6_camera'))
            except (ValueError,TypeError,KeyError):reasons.append('外参不是刚体变换')
        accepted=not reasons
        return {'observer':side,'accepted':accepted,'reasons':reasons,
                'display_identity_valid':bool(result and result.get('observer')==side and result.get('contract_sha256')==digest and result.get('head_snapshot_sha256')==snapshot['snapshot_sha256'] and result.get('environment','simulation')==contract.get('environment','simulation')),
                'holdout':result.get('holdout'),'jacobian_rank':result.get('jacobian_rank'),
                'coverage_complete':result.get('coverage_complete'),
                'T_link6_camera':result.get('T_link6_camera'),
                'simulation_only':result.get('simulation_only'),
                'robot_source':result.get('robot_source')}

    def marker_prior(self, rows=None):
        """The sticker installation a head re-fit must hold fixed, or None.

        The head half solves a camera, twelve joint offsets, the mesh and two sticker
        installations at once, which is why those two are its weakest parameters; the
        wrist half watches the same stickers from six degrees of freedom. So the prior
        is never a value an operator types in - it is a wrist result, named here, and
        the mounts are read back from it so the state cannot drift from its source.
        """
        source=getattr(self,'marker_prior_source',None)
        if not source:return None
        try:
            # `rows` lets a caller that already listed the wrist results pass them in:
            # the capture loop polls owner feedback under the same lock, and a second
            # full scan per state read is enough of a stall to break its frame bracket.
            row=next((r for r in (self.wrist_results() if rows is None else rows) if r['id']==source),None)
            if row is None:
                raise ValueError('来源腕结果已不可用（被删除或几何指纹已变更）')
            mounts=row.get('marker_mounts') or {}
            if not mounts:
                raise ValueError('来源腕结果没有估计标识安装：该批次规划时未开启标识估计，请重跑腕半场')
            return {'source_result_id':source,'batch':row.get('batch'),'mounts':mounts,
                    'deltas':row.get('marker_mount_deltas') or {},
                    'wrist_pair_status':row.get('pair_status')}
        except ValueError as exc:
            return {'source_result_id':source,'error':str(exc),'mounts':{},'deltas':{}}

    def wrist_results(self):
        # One record per solved dataset, carrying both sides. The solver returns the
        # two sides together under a single contract, so a record is paired by
        # nature; splitting it into independent left/right entries made users pick
        # two versions that never existed independently (G5). Records stored before
        # this change stay single-sided and keep kind left/right.
        results=[]
        for path in sorted(self.output.glob('batch-*/capture/wrist-fit-*.json')):
            try:
                if not path.resolve().is_relative_to(self.output.resolve()):continue
                raw=path.read_bytes();report=json.loads(raw);dataset=read(path.parent/'dataset.json');contract=dataset['wrist_contract']
                completion=read(path.parent/'completion.json')
                if contract.get('geometry_id')!=self.geometry_id:continue
                from arx_env.joint_head_wrist_calibration import wrist_capture_coverage
                coverage=None
                partial=any(report.get(side,{}).get('partial_capture_authorized') is True for side in ('left','right'))
                if partial:
                    coverage=wrist_capture_coverage(dataset,completion)
                    if not all(report.get(side,{}).get('partial_capture_authorized') is True and
                               report[side].get('capture_coverage')==coverage for side in ('left','right')):continue
                snapshot=self.calibration_snapshot(contract['head_snapshot'])
                if not snapshot['snapshot_complete']:continue
                digest=hashlib.sha256(json.dumps(contract,sort_keys=True,allow_nan=False).encode()).hexdigest()
                sides={side:self.wrist_side_status(report.get(side,{}),side,digest,snapshot,contract) for side in ('left','right')}
                if not completion.get('passed') and not partial:
                    for side in sides.values():
                        side['accepted']=False;side['reasons'].append('采集未完成')
                failed=[side for side in ('left','right') if not sides[side]['accepted']]
                status='complete' if not failed else ('incomplete_both' if len(failed)==2 else 'incomplete_'+failed[0])
                environment=contract.get('environment','simulation')
                simulated=next((sides[s]['simulation_only'] for s in ('left','right') if sides[s]['simulation_only'] is not None),None)
                source=next((sides[s]['robot_source'] for s in ('left','right') if sides[s]['robot_source']),None)
                objective=next((((report.get(s) or {}).get('objective')) for s in ('left','right') if (report.get(s) or {}).get('objective')),None)
                # One estimate per side: the left solve watches marker 30, the right solve
                # marker 24, so a complete pair carries both stickers' installations. An
                # older batch (or one planned with the estimate switched off) carries none.
                marker_mounts={};marker_mount_deltas={}
                for side in ('left','right'):
                    estimate=(report.get(side) or {}).get('marker_mount')
                    if not estimate:continue
                    marker_mounts[str(estimate['marker_id'])]=estimate['T_link6_marker']
                    marker_mount_deltas[str(estimate['marker_id'])]=estimate.get('delta_from_head_snapshot')
                row={'id':hashlib.sha256(b'paired'+raw).hexdigest(),'kind':'wrist','pair_status':status,
                     'file':str(path.relative_to(self.output)),'batch':path.parent.parent.name,
                     'left':sides['left'],'right':sides['right'],'environment':environment,
                     'robot_source':source,'simulation_only':simulated,
                     'asset_geometry_id':contract.get('asset_geometry_id'),
                     'asset_seal_sha256':contract.get('asset_seal_sha256'),
                     'accepted':status=='complete','label':report.get('display_label'),'capture_coverage':coverage,
                     'partial_capture_authorized':partial,
                     'acceptance':contract.get('acceptance',{}),
                     'marker_mounts':marker_mounts,'marker_mount_deltas':marker_mount_deltas,
                     'objective':objective,
                     'holdout':{'left':sides['left']['holdout'],'right':sides['right']['holdout']},
                     'result':{'contract_sha256':digest,'head_snapshot_sha256':snapshot['snapshot_sha256'],
                               'objective':objective,'environment':environment,'simulation_only':simulated}}
                results.append(row)
            except (OSError,ValueError,TypeError,KeyError):continue
        return list({row['id']:row for row in results+self.stored_results(('left','right','wrist'))}.values())

    @staticmethod
    def wrist_result_summary(row):
        # Paired records carry their verdict at the top level (accepted / objective /
        # acceptance); single-sided records keep it inside `result`. Project both so
        # a paired version is never reported as "not yet accepted".
        result=row.get('result',{})
        summary={k:v for k,v in row.items() if k!='result'}
        summary.update({
            'accepted':result['accepted'] if 'accepted' in result else row.get('accepted'),
            'objective':result.get('objective') or row.get('objective'),
            'acceptance':result.get('acceptance') or row.get('acceptance') or {},
            'simulation_only':result.get('simulation_only') if result.get('simulation_only') is not None else row.get('simulation_only'),
            'robot_source':result.get('robot_source') or row.get('robot_source'),
        })
        return summary

    def restore_wrist_display(self, remote):
        """Restore frozen display parameters; the mirror loop resumes when the lock exits."""
        with self.mode_lock:
            selected=dict(getattr(self,'wrist_selected',{}) or {})
            if (not selected or self.status!='READY' or getattr(self,'pipeline_running',False)
                    or remote.get('status') not in ('READY','MIRRORING') or remote.get('wrist_config_pending')):return
            for state in (getattr(self,'control_status',None),getattr(self,'hardware_feedback',None)):
                owner=(state or {}).get('owner') or {}
                if (owner.get('automatic_enabled') or owner.get('automatic')
                        or owner.get('state') in ('VERIFYING','RUNNING','SETTLING','AWAITING_CAPTURE')
                        or (owner.get('motion') or {}).get('status') in ('VERIFYING','RUNNING','SETTLING','AWAITING_CAPTURE')):return
            catalog={row['id']:row for row in self.environment_results(self.wrist_results())}
            row=catalog.get(selected.get('id'))
            if row is None or self.wrist_overlay_error(row):return
            try:self.require_calibration_environment(row)
            except ValueError:return
            digest=row['result']['contract_sha256']
            if selected.get('contract_sha256')!=digest:return
            configured=remote.get('wrist_contract_sha256')==digest
            missing=[side for side in ('left','right') if not configured or remote.get('wrist_calibration',{}).get(side,{}).get('id')!=row['id']]
            if not missing:return
            contract=self.wrist_contract(row)
            # A READY snapshot may already have become MIRRORING before this lock.
            # Clear only when repair is needed; never pulse a healthy mirror.
            mirror=self.control_mode=='hardware_sync' or bool(remote.get('mirror')) or remote.get('status')=='MIRRORING'
            if mirror:self.remote_request('/mirror',{'clear':True})
            if not configured:self.remote_request('/wrist-config',{'wrist_contract':contract})
            for side in missing:
                self.remote_request('/wrist-calibration',{'id':row['id'],'observer':side,
                    'T_link6_camera':row[side]['T_link6_camera'],
                    'head_snapshot_sha256':row['result']['head_snapshot_sha256'],'contract_sha256':digest})
            if mirror and not configured:
                # Resuming a mirror before the queued config reaches a render tick
                # cancels the config and drops the overlays. Bound this wait; the
                # context manager releases mode_lock even on failure/timeout so the
                # existing read-only mirror loop always regains control.
                deadline=time.monotonic()+3
                while True:
                    restored=self.remote_request('/state')
                    if not restored.get('wrist_config_pending') and restored.get('wrist_contract_sha256')==digest:break
                    if time.monotonic()>=deadline:raise ValueError('腕叠加契约恢复超时；镜像继续，稍后重新核验')
                    time.sleep(.03)

    def plan_wrists(self):
        from importlib import util
        spec=util.spec_from_file_location('joint_head_wrist_plan',Path(__file__).with_name('joint_head_wrist_plan.py'))
        module=util.module_from_spec(spec);spec.loader.exec_module(module)
        plan_wrist=module.plan_wrist
        import copy
        mode=self.control_mode
        if mode=='hardware_sync' and not self.hardware_connected():raise ValueError('真机反馈不可用，不能规划真机标定')
        choice=getattr(self,'workflow',{}).get('head_result_id')
        selected=next((row for row in self.calibration_results() if row['id']==choice),None)
        if not selected:raise ValueError('请在采集与求解中明确选择依赖的头标定版本')
        self.require_calibration_environment(selected)
        self.require_joint_head_dependency(selected)
        self.register_result(selected)
        snapshot=self.calibration_snapshot(selected['parameters'])
        if not snapshot['snapshot_complete']:raise ValueError('头标定快照缺少贴纸安装；请重新应用完整的历史标定结果')
        mounts=self.config.get('wrist_nominal_mounts')
        if not mounts:raise ValueError('未配置仿真腕相机名义安装')
        cameras=({side:adapter.read_calibration_frame()[1]['intrinsics'] for side,adapter in self.wrist_cameras.items()} if mode=='hardware_sync' else self.config.get('wrist_simulation_cameras')) or {s:{**self.geometry.k,'rectified':True,'coefficients':[0.]*5} for s in ('left','right')}
        geometry=self.geometry.with_gaps(80.,80.);geometry.config={**geometry.config,'table_z_m':0.,'table_clearance_m':.005}
        start=np.array(self.hardware_feedback['owner']['calibration_feedback']['qpos'][:6]+self.hardware_feedback['owner']['calibration_feedback']['qpos'][7:13]) if mode=='hardware_sync' else self.q.copy()
        cameras={side:{**k,'rectified':True,'coefficients':[0.]*5,'distortion_model':'distortion.inverse_brown_conrady'} for side,k in cameras.items()}
        offsets=np.concatenate([snapshot['joint_offsets_rad'][s] for s in ('left','right')])
        planned_fraction={'left':0.,'right':0.}
        def progress(value):
            with self.lock:self.wrist_plan_progress=value
            planned_fraction[value['observer']]=min(1.,value['accepted']/max(1,value['target']))
            self.pipeline_subprogress(3,{'fraction':sum(planned_fraction.values())/2,
                'label':'双腕有效采样姿态检查', 'completed':value['accepted'],'total':value['target']})
        self.wrist_plan_progress=None
        try:
            seed_file=self.config.get('wrist_seed_file')
            seed_path=Path(__file__).resolve().parents[3]/seed_file if seed_file else None
            seed_bytes=seed_path.read_bytes() if seed_path else None
            result=plan_wrist(geometry,mounts,poses_per_side=16,seed=20,start_q=start,joint_offsets=offsets,cameras=cameras,
                             marker_mounts=snapshot['T_link6_marker'],progress=progress,
                             motion_seed=json.loads(seed_bytes) if seed_bytes is not None else None)
            if seed_bytes is not None:result['motion_seed_sha256']=hashlib.sha256(seed_bytes).hexdigest()
        finally:
            with self.lock:self.wrist_plan_progress=None
        contract={'asset_geometry_id':self.config.get('asset_geometry_id'),'asset_seal_sha256':self.config.get('asset_seal_sha256'),
                  'head_snapshot':snapshot,'solver':'corners','cameras':cameras,'nominal_mounts':mounts,
                  'planned_frames':result['planned_frames'],'table_z_m':0.,'table_clearance_m':.005,
                  # The holdout gate is declared here and travels with the capture, so a pair is
                  # judged by the numbers it was accepted under rather than by today's default.
                  'acceptance':{'max_holdout_rms_px':self.wrist_acceptance_px()},
                  # Wrist cameras use opposite sticker corners with the head mounts frozen.
                  'estimate_marker_mount':False,
                  'geometry_id':self.geometry_id,'environment':'hardware' if mode=='hardware_sync' else 'simulation','robot_source':self.robot_source(),'simulation_only':self.robot_source()!='physical','marker_size_m':self.geometry.marker_size,
                  'chassis_sha256':hashlib.sha256(Path(self.config['chassis_mesh']).read_bytes()).hexdigest() if self.config.get('chassis_mesh') else None}
        if result.get('motion_seed'):
            contract['motion_seed']=result['motion_seed'];contract['motion_seed_sha256']=result.get('motion_seed_sha256')
        with self.lock:
            current=np.array(self.hardware_feedback['owner']['calibration_feedback']['qpos'][:6]+self.hardware_feedback['owner']['calibration_feedback']['qpos'][7:13]) if mode=='hardware_sync' and self.hardware_connected() else self.q
            if self.control_mode!=mode or self.workflow.get('head_result_id')!=selected['id'] or np.max(abs(current-start))>.002:
                raise ValueError('规划期间起点或头标定已变化，请重新规划')
            result['waypoints']=self.calibration_waypoints(result['waypoints'])
            self.plan={**result,'wrist_contract':contract};self.plan_geometry_id=self.geometry_id
            self.queue=[dict(w,name=w['id'],speeds=[.4]*12) for w in result['waypoints']]
            self.persist()

    def fit_wrists(self, allow_partial=False):
        from arx_env.joint_head_wrist_calibration import solve_wrist_dataset
        dataset=read(self.dataset/'dataset.json')
        completion=read(self.dataset/'completion.json')
        if not completion.get('passed') and not allow_partial:raise ValueError('当前腕采集未完整结束')
        contract=dataset.get('wrist_contract')
        if not contract:raise ValueError('当前历史批次不是双腕同步采集')
        self.require_joint_head_dependency({'parameters':contract['head_snapshot']})
        if contract.get('solver', 'corners') not in ('corners', 'hybrid'):
            raise ValueError('不支持的腕部求解目标')
        if contract.get('geometry_id')!=self.geometry_id:raise ValueError('腕采集几何与当前模型不一致，禁止使用当前几何重算历史数据')
        # Hybrid wrist fitting consumes the captured observer RGB files.  Keep
        # the dataset root explicit so missing evidence fails instead of
        # silently falling back to sticker-only fitting.
        result=solve_wrist_dataset(dataset,self.geometry,contract['nominal_mounts'],
                                   dataset_root=self.dataset,progress=lambda value:self.pipeline_subprogress(5,value),
                                   allow_partial=allow_partial,source_completion=completion)
        path=self.dataset/('wrist-fit-'+secrets.token_hex(4)+'.json')
        with path.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
        self.fit_result=result
        # Publish through the same skip-as-you-go backfill: an earlier *rejected* draft is
        # still on disk and `register_result` refuses it, which used to end a successful
        # solve as REJECTED with the draft's reason.
        self.register_existing_calibrations()
        return result

    @staticmethod
    def camera_frame_age(meta):
        """Age of a hardware camera frame, on whichever basis the camera provides.

        Same rule as Camera.snapshot(): a camera that computed its own age reports
        source_age_s plus the monotonic receive mark, and we extend that with the local
        elapsed time; a camera that only stamps the frame is aged against the wall clock.
        When neither basis is present the frame cannot be aged - say so instead of
        subtracting None, which used to surface as
        "unsupported operand type(s) for -: 'float' and 'NoneType'"."""
        received, age = meta.get('received_monotonic'), meta.get('source_age_s')
        if isinstance(received, (int, float)) and isinstance(age, (int, float)):
            return age + time.monotonic() - received
        stamp = meta.get('frame_timestamp_ns')
        if isinstance(stamp, int) and stamp > 0:
            return (time.time_ns() - stamp) / 1e9
        raise ValueError('相机帧缺少年龄基准（source_age_s / frame_timestamp_ns）')

    @staticmethod
    def overlay_ages(source_age_s, render_s):
        """The two ages a served overlay frame must report, kept apart on purpose.

        `source_age_s` is how old the camera frame already was when we read it. It is
        the only basis a freshness gate may judge, because it describes the
        observation. `observed_age_s` adds the render round trip, which describes the
        bridge: the panel expires the displayed frame on it. Using the sum as the gate
        - which is what happened until 2026-09-22 - turns a slow renderer into a stale
        camera, and with a 1.4 s render the wrist overlay reported "源帧已过期" on every
        single request even though the frame was 0.05 s old when it was read."""
        if not all(isinstance(value,(int,float)) for value in (source_age_s,render_s)):
            raise ValueError('叠加年龄需要数值源帧龄与渲染耗时')
        return float(source_age_s), float(source_age_s)+max(0.,float(render_s))

    def wrist_selection_id(self):
        """The id of the selected paired record, or None.

        Both sides of a pair carry the same id, so callers must never key the selection
        by side; numerical acceptance does not control display selection."""
        choice=getattr(self,'wrist_selected',{}) or {}
        return choice.get('id') if choice.get('pair_status') else None

    def hardware_wrist_overlay_frame(self, observer, show=True):
        if show and not getattr(self,'mesh_rendering_enabled',True):raise ValueError('实时 mesh 叠加已关闭')
        from arx_env.joint_head_workbench_overlay import pair_real_feedback
        import base64
        selected_id=self.wrist_selection_id()
        selected=next((row for row in self.wrist_results() if row['id']==selected_id and not self.wrist_overlay_error(row)),None)
        if selected and selected.get('environment')!='hardware':raise ValueError('真机叠加禁止使用仿真或来源不明的标定')
        camera=self.wrist_cameras.get(observer)
        camera_path=self.config.get('hardware_wrist_camera_files',{}).get(observer)
        if not selected or not camera or not camera_path or not self.hardware_connected():
            raise ValueError('腕相机、实际内参、关节反馈或有效腕结果尚未接入')
        snapshot=self.wrist_contract(selected)['head_snapshot']
        result=selected['result']
        if result['head_snapshot_sha256']!=snapshot['snapshot_sha256']:raise ValueError('腕结果与头标定快照不一致')
        raw,meta=camera.read_frame()
        source_age=self.camera_frame_age(meta)
        if source_age>=2:raise ValueError('腕叠加源帧已过期')
        history=self.feedback_history_snapshot()
        sample,timing=pair_real_feedback(meta,history,self.hardware_feedback['owner']['pid'])
        q=np.r_[sample['qpos'][:6],sample['qpos'][7:13]]
        offsets=np.concatenate([snapshot['joint_offsets_rad'][s] for s in ('left','right')])
        pose=self.geometry.frames(q+offsets)[observer]['link6']@rigid(selected[observer]['T_link6_camera'])
        candidate={**snapshot,'id':selected['id'],'camera_T_world_optical':pose.tolist()}
        started=time.monotonic()
        encoded=self.remote_request('/real-overlay',{'jpeg_base64':base64.b64encode(raw).decode(),
            'qpos':sample['qpos'],'candidate':candidate,'camera':read(camera_path)},binary=True) if show else raw
        with self.lock:
            if (getattr(self,'wrist_selected',{}) or {}).get('id')!=selected['id'] or not self.hardware_connected():raise ValueError('腕标定选择或真机连接已变化')
        image_age,observed_age=self.overlay_ages(source_age,time.monotonic()-started)
        return encoded,dict(timing,candidate_id=selected['id'],observer=observer,image_age_s=image_age,
                            observed_age_s=observed_age,render_ms=round((observed_age-image_age)*1000,1),
                            simulation_only=result.get('simulation_only',True),physical_validated=False)

    def hardware_overlay_frame(self, show=True):
        if show and not getattr(self,'mesh_rendering_enabled',True):raise ValueError('实时 mesh 叠加已关闭')
        """Real pixels paired with nearest same-host joint sample; never DSW pixels."""
        from arx_env.joint_head_workbench_overlay import pair_real_feedback
        if not self.hardware_camera or not self.hardware_connected():
            raise ValueError('真机相机或关节反馈未接入/已过期')
        with self.lock:
            selected = dict(self.calibration) if self.calibration else None
            candidate = (selected.get('parameters') or self.calibration_candidate(selected['candidate'],selected['batch'])) if selected else None
        if not selected:raise ValueError('尚未选择实时叠加标定参数')
        if selected.get('environment')!='hardware':raise ValueError('真机叠加禁止使用仿真或来源不明的标定')
        if candidate['id']!=selected['id']:raise ValueError('标定来源内容已变化，请重新选择实时参数')
        raw, meta = self.hardware_camera.read_frame()
        # Judge the source frame before rendering: the age our own render adds is a
        # property of the renderer, never of the observation.
        source_age = self.camera_frame_age(meta)
        if source_age>2.:raise ValueError('真机源图像已过期')
        history=self.feedback_history_snapshot()
        sample, timing = pair_real_feedback(meta, history, self.hardware_feedback['owner']['pid'])
        started = time.monotonic()
        if show:
            import base64
            self.require_remote()
            encoded = self.remote_request('/real-overlay', {
                'jpeg_base64': base64.b64encode(raw).decode('ascii'),
                'qpos': sample['qpos'], 'candidate': candidate,
                'camera': read(self.config['camera'])}, binary=True)
        else:
            encoded = raw
        with self.lock:
            if self.calibration != selected or not self.hardware_connected():raise ValueError('参数或真机连接已变化')
        image_age, observed_age = self.overlay_ages(source_age, time.monotonic()-started)
        return encoded,dict(timing,candidate_id=selected['id'],frame_sequence=meta['frame_seq'],image_age_s=image_age,
                            observed_age_s=observed_age,render_ms=round((observed_age-image_age)*1000,1),mesh_visible=show,
                            jaw_sdk_mm=[sample['qpos'][6]*1000,sample['qpos'][13]*1000],jaw_gap_mm=feedback_gaps(sample['qpos']),jaw_mapping=feedback_state(sample['qpos'])['mapping'])

    def counts(self):
        captures=sum(w.get('capture') is not False for w in self.queue)
        active=getattr(self,'active_capture',None) if getattr(self,'status',None)=='CAPTURING' else None
        if active is not None:
            return {'waypoints':len(self.queue),'capture_points':captures,'transition_points':len(self.queue)-captures,
                    'captured':active['captured'],'valid':active['captured'],'failed':0,'unclassified':0}
        valid=sum(row.get('valid_for_fit') is True for row in self.frames)
        failed=sum(row.get('valid_for_fit') is False for row in self.frames)
        return {'waypoints':len(self.queue),'capture_points':captures,'transition_points':len(self.queue)-captures,
                'captured':len(self.frames),'valid':valid,'failed':failed,'unclassified':len(self.frames)-valid-failed}

    def display(self,q,gaps=None):
        gaps=jaw_gaps(self.jaw_mm if gaps is None else gaps)
        geometry=self.geometry.with_gaps(*gaps)
        p=geometry.preview(q,False);frames=geometry.frames(q);parts=[]
        for side,name,v,f,signs in self.display_meshes:
            if name=='link6':
                v=v.copy();v[:,1]+=signs*(geometry.gaps[side]-self.geometry.gaps[side])/2
            T=frames[side][name]
            parts.append({'side':side,'link':name,'vertices':np.round(v@T[:3,:3].T+T[:3,3],6).tolist(),'faces':f.tolist()})
        for side in ('left','right'):
            T=frames[side]['link6']@self.geometry.mounts[side];z=self.geometry.marker_size/2
            corners=np.array([[-z,-z,0],[z,-z,0],[z,z,0],[-z,z,0]])
            parts.append({'side':side,'link':'marker','marker_id':24 if side=='left' else 30,'vertices':(corners@T[:3,:3].T+T[:3,3]).tolist(),'faces':[[0,1,2],[0,2,3]]})
        p['parts']=parts
        p['jaw_gap_mm']=gaps.tolist()
        p['mount_status']=self.geometry.mount_status
        k=self.geometry.k
        p['camera']={'T_world_optical':self.geometry.camera.tolist(),'K':[[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1]],'width':k['width'],'height':k['height']}
        return p

    def diagnostics(self):
        return {'live':bool(self.live),'capabilities':{'preview':True,'execute':True,'capture':True,'hardware':False},
          'hardware_camera_configured':self.hardware_camera is not None,'hardware_adapter':{'read_adapter':'arx_env.joint_head_workbench_hardware.HardwareReadAdapter','read_configured':self.hardware_reader is not None,'verified':False,'owner':'joint_head_real_owner.py','joint_order':'left joint1..6, right joint1..6; jaws separate','required':['per-side/per-motor feedback age and validity','image exposure and joint bracketing timestamps','owner PID and command epoch','pedal and damping-stop acknowledgement','verified SDK speed and jaw limits','explicit takeover; no automatic reconnect/resume']},
          'backend':'DSW persistent Isaac RTX' if self.live else 'X5 kinematics; configured Isaac RTX image capture','robot_connected':False,
          'hardware_velocity_limit_verified':False,'velocity_source':'simulation cap 1 rad/s intersected with URDF; not hardware limit',
          'feedback':[{'joint':i+1,'source':'simulated','valid':not self.live or not self.remote_error and time.monotonic()-self.remote_received<=3} for i in range(12)],
          'capture_configured':bool(self.live or self.config.get('capture_command')),
          'collision_scope':self.geometry.collision_method,'mount_chain':self.geometry.mount_chain,
          'jaw':{'metal_inner_gap_mm':83,'tip_center_distance_mm':85,'sdk_mapping':'four-point piecewise linear; ruler +2 mm correction included; command and feedback maps separate'},
          'pending_hardware':['per-motor CAN freshness','pedal stop chain','SDK speed limits','jaw mapping independent validation and below-first-point range']}

    def runtime_state(self):
        """Cached operational snapshot; never wait for planning or scan artifacts."""
        now=time.monotonic()
        remote=self.remote or {}
        fresh=not self.remote_error and now-self.remote_received<=3
        hardware=self.hardware_connected()
        synced=self.control_mode=='hardware_sync' and hardware
        feedback=self.hardware_feedback if hardware else None
        owner=(feedback or {}).get('owner') or {}
        q=owner.get('calibration_feedback',{}).get('qpos')
        control=self.control_state()
        pipeline=dict(getattr(self,'pipeline_report',{}) or {})
        pipeline['steps']=[dict(step) for step in pipeline.get('steps',[])]
        pipeline.update(running=bool(getattr(self,'pipeline_running',False)),
                        review=getattr(self,'pipeline_review',None))
        # Feedback diagnostics/events and artifact catalogs belong to the slow endpoint.
        summary={'owner':{key:owner.get(key) for key in
                         ('pid','command_epoch','state','simulation_only','physical_hardware_connected','evidence_source')},
                 'source_age_s':(feedback or {}).get('source_age_s')} if hardware else None
        return {'live_backend':bool(self.live),'backend_connected':fresh if self.live else True,
                'remote':{key:remote.get(key) for key in ('status','state','reason','render','frame_timestamp_s','timestamp_s')},
                'hardware_connected':hardware,'hardware_feedback':summary,'hardware_sync_active':synced,
                'hardware_control_connected':self.control_available(),'hardware_error':self.hardware_error,
                'hardware_last_disconnect':getattr(self,'hardware_last_disconnect',None),
                'control_error':getattr(self,'control_error',None),'control_last_disconnect':getattr(self,'control_last_disconnect',None),
                'mirror_error':self.mirror_error,'render_connection_error':self.remote_error,
                'render_last_disconnect':getattr(self,'remote_last_error',None),
                'display_restore_error':getattr(self,'display_restore_error',None),
                'control_mode':self.control_mode,'mode':'hardware' if self.control_mode=='hardware_sync' else 'simulation',
                'simulation_backend':'上海 DSW · Isaac RTX 实时渲染' if self.live else 'X5 运动学 / Isaac RTX 采集',
                'csrf_token':self.csrf,'model_family':self.config.get('model_family','x5'),**control,
                'qpos':list(q[:6])+list(q[7:13]) if synced else self.q.tolist(),
                'jaw_gap_mm':feedback_gaps(q) if synced else self.jaw_mm.tolist(),
                'hardware_jaw':feedback_state(q) if hardware else None,
                'limits':self.geometry.limits.tolist(),'velocity_limits':self.velocity_limits.tolist(),
                'status':('DISCONNECTED' if not fresh else remote.get('status',remote.get('state')))
                         if self.live and self.control_mode=='simulation' and self.status=='READY' else self.status,
                'reason':(self.remote_error or self.import_error or remote.get('reason') or self.reason) if self.live else self.reason,
                'hardware_fault':control['execution_blocker']['code'].startswith('owner_fault'),
                'counts':self.counts(),'selected_batch':self.selected_batch,
                'workflow':dict(getattr(self,'workflow',{'kind':'head','head_result_id':None})),
                'pipeline':pipeline,'motion_preflight':dict(getattr(self,'motion_preflight',{}) or {}),
                'review_generation':getattr(self,'review_generation',0)}

    def state(self):
        with self.lock:
            remote=self.remote or {};fresh=not self.remote_error and time.monotonic()-self.remote_received<=3
            wrist_rows=self.environment_results(self.wrist_results())
            return {'live_backend':bool(self.live),'remote':remote,'backend_connected':fresh if self.live else True,'hardware_connected':self.hardware_connected(),'hardware_feedback':self.hardware_feedback if self.hardware_connected() else None,'hardware_error':self.hardware_error,'hardware_last_disconnect':getattr(self,'hardware_last_disconnect',None),'control_error':getattr(self,'control_error',None),'control_last_disconnect':getattr(self,'control_last_disconnect',None),'mirror_error':self.mirror_error,'render_connection_error':self.remote_error,'render_last_disconnect':getattr(self,'remote_last_error',None),'display_restore_error':getattr(self,'display_restore_error',None),'render_stream_last_errors':dict(getattr(self,'live_frame_errors',{})),'control_mode':self.control_mode,'hardware_sync_active':self.hardware_sync_active(),'mode':'hardware' if self.control_mode=='hardware_sync' else 'simulation','simulation_backend':'上海 DSW · Isaac RTX 实时渲染' if self.live else 'X5 运动学 / Isaac RTX 采集','csrf_token':self.csrf,
             'model_family':self.config.get('model_family','x5'),'asset_geometry_id':self.config.get('asset_geometry_id'),'asset_seal_sha256':self.config.get('asset_seal_sha256'),
             **self.control_state(),
             'arrival_statistics':self.arrival_state(),
             'jaw_gap_mm':feedback_gaps(self.hardware_feedback['owner']['calibration_feedback']['qpos']) if self.hardware_sync_active() else self.jaw_mm.tolist(),'jaw_definition':'actual inner gap mm; ruler +2 mm correction included','hardware_jaw':feedback_state(self.hardware_feedback['owner']['calibration_feedback']['qpos']) if self.hardware_connected() else None,
             'qpos':(self.hardware_feedback['owner']['calibration_feedback']['qpos'][:6]+self.hardware_feedback['owner']['calibration_feedback']['qpos'][7:13]) if self.hardware_sync_active() else self.q.tolist(),'limits':self.geometry.limits.tolist(),'velocity_limits':self.velocity_limits.tolist(),
             'hardware_control_connected':self.control_available(),'camera_processes':getattr(self,'camera_processes',{}),'camera_health':{name:camera.snapshot() for name,camera in {'head':self.hardware_camera,**self.wrist_cameras}.items() if camera},
             'status':('DISCONNECTED' if not fresh else remote.get('status',remote.get('state'))) if self.live and self.control_mode=='simulation' and self.status=='READY' else self.status,'reason':(self.remote_error or self.import_error or remote.get('reason') or self.reason) if self.live else self.reason,'hardware_fault':self.execution_blocker()['code'].startswith('owner_fault'),'queue':self.queue,
             'calibration':self.calibration_state(),'wrist':{'selected':getattr(self,'wrist_selected',{}),'planning':getattr(self,'wrist_plan_progress',None) if self.status=='PLANNING' else None,'results':[{**self.wrist_result_summary(r),'stale':self.deleted_batch(r.get('batch'))} for r in wrist_rows]},'frames':self.frames,'fit_result':self.fit_result,'evaluation':self.evaluation,'batch':self.batch,'plan':self.plan,
             'counts':self.counts(),'history':self.history(),'selected_batch':self.selected_batch,'workflow':getattr(self,'workflow',{'kind':'head','head_result_id':None}),'library':self.library_state(),
             'marker_prior':self.marker_prior(wrist_rows),'pipeline':{**dict(getattr(self,'pipeline_report',{}) or {}),'running':bool(getattr(self,'pipeline_running',False)),'review':getattr(self,'pipeline_review',None)},'plan_stale':bool(self.queue) and self.plan_geometry_id!=self.geometry_id,'mount_chain':self.geometry.mount_chain,
             'actual_parts':self.actual['parts'],'camera':self.actual['camera'],'diagnostics':self.diagnostics()}

    def idle(self):
        if self.importing:raise ValueError('正在保存远端采集批次')
        if self.live and (self.remote or {}).get('status',(self.remote or {}).get('state')) in ('VALIDATING','EXECUTING','RUNNING','SETTLING','AWAITING_CAPTURE'):raise ValueError('DSW job is active')
        if self.status in ('RUNNING','PLANNING','CAPTURING','FITTING'): raise ValueError('job is active')

    def send_previews(self):
        # Only the newest unsent target survives a slow network round trip.
        while True:
            with self.preview_condition:
                self.preview_condition.wait_for(lambda:self.preview_pending is not None)
                body=self.preview_pending;self.preview_pending=None
            try:
                result=(self.preview(body,display_only=True) if self.control_mode=='hardware_sync'
                        else self.action('/api/preview',body))
                with self.preview_condition:
                    if body.get('_dispatch_id',0)>=getattr(self,'preview_completed_dispatch',-1):
                        self.preview_completed_dispatch=body.get('_dispatch_id',0)
                        self.preview_result={'client_revision':body['revision'],**result}
            except Exception as exc:
                with self.preview_condition:
                    if body.get('_dispatch_id',0)>=getattr(self,'preview_completed_dispatch',-1):
                        self.preview_completed_dispatch=body.get('_dispatch_id',0)
                        self.preview_result={'client_revision':body['revision'],'error':str(exc)}

    def queue_preview(self,body):
        self.geometry._q(body['q']);jaw_gaps(body.get('jaw_gap_mm',self.jaw_mm))
        if type(body.get('revision')) is not int:raise ValueError('preview revision required')
        with self.preview_condition:
            self.preview_dispatch=getattr(self,'preview_dispatch',0)+1
            self.preview_pending={**body,'_dispatch_id':self.preview_dispatch};self.preview_condition.notify()
        return {'queued':True}

    def preview(self,body,*,display_only=False):
        review=None if display_only else self.review_snapshot()
        gaps=jaw_gaps(body.get('jaw_gap_mm',self.jaw_mm))
        jaw_speed=body.get('jaw_speed_mm_s',10.)
        if type(jaw_speed) not in (int,float) or not np.isfinite(jaw_speed) or jaw_speed<=0:raise ValueError('positive jaw speed required')
        q=self.geometry._q(body['q']); speeds=np.asarray(body.get('speeds',[.2]*12),float)
        if speeds.shape!=(12,) or not np.isfinite(speeds).all() or np.any(speeds<=0) or (not self.hardware_sync_active() and np.any(speeds>self.velocity_limits)): raise ValueError('invalid per-joint speeds')
        if self.live:
            remote=self.require_remote()
            if not remote.get('capabilities',{}).get('jaw_gap_mm'):raise ValueError('DSW 后端尚不支持夹爪开度，请更新后端；不会忽略目标开度')
            if display_only:
                with self.preview_condition:
                    self.preview_revision=max(self.preview_revision+1,int(remote.get('preview_revision',0))+1)
                    requested_revision=self.preview_revision
            else:
                self.preview_revision=max(self.preview_revision+1,int(remote.get('preview_revision',0))+1)
                requested_revision=self.preview_revision
            result=self.remote_request('/preview',{'q':q.tolist(),'jaw_gap_mm':gaps.tolist(),'revision':requested_revision})
            if display_only or self.control_mode=='hardware_sync':
                # Display only: hardware authority comes from control_request's
                # frozen owner plan checks, never from this renderer preview.
                return {**result,'revision':requested_revision,'live':True,'token':None,
                        'collision_free':None,'display_only':True}
            # A DSW preview renders the target; it says nothing about the swept path,
            # and the scene has self-collision disabled. Check locally, so the renderer
            # path cannot mint execution authority for a self-colliding target either.
            path=self.geometry.with_gaps(83.,83.).path_check(self.q,q)
            token=secrets.token_urlsafe(20) if path['ok'] else None
            with self.review_lock:
                self.require_review(review)
                if token:
                    if len(self.tokens)>40:self.tokens.clear()
                    self.tokens[token]={'start':self.q.copy(),'start_jaw':self.jaw_mm.copy(),'jaw_gap_mm':gaps.copy(),'q':q.copy(),'speeds':speeds.copy(),'jaw_speed_mm_s':jaw_speed,'expires':time.monotonic()+120,'collision_free':True}
            return {**result,'revision':requested_revision,'live':True,'token':token,'collision_free':path['ok'],'path_check':path,'duration_s':max(.05,float(np.max(1.875*np.abs(q-self.q)/speeds)),float(np.max(1.875*abs(gaps-self.jaw_mm)/jaw_speed)))}
        # Full-open convex hull encloses every intermediate finger position.
        p=self.display(q,gaps); path=self.geometry.with_gaps(83.,83.).path_check(self.q,q)
        p['collision_free']=path['ok']; p['path_check']=path
        duration=max(float(np.max(1.875*np.abs(q-self.q)/speeds)),float(np.max(1.875*abs(gaps-self.jaw_mm)/jaw_speed)),.05)
        token=secrets.token_urlsafe(20) if path['ok'] else None
        with self.review_lock:
            self.require_review(review)
            if token:
                # Execution authority stays in memory and expires; saved plans cannot resume execution.
                if len(self.tokens)>40:self.tokens.clear()
                self.tokens[token]={'start':self.q.copy(),'start_jaw':self.jaw_mm.copy(),'jaw_gap_mm':gaps.copy(),'q':q.copy(),'speeds':speeds.copy(),'jaw_speed_mm_s':jaw_speed,'duration':duration,'expires':time.monotonic()+120,'collision_free':True}
        return {**p,'token':token,'duration_s':duration}

    def fit_head_batch(self):
        """Solve the selected head batch, holding the sticker mounts a prior names.

        Extracted from the `/api/fit` handler so the alternating pipeline can re-solve
        the same capture with new evidence without going through HTTP or the operator.
        """
        if self.config.get('asset_geometry_id') and read(self.dataset/'dataset.json').get('asset_geometry_id')!=self.config['asset_geometry_id']:
            raise ValueError('当前 A1 不可复用旧几何的头部观测')
        result=self.dataset/('fit-'+secrets.token_hex(4)+'.json')
        geometry_config=result.with_suffix('.geometry.json')
        geometry_keys=('model_family','asset_geometry_id','asset_seal_sha256','parking_support_contact','jaw_mapping_file','urdf','scene_seed','candidate','camera','mesh_manifest','gap_m','marker_size_m','margin_px','wrist_support_proxy','chassis_mesh','chassis_scale','chassis_T_world_mesh','static_boxes','wrist_camera_housings')
        geometry_config.write_text(json.dumps({key:self.config[key] for key in geometry_keys if key in self.config},indent=2))
        argv=[sys.executable,str(Path(__file__).with_name('joint_head_real_fit.py')),'--dataset',str(self.dataset),'--output',str(result),'--urdf',self.config['urdf'],'--scene',self.config['scene_seed'],'--camera-seed',self.config['camera_seed'],'--solver','hybrid','--workbench-config',str(geometry_config)]
        prior=self.marker_prior()
        if prior and prior.get('error'):raise ValueError('标识先验不可用：'+prior['error'])
        if prior and prior['mounts']:
            marker_file=result.with_suffix('.marker-mounts.json')
            marker_file.write_text(json.dumps(prior['mounts'],indent=2))
            argv+= ['--marker-mounts',str(marker_file),
                    '--marker-mounts-provenance','腕结果 %s（批次 %s）估计的贴纸安装'%(prior['source_result_id'][:12],prior['batch'])]
        self.fit_progress_path=result.with_suffix('.progress.json')
        try:self.run_process(argv,result.with_suffix('.log'))
        finally:self.fit_progress_path=None
        report=read(result);environment=read(self.dataset/'dataset.json').get('environment','simulation') if (self.dataset/'dataset.json').exists() else 'simulation'
        dataset_meta=read(self.dataset/'dataset.json') if (self.dataset/'dataset.json').exists() else {}
        report.update(robot_source=dataset_meta.get('robot_source','unknown'),simulation_only=dataset_meta.get('simulation_only',True),
                      asset_geometry_id=dataset_meta.get('asset_geometry_id'),asset_seal_sha256=dataset_meta.get('asset_seal_sha256'))
        report.update(status='HARDWARE_IMAGE_FIT' if environment=='hardware' else 'SIMULATED_RENDERED_IMAGE_FIT',environment=environment,hardware_validation=False)
        result.write_text(json.dumps(report,indent=2))
        with self.lock:
            self.fit_result=report
            for row in self.calibration_results():self.register_result(row)
            for frame in self.frames:
                overlay=result.parent/(result.stem+'-'+Path(frame['image']).stem+'-overlay.png')
                if overlay.exists():frame['overlay_url']='/artifacts/'+str(overlay.relative_to(self.output))
        return report

    def pipeline_subprogress(self, stage, value):
        """Record measured work units; never grant stage completion or execution authority."""
        if not getattr(self,'pipeline_running',False):return
        fraction=float(value['fraction'])
        if not np.isfinite(fraction):return
        with self.lock:
            old=self.pipeline_report.get('subprogress') or {}
            fraction=max(0.,min(.99,fraction))
            if old.get('stage')==stage:fraction=max(fraction,old['fraction'])
            self.pipeline_report['subprogress']={**value,'stage':stage,'fraction':fraction}

    def pipeline_progress(self, **fields):
        # Update the report in place. Copying it here used to leave the job holding one
        # object and the state another, so fields written by the job (the retry list) and by
        # the state (the phase) ended up in different dictionaries and the last write won.
        with self.lock:
            report=getattr(self,'pipeline_report',None)
            if not isinstance(report,dict):
                report={}
                self.pipeline_report=report
            if fields.get('phase') and fields['phase']!=report.get('phase'):
                report.pop('subprogress',None)
            report.update(fields)
            # Write it through at every step boundary so a restart still shows what the last
            # run did; a disk problem must not abort a calibration that is otherwise fine.
            try:
                self.persist()
            except OSError:
                pass

    def wait_for_step(self, timeout, label, settle=('READY','STOPPED')):
        """Block until a sub-job the pipeline started has stopped running."""
        began=time.monotonic()
        budget=None;budget_loaded=False
        while True:
            with self.lock: status=self.status; reason=self.reason
            if label.endswith('采集') and getattr(self,'pipeline_running',False):
                counts=self.counts();total=counts['capture_points']
                if total:self.pipeline_subprogress(4 if label.startswith('腕') else 1,
                    {'fraction':counts['captured']/total,'label':'已采集图像',
                     'completed':counts['captured'],'total':total})
            if status in settle:return
            if status=='REJECTED':raise ValueError('%s 失败：%s'%(label,reason or '未给出原因'))
            if not budget_loaded:
                budget=self.resident_capture_budget(check=False) if label.endswith('采集') else None
                budget_loaded=True
            if budget:
                # The collector owns the earlier cutoff and its detailed failure.
                # Only the original resident end bounds a stuck cleanup/status path.
                if time.monotonic()>=budget['resident_deadline_monotonic_s'] or time.time_ns()>=budget['resident_deadline_wall_ns']:
                    raise ValueError('Fake 采集未能在原驻留期限内结束；检查批次 completion.json')
            elif time.monotonic()-began>timeout:raise ValueError('%s 超时（%.0f s 未回到 READY）'%(label,timeout))
            time.sleep(2)

    def pipeline_stop(self):
        """Leave the job at a step boundary when the operator asked it to stop."""
        if self.stop.is_set():
            raise PipelineStopped('操作者请求停止流水线：已完成的步骤保留，未发布新的标定组')

    # A batch can die on infrastructure rather than on the motion: the frame/feedback
    # clock, or the renderer behind the DSW tunnel dropping a request. Neither says
    # anything about the plan, so both are worth another flight; a plan, geometry or
    # control error will simply reproduce and must not be retried.
    # Matched against the message, so the list is deliberately about infrastructure: the
    # renderer behind the DSW tunnel, the frame/feedback cadence, the camera service. A plan,
    # geometry or control error will reproduce on the next attempt and is not listed.
    TRANSIENT_CAPTURE = ('采集', 'DSW', 'unavailable', 'Connection reset', 'feedback bracket',
                         'HTTP Error', 'Service Unavailable')

    def capture_batch(self,report,label,attempts=3):
        """Preview, confirm and wait for one batch, recording retries as they happen.

        Retries are written into the report as they occur, not returned at the end, so a
        run that ultimately fails still shows which failures preceded it.
        """
        automatic=bool(self.config.get('pipeline_auto_confirm')) and self.robot_source()=='simulated'
        if not automatic:attempts=1
        for attempt in range(attempts):
            self.pipeline_stop()
            if transport_environment(self.config)=='fake':self.resident_capture_budget()
            try:
                if automatic:
                    preview=self.preview_calibration()
                    started=self.confirm_calibration({'token':preview['token'],'confirmation':'START_REVIEWED_SINGLE_ARM','physical_clearance':True})
                else:
                    self.pipeline_review_result=None
                    self.pipeline_review={'label':label,'kind':self.workflow['kind']}
                    self.pipeline_progress(phase=label+'：等待现场审阅并确认净空')
                    try:
                        while self.pipeline_review_result is None:
                            if self.status=='REJECTED':raise ValueError(self.reason or '真机确认或采集失败')
                            if self.stop.wait(.1):self.pipeline_stop()
                        started=self.pipeline_review_result
                    finally:
                        self.pipeline_review=None
                self.pipeline_progress(phase=label+'：采集')
                self.wait_for_step(1800,'%s采集'%label)
                self.pipeline_stop()
                batch=started['batch_id']
                if not any(row['id']==batch and row.get('status')=='completed' and (row.get('completion') or {}).get('passed') for row in self.history()):
                    raise ValueError('本次采集未产生通过完整性检查的批次')
                return batch
            except (ValueError,OSError) as exc:
                if transport_environment(self.config)=='fake':self.resident_capture_budget()
                # OSError too: the camera and renderer clients raise urllib's HTTPError, so a
                # 503 from an unreachable service is not a ValueError and used to escape the
                # retry entirely (seen live: 'HTTP Error 503: Service Unavailable').
                if attempt==attempts-1 or not any(mark in str(exc) for mark in self.TRANSIENT_CAPTURE):raise
                report.setdefault('capture_retries',[]).append({'batch':label,'reason':str(exc)})
                self.pipeline_progress(phase='%s：采集失败（%s），重试第 %d 次'%(label,str(exc)[:60],attempt+1))
                # Pause so the owner, the renderer and the frame/feedback cadence settle
                # before the batch is flown again.
                if self.stop.wait(20):self.pipeline_stop()

    def newest_batch(self,kind):
        """Newest capture of one kind whose completion says the whole batch passed.

        A retried capture leaves a partial batch behind, so recency alone is not enough.
        """
        rows=sorted((row for row in self.history() if row['kind']==kind
                     and row.get('status')=='completed' and (row.get('completion') or {}).get('passed')),
                    key=lambda row:row.get('updated_at') or 0)
        if not rows:raise ValueError('%s半场没有产生通过完整性检查的采集批次'%kind)
        return rows[-1]['id']

    def run_calibration_pipeline(self,body):
        """One action that takes both halves from planning to an installed group.

        Two modes, because they answer different questions.

        `full` is the production button: plan the head capture, fly it, solve it, then
        plan, fly and solve the wrist half against that head, and publish and install the
        pair. Nothing is reused - the point is a robot that has no fresh capture.

        `refit` is the two-round experiment on a capture that is already selected: solve
        the head, let the wrist half estimate the sticker installation, re-solve the head
        with it held, and install the best complete pair. It stays an experiment because
        the held-out check inside decides whether the outside installation is adopted at
        all: measurements on the real capture say the installation is the *head* half's to
        measure, not the wrist's, so a handed-back value usually loses (architecture doc,
        the identifiability section).

        Physical capture pauses for a fresh operator review before every batch.
        Only an explicitly simulated robot may use automatic confirmation/retries.
        """
        if getattr(self,'pipeline_running',False):raise ValueError('标定流水线正在运行')
        if self.config.get('pipeline_auto_confirm') and self.robot_source()!='simulated':
            raise ValueError('pipeline_auto_confirm 只允许明确 simulated 的假真机')
        if self.control_mode!='hardware_sync' or not self.hardware_connected():raise ValueError('标定流水线需要真机反馈与同步控制')
        mode=body.get('mode') or ('refit' if body.get('rounds') else 'full')
        if mode not in ('full','wrist','refit'):raise ValueError('未知的流水线模式 '+str(mode))
        capture_only=body.get('capture_only',False)
        if type(capture_only) is not bool:raise ValueError('capture_only 必须为布尔值')
        if 'capture_only' in body and mode!='wrist':raise ValueError('capture_only 仅支持腕部模式')
        rounds=1;head_capture=None;resume_head=None
        if mode=='wrist':
            resume_head=next((r for r in self.calibration_results() if r['id']==body.get('head_result_id')),None)
            if resume_head is None:raise ValueError('恢复双腕需要明确指定已通过的头部结果')
            self.require_calibration_environment(resume_head)
            self.require_joint_head_dependency(resume_head)
        if mode=='refit':
            rounds=int(body.get('rounds',2) or 2)
            if not 1<=rounds<=4:raise ValueError('迭代轮数需在 1–4 之间')
            if not hasattr(self,'dataset') or not self.frames:raise ValueError('重解模式需要先选中一批已采集完成的头部数据：它在同一批像素上反复重解，不重复飞头部')
            if (self.dataset/'dataset.json').exists() and read(self.dataset/'dataset.json').get('wrist_contract'):
                raise ValueError('选中的批次是腕标定批次，请选中头部批次')
            if not (self.dataset/'completion.json').exists() or not read(self.dataset/'completion.json').get('passed'):
                raise ValueError('选中的头部批次采集未完成，不能作为重解的头部证据')
            head_capture=str(self.dataset)

        def job():
            self.pipeline_thread=threading.get_ident()
            report={'schema':'joint-head-calibration-pipeline-v2','run_id':self.pipeline_run_id,'mode':mode,'rounds_requested':rounds,
                    'auto_confirm':bool(self.config.get('pipeline_auto_confirm')),'steps':[],'aborted':None,'stopped':False}
            if capture_only:report.update(capture_only=True,capture_completed=False)
            if resume_head:
                report.update(mode='full',resumed_from='wrist',head_capture=resume_head['batch'])
            if head_capture:
                report['head_capture']=(str(Path(head_capture).relative_to(self.output))
                                        if Path(head_capture).is_relative_to(self.output) else head_capture)
            self.pipeline_report=report
            try:
                if mode=='full':
                    # Head half: nothing exists yet, so it is planned, flown and solved.
                    self.pipeline_stop()
                    self.pipeline_progress(phase='头部：规划')
                    self.action('/api/workflow',{'kind':'head'})
                    self.action('/api/auto-plan',{})
                    self.wait_for_step(900,'头部规划')
                    counts=self.counts()
                    report['steps'].append({'step':'plan','half':'head','waypoints':counts['waypoints'],
                                            'capture_points':counts['capture_points']})
                    self.pipeline_stop()
                    self.pipeline_progress(phase='头部：审阅与采集（%d 个采样点）'%counts['capture_points'])
                    head_batch=self.capture_batch(report,'头部')
                    report['head_capture']=head_batch
                    report['steps'].append({'step':'capture','half':'head','batch':head_batch,
                                            'counts':dict(self.counts())})
                    self.pipeline_stop()
                    self.pipeline_progress(phase='头部：求解')
                    self.action('/api/fit-source',{'id':head_batch})
                    self.action('/api/fit',{})
                    self.wait_for_step(1800,'头部求解')
                    head=self.newest_head()
                    report['steps'].append({'step':'solve','half':'head','result_id':head['id'],
                                            'metrics':self.pipeline_head_metrics(head)})
                elif mode=='wrist':
                    head=resume_head
                    report['steps'].append({'step':'solve','half':'head','reused':True,'result_id':head['id'],
                                            'metrics':self.pipeline_head_metrics(head)})
                else:
                    # Round 0: the head half as it stands, mounts free - the baseline the
                    # iteration has to beat.
                    self.action('/api/workflow',{'kind':'head'})
                    self.action('/api/fit-source',{'id':report['head_capture']})
                    self.marker_prior_source=None;self.persist()
                    self.pipeline_progress(phase='头部第 1 次求解（贴纸自由估计）')
                    self.fit_head_batch()
                    head=self.newest_head()
                    report['steps'].append({'step':'head','round':0,'source':'capture','result_id':head['id'],
                                            'marker_prior':None,'metrics':self.pipeline_head_metrics(head)})
                for number in range(1,rounds+1):
                    prefix=('腕半场' if mode in ('full','wrist') else '第 %d 轮腕半场'%number)
                    self.pipeline_stop()
                    self.pipeline_progress(phase=prefix+'：规划')
                    self.action('/api/workflow',{'kind':'wrist','head_result_id':head['id']})
                    self.plan_wrists()
                    plan=self.plan or {}
                    self.pipeline_stop()
                    self.pipeline_progress(phase=prefix+'：审阅与采集')
                    wrist_batch=self.capture_batch(report,prefix)
                    self.pipeline_stop()
                    if capture_only:
                        counts=dict(self.counts())
                        report.update(capture_completed=True,wrist_capture=wrist_batch,counts=counts,
                                      message='双腕采集完成，诊断批次已保留；未求解或安装，下一次需重新生成并审阅计划')
                        report['steps'].append({'step':'capture','half':'wrist','batch':wrist_batch,'counts':counts})
                        return
                    self.action('/api/fit-source',{'id':wrist_batch})
                    self.pipeline_progress(phase=prefix+'：求解')
                    self.action('/api/wrist-fit',{})
                    self.wait_for_step(1800,prefix+'求解')
                    wrist=self.newest_wrist()
                    mounts={str(row['marker_id']):row['T_link6_marker']
                            for row in ((wrist.get('left') or {}).get('marker_mount'),(wrist.get('right') or {}).get('marker_mount')) if row}
                    metrics=self.pipeline_wrist_metrics(wrist)
                    step={'step':'wrist','round':number,'half':'wrist','result_id':wrist['id'],
                          'batch':wrist_batch,'motion_seed':plan.get('motion_seed'),
                          'min_clearance_m':plan.get('min_clearance_m'),
                          'pair_status':wrist.get('pair_status'),'marker_mounts':sorted(mounts),
                          'acceptance':{'max_holdout_rms_px':self.wrist_acceptance_px()},
                          'metrics':metrics}
                    report['steps'].append(step)
                    if wrist.get('pair_status')=='complete':
                        report.setdefault('pairs',[]).append({
                            'round':number,'head_result_id':head['id'],'wrist_result_id':wrist['id'],
                            'worst_holdout_rms_px':max(metrics[side]['holdout_rms_px'] for side in ('left','right'))})
                    if mode in ('full','wrist'):
                        continue
                    if not mounts:
                        step['adopted']=None
                        report['aborted']='腕半场没有给出贴纸安装估计（该批次未开启标识估计），重解迭代到此为止'
                        break
                    self.marker_prior_source=wrist['id'];self.persist()
                    self.pipeline_progress(phase=prefix+'：头部重解（冻结腕估计的贴纸安装）')
                    self.action('/api/workflow',{'kind':'head'})
                    self.action('/api/fit-source',{'id':report['head_capture']})
                    self.fit_head_batch()
                    head_frozen=self.newest_head(marker_frozen=True)
                    frozen_metrics=self.pipeline_head_metrics(head_frozen)
                    step={'step':'head','round':number,'source':'re-fit','result_id':head_frozen['id'],
                          'marker_prior':wrist['id'],'metrics':frozen_metrics,'adopted':None}
                    report['steps'].append(step)
                    # The prior has to earn its place: the head half re-fits the same pixels
                    # with the mount held, so its held-out stickers say whether the outside
                    # value describes this robot better than the head's own. Freezing can
                    # never help in sample - it removes twelve parameters - so a *held-out*
                    # regression is the signal to stop rather than publish a worse head.
                    free_holdout=report['steps'][0]['metrics'].get('holdout_marker_rms_px')
                    frozen_holdout=frozen_metrics.get('holdout_marker_rms_px')
                    if free_holdout is not None and frozen_holdout is not None and frozen_holdout>free_holdout+1e-9:
                        step['adopted']=False
                        report['aborted']=('第 %d 轮的贴纸安装先验未通过：头部在同一批像素上带着它重解，'
                            '留出贴纸 rms %.3f → %.3f px（自由估计 → 冻结外部先验）；'
                            '外部值没有让头部自己的留出像素更好，因此不再继续迭代，也不采用它'
                            %(number,free_holdout,frozen_holdout))
                        self.marker_prior_source=None;self.persist()
                        break
                    step['adopted']=True
                    head=head_frozen
                # Publish and install the best complete pair the loop produced. Iterating
                # does not guarantee the last round is the best one, so the choice is made
                # on the numbers rather than on the order they arrived in.
                pairs=sorted(report.get('pairs',[]),key=lambda row:row['worst_holdout_rms_px'])
                if not pairs:
                    report['aborted']=report.get('aborted') or '双腕没有成对通过留出检查，因此没有发布，也没有安装'
                else:
                    best=pairs[0];report['best_pair']=best
                    self.pipeline_progress(phase='发布并安装（最差一侧留出 %.3f px）'%best['worst_holdout_rms_px'])
                    self.pipeline_stop()
                    installed=self.action('/api/calibration-install',{'head_result_id':best['head_result_id'],
                                                                      'wrist_result_id':best['wrist_result_id']})
                    report['installed']=installed.get('installed');report['message']=installed.get('message')
                    if len(pairs)<rounds:
                        report['aborted']=('只有 %d/%d 轮的双腕成对通过留出检查；已安装其中最好的一轮，'
                                           '其余轮次未成对的原因见各轮记录')%(len(pairs),rounds)
            except PipelineStopped as stop:
                report['stopped']=True
                report['aborted']=str(stop)
            except Exception as exc:
                # `/api/state`'s `reason` prefers the hardware link's own error when the
                # workbench is live, which can hide why a pipeline step died; the pipeline
                # therefore keeps its own account, including the phase it died in.
                report['failed_phase']=report.get('phase')
                report['aborted']='%s：%s'%(type(exc).__name__,exc)
                raise
            finally:
                self.pipeline_progress(phase=None)
                with self.lock:
                    self.pipeline_report=report
                    self.pipeline_running=False;self.pipeline_review=None;self.pipeline_run_id=None
                    self.calibration_previews={}

        self.idle()
        self.pipeline_running=True;self.pipeline_run_id=secrets.token_hex(12)
        try:self.background('PIPELINE',job)
        except Exception:
            self.pipeline_running=False;self.pipeline_run_id=None
            raise
        message=('完整标定已启动：头部（规划→采集→求解）→ 腕半场（规划→采集→求解）→ 发布并安装一组'
                 if mode=='full' else
                 '重解迭代已启动：头部重解 → %d 轮（腕半场估计贴纸安装 → 头部冻结重解）→ 发布安装'%rounds)
        if mode=='wrist':message='已复用指定头部结果：重新规划双腕，等待现场审阅后采集、求解及安装'
        if capture_only:message='已复用指定头部结果：重新规划双腕，等待现场审阅后仅采集诊断批次'
        return {'message':message,'mode':mode,'rounds':rounds}

    def head_fit_file(self,row):
        return self.output/row['batch']/row['file']

    def pipeline_head_metrics(self,row):
        # Read the fit report itself: the candidate projection the panel uses drops the
        # mount bookkeeping, and a round is only a round if the freeze actually happened.
        report=read(self.head_fit_file(row)) if self.head_fit_file(row).exists() else {}
        metrics=report.get('metrics_rectified_px') or row.get('metrics_rectified_px') or {}
        mesh=report.get('mesh_metrics') or row.get('mesh_metrics') or {}
        return {'holdout_marker_rms_px':(metrics.get('holdout') or {}).get('rms_px'),
                'holdout_mesh_coverage':(mesh.get('holdout') or {}).get('macro_coverage'),
                'holdout_mesh_px':(mesh.get('holdout') or {}).get('macro_mean_penalized_px'),
                'marker_mounts_frozen':sorted(str(key) for key in (report.get('frozen_mounts') or {})),
                'mount_mode':report.get('mount_mode'),
                'frozen_mount_provenance':report.get('frozen_mount_provenance')}

    def pipeline_wrist_metrics(self,row):
        return {side:{'holdout_rms_px':((row.get(side) or {}).get('holdout') or {}).get('rms_px'),
                      'fit_rms_px':((row.get(side) or {}).get('fit') or {}).get('rms_px'),
                      'marker_mount_delta':((row.get(side) or {}).get('marker_mount') or {}).get('delta_from_head_snapshot')}
                for side in ('left','right')}

    def result_mtime(self,row):
        try:return (self.output/row['batch']/row['file']).stat().st_mtime
        except (OSError,KeyError,TypeError):return 0.

    def newest_head(self, marker_frozen=False):
        # One capture can hold several fits (each round writes its own), so recency comes
        # from the file, not from a field the records do not carry.
        rows=[row for row in self.calibration_results() if row.get('candidate')=='joint_candidate' and (not getattr(self,'pipeline_running',False) or row.get('batch')==self.selected_batch)]
        if not rows:raise ValueError('头部求解没有产生结果')
        rows=sorted(rows,key=self.result_mtime)
        if marker_frozen:
            frozen=[row for row in rows if (read(self.head_fit_file(row)) if self.head_fit_file(row).exists() else {}).get('frozen_mounts')]
            if not frozen:
                raise ValueError('头部重解没有冻结贴纸安装，流水线不会把一个没有生效的先验算作一轮')
            return frozen[-1]
        return rows[-1]

    def newest_wrist(self):
        rows=sorted((row for row in self.wrist_results() if not getattr(self,'pipeline_running',False) or row.get('batch')==str(Path(self.selected_batch).parent)),key=self.result_mtime)
        if not rows:raise ValueError('腕半场求解没有产生结果')
        return rows[-1]

    def background(self,status,job):
        self.idle()
        if getattr(self,'pipeline_running',False) and status!='PIPELINE':self.pipeline_stop()
        self.invalidate_reviews()
        self.status=status;self.reason=None
        if status=='PIPELINE' or not getattr(self,'pipeline_running',False):self.stop.clear()
        def run():
            try:
                job()
                with self.lock:self.status='STOPPED' if self.stop.is_set() else 'READY'
            except Exception as exc:
                with self.lock:self.status='REJECTED';self.reason=str(exc)
        threading.Thread(target=run,daemon=True).start()

    def move(self,q,duration,gaps):
        start_jaw=self.jaw_mm.copy();start=self.q.copy(); began=time.monotonic()
        while not self.stop.is_set():
            u=min((time.monotonic()-began)/duration,1); blend=10*u**3-15*u**4+6*u**5
            value=start+(q-start)*blend; jaw=start_jaw+(gaps-start_jaw)*blend;actual=self.display(value,jaw)
            with self.lock:self.q=value;self.jaw_mm=jaw;self.actual=actual
            if u>=1:break
            self.stop.wait(.08)

    def capture_request(self,waypoints):
        g=self.geometry; stations=[]; shared=[]
        for side in ('left','right'):
            for name,v,_ in g.side_meshes[side]:
                unique,inverse=np.unique(v,axis=0,return_inverse=True)
                shared.append({'side':side,'link':name,'vertices':unique.tolist(),'faces':inverse.reshape(-1,3).tolist()})
        for i,w in enumerate(waypoints):
            if w.get('capture') is False:continue
            q=np.array(w['q']);frames=g.frames(q);markers=[]
            for side in ('left','right'):
                T=frames[side]['link6']@g.mounts[side];s=g.marker_size/2
                corners=np.array([[-s,-s,0],[s,-s,0],[s,s,0],[-s,s,0]])
                markers.append({'id':24 if side=='left' else 30,'corners_world':(corners@T[:3,:3].T+T[:3,3]).tolist()})
            stations.append({'id':f'frame-{i:03d}','q':q.tolist(),'arm':w.get('arm'),'transition':bool(w.get('transition')),'split':w.get('split','holdout' if len(stations)%5==4 else 'fit'),'transforms':{side:{name:T.tolist() for name,T in links.items()} for side,links in frames.items()},'markers':markers})
        centers={}
        for station,w in zip(stations,[w for w in waypoints if w.get('capture') is not False]):
            side=w.get('arm')
            if side in ('left','right'):
                station['expected_marker_id']=24 if side=='left' else 30
                if w.get('cell')==[1,1] and w.get('kind')=='grid':centers[side]=station
        for station in stations:
            side=station.get('arm');center=centers.get(side)
            if center is None:continue
            recovery=[]
            for fraction in (.5,.8):
                q=np.array(station['q'])*(1-fraction)+np.array(center['q'])*fraction
                if not g.path_check(station['q'],q)['ok']:continue
                frames=g.frames(q);markers=[]
                for arm in ('left','right'):
                    T=frames[arm]['link6']@g.mounts[arm];z=g.marker_size/2
                    corners=np.array([[-z,-z,0],[z,-z,0],[z,z,0],[-z,z,0]])
                    markers.append({'id':24 if arm=='left' else 30,'corners_world':(corners@T[:3,:3].T+T[:3,3]).tolist()})
                recovery.append({'q':q.tolist(),'transforms':{arm:{n:T.tolist() for n,T in links.items()} for arm,links in frames.items()},'markers':markers})
            station['recovery']=recovery
        return {'camera':{**g.k,'T_world_optical':g.camera.tolist()},'shared_parts':shared,'stations':stations,'mount_chain':g.mount_chain,'mount_status':g.mount_status}

    def run_process(self,argv,log):
        with log.open('w') as out:
            self.process=subprocess.Popen(argv,stdout=out,stderr=subprocess.STDOUT)
            while self.process.poll() is None:
                progress_path=getattr(self,'fit_progress_path',None)
                if progress_path and progress_path.exists():
                    try:self.pipeline_subprogress(2,read(progress_path))
                    except (OSError,ValueError,KeyError):pass
                if self.stop.wait(.2):
                    self.process.terminate()
                    try:self.process.wait(timeout=20)
                    except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
                    raise ValueError('任务已停止；远端停止记录见 capture.log')
            code=self.process.returncode;self.process=None
            if code:
                with log.open('rb') as stream:
                    stream.seek(max(0,log.stat().st_size-4096));lines=stream.read().decode(errors='replace').strip().splitlines()
                detail=lines[-1][:1000] if lines else '无错误详情'
                raise ValueError(f'worker exited {code}: {detail}; see {log}')

    def action(self,path,body):
        if path in ('/api/stop','/api/control/stop') or path=='/api/calibration-pipeline' and body.get('stop'):
            return self.request_stop(body if path=='/api/control/stop' else {},hardware=path=='/api/control/stop')
        if path=='/api/preview-target' and self.live:
            if getattr(self,'pipeline_running',False):raise ValueError('标定运行期间不接受手动预览')
            return self.queue_preview(body)
        review=self.review_snapshot() if path in Workbench.PREFLIGHT_PATHS else None
        with self.mode_lock:
            if getattr(self,'fake_restart_in_progress',False) and path!='/api/stop':
                raise ValueError('Fake 控制器正在恢复，暂不接受新操作')
            if getattr(self,'pipeline_running',False) and threading.get_ident()!=getattr(self,'pipeline_thread',None):
                allowed=('/api/stop','/api/plan-preview','/api/overview-view')
                if path=='/api/calibration-pipeline' and body.get('stop'):return self._action('/api/stop',{})
                if path not in allowed and not (getattr(self,'pipeline_review',None) and path in ('/api/calibration-preview','/api/calibration-confirm')):
                    raise ValueError('标定流水线正在运行；请先停止或完成当前审阅')
            if path not in Workbench.PREFLIGHT_PATHS:return self._pipeline_action(path,body)
        return self.preflight_action(path,body,review)

    def preflight_action(self,path,body,review):
        if path=='/api/hardware-trajectory-preview':return self.hardware_trajectory_preview(body,review=review)
        if path=='/api/queue':return self.queue_pose(body,review=review)
        if path=='/api/calibration-preview':return self.preview_calibration(review=review)
        if path=='/api/hardware-jaw-preview':
            if set(body) != {'gap_mm'} or not isinstance(body['gap_mm'],dict) or set(body['gap_mm']) != {'left','right'}:
                raise ValueError('left/right actual gap_mm required')
            body={'active_arm':'both','sdk_gap_mm':{side:command_sdk(side,body['gap_mm'][side])*1000 for side in ('left','right')}}
            path='/api/control/jaw-preview'
        return self.control_request(path.removeprefix('/api/control/'),body,review=review)

    def _pipeline_action(self,path,body):
        if path in ('/api/stop','/api/control/stop') or path=='/api/calibration-pipeline' and body.get('stop'):
            return self.request_stop(body if path=='/api/control/stop' else {},hardware=path=='/api/control/stop')
        if path in Workbench.PREFLIGHT_PATHS:return self.preflight_action(path,body,self.review_snapshot())
        if path=='/api/overview-view':return self.remote_request('/overview-view',body)
        if path=='/api/preview-target' and self.live:return self.queue_preview(body)
        with self.mode_lock:return self._action(path,body)

    def _action(self,path,body):
        if path in ('/api/stop','/api/control/stop') or path=='/api/calibration-pipeline' and body.get('stop'):
            return self.request_stop(body if path=='/api/control/stop' else {},hardware=path=='/api/control/stop')
        if path in Workbench.PREFLIGHT_PATHS:return self.preflight_action(path,body,self.review_snapshot())
        if path=='/api/overlay-rendering':
            if set(body)!={'enabled'} or type(body['enabled']) is not bool:raise ValueError('enabled must be boolean')
            if body['enabled'] and 'mesh_rendering_enabled' not in self.require_remote():
                raise ValueError('渲染器尚不支持实时叠加开关，请先更新对应后端')
            self.mesh_rendering_enabled=body['enabled'];self.persist()
            try:
                self.remote_request('/overlay-rendering',body)
                applied=True
            except ValueError:applied=False
            return {'enabled':self.mesh_rendering_enabled,'applied':applied}
        if path=='/api/calibration-preview':return self.preview_calibration()
        if path=='/api/calibration-confirm':
            result=self.confirm_calibration(body)
            if getattr(self,'pipeline_review',None):
                self.pipeline_review=None
                self.pipeline_review_result=result
            return result
        if path=='/api/mesh-view-mode':
            mode=body.get('mode')
            if mode not in ('simulation','hardware'):raise ValueError('invalid mesh view mode')
            with self.lock:self._mesh_view_mode=mode;self.persist()
            return {'mesh_view_mode':mode,'environment':self.active_environment()}
        if path=='/api/workflow':
            self.idle();kind=body.get('kind');head=body.get('head_result_id')
            if kind not in ('head','wrist'):raise ValueError('invalid calibration kind')
            if kind=='wrist' and head:
                chosen=next((r for r in self.calibration_results() if r['id']==head and r['parameters']['snapshot_complete']),None)
                if not chosen:raise ValueError('腕标定需要明确选择完整的头标定版本')
                self.require_calibration_environment(chosen)
                self.require_joint_head_dependency(chosen)
                self.register_result(chosen)
            with self.lock:
                updated={'kind':kind,'head_result_id':(head or None) if kind=='wrist' else None}
                if updated!=self.workflow:
                    self.invalidate_reviews();self.workflow=updated;self.queue=[];self.plan=None;self.persist()
            return {'message':'标定类型与依赖已更新；变更后需重新规划','workflow':self.workflow}
        if path=='/api/calibration-register':
            with self.lock:
                # One group = one head + the wrist result solved against it. Saving the two
                # together is what makes the pair meaningful; a head on its own would be a
                # version nothing can use, so it is refused with the reason for the refusal.
                group=next((g for g in self.calibration_groups() if g['id']==body.get('head_result_id')),None)
                if not group:
                    # A page from before this rule posts `result_id`; tell it the new unit
                    # instead of leaving it with a vague rejection.
                    if body.get('result_id'):raise ValueError('标定版本按“头部 + 匹配的双腕”一组保存，请重新选择一组')
                    raise ValueError('请选择已验证的标定结果')
                head=next((r for r in self.calibration_results() if r['id']==group['id']),None)
                if not head:raise ValueError('请选择已验证的标定结果')
                self.require_calibration_environment(head)
                if not group['wrist'] or body.get('wrist_result_id')!=group['wrist']['id']:
                    raise ValueError('标定版本按“头部 + 匹配的双腕”一组保存：'+str(group.get('reason') or '请选择包含匹配双腕的一组'))
                if not group['saveable']:raise ValueError(group['reason'] or '该组当前不能保存')
                wrist=next((r for r in self.wrist_results() if r['id']==group['wrist']['id']),None)
                if not wrist:raise ValueError('匹配的双腕结果已变化，请重新选择')
                self.require_calibration_environment(wrist)
                # The wrist record carries the head as its dependency, so publishing it
                # publishes the head too; the head call is the idempotent second half.
                wrist_record=self.register_result(wrist)
                head_record=self.register_result(head)
                return {'message':'标定版本已保存：头部 ＋ 双腕一组','group':group['id'],
                        'wrist_result_id':wrist['id'],
                        'head_version':(head_record or {}).get('version'),'version':(wrist_record or {}).get('version')}
        if path=='/api/calibration-install':
            if body.get('confirmation')=='INSTALL_USER_ACCEPTED_PROVISIONAL':
                return self.install_provisional_pair(body)
            with self.lock:
                # Installing is the operator's decision about which calibration is in
                # force; publishing to the library is not, because a solved head is
                # published on its own and would otherwise make "save" the only step.
                group=next((g for g in self.calibration_groups() if g['id']==body.get('head_result_id')),None)
                if not group:
                    if body.get('result_id'):raise ValueError('安装的单位是“头部 + 匹配的双腕”一组，请重新选择一组')
                    raise ValueError('请选择一组已验证的标定结果')
                if not group['wrist'] or body.get('wrist_result_id')!=group['wrist']['id']:
                    raise ValueError('安装的单位是“头部 + 匹配的双腕”一组：'+str(group.get('reason') or '请选择包含匹配双腕的一组'))
                if not group['installable'] and not group['installed']:
                    raise ValueError(group['reason'] or '该组当前不能安装')
                head=next((r for r in self.calibration_results() if r['id']==group['id']),None)
                wrist=next((r for r in self.wrist_results() if r['id']==group['wrist']['id']),None)
                if not head or not wrist:raise ValueError('匹配的双腕结果已变化，请重新选择')
                self.require_calibration_environment(head)
                self.require_calibration_environment(wrist)
                store=self.store_for(group.get('environment','simulation'))
                head_record=self.register_result(head)
                wrist_record=self.register_result(wrist)
                if not head_record or not wrist_record:
                    raise ValueError('标定库不可用，无法安装')
                manifest=store.install(head_record,wrist_record,geometry_id=self.geometry_id,
                                       acceptance=self.install_acceptance(wrist),
                                       notes=self.install_notes(head_record,wrist_record),robot_source=self.robot_source())
                return {'message':'已安装为当前有效标定：头部 %s ＋ 双腕 %s'%(head_record.get('version'),wrist_record.get('version')),
                        'group':group['id'],'installed':self.notify_installation(),
                        'path':manifest.get('path')}
        if path=='/api/calibration-pipeline':
            if body.get('stop'):
                return self._action('/api/stop',{})
            return self.run_calibration_pipeline(body)
        if path=='/api/marker-prior':
            with self.lock:
                # The prior is a wrist result id, never a hand-typed transform: the head
                # re-fit reads the mounts back from that result, so the state cannot
                # drift from the evidence that produced it.
                source=body.get('source')
                if source in (None,'','clear'):
                    self.marker_prior_source=None;self.persist()
                    return {'message':'已清除标识先验：头部求解会重新自由估计贴纸安装','marker_prior':None}
                if not isinstance(source,str):raise ValueError('请选择一个腕结果作为标识先验来源')
                row=next((r for r in self.wrist_results() if r['id']==source),None)
                if not row:raise ValueError('请选择一个腕结果作为标识先验来源')
                self.require_calibration_environment(row)
                if not row.get('marker_mounts'):
                    raise ValueError('该腕结果没有估计标识安装（规划时未开启标识估计），请重跑腕半场')
                self.marker_prior_source=source;self.persist()
                prior=self.marker_prior()
                if prior and prior.get('error'):raise ValueError(prior['error'])
                return {'message':'已把腕结果 %s 估计的贴纸安装设为头部求解的先验：%s'
                                  %(source[:12],'、'.join('标识 '+marker for marker in sorted(prior['mounts']))),
                        'marker_prior':prior}
        if path in ('/api/history-delete-preview','/api/history-delete'):
            with self.lock:return self.delete_history(body,preview=path.endswith('-preview'))
        if path=='/api/history-select':
            with self.lock:return self.review_batch(body.get('id'))
        if path=='/api/calibration-historical':
            with self.lock:return self.historical_overlay(body)
        if path=='/api/fit-source':
            self.idle()
            with self.lock:
                identity=body.get('id')
                if identity is None:
                    self.selected_batch=None;self.frames=[];self.fit_result=None;self.evaluation=None;self.batch=None
                    if hasattr(self,'dataset'):del self.dataset
                else:
                    source=next((r for r in self.history() if r['id']==identity),None)
                    if not source or source['kind']!=self.workflow['kind']:raise ValueError('求解批次与当前标定类型不一致')
                    self.load_batch(identity)
                self.persist()
            return {'message':'求解输入已选择；历史回放保持独立'}
        if path=='/api/wrist-plan':
            if getattr(self,'workflow',{}).get('kind')!='wrist':raise ValueError('请先选择腕部标定及头标定依赖')
            self.idle();self.background('PLANNING',self.plan_wrists);return {'message':'正在规划双腕互看贴纸，固定头标定并检查全路径'}
        if path=='/api/wrist-fit':
            if getattr(self,'workflow',{}).get('kind')!='wrist':raise ValueError('请先选择腕部标定')
            if not hasattr(self,'dataset'):raise ValueError('请先选择腕部求解批次')
            allow_partial=body.get('allow_partial',False)
            if not isinstance(allow_partial,bool):raise ValueError('allow_partial must be boolean')
            self.idle();self.background('FITTING',lambda:self.fit_wrists(allow_partial=allow_partial));return {'message':'正在求解左右腕外参并检查独立留出与完整采集覆盖'}
        if path=='/api/wrist-select':
            self.idle();remote=self.remote or {}
            selected=next((r for r in self.wrist_results() if r['id']==body.get('result_id')),None)
            if selected is None:raise ValueError('所选腕结果不存在')
            # Display selection is independent of numerical acceptance and installation.
            error=self.wrist_overlay_error(selected)
            if error:raise ValueError(error)
            self.require_calibration_environment(selected)
            contract=self.wrist_contract(selected)
            digest=selected['result']['contract_sha256']
            # Persist the camera's selection even when the optional DSW display is unavailable.
            display_error=None
            if not self.remote_error:
                try:
                    self.remote_request('/wrist-config',{'wrist_contract':contract})
                    for side in ('left','right'):
                        self.remote_request('/wrist-calibration',{'id':selected['id'],'observer':side,
                            'T_link6_camera':selected[side]['T_link6_camera'],
                            'head_snapshot_sha256':selected['result']['head_snapshot_sha256'],'contract_sha256':digest})
                except ValueError as exc:display_error=str(exc)
            self.wrist_selected={'id':selected['id'],'contract_sha256':digest,'pair_status':selected.get('pair_status'),
                                 'file':selected.get('file'),'environment':selected.get('environment'),
                                 'robot_source':selected.get('robot_source'),'simulation_only':selected.get('simulation_only'),
                                 'left':selected['left'],'right':selected['right']}
            (self.output/'wrist-selected.json').write_text(json.dumps(self.wrist_selected,allow_nan=False))
            message='双腕标定已选择' if not display_error else '双腕标定已记录；DSW 显示暂不可用：'+display_error
            return {'message':message,'selected':self.wrist_selected,'display_error':display_error}

        if path=='/api/control-mode':return self.set_control_mode(body.get('mode'))
        if path.startswith('/api/control/'):
            return self.control_request(path.removeprefix('/api/control/'),body)
        with self.lock:
            if self.live and path in ('/api/execute','/api/batch','/api/run-batch') and (self.pending_capture or self.importing):raise ValueError('上一批采集尚未保存，禁止覆盖远端产物；保存失败时点击已读重试')
            if path=='/api/calibration':
                if not self.live:raise ValueError('DSW live renderer is required')
                if body.get('clear'):
                    self.calibration=None;(self.output/'realtime-calibration.json').write_text('{}');return self.remote_request('/calibration',{'clear':True})
                result=next((row for row in self.calibration_results() if row['id']==body.get('result_id')),None)
                if result is None:raise ValueError('请选择参数有效的标定结果')
                self.require_calibration_environment(result)
                candidate=result['parameters']
                self.calibration=dict(result)
                (self.output/'realtime-calibration.json').write_text(json.dumps(self.calibration))
                try:self.remote_request('/calibration',candidate)
                except ValueError:pass
                return {'message':'已选择两侧实时叠加参数；DSW 断线不影响真机侧','calibration':self.calibration}
            if path=='/api/plan-preview':
                index=body.get('index')
                if type(index) is not int or not 0<=index<len(self.queue):raise ValueError('invalid plan index')
                if self.live:return {**self.preview({**self.queue[index],'revision':body.get('revision',0)}),'plan_index':index,'waypoint':self.queue[index]}
                return {**self.display(self.queue[index]['q'],self.queue[index].get('jaw_gap_mm',self.jaw_mm)),'plan_index':index,'waypoint':self.queue[index]}
            if path=='/api/preview':return self.preview(body)
            if path=='/api/ack':
                if self.live and self.import_error:
                    self.import_error=None;return {'message':'重新下载上一批采集；保存完成前禁用执行'}
                self.idle()
                if self.live and (self.remote or {}).get('status',(self.remote or {}).get('state')) in ('STOPPED','ERROR'):
                    self.remote_request('/ack',{})
                self.reason=None;self.status='READY';self.tokens.clear();return {'message':'已读并恢复规划；旧预览失效'}
            if self.hardware_sync_active() and path in ('/api/execute','/api/batch','/api/run-batch'):raise ValueError('真机已接入；请使用统一真机轨迹审阅与执行')
            if path=='/api/execute':
                self.idle();t=self.tokens.pop(body.get('token'),None)
                if not t:raise ValueError('预览已失效；请重新预览，通过路径检查后再执行')
                if t.get('collision_free') is not True:raise ValueError('该预览未通过路径检查，不能执行')
                if t['expires']<time.monotonic() or np.max(abs(t['start']-self.q))>.002 or np.max(abs(t['start_jaw']-self.jaw_mm))>.1:raise ValueError('预览过期或起点改变，请重新预览')
                if self.live:
                    self.require_remote();return self.remote_request('/execute',{'waypoints':[{'q':t['q'].tolist(),'jaw_gap_mm':t['jaw_gap_mm'].tolist(),'speeds':t['speeds'].tolist(),'jaw_speed_mm_s':t.get('jaw_speed_mm_s',10.),'capture':False}],'expected_start':t['start'].tolist(),'expected_jaw_gap_mm':t['start_jaw'].tolist()})
                self.background('RUNNING',lambda:self.move(t['q'],t['duration'],t['jaw_gap_mm']));return {'message':'执行仿真动作'}
            if path=='/api/queue-clear':
                self.idle();self.invalidate_reviews();self.queue=[];self.plan=None;self.plan_geometry_id=self.geometry_id;self.persist();return {'message':'队列已清空'}
            if path=='/api/queue-remove':
                self.idle();index=body.get('index')
                if type(index) is not int or not 0<=index<len(self.queue):raise ValueError('invalid queue index')
                self.invalidate_reviews();self.queue.pop(index);self.plan=None;self.persist();return {'message':'姿态已移除'}
            if path in ('/api/plan','/api/auto-plan'):
                if getattr(self,'workflow',{'kind':'head'})['kind']!='head':raise ValueError('请先选择头部标定')
                self.idle()
                def plan():
                    if self.control_mode=='hardware_sync' and not self.hardware_connected():raise ValueError('真机反馈不可用，不能规划真机标定')
                    start=np.array(self.state()['qpos'])
                    with self.lock:
                        self.queue=[];self.plan=None;self.plan_geometry_id=self.geometry_id;self.persist()
                    result=self.geometry.with_gaps(80.,80.).auto_plan(start)
                    result['waypoints']=self.calibration_waypoints(result['waypoints'])
                    if result.get('plan_complete') is False:
                        with self.lock:
                            self.plan=result;self.plan_geometry_id=self.geometry_id;self.queue=[];self.persist()
                        raise ValueError('头部标定计划未通过采样检查：'+('；'.join(result.get('deficiencies',[])) or '见规划详情'))
                    with self.lock:
                        self.plan=result;self.plan_geometry_id=self.geometry_id;self.queue=[dict(w,name=f"{w['arm']} {w['kind']} {w['cell']}",jaw_gap_mm=w.get('jaw_gap_mm',self.jaw_mm.tolist()),speeds=[.2]*12) for w in result['waypoints']]
                        self.persist()
                self.background('PLANNING',plan);return {'message':'正在生成双臂视野覆盖与关节变化采样计划'}
            if path in ('/api/batch','/api/run-batch'):
                if self.control_mode=='hardware_sync':raise ValueError('真机采集必须先预览并确认标定轨迹')
                self.idle()
                if not self.queue:raise ValueError('队列为空')
                trusted_seed=self.standard_motion_seed_active()
                if not trusted_seed and (self.plan or {}).get('plan_complete') is False:raise ValueError('当前标定计划未通过采样检查，请重新规划')
                if not trusted_seed and self.plan_geometry_id!=self.geometry_id:raise ValueError('历史计划缺少当前几何依据，请重新生成计划，或清空后重新添加姿态')
                if self.live:
                    remote=self.require_remote()
                    if not remote.get('capabilities',{}).get('jaw_gap_mm'):raise ValueError('DSW 后端尚不支持夹爪开度')
                    frozen=json.loads(json.dumps(self.queue));sample=0
                    if any(w.get('observer') for w in frozen) and not (self.plan or {}).get('wrist_contract'):
                        raise ValueError('腕队列已编辑导致冻结合同失效，请重新规划双腕')
                    for w in frozen:
                        if w.get('capture') is not False:w.setdefault('split','holdout' if sample%5==4 else 'fit');sample+=1
                        else:w.setdefault('split','fit')
                        # DSW's waypoint schema has no ``both`` arm value;
                        # q already contains both arms, so omit this metadata
                        # for the initial all-joint zero transition.
                        if w.get('arm') not in ('left','right'):w.pop('arm',None)
                    payload={'waypoints':frozen,'expected_start':self.q.tolist(),'expected_jaw_gap_mm':self.jaw_mm.tolist()}
                    if not trusted_seed and not self.queue_is_plan_checked(frozen):self.require_checked_path(frozen,self.q)
                    if trusted_seed:payload['standard_motion_seed']=self.STANDARD_MOTION_SEED
                    if self.plan and self.plan.get('wrist_contract'):
                        if self.control_mode!='simulation':raise ValueError('腕队列仅支持纯仿真')
                        payload['wrist_contract']=self.plan['wrist_contract']
                        if getattr(self,'workflow',{}).get('head_result_id')!=payload['wrist_contract']['head_snapshot']['id']:
                            raise ValueError('头标定已变化，请重新规划双腕')
                    result=self.remote_request('/execute',payload)
                    self.pending_capture=result['capture_id'];return {**result,'message':'DSW 逐点执行队列；反馈与采集来自远端'}
                if all(w.get('capture') is False for w in self.queue):
                    frozen=json.loads(json.dumps(self.queue))
                    # Manual motion queue is usable without any camera/capture worker.
                    start=self.q.copy()
                    for w in frozen:
                        target=self.geometry._q(w['q']);gaps=jaw_gaps(w.get('jaw_gap_mm',self.jaw_mm))
                        speeds=np.asarray(w.get('speeds',[.2]*12),float)
                        if speeds.shape!=(12,) or not np.isfinite(speeds).all() or np.any(speeds<=0) or np.any(speeds>self.velocity_limits):raise ValueError('invalid queued speeds')
                        if not self.geometry.with_gaps(83.,83.).path_check(start,target)['ok']:raise ValueError('queued path rejected')
                        start=target
                    def manual_queue():
                        for w in frozen:
                            if self.stop.is_set():break
                            target=np.asarray(w['q']);gaps=jaw_gaps(w.get('jaw_gap_mm',self.jaw_mm))
                            duration=max(.05,float(np.max(1.875*abs(target-self.q)/w['speeds'])),float(np.max(1.875*abs(gaps-self.jaw_mm)/w.get('jaw_speed_mm_s',10.))))
                            self.move(target,duration,gaps)
                    self.background('RUNNING',manual_queue);return {'message':'执行仿真人工动作队列（不采集）'}
                if any(not np.allclose(w.get('jaw_gap_mm',self.jaw_mm),[self.geometry.gaps[s]*1000 for s in ('left','right')]) for w in self.queue):raise ValueError('变开度批量采集需更新后的 DSW live 后端；离线采集器只支持固定开度')
                if not self.config.get('capture_command'):raise ValueError('Isaac capture worker 未配置，不生成假采集成绩')
                frozen=json.loads(json.dumps(self.queue));start=self.q.copy()
                def capture():
                    current=start
                    for w in frozen:
                        if not trusted_seed:
                            check=self.geometry.path_check(current,w['q'])
                            if not check['ok']:raise ValueError('批次路径检查失败: '+str(check))
                        current=np.array(w['q'])
                    root=self.output/('batch-'+secrets.token_hex(5));root.mkdir()
                    (root/'plan.json').write_text(json.dumps({'waypoints':frozen},allow_nan=False))
                    request=root/'request.json';request.write_text(json.dumps(self.capture_request(frozen)))
                    self.run_process([*self.config['capture_command'],'--request',str(request),'--output',str(root/'capture')],root/'capture.log')
                    with self.lock:
                        self.load_batch(str((root/'capture').relative_to(self.output)))
                        if not self.frames:raise ValueError('capture returned no image rows')
                        self.q=np.array(self.frames[-1]['q_reported_rad']);self.actual=self.display(self.q)
                        self.persist()
                self.background('CAPTURING',capture);return {'message':'冻结队列并启动 Isaac 图像采集'}
            if path=='/api/fit':
                self.idle()
                if getattr(self,'workflow',{}).get('kind','head')!='head':raise ValueError('请先选择头部标定')
                if not hasattr(self,'dataset') or not self.frames:raise ValueError('请先完成图像采集')
                if (self.dataset/'dataset.json').exists() and read(self.dataset/'dataset.json').get('wrist_contract'):raise ValueError('此批次为腕标定，请使用腕求解')
                completion=self.dataset/'completion.json'
                if not completion.exists() or not read(completion).get('passed'):raise ValueError('当前批次采集未完成，不能求解')
                self.background('FITTING',self.fit_head_batch);return {'message':'整批一次求解并评估留出帧'}
            raise ValueError('unknown action')


def handler(app, fake_app=None):
    apps={'physical':app}
    if fake_app is not None:
        if fake_app is app or fake_app.csrf==app.csrf:raise ValueError('runtime instances and request tokens must be independent')
        if fake_app.output.resolve()==app.output.resolve():raise ValueError('runtime outputs must be independent')
        physical_root=getattr(app,'calibration_root',None);fake_root=getattr(fake_app,'calibration_root',None)
        if physical_root is not None and fake_root is not None and Path(physical_root).resolve()==Path(fake_root).resolve():
            raise ValueError('runtime calibration roots must be independent')
        apps['fake']=fake_app
    switch_lock=threading.RLock()

    def prefixed(value,prefix):
        if isinstance(value,str) and value.startswith(('/api/','/artifacts/')):return prefix+value
        if isinstance(value,list):return [prefixed(item,prefix) for item in value]
        if isinstance(value,dict):return {key:prefixed(item,prefix) for key,item in value.items()}
        return value

    def switch_runtime(body):
        name,mode=body.get('runtime'),body.get('mode')
        if name not in apps or mode!='hardware_sync':
            raise ValueError('unified runtime only supports physical and fake hardware control')
        # Fixed lock order prevents concurrent switches/actions from changing either
        # instance between the idle check and the mode change. No owner lifecycle call.
        with switch_lock, apps['physical'].mode_lock:
            other=apps.get('fake',apps['physical'])
            with other.mode_lock:
                for instance in apps.values():
                    instance.idle()
                    if getattr(instance,'fake_restart_in_progress',False):raise ValueError('Fake 控制器正在恢复，暂不能切换模式')
                    if getattr(instance,'pipeline_running',False):raise ValueError('runtime pipeline is active')
                    if getattr(instance,'status','READY') in ('PIPELINE','RUNNING','PLANNING','CAPTURING','FITTING'):
                        raise ValueError('runtime job is active')
                    if getattr(instance,'control',None):
                        gateway=instance.control_request('state')
                        owner=gateway.get('owner') or {}
                        if not owner and gateway.get('reachable') is False and gateway.get('starting_pid') is None and gateway.get('motion_enabled') is False:
                            # Inspect only. This refreshes a takeover-preview token, never
                            # confirms takeover or starts/stops a controller.
                            inspection=instance.control_request('takeover-preview',{})
                            if inspection.get('processes')==[]:continue
                        if not isinstance(owner,dict) or not owner:raise ValueError('runtime owner state is unavailable or a controller still exists')
                    else:owner=(getattr(instance,'control_status',None) or {}).get('owner') or {}
                    if owner.get('automatic_enabled') or owner.get('automatic') or owner.get('state') in ('VERIFYING','RUNNING','SETTLING','AWAITING_CAPTURE') or (owner.get('motion') or {}).get('status') in ('VERIFYING','RUNNING','SETTLING','AWAITING_CAPTURE'):
                        raise ValueError('runtime motion is active')
                result=apps[name].set_control_mode(mode)
                for instance in apps.values():
                    instance.invalidate_reviews()
                return {**result,'runtime':name,'url':'/'+name+'/'}

    class Handler(BaseHTTPRequestHandler):
        app=apps['physical']
        prefix='/physical'
        runtime_name='physical'
        def select_runtime(self):
            parsed=urlsplit(self.path)
            pieces=parsed.path.split('/')
            name=pieces[1] if len(pieces)>1 and pieces[1] in ('physical','fake') else 'physical'
            if name not in apps:
                self.runtime_name='physical';self.app=apps['physical'];self.prefix='/physical'
                self.send(404,{'error':'runtime unavailable'});return False
            self.runtime_name=name;self.app=apps[name];self.prefix='/'+name
            if len(pieces)>1 and pieces[1] in ('physical','fake'):
                path='/'+'/'.join(pieces[2:])
                self.path=path+('?' + parsed.query if parsed.query else '')
            return True
        def runtime_context(self):
            return {'name':self.runtime_name,'prefix':self.prefix,'available':list(apps)}
        def setup(self):
            super().setup();self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        def log_message(self,*args):pass
        def send(self,code,data,kind='application/json',headers=None,prefix_json=True):
            raw=json.dumps(prefixed(data,self.prefix) if prefix_json else data,allow_nan=False).encode() if kind=='application/json' else data
            self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(raw)));self.send_header('Cache-Control','no-store')
            for key,value in (headers or {}).items():self.send_header(key,value)
            self.end_headers();self.wfile.write(raw)
        def stream(self,source):
            self.connection.settimeout(5)
            self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame');self.send_header('Cache-Control','no-store');self.end_headers()
            try:
                for raw in source:self.wfile.write(raw);self.wfile.flush()
            except (OSError,ValueError):pass
            finally:self.close_connection=True
        def camera_stream(self):
            for jpeg,meta in self.app.hardware_camera.iter_frames():
                yield b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n'
        def dsw_stream(self,view):
            request=Request(self.app.live['url'].rstrip('/')+'/stream.mjpg?view='+view,headers={'Authorization':'Bearer '+self.app.live_token,'Host':'127.0.0.1:'+str(self.app.live.get('remote_port',8118))})
            while True:
                try:
                    with self.app.http.open(request,timeout=5) as response:
                        while True:
                            raw=response.read1(65536)
                            if not raw:break
                            yield raw
                except (OSError,URLError):
                    pass
                # Keep the browser stream alive across upstream tunnel loss or idle preview timeout.
                time.sleep(.5)
        def valid_host(self):
            return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}',*(host for instance in apps.values() for host in instance.config.get('allowed_hosts',[])))
        def do_GET(self):
            if not self.select_runtime():return
            if not self.valid_host():return self.send(403,{'error':'invalid host'})
            path=urlsplit(self.path).path
            if path=='/':
                page=Path(__file__).with_suffix('.html').read_text()
                page=page.replace('/api/',self.prefix+'/api/').replace('/artifacts/',self.prefix+'/artifacts/')
                context=json.dumps(self.runtime_context()).replace('<','\\u003c')
                page=page.replace('<script>','<script>window.__WORKBENCH_RUNTIME__='+context+';</script><script>',1)
                return self.send(200,page.encode(),'text/html; charset=utf-8')
            if path=='/api/state':return self.send(200,{**self.app.state(),'runtime':self.runtime_context(),'dsw_recovery_guide':self.app.config.get('dsw_recovery_guide')})
            if path=='/api/runtime-state':return self.send(200,{**self.app.runtime_state(),'runtime':self.runtime_context()})
            if path=='/api/calibration-file':
                try:
                    record=self.app.calibration_store_record(parse_qs(urlsplit(self.path).query).get('id',[None])[0])
                    return self.send(200,record,headers={'Content-Disposition':'attachment; filename="'+record['version']+'.json"'},prefix_json=False)
                except ValueError as exc:return self.send(404,{'error':str(exc)})
            if path=='/api/preview-result':return self.send(200,self.app.preview_result or {})
            if path=='/api/control/state':
                try:return self.send(200,self.app.control_request('state'))
                except ValueError as exc:return self.send(503,{'error':str(exc)})
            if path=='/api/hardware-stream':
                if self.app.hardware_camera is None:return self.send(409,{'error':'camera not configured'})
                return self.stream(self.camera_stream())
            if path=='/api/live-stream':
                view=parse_qs(urlsplit(self.path).query).get('view',['actual'])[0]
                if not self.app.live or view not in ('actual','preview','head'):return self.send(400,{'error':'invalid stream'})
                return self.stream(self.dsw_stream(view))
            if path=='/api/hardware-overlay':
                try:
                    raw,meta=self.app.hardware_overlay_frame(parse_qs(urlsplit(self.path).query).get('mesh',['1'])[0]!='0')
                    return self.send(200,raw,'image/jpeg',{'X-Frame-Metadata':json.dumps(meta,allow_nan=False)})
                except Exception as exc:return self.send(503,{'error':str(exc)})
            if path=='/api/wrist-overlay':
                query=parse_qs(urlsplit(self.path).query);observer=query.get('camera',[''])[0]
                if observer not in ('left','right'):return self.send(400,{'error':'invalid wrist camera'})
                try:
                    raw,meta=self.app.hardware_wrist_overlay_frame(observer,query.get('mesh',['1'])[0]!='0')
                    return self.send(200,raw,'image/jpeg',{'X-Frame-Metadata':json.dumps(meta,allow_nan=False)})
                except Exception as exc:return self.send(503,{'error':str(exc)})
            if path=='/api/wrist-frame':
                side=parse_qs(urlsplit(self.path).query).get('camera',[''])[0]
                camera=self.app.wrist_cameras.get(side)
                if camera is None:return self.send(409,{'error':'腕相机未接入'})
                try:
                    raw,meta=camera.read_frame();meta.update(camera.snapshot())
                    return self.send(200,raw,'image/jpeg',{'X-Frame-Metadata':json.dumps(meta,ensure_ascii=True,allow_nan=False)})
                except Exception as exc:return self.send(503,{**camera.snapshot(),'error':str(exc)})
            if path=='/api/hardware-frame':
                if self.app.hardware_camera is None:return self.send(409,{'error':'真机相机尚未配置；仿真画面仍可使用','ready':False})
                try:
                    raw,metadata=self.app.hardware_camera.read_frame()
                    return self.send(200,raw,'image/jpeg',{'X-Frame-Metadata':json.dumps(metadata,ensure_ascii=True,allow_nan=False)})
                except Exception as exc:return self.send(503,{'error':str(exc),'ready':False})
            if path=='/api/hardware-camera-state':
                return self.send(200,self.app.hardware_camera.snapshot() if self.app.hardware_camera else {'ready':False,'error':'camera not configured'})
            if path=='/api/hardware-state':
                if self.app.hardware_reader is None:return self.send(409,{'error':'read-only hardware owner port is not configured','capabilities':{'execute_queue':False,'stop':False,'capture':False}})
                try:return self.send(200,self.app.hardware_reader.read_state())
                except Exception as exc:return self.send(503,{'error':str(exc),'capabilities':{'execute_queue':False,'stop':False,'capture':False}})
            if path=='/api/live-frame':
                try:
                    self.app.require_remote();view=parse_qs(urlsplit(self.path).query).get('view',['preview'])[0]
                    if view not in ('preview','actual','head','left','right','left-calibration','right-calibration','left-calibration-original','right-calibration-original','calibration','calibration-original'):raise ValueError('invalid view')
                    if view in ('calibration','calibration-original') and not self.app.calibration_state()['ready']:raise ValueError('calibration overlay unavailable or rendering')
                    # Display the latest completed preview while a newer target renders.
                    # Execution approval is independently tied to the target token; the
                    # 400 ms status cache must not starve frames during slider input.
                    if view in ('preview','actual','head'):
                        raw=self.app.live_frame(view)
                        tag='"'+hashlib.sha256(raw).hexdigest()[:24]+'"'
                        if self.headers.get('If-None-Match')==tag:
                            return self.send(304,b'','image/jpeg',{'ETag':tag})
                        return self.send(200,raw,'image/jpeg',{'ETag':tag})
                    query='/frame.jpg?view='+view
                    if view.startswith(('left','right')):
                        side=view.split('-')[0];overlay='-calibration' in view
                        # Both sides share one id (S3); keying this by side silently broke
                        # the overlay view after the paired change.
                        selected=self.app.wrist_selection_id()
                        remote=self.app.remote or {};received=self.app.remote_received;info=remote.get('wrist_calibration',{}).get(side,{})
                        if overlay:
                            requested=parse_qs(urlsplit(self.path).query).get('candidate_id',[None])[0]
                            if not selected or requested!=selected or info.get('id')!=selected or not info.get('ready'):raise ValueError('腕候选尚未就绪或已变化')
                            query+='&candidate_id='+selected
                        stamp=remote.get('frame_timestamp_s',{}).get(view)
                        if not isinstance(stamp,(int,float)) or not np.isfinite(stamp):raise ValueError('腕图像缺少源时间戳')
                        raw=self.app.remote_request(query,binary=True)
                        age=max(0.,float(remote['timestamp_s'])-stamp)+time.monotonic()-received
                        if age>=2 or overlay and self.app.wrist_selection_id()!=selected:raise ValueError('腕图像已过期或候选已变化')
                        return self.send(200,raw,'image/jpeg',{'X-Frame-Metadata':json.dumps({'candidate_id':selected if overlay else None,'image_age_s':age})})
                    if view in ('calibration','calibration-original'):
                        from urllib.parse import quote
                        frame_id=parse_qs(urlsplit(self.path).query).get('frame_id',[None])[0]
                        if frame_id:query+='&frame_id='+quote(frame_id,safe='')
                    return self.send(200,self.app.remote_request(query,binary=True),'image/jpeg')
                except ValueError as exc:return self.send(503,{'error':str(exc)})
            if path=='/api/diagnostics':return self.send(200,self.app.diagnostics())
            if path.startswith('/artifacts/'):
                file=(self.app.output/path.removeprefix('/artifacts/')).resolve()
                if file.is_relative_to(self.app.output) and file.suffix=='.png' and file.is_file():return self.send(200,file.read_bytes(),'image/png')
            return self.send(404,{'error':'not found'})
        def do_POST(self):
            if not self.select_runtime():return
            if not self.valid_host() or self.headers.get('X-CSRF-Token')!=self.app.csrf:return self.send(403,{'error':'invalid request token'})
            if self.headers.get('Origin') not in (None,'http://'+self.headers['Host']):return self.send(403,{'error':'invalid origin'})
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=4*1024*1024:raise ValueError('invalid request size')
                body=json.loads(self.rfile.read(size))
                if not isinstance(body,dict):raise ValueError('object required')
                path=urlsplit(self.path).path
                if path=='/api/runtime-switch':return self.send(200,switch_runtime(body))
                if path in ('/api/fake-controller-restart-preview','/api/fake-controller-restart-confirm'):
                    if fake_app is None or self.runtime_name!='fake':raise ValueError('Fake 控制器恢复仅限 Fake 入口')
                    operation=(self.app.fake_controller_restart_preview if path.endswith('preview')
                               else self.app.fake_controller_restart_confirm)
                    return self.send(200,operation(body))
                if fake_app is not None and path=='/api/control-mode':
                    return self.send(200,switch_runtime({**body,'runtime':self.runtime_name}))
                self.send(200,self.app.action(path,body))
            except (ValueError,KeyError,TypeError) as exc:self.send(400,{'error':str(exc)})
    return Handler


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True,type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--port',type=int,default=8117);p.add_argument('--preflight',action='store_true');p.add_argument('--fake-config',type=Path);p.add_argument('--fake-output',type=Path);a=p.parse_args()
    if bool(a.fake_config)!=bool(a.fake_output):p.error('--fake-config and --fake-output must be supplied together')
    config=read(a.config);fake_config=read(a.fake_config) if a.fake_config else None
    if fake_config is not None:
        if config.get('simulation_only') is not False or fake_config.get('simulation_only') is not True:
            p.error('unified runtime needs physical simulation_only=false and fake simulation_only=true')
        if a.output.resolve()==a.fake_output.resolve():p.error('runtime outputs must be independent')
        if config.get('calibration_root') and fake_config.get('calibration_root') and Path(config['calibration_root']).resolve()==Path(fake_config['calibration_root']).resolve():
            p.error('runtime calibration roots must be independent')
    app=Workbench(config,a.output)
    fake=Workbench(fake_config,a.fake_output) if fake_config is not None else None
    if fake and app.control_mode!='hardware_sync':app.set_control_mode('hardware_sync')
    if fake and fake.control_mode!='hardware_sync':fake.set_control_mode('hardware_sync')
    if a.preflight:print(json.dumps({'physical':app.diagnostics(),'fake':fake.diagnostics()} if fake else app.diagnostics(),indent=2));return
    server=ThreadingHTTPServer((app.config.get('bind_host','127.0.0.1'),a.port),handler(app,fake));print(f'http://127.0.0.1:{a.port}',flush=True);server.serve_forever()
if __name__=='__main__':main()
