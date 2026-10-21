"""Read-only CPU mesh over real camera frames and same-host measured feedback.

The Workbench owns its existing camera readers and feedback history. This adapter
never opens a sensor, acquires a controller, starts a process or sends a target.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import secrets
import threading
import time

from .target_preview import PendingFrame


def _digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()


class PhysicalPreview:
    def __init__(self,workbench,*,max_delta_ms=200.,interval_s=.2):
        if workbench.config.get('environment')!='physical':
            raise ValueError('physical preview requires a physical Workbench')
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0
               for v in (max_delta_ms,interval_s)) or max_delta_ms>200.:
            raise ValueError('positive preview interval and pairing limit at most 200 ms required')
        self.workbench=workbench;self.max_delta_ms=max_delta_ms;self.interval_s=interval_s
        self.instance=secrets.token_hex(16);self.lock=threading.Lock()
        self.closed=threading.Event();self.changed=threading.Event();self.thread=None
        self.display_context=None;self.generation=0;self.revisions=dict(target=0,view=0,mesh=0)
        self.selection=dict(enabled=False,camera='head',candidate_id=None)
        self.frame=None;self.error=None

    def start(self):
        with self.lock:
            if self.closed.is_set():raise ValueError('physical preview is closed')
            if self.thread is not None:return
            self.thread=threading.Thread(target=self._loop,daemon=True,name='physical-mesh-preview')
            self.thread.start()

    def bind_context(self,display_context):
        if not isinstance(display_context,str) or not display_context:raise ValueError('display context required')
        with self.lock:
            if self.display_context==display_context:return
            self.display_context=display_context;self.generation+=1;self.revisions['mesh']+=1
            self.frame=None;self.error=None
        self.changed.set()

    @staticmethod
    def _physical(row):
        return (row.get('environment')=='hardware' and row.get('robot_source')=='physical'
                and row.get('simulation_only') is False)

    def _candidates(self,camera,identity):
        app=self.workbench
        heads={r['id']:r for r in app.calibration_results() if self._physical(r) and app.calibration_asset_matches(r)}
        wrists={r['id']:r for r in app.wrist_results() if self._physical(r) and app.calibration_asset_matches(r)}
        installed=app.installed_pair() or {}
        if identity is None:
            identity=installed.get('head_id' if camera=='head' else 'wrist_id')
        wrist=None
        if identity in wrists:
            wrist=wrists[identity];head_id=app.wrist_contract(wrist)['head_snapshot']['id']
        elif identity in heads:
            head_id=identity
            if camera!='head':
                matching=[r for r in wrists.values() if app.wrist_contract(r)['head_snapshot']['id']==head_id]
                wrist=next((r for r in matching if r['id']==installed.get('wrist_id')),None)
                if wrist is None and matching:wrist=matching[0]
        else:raise ValueError('请选择真实头部或双腕标定；当前安装组尚未接入')
        head=heads.get(head_id)
        if head is None:raise ValueError('所选双腕依赖的真实头部标定缺失')
        parameters=app.calibration_snapshot(head['parameters'])
        if parameters.get('id')!=head_id or not parameters.get('snapshot_complete'):
            raise ValueError('头部标定快照不完整或身份不一致')
        if camera!='head':
            if wrist is None:raise ValueError('所选头部标定没有关联的双腕结果')
            error=app.wrist_overlay_error(wrist)
            if error:raise ValueError(error)
            contract=app.wrist_contract(wrist)
            snapshot=app.calibration_snapshot(contract['head_snapshot'])
            if snapshot['id']!=head_id or snapshot['snapshot_sha256']!=parameters['snapshot_sha256']:
                raise ValueError('双腕依赖快照与所选头部标定不一致')
        return deepcopy(dict(head=parameters,wrist=wrist if camera!='head' else None,
            wrist_cameras=contract['cameras'] if camera!='head' else None,
            candidate_id=head_id if camera=='head' else wrist['id']))

    def mesh(self,enabled,*,camera='head',candidate_id=None):
        if type(enabled) is not bool or camera not in ('head','left','right','all'):
            raise ValueError('explicit mesh switch and camera required')
        if candidate_id is not None and (not isinstance(candidate_id,str) or not candidate_id):
            raise ValueError('valid calibration identity required')
        with self.lock:
            if self.closed.is_set():raise ValueError('physical preview is closed')
            previous=self.selection['candidate_id']
        identity=self._candidates(camera,candidate_id or previous)['candidate_id'] if enabled else candidate_id or previous
        with self.lock:
            self.selection=dict(enabled=enabled,camera=camera,candidate_id=identity)
            self.generation+=1;self.revisions['mesh']+=1;self.frame=None;self.error=None
            revision=self.revisions['mesh']
        self.changed.set()
        return dict(enabled=enabled,camera=camera,candidate_id=identity,client_revision=revision,
            display_only=True,warning='开启 mesh 叠加会增加 CPU 负载并降低预览刷新速度')

    def target_pose(self,*args,**kwargs):
        raise ValueError('当前真机显示仅支持实测相机 mesh；目标姿态预览尚未配置')

    def overview(self,*args,**kwargs):
        raise ValueError('当前真机显示仅支持实测相机 mesh；整机仿真视角尚未配置')

    def _owner(self):
        app=self.workbench
        if not app.hardware_connected():raise ValueError('等待新鲜真机关节反馈，未启动任何控制器')
        owner=(app.hardware_feedback or {}).get('owner') or {}
        if type(owner.get('pid')) is not int or owner['pid']<=0:raise ValueError('真机反馈缺少 Owner 身份')
        return tuple(owner.get(k) for k in ('pid','command_epoch','runtime_nonce'))

    def _render_camera(self,role,chosen,owner):
        import cv2
        import numpy as np
        from arx_env.joint_head_isaac_fit import rigid
        from arx_env.joint_head_workbench_overlay import pair_real_feedback,real_mesh_overlay
        app=self.workbench
        reader=app.hardware_camera if role=='head' else app.wrist_cameras.get(role)
        if reader is None:raise ValueError('当前相机尚未接入：'+role)
        path=app.config['camera'] if role=='head' else app.config['hardware_wrist_camera_files'][role]
        camera=json.loads(Path(path).read_text())
        if role!='head' and chosen['wrist_cameras'][role]!=camera:
            raise ValueError('腕相机内参与所选标定契约不一致：'+role)
        raw,source=reader.read_calibration_frame()
        if (source.get('environment') not in (None,'physical') or source.get('simulation_only') is True
                or source.get('timestamp_kind')=='simulation_step' or source.get('source_clock_id') is not None):
            raise ValueError('真机 mesh 禁止使用仿真图像')
        observed=source.get('intrinsics') or {}
        expected_serial=app.config['hardware_camera_serials'][role]
        if (source.get('serial')!=expected_serial or camera.get('serial')!=expected_serial
                or observed.get('serial',expected_serial)!=expected_serial):
            raise ValueError('实际相机与标定序列号不匹配：'+role)
        def valid_coefficients(value):
            return (isinstance(value,list) and len(value)==5
                    and all(type(v) in (int,float) and math.isfinite(v) for v in value))
        rectified_differs=observed.get('rectified')!=camera.get('rectified')
        zero_distortion_equivalent=(rectified_differs
            and all(type(k.get('rectified')) is bool and valid_coefficients(k.get('coefficients'))
                    and not any(k['coefficients'])
                    for k in (observed,camera)))
        # Rectification changes image coordinates, not the optical-frame extrinsic.
        # Project onto untouched raw pixels using that frame's SDK distortion.
        rectified_to_raw=(camera.get('rectified') is True and observed.get('rectified') is False
            and valid_coefficients(camera.get('coefficients')) and not any(camera['coefficients'])
            and valid_coefficients(observed.get('coefficients'))
            and observed.get('distortion_model') in ('distortion.inverse_brown_conrady',
                'distortion.brown_conrady','distortion.modified_brown_conrady')
            and source.get('raw') is True and source.get('annotated') is False and source.get('image_domain')=='raw')
        if (any(observed.get(k)!=camera.get(k) for k in
                ('width','height','fx','fy','cx','cy','distortion_model'))
                or (observed.get('coefficients')!=camera.get('coefficients') or rectified_differs)
                and not (zero_distortion_equivalent or rectified_to_raw)):
            raise ValueError('实际相机内参与冻结标定内参不一致：'+role)
        projection_camera=deepcopy(observed if rectified_to_raw else camera)
        stamp=source.get('host_receive_ns')
        if isinstance(stamp,str) and stamp.isdecimal():stamp=int(stamp)
        if type(stamp) is not int or stamp<=0:raise ValueError('相机缺少真实主机接收时间')
        age=(time.time_ns()-stamp)/1e9
        if not 0<=age<2:raise ValueError('真机源图像已过期或主机时钟不一致')
        history=[row for row in app.feedback_history_snapshot()
                 if tuple((row.get('owner') or {}).get(k) for k in ('pid','command_epoch','runtime_nonce'))==owner]
        sample,timing=pair_real_feedback(dict(frame_timestamp_ns=stamp),history,owner[0],self.max_delta_ms)
        candidate=deepcopy(chosen['head'])
        if role!='head':
            q=np.asarray(sample['qpos'],float)
            if q.shape!=(14,) or not np.isfinite(q).all():raise ValueError('真机关节无效')
            offsets=np.concatenate([candidate['joint_offsets_rad'][side] for side in ('left','right')])
            pose=app.geometry.frames(np.r_[q[:6],q[7:13]]+offsets)[role]['link6']@rigid(chosen['wrist'][role]['T_link6_camera'])
            candidate.update(id=chosen['wrist']['id'],head_result_id=chosen['head']['id'],camera_T_world_optical=pose.tolist())
        if not isinstance(raw,bytes) or len(raw)>8*1024*1024:raise ValueError('bounded real camera JPEG required')
        bgr=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        if bgr is None:raise ValueError('相机 JPEG 无法解码')
        began=time.monotonic();image=real_mesh_overlay(app.geometry,bgr,sample['qpos'],candidate,projection_camera)
        return image,dict(timing,camera=role,candidate_id=candidate['id'],head_candidate_id=chosen['head']['id'],
            source_wall_ns=stamp,source_frame_number=source.get('frame_number'),source_jpeg_sha256=hashlib.sha256(raw).hexdigest(),
            source_camera_metadata=deepcopy(source),calibration_intrinsics=deepcopy(camera),
            projection_intrinsics=projection_camera,
            projection_domain='actual_raw_sdk_intrinsics_from_rectified_calibration' if rectified_to_raw else 'frozen_calibration_intrinsics',
            intrinsics_equivalence=('identical_zero_distortion_with_different_rectified_flag' if zero_distortion_equivalent
                else 'different_distortion_domains' if rectified_to_raw else 'identical'),
            frame_id=_digest([role,source.get('frame_number'),stamp,candidate,sample]),
            source_age_s=age,render_ms=(time.monotonic()-began)*1000,owner_pid=owner[0],command_epoch=owner[1],
            qpos=sample['qpos'],environment='physical',clock_domain='host_wall',source_clock_id=None,
            synchronized_capture=False,display_only=True,measurement_qualified=False)

    def _render(self,selection,chosen,owner):
        import cv2
        import numpy as np
        roles=('head','left','right') if selection['camera']=='all' else (selection['camera'],)
        images=[];records=[]
        for role in roles:
            if self.closed.is_set():raise ValueError('physical preview closed')
            image,metadata=self._render_camera(role,chosen,owner);images.append(image);records.append(metadata)
        image=images[0]
        if len(images)>1:
            height=max(v.shape[0] for v in images);width=max(v.shape[1] for v in images)
            image=np.full((height+24,width*3,3),24,np.uint8)
            for index,(label,part) in enumerate(zip(('Head','Left wrist','Right wrist'),images)):
                image[24:24+part.shape[0],index*width:index*width+part.shape[1]]=part
                cv2.putText(image,label,(index*width+8,17),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
        ok,encoded=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,90])
        if not ok:raise ValueError('CPU mesh JPEG 编码失败')
        age=(time.time_ns()-min(m['source_wall_ns'] for m in records))/1e9
        if not 0<=age<2:raise ValueError('mesh 渲染完成时源帧已过期')
        metadata=dict(records[0],camera=selection['camera'],candidate_id=chosen['candidate_id'],
            source_wall_ns=min(m['source_wall_ns'] for m in records),observed_age_s=age,
            render_ms=sum(m['render_ms'] for m in records),frame_id=_digest([m['frame_id'] for m in records]),
            cameras=records,wrist_candidate_id=(chosen['wrist'] or {}).get('id'),source='physical_camera_cpu_mesh')
        return encoded.tobytes(),metadata

    def _loop(self):
        while not self.closed.is_set():
            self.changed.clear();began=time.monotonic()
            with self.lock:selection=dict(self.selection);generation=self.generation;revision=self.revisions['mesh']
            if selection['enabled']:
                try:
                    owner=self._owner();chosen=self._candidates(selection['camera'],selection['candidate_id'])
                    raw,metadata=self._render(selection,chosen,owner)
                    if owner!=self._owner() or chosen!=self._candidates(selection['camera'],selection['candidate_id']):
                        raise ValueError('真机连接或标定参数在渲染期间变化')
                    with self.lock:
                        if generation==self.generation and not self.closed.is_set():
                            self.frame=(raw,dict(metadata,revision=revision,renderer_instance_id=self.instance),time.monotonic(),owner)
                            self.error=None
                except Exception as error:
                    with self.lock:
                        if generation==self.generation:self.error=str(error);self.frame=None
            self.changed.wait(max(.01,self.interval_s-(time.monotonic()-began)))

    def _current(self):
        frame=self.frame
        if frame and self.selection['enabled'] and frame[1]['observed_age_s']+time.monotonic()-frame[2]<2:
            try:
                if frame[3]==self._owner():return frame
            except ValueError:pass
        return None

    def state(self):
        with self.lock:
            frame=self._current();running=bool(self.thread and self.thread.is_alive() and not self.closed.is_set())
            return dict(renderer_instance_id=self.instance,connected=running,status='ready' if running else 'stopped',
                display_context=self.display_context,revisions=dict(self.revisions),mesh=dict(self.selection),
                frames={'mesh':deepcopy(frame[1])} if frame else {},error=None,mesh_error=self.error,
                capabilities=dict(mesh=True,target=False,overview=False),
                unsupported_reason='仅提供真实相机与实测关节的 CPU mesh 叠加；目标姿态与整机仿真视角尚未配置',
                recovery=dict(allowed=False,message='相机与关节反馈恢复后会自动刷新 mesh；本显示服务不重连控制器'),
                display_only=True,control_authority=False)

    def media(self,view,*args,**kwargs):
        if view!='mesh':raise ValueError('当前真机显示仅支持 mesh；目标姿态与整机仿真视角尚未配置')
        with self.lock:
            frame=self._current()
            if frame is None:raise PendingFrame(self.error or '等待当前真实图像与关节反馈配对的 mesh')
            return frame[0],deepcopy(frame[1])

    def close(self):
        self.closed.set();self.changed.set()
        with self.lock:self.generation+=1;self.frame=None
        if self.thread:self.thread.join(timeout=.1)
