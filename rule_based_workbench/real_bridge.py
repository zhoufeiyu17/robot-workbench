"""Workbench transport to its separate robot services; never imports the SDK."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import secrets
import threading
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener
from real_contract import namespace, owner_matches_environment

CAMERAS = ('base_0', 'left_wrist_0', 'right_wrist_0')


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        raise ValueError('robot service redirects are forbidden')


class RealBridge:
    def __init__(self, config):
        self.config = config
        namespace(config.get('environment', 'physical'))
        for url in [config['control_url'], *[v['url'] for v in config['cameras'].values()]]:
            parsed = urlsplit(url)
            if (parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost')
                    or parsed.username or parsed.password or parsed.path not in ('', '/')
                    or parsed.query or parsed.fragment):
                raise ValueError('real services must use explicit loopback SSH forwards')
        if set(config['cameras']) != set(CAMERAS):
            raise ValueError('all three camera identities are required')
        self.token = Path(config['token_file']).read_text().strip()
        if len(self.token) < 32:
            raise ValueError('control token must contain at least 32 characters')
        self.opener = build_opener(ProxyHandler({}), NoRedirect())
        self.ros_camera=self.ros_owner=None;self.ros_lock=threading.RLock()
        if config.get('data_plane')is not None:
            from robot_workbench.ros_data import plane_config
            plane_config(config['data_plane'])
            if not config.get('control_owner_descriptor'):raise ValueError('ROS input requires the registered control Owner')
            if any(value.get('recording_directory')for value in config['cameras'].values()):
                raise ValueError('ROS input cannot use a parallel camera recording directory')

    def _owner_client(self):
        with self.ros_lock:
            if self.ros_owner is None:
                from robot_workbench.control_owner import ControlRuntimeClient
                client=ControlRuntimeClient(self.config['control_owner_descriptor'])
                if client.device_id!=self.config['robot_id']or client.environment!=self.environment:
                    client.detach();raise ValueError('ROS Owner belongs to another device/environment')
                plane=(client.state.get('control_owner')or{}).get('data_plane')
                if client.state.get('connected') and plane!=self.config['data_plane']:
                    client.detach();raise ValueError('ROS Owner data-plane binding differs')
                self.ros_owner=client
            return self.ros_owner

    def _camera_view(self):
        with self.ros_lock:
            if self.ros_camera is None:
                from robot_workbench.ros_data import RosNode,SourceDirectory
                from robot_workbench.ros_inputs import RosCameraView
                plane=self.config['data_plane']
                probe=RosNode('collection_bind_'+secrets.token_hex(6),namespace=plane['namespace'],domain_id=plane['domain_id'],localhost_only=False)
                try:
                    rows=SourceDirectory(probe,self.config['robot_id']).resolve({name:dict(kind='image')for name in CAMERAS})
                    intrinsics={row['stream_id']:row['metadata'].get('intrinsics',row['metadata'])for row in rows}
                    for name,value in intrinsics.items():
                        if value.get('serial')!=self.config['cameras'][name]['serial'] or not all(k in value for k in ('width','height')):
                            raise ValueError('original ROS camera intrinsics/serial differ from collection binding')
                finally:probe.close()
                self.ros_camera=RosCameraView(plane,self.config['robot_id'],intrinsics,self.environment)
            return self.ros_camera

    def close(self):
        with self.ros_lock:
            if self.ros_camera is not None:self.ros_camera.close();self.ros_camera=None
            if self.ros_owner is not None:self.ros_owner.detach();self.ros_owner=None

    @property
    def environment(self):
        return namespace(self.config.get('environment','physical'))['environment']

    def request(self, url, payload=None, *, control=False):
        if self.config.get('hardware_access_enabled', True) is False:
            raise ValueError('相机与控制权当前由标定工作台使用；数采工作台处于开发模式')
        headers = {'Accept': 'application/json'}
        if control:
            headers['Authorization'] = 'Bearer ' + self.token
        data = None if payload is None else json.dumps(payload, allow_nan=False).encode()
        if data is not None:
            headers['Content-Type'] = 'application/json'
        action = payload.get('action') if control and payload is not None else None
        timeout = 15 if action == 'preview_frozen' else 2
        request = Request(url, data=data, headers=headers)
        try:
            with self.opener.open(request, timeout=timeout) as response:
                body = response.read(16 * 1024 * 1024 + 1)
                if len(body) > 16 * 1024 * 1024:
                    raise ValueError('robot service response too large')
                return body, response.headers.get('Content-Type', 'application/octet-stream')
        except HTTPError as error:
            raise ValueError('robot service rejected request: ' + error.read(2048).decode(errors='replace')) from None
        except (TimeoutError, URLError) as error:
            if isinstance(error, URLError) and not isinstance(error.reason, TimeoutError):
                raise
            raise TimeoutError(f'robot service timed out: {request.get_method()} {urlsplit(url).path} '
                               f'action={action or "none"} budget={timeout}s') from None

    def owner(self):
        if self.config.get('hardware_access_enabled', True) is False:
            return dict(connected=False,state='CALIBRATION_OWNED',feedback_healthy=False,hardware_access_enabled=False)
        if self.config.get('data_plane')is not None:
            value=self._owner_client().refresh('collection')
            if value.get('connected')and (value.get('control_owner')or{}).get('data_plane')!=self.config['data_plane']:
                raise ValueError('ROS Owner data-plane binding differs')
            return value
        raw, _ = self.request(self.config['control_url'] + '/state', control=True)
        return json.loads(raw)

    def camera(self, name, path):
        if name not in CAMERAS or path not in ('status', 'frame.jpg', 'snapshot.zip', 'depth.png'):
            raise ValueError('unknown camera or resource')
        if self.config.get('data_plane')is not None:
            if self.config.get('hardware_access_enabled',True)is False:raise ValueError('camera access is disabled')
            view=self._camera_view();formats={'status':(),'frame.jpg':('jpeg',),'depth.png':('depth_png',),
                'snapshot.zip':('color_png','depth_png')}[path]
            frame=view.frames[name].snapshot(formats=formats)
            if path=='status':return json.dumps(frame['status']).encode(),'application/json'
            if not frame['status'].get('ready'):raise ValueError(frame['status'].get('error')or'original ROS camera is not ready')
            if path=='snapshot.zip':
                from real_rgbd import snapshot_bundle
                return snapshot_bundle(frame['status'],frame['color_png'],frame['depth_png']),'application/zip'
            return (frame['jpeg'],'image/jpeg')if path=='frame.jpg'else(frame['depth_png'],'image/png')
        return self.request(self.config['cameras'][name]['url'] + '/' + path)

    def camera_stream(self, name):
        if self.config.get('hardware_access_enabled', True) is False:
            raise ValueError('相机与控制权当前由标定工作台使用；数采工作台处于开发模式')
        if name not in CAMERAS:
            raise ValueError('unknown camera')
        return self.opener.open(self.config['cameras'][name]['url'] + '/stream.mjpg', timeout=5)

    def inputs(self):
        from real_workflow import load
        data = load(self.config)
        if data['blockers'] or not data['source'] or not data['plan']:
            raise ValueError('；'.join(data['blockers']) or '真实采集产物尚未就绪')
        return data['source'], data['plan']

    def workflow_status(self, owner=None):
        from real_workflow import summary
        return summary(self.config, self.owner() if owner is None else owner)

    def playback_status(self):
        from real_workflow import playback
        return playback(self.config)[0]

    def playback_path(self, name):
        from real_workflow import playback, FILES
        if name not in FILES:
            return None
        return playback(self.config)[1].get(name)

    def live_preview(self, side, camera='base_0'):
        from real_workflow import live_preview
        return live_preview(self.config,side,camera)

    def image_evidence(self, name):
        from real_workflow import image_evidence
        return image_evidence(self.config,name)

    def prepare_collection(self):
        """Load this physical run and issue a preview only; operator confirms later."""
        status = self.status()
        if not status['collection_ready']:
            raise ValueError('；'.join(status['blockers']))
        preview = self.control({'action':'preview_frozen'})
        spec = namespace(self.environment)
        return dict(status='awaiting-physical-confirmation',mode=spec['mode'],environment=self.environment,preview=preview,
                    message=spec['label']+'轨迹已预览，等待确认；尚未发送运动指令')

    def status(self):
        if self.config.get('hardware_access_enabled', True) is False:
            return dict(configured=True,connected=False,owner=self.owner(),cameras={c:dict(ready=False) for c in CAMERAS},
                        collection_ready=False,blockers=['相机与控制权当前由标定工作台使用；数采工作台处于开发模式'],
                        robot_id=self.config['robot_id'],control_hz=30,control_decoupled_from_rendering=True,hardware_access_enabled=False)
        def read(name):
            try:
                value = self.owner() if name == 'owner' else json.loads(self.camera(name, 'status')[0])
                if name != 'owner' and value.get('serial') != self.config['cameras'][name]['serial']:
                    return {'ready': False, 'error': 'camera serial mismatch'}
                return value
            except (OSError, ValueError) as error:
                return {'connected': False, 'ready': False, 'error': str(error)}
        with ThreadPoolExecutor(max_workers=4) as pool:
            values = dict(zip(('owner', *CAMERAS), pool.map(read, ('owner', *CAMERAS))))
        owner = values.pop('owner')
        blockers = []
        if not owner.get('connected'):
            blockers.append('尚未申请真机控制权；连接前请支撑双臂')
        elif not owner_matches_environment(owner, self.environment):
            blockers.append('当前 owner 与工作台 '+self.environment+' 环境不一致')
        elif not owner.get('feedback_healthy'):
            blockers.append('真机关节反馈尚未健康')
        if owner.get('feedback_source_error'):
            blockers.append('ROS反馈尚不可用：'+str(owner['feedback_source_error']))
        if owner.get('connected'):
            if owner.get('holder', 'collection') != 'collection':
                blockers.append('请先交接控制权限：当前由 '+str(owner.get('holder'))+' 持有')
            permission=(owner.get('allowed_operations') or {}).get('preview')
            if permission is not None and permission.get('allowed') is not True:
                blockers.append(permission.get('reason') or '当前控制服务不允许新轨迹预览')
            if not owner.get('stationary'):
                blockers.append('真机尚未静止，不能预览新轨迹')
            if owner.get('automatic_enabled'):
                blockers.append('真机正在执行，不能开始另一轮采集')
            if owner.get('state') not in ('READY','STOPPED_DAMPING_HOLD','MANUAL_TAKEOVER'):
                blockers.append('控制器状态尚不允许新轨迹预览')
            pedal = owner.get('pedal',{})
            if not pedal.get('input_healthy') or pedal.get('physically_down'):
                blockers.append('脚踏输入不健康或正在按下')
            age = owner.get('sample_age_s')
            if age is not None and (type(age) not in (int,float) or not 0 <= age <= .1):
                blockers.append('设备反馈已过期')
        capture_ready=bool(owner.get('recording_configured')) and all(c.get('ready') for c in values.values())
        if self.config.get('data_plane')is not None and any(c.get('ready')and c.get('rgbd_paired')is not True for c in values.values()):
            capture_ready=False;blockers.append('规则数采需要已绑定的原始RGB-D；当前来源缺少对齐深度')
        try:
            self.inputs()
        except (OSError, ValueError, KeyError) as error:
            blockers.append(str(error))
        return dict(configured=True, connected=bool(owner.get('connected')), owner=owner,
                    environment=self.environment,environment_label=namespace(self.environment)['label'],
                    cameras=values, collection_ready=not blockers, blockers=blockers,
                    robot_id=self.config['robot_id'], control_hz=30,capture_ready=capture_ready,
                    control_decoupled_from_rendering=True)

    def control(self, payload, *, cancel=None, before_confirm=None):
        if self.config.get('hardware_access_enabled', True) is False:
            raise ValueError('相机与控制权当前由标定工作台使用；数采工作台处于开发模式')
        action = payload.get('action')
        fields = {
            'acquire': {'action', 'confirmation'}, 'stop': {'action'},
            'release': {'action', 'confirmation'},
            'recover': {'action', 'confirmation', 'owner_pid', 'command_epoch'},
        }
        if action == 'preview_frozen':
            if set(payload) != {'action'}:
                raise ValueError('preview inputs must come from the configured real acquisition')
            source, plan = self.inputs()
            payload = {'action': action, 'source': source, 'plan': plan}
            fields[action] = set(payload)
        elif action == 'confirm':
            self.inputs()
            if cancel is not None and cancel():raise InterruptedError('执行确认前已中断')
            if before_confirm is not None:before_confirm()
            fields[action] = {'action', 'token', 'confirmation', 'physical_clearance'}
        if action not in fields or set(payload) != fields[action]:
            raise ValueError('unsupported real control action or fields')
        if action=='confirm' and cancel is not None and cancel():raise InterruptedError('执行确认前已中断')
        raw, _ = self.request(self.config['control_url'] + '/operator', payload, control=True)
        if action=='acquire'and self.ros_owner is not None:
            self.ros_owner.detach();self.ros_owner=None
        return json.loads(raw)
