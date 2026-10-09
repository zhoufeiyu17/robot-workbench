"""Workbench transport to its separate robot services; never imports the SDK."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

CAMERAS = ('base_0', 'left_wrist_0', 'right_wrist_0')


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        raise ValueError('robot service redirects are forbidden')


class RealBridge:
    def __init__(self, config):
        self.config = config
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

    def request(self, url, payload=None, *, control=False):
        if self.config.get('hardware_access_enabled', True) is False:
            raise ValueError('相机与控制权当前由标定工作台使用；数采工作台处于开发模式')
        headers = {'Accept': 'application/json'}
        if control:
            headers['Authorization'] = 'Bearer ' + self.token
        data = None if payload is None else json.dumps(payload, allow_nan=False).encode()
        if data is not None:
            headers['Content-Type'] = 'application/json'
        try:
            with self.opener.open(Request(url, data=data, headers=headers), timeout=2) as response:
                body = response.read(16 * 1024 * 1024 + 1)
                if len(body) > 16 * 1024 * 1024:
                    raise ValueError('robot service response too large')
                return body, response.headers.get('Content-Type', 'application/octet-stream')
        except HTTPError as error:
            raise ValueError('robot service rejected request: ' + error.read(2048).decode(errors='replace')) from None

    def owner(self):
        if self.config.get('hardware_access_enabled', True) is False:
            return dict(connected=False,state='CALIBRATION_OWNED',feedback_healthy=False,hardware_access_enabled=False)
        raw, _ = self.request(self.config['control_url'] + '/state', control=True)
        return json.loads(raw)

    def camera(self, name, path):
        if name not in CAMERAS or path not in ('status', 'frame.jpg', 'snapshot.zip', 'depth.png'):
            raise ValueError('unknown camera or resource')
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
        return dict(status='awaiting-physical-confirmation',mode='real_robot',preview=preview,
                    message='本轮真实轨迹已预览，等待现场确认；尚未发送运动指令')

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
        elif owner.get('mode') != 'hardware':
            blockers.append('当前 owner 不是真实硬件模式，不能作为真机采集')
        elif not owner.get('feedback_healthy'):
            blockers.append('真机关节反馈尚未健康')
        if owner.get('connected'):
            if not owner.get('stationary'):
                blockers.append('真机尚未静止，不能预览新轨迹')
            if owner.get('automatic_enabled'):
                blockers.append('真机正在执行，不能开始另一轮采集')
            if owner.get('state') not in ('READY','STOPPED_DAMPING_HOLD','MANUAL_TAKEOVER'):
                blockers.append('控制器状态尚不允许新轨迹预览')
            pedal = owner.get('pedal',{})
            if not pedal.get('input_healthy') or pedal.get('physically_down'):
                blockers.append('脚踏输入不健康或正在按下')
        capture_ready=bool(owner.get('recording_configured')) and all(c.get('ready') for c in values.values())
        try:
            self.inputs()
        except (OSError, ValueError, KeyError) as error:
            blockers.append(str(error))
        return dict(configured=True, connected=bool(owner.get('connected')), owner=owner,
                    cameras=values, collection_ready=not blockers, blockers=blockers,
                    robot_id=self.config['robot_id'], control_hz=30,capture_ready=capture_ready,
                    control_decoupled_from_rendering=True)

    def control(self, payload):
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
            fields[action] = {'action', 'token', 'confirmation', 'physical_clearance'}
        if action not in fields or set(payload) != fields[action]:
            raise ValueError('unsupported real control action or fields')
        raw, _ = self.request(self.config['control_url'] + '/operator', payload, control=True)
        return json.loads(raw)
