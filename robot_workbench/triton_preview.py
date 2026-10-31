"""CPU-only proxy to the separately owned conversion / inference-preview service.

This module cannot construct an SDK, load a model, or publish robot commands.
"""
import json
import math
import os
from pathlib import Path
import re
import stat
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import Request, HTTPRedirectHandler, build_opener, ProxyHandler

IDENTIFIER = re.compile(r'[A-Za-z0-9_-]{1,128}\Z')
SHA256 = re.compile(r'[a-f0-9]{64}\Z')
LIMIT = 2 * 1024 * 1024


def identifier(value):
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        raise ValueError('明确的已登记任务或模型ID必需')
    return value


def validate_prediction_result(result):
    p=result.get('prediction') or {};rows=p.get('values')
    if (result.get('hardware_control') is not False or result.get('command_publication') is not False
            or p.get('shape') != [60,14] or p.get('joint_units') != 'rad' or p.get('gripper_units') != 'm'
            or not isinstance(rows,list) or len(rows)!=60 or any(not isinstance(row,list) or len(row)!=14
                or any(type(x) not in (int,float) or not math.isfinite(x) for x in row) for row in rows)):
        raise ValueError('Triton输出预览的形状、单位或不控机契约无效')


def validate_config(config):
    if not isinstance(config, dict) or set(config) != {'endpoint', 'token_file'}:
        raise ValueError('triton_preview requires endpoint and private token_file')
    u = urlsplit(config['endpoint'])
    if (u.scheme != 'http' or u.hostname != '127.0.0.1' or not u.port
            or u.username or u.password or u.path not in ('', '/') or u.query or u.fragment):
        raise ValueError('Triton preview endpoint must be an explicit local service')
    p = Path(config['token_file'])
    if not p.is_absolute() or '..' in p.parts:
        raise ValueError('absolute private Triton token file required')
    return config


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


class TritonPreviewClient:
    def __init__(self, config):
        self.config = dict(validate_config(config))
        self.http = build_opener(ProxyHandler({}), NoRedirect())

    def request(self, path, body=None):
        token_path = Path(self.config['token_file'])
        info = token_path.lstat(); mode = info.st_mode
        if not stat.S_ISREG(mode) or mode & 0o077 or info.st_uid != os.getuid():
            raise ValueError('Triton凭据文件权限必须为当前用户私有')
        token = token_path.read_text().strip()
        if not token or len(token) > 4096 or any(c.isspace() for c in token):
            raise ValueError('Triton服务凭据无效')
        req = Request(self.config['endpoint'].rstrip('/') + path,
                      None if body is None else json.dumps(body, allow_nan=False).encode(),
                      headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
        try:
            response = self.http.open(req, timeout=5)
        except HTTPError as error:
            # Never retry a submitted job, including an unknown transport outcome.
            if error.code >= 500:
                raise ConnectionError('Triton服务暂不可用；提交过的请求须按request_id核验，不能自动重试') from None
            try:
                value = json.loads(error.read(LIMIT + 1))
                reason = value.get('error', {})
                message = reason.get('message', reason.get('code', '请求未通过')) if isinstance(reason, dict) else str(reason)
            except (ValueError, AttributeError):
                message = 'Triton服务拒绝请求'
            raise ValueError(str(message)[:512]) from None
        with response:
            raw = response.read(LIMIT + 1)
        if len(raw) > LIMIT:
            raise ValueError('Triton回执超过大小上限')
        try:value = json.loads(raw, parse_constant=lambda _: (_ for _ in ()).throw(ValueError('非有限数值回执')))
        except ValueError:
            if body is not None:raise ConnectionError('Triton提交回执无效，请按request_id核验') from None
            raise
        if not isinstance(value, dict):
            raise ValueError('Triton对象回执必需')
        return value

    def catalog(self):
        value = self.request('/v1/catalog')
        for key in ('sources', 'models'):
            if not isinstance(value.get(key), list) or len(value[key]) > 500:
                raise ValueError('Triton目录格式无效')
            for row in value[key]:
                identifier(row['id'])
                if not isinstance(row.get('display_name'), str) or not isinstance(row.get('status'), str):
                    raise ValueError('Triton目录身份缺失')
                if key == 'models':
                    row['selectable'] = (row.get('selectable') is True and row['status'] == 'READY'
                                         and bool(SHA256.fullmatch(str(row.get('deployment_sha256', '')))))
        return {**value, 'configured': True, 'available': True, 'hardware_control': False}

    def start(self, action, body):
        field = {'conversions': 'source_id', 'infer': 'model_id'}.get(action)
        if field is None or set(body) != {field, 'request_id'}:
            raise ValueError('Triton操作只接受登记ID与请求身份')
        identifier(body[field]); identifier(body['request_id'])
        catalog = self.catalog();rows = catalog['sources' if action == 'conversions' else 'models']
        selected = next((x for x in rows if x['id'] == body[field]), None)
        known = any(x.get('request_id') == body['request_id'] for x in catalog.get('recent_jobs', []))
        if not known and (selected is None or selected.get('selectable') is not True):
            raise ValueError('请选择可转换来源' if action == 'conversions' else '仅通过验证的READY模型允许推理')
        value = self.request('/v1/' + action, body)
        try:identifier(value.get('job_id'))
        except ValueError:raise ConnectionError('Triton任务回执缺少有效身份，请按request_id核验') from None
        return value

    def job(self, job_id):
        value = self.request('/v1/jobs/' + identifier(job_id))
        if value.get('job_id') != job_id:
            raise ValueError('Triton任务身份不匹配')
        result = value.get('result') or {}
        prediction = result.get('prediction')
        if prediction is not None:
            validate_prediction_result(result)
        return value
