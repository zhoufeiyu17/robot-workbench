"""Lightweight Base566 checkpoint discovery; never loads or reads model weights.

The deployment owns the catalog root and family template. Checkpoints contribute
only model/metadata hashes and training identity, never runtime paths or code.
Core validate_bundle(verify_files=True) remains the load-time content gate.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
import stat

from . import inference_model as model


SMALL_FILE_LIMIT = 16 * 1024**2
_SHA = re.compile(r'[0-9a-f]{64}')
_STAGING = re.compile(r'(?:^|[._-])(?:partial|part|incomplete|download|tmp)(?:$|[._-])')


def _small(path):
    path = Path(path)
    if not path.is_file() or path.stat().st_size > SMALL_FILE_LIMIT:
        raise ValueError('missing or oversized small catalog asset: ' + path.name)
    with path.open('rb') as stream:
        data = stream.read(SMALL_FILE_LIMIT + 1)
    if len(data) > SMALL_FILE_LIMIT:
        raise ValueError('catalog metadata grew beyond its size limit: ' + path.name)
    return data


def _json(data):
    value = json.loads(data)
    if not isinstance(value, dict):
        raise ValueError('catalog JSON must be an object')
    return value


def _family(template):
    template = deepcopy(template) if isinstance(template, dict) else _json(_small(template))
    if template.get('schema') != 'workbench.base566.v1':
        raise ValueError('trusted Base566 family template required')
    template = model.validate_bundle(template, verify_files=False)
    size = template['artifacts']['checkpoint'].get('size_bytes')
    if type(size) is not int or size <= 0:
        raise ValueError('family template requires approved checkpoint size_bytes; standard complete.json has no size')
    for name, entry in template['artifacts'].items():
        if name in ('checkpoint', 'metadata'):
            continue
        path = Path(entry['path'])
        if not path.is_file():
            return template, None, ('missing_assets', '缺少家族配套：' + name)
        if hashlib.sha256(_small(path)).hexdigest() != entry['sha256']:
            return template, None, ('incompatible', '家族配套内容校验不符：' + name)
    if not Path(template['python']).is_file() or not Path(template['model_source']).is_dir():
        return template, None, ('missing_assets', '缺少已批准的 Python 或模型源码目录')
    source_root = Path(template['model_source']).resolve()
    sources = _json(_small(template['artifacts']['source_manifest']['path']))
    for relative, pin in sources.items():
        path = Path(relative)
        if path.is_absolute() or '..' in path.parts:
            return template, None, ('incompatible', '家族源码清单含越界路径')
        target = (source_root / path).resolve()
        if not target.is_relative_to(source_root):
            return template, None, ('incompatible', '家族源码路径越界')
        if not target.is_file() or target.stat().st_size != pin.get('size_bytes'):
            return template, None, ('missing_assets', '缺少或未完成家族源码：' + relative)
    stats = _json(_small(template['artifacts']['stats']['path']))
    return template, stats, None


def _entry(directory, root, template, stats, family_error):
    entry = dict(directory_name=directory.name, label=directory.name, model_id=None,
                 epoch=None, step=None, status='missing_assets', selectable=False, reason='缺少 checkpoint 文件')
    def unavailable(status, reason):
        entry.update(status=status, reason=reason, selectable=False)
        return entry
    if directory.is_symlink() or directory.resolve().parent != root:
        return unavailable('incompatible', 'checkpoint 目录不能是符号链接或越界路径')
    if _STAGING.search(directory.name) or any(_STAGING.search(path.name) for path in directory.iterdir()):
        return unavailable('downloading', '存在下载中或未完成文件')
    paths = {name: directory / name for name in ('model.pt', 'metadata.json', 'complete.json')}
    for name, path in paths.items():
        if path.is_symlink() or path.exists() and not stat.S_ISREG(path.lstat().st_mode):
            return unavailable('incompatible', 'checkpoint 必须是普通文件：' + name)
    if not paths['metadata.json'].exists():
        return unavailable('missing_assets', '缺少 metadata.json，不能确认模型与归一化参数')
    metadata_bytes = _small(paths['metadata.json'])
    metadata = _json(metadata_bytes)
    epoch, step = metadata.get('epoch'), metadata.get('step')
    if type(epoch) is not int or epoch < 0 or type(step) is not int or step <= 0:
        return unavailable('incompatible', 'metadata 缺少有效 epoch / step')
    model_id = f'base566-epoch{epoch:03d}-step{step}'
    entry.update(model_id=model_id, epoch=epoch, step=step, label=f'Base566 · epoch {epoch:03d} / step {step}')
    if not paths['complete.json'].exists():
        return unavailable('downloading', '等待 complete.json 完整性回执')
    complete_bytes = _small(paths['complete.json'])
    complete = _json(complete_bytes)
    if complete.get('complete') is not True:
        return unavailable('downloading', 'checkpoint 尚未标记完成')
    expected = dict(mode='train', epoch=epoch, step=step, world_size=16,
                    identity_sha256=model.BASE566_FAMILY_ID,
                    fixed_input_sha256=metadata.get('fixed_input_sha256'))
    if (any(complete.get(k) != value for k, value in expected.items())
            or not _SHA.fullmatch(str(expected['fixed_input_sha256']))):
        return unavailable('incompatible', 'complete 与 metadata / 固定家族身份不一致')
    declared = complete.get('files')
    if not isinstance(declared, dict) or any(not isinstance(name, str) or Path(name).is_absolute()
            or '..' in Path(name).parts for name in declared):
        return unavailable('incompatible', 'complete 文件清单包含无效或越界路径')
    for name, key in (('model.pt', 'model_sha256'), ('metadata.json', 'metadata_sha256')):
        digest = complete.get(key)
        if not isinstance(digest, str) or not _SHA.fullmatch(digest) or complete.get('files', {}).get(name) != digest:
            return unavailable('incompatible', 'complete 缺少一致的文件 SHA256：' + name)
    metadata_sha = hashlib.sha256(metadata_bytes).hexdigest()
    if metadata_sha != complete['metadata_sha256']:
        return unavailable('incompatible', 'metadata.json 内容与 complete SHA256 不一致')
    if family_error:
        return unavailable(*family_error)
    bundle = deepcopy(template)
    bundle.update(model_id=model_id, epoch=epoch, step=step)
    bundle['artifacts']['checkpoint'] = dict(path=str(paths['model.pt']), sha256=complete['model_sha256'],
        size_bytes=template['artifacts']['checkpoint']['size_bytes'])
    bundle['artifacts']['metadata'] = dict(path=str(paths['metadata.json']), sha256=metadata_sha, size_bytes=len(metadata_bytes))
    model.validate_base566_metadata(metadata, bundle, stats)
    model.validate_bundle(bundle, verify_files=False)
    if not paths['model.pt'].exists():
        return unavailable('downloading', '等待 model.pt 下载')
    before = paths['model.pt'].stat()
    actual = before.st_size
    expected_size = bundle['artifacts']['checkpoint']['size_bytes']
    entry.update(model_size_bytes=actual, expected_model_size_bytes=expected_size)
    if actual != expected_size:
        return unavailable('downloading' if actual < expected_size else 'incompatible',
                           f'model.pt 大小不符：{actual} / {expected_size} bytes')
    # Repeat cheap identities after the scan; a concurrently replaced download
    # is not selectable. The core rehashes the entire model at actual load time.
    after = paths['model.pt'].stat()
    if (paths['model.pt'].is_symlink() or directory.is_symlink()
            or any(getattr(before, key) != getattr(after, key) for key in ('st_dev','st_ino','st_size','st_mtime_ns','st_ctime_ns'))
            or _small(paths['metadata.json']) != metadata_bytes or _small(paths['complete.json']) != complete_bytes):
        return unavailable('downloading', '扫描期间 checkpoint 发生变化，请刷新')
    entry.update(status='ready', selectable=True, reason='可选择；加载时校验完整权重 SHA256',
        derived_manifest=bundle, checkpoint_sha256=complete['model_sha256'], metadata_sha256=metadata_sha,
        complete_sha256=hashlib.sha256(complete_bytes).hexdigest(), family_identity_sha256=model.BASE566_FAMILY_ID,
        weight_content_verified=False, model_size_source='trusted_family_template')
    return entry


def scan_catalog(catalog_root, family_template):
    """Return internal entries; expose labels/identity, not manifests, to devices."""
    root = Path(catalog_root)
    if not root.is_absolute() or '..' in root.parts:
        raise ValueError('absolute trusted catalog root required')
    root = root.resolve()
    template, stats, family_error = _family(family_template)
    if not root.exists():
        return []
    if not root.is_dir():
        raise ValueError('catalog root must be a directory')
    entries = []
    for directory in sorted(root.iterdir()):
        if not directory.is_dir() and not directory.is_symlink():
            continue
        try:
            entry = _entry(directory, root, template, stats, family_error)
        except (OSError, ValueError, TypeError, KeyError, AttributeError) as error:
            entry = dict(directory_name=directory.name, label=directory.name, model_id=None, epoch=None, step=None,
                         status='incompatible', selectable=False, reason=str(error))
        entries.append(entry)
    for entry in entries:
        identity = entry['model_id']
        if identity and sum(row['model_id'] == identity for row in entries) > 1:
            entry.update(status='incompatible', selectable=False, reason='多个目录声明同一模型身份，请保留唯一候选')
            entry.pop('derived_manifest', None)
    return sorted(entries, key=lambda row: (-(row['epoch'] or 0), -(row['step'] or 0), row['directory_name']))


list_checkpoints = scan_catalog


def catalog_manifests(catalog_root, family_template):
    return {row['model_id']: row['derived_manifest'] for row in scan_catalog(catalog_root, family_template) if row['selectable']}


def resolve_checkpoint(catalog_root, family_template, model_id):
    for entry in scan_catalog(catalog_root, family_template):
        if entry['model_id'] == model_id:
            if entry['selectable']:
                return entry['derived_manifest']
            raise ValueError(entry['reason'])
    raise ValueError('checkpoint is not present in the trusted catalog: ' + str(model_id))
