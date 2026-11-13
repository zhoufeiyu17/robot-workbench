"""Version locks for the interpreter and distributions used by a model worker."""
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import re
import subprocess
import sys

SCHEMA = 'workbench.model-runtime.v1'


def snapshot():
    packages = {}
    for distribution in importlib.metadata.distributions():
        name = re.sub(r'[-_.]+', '-', distribution.metadata['Name']).lower()
        version = distribution.version
        # Match importlib.metadata.version: the first distribution on sys.path wins.
        packages.setdefault(name, version)
    return dict(schema=SCHEMA, python=platform.python_version(),
                implementation=sys.implementation.name, abi=sys.implementation.cache_tag,
                architecture=platform.machine(), packages=dict(sorted(packages.items())))


def capture(python):
    from rule_based_workbench.launch_environment import clean_environment
    raw = subprocess.check_output([str(python), '-I', str(Path(__file__).resolve()), '--snapshot'],
                                  env=clean_environment(), timeout=20)
    if len(raw) > 1024 * 1024:
        raise ValueError('runtime environment inventory exceeds limit')
    value = json.loads(raw)
    validate(value)
    return value


def validate(value):
    fields = {'schema', 'python', 'implementation', 'abi', 'architecture', 'packages'}
    if (not isinstance(value, dict) or set(value) != fields or value['schema'] != SCHEMA
            or any(not isinstance(value[k], str) or not value[k] for k in fields - {'packages'})
            or not isinstance(value['packages'], dict)
            or any(not isinstance(k, str) or not k or not isinstance(v, str) or not v
                   for k, v in value['packages'].items())):
        raise ValueError('explicit interpreter and package version lock required')
    return value


def verify(python, expected):
    validate(expected)
    actual = capture(python)
    if actual != expected:
        changed = [k for k in expected if k != 'packages' and expected[k] != actual[k]]
        changed += ['package ' + k for k in sorted(expected['packages'].keys() | actual['packages'].keys())
                    if expected['packages'].get(k) != actual['packages'].get(k)]
        raise ValueError('model runtime version lock mismatch: ' + ', '.join(changed[:12]))
    return actual


def read_pin(pin):
    if not isinstance(pin, dict) or set(pin) != {'path', 'sha256'}:
        raise ValueError('runtime environment lock path and digest required')
    path = Path(pin['path'])
    if not path.is_absolute() or not path.is_file() or path.stat().st_size > 1024 * 1024:
        raise ValueError('bounded runtime environment lock required')
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != pin['sha256']:
        raise ValueError('runtime environment lock content changed')
    return validate(json.loads(raw))


def interpreter(registry):
    if registry.get('python'):
        return Path(registry['python'])
    environment = Path(registry['root']) / 'env'
    if registry.get('backend', 'torch') == 'torch':
        return environment / 'bin/python'
    candidates = [path for path in environment.glob('anyinfer-*/bin/python') if path.is_file()]
    if len(candidates) != 1:
        raise ValueError('legacy model must declare one unambiguous runtime interpreter')
    return candidates[0]


def lock_path(backend):
    if backend not in ('torch', 'anyinfer'):
        raise ValueError('unsupported model runtime backend')
    return 'runtime/' + backend + '-environment.json'


def verify_registered(registry):
    name = lock_path(registry.get('backend', 'torch'))
    rows = [row for row in registry['files'] if row['path'] == name]
    if len(rows) != 1:
        raise ValueError('model runtime version lock missing; run install_model.py lock-runtime')
    row = rows[0]
    root = Path(registry['root']).resolve(strict=True)
    path = (root / name).resolve(strict=True)
    if not path.is_relative_to(root) or path.stat().st_size != row['bytes']:
        raise ValueError('runtime environment lock path or size differs')
    pin = dict(path=str(path), sha256=row['sha256'])
    return verify(interpreter(registry), read_pin(pin))


if __name__ == '__main__':
    if sys.argv[1:] != ['--snapshot']:
        raise SystemExit('use --snapshot')
    print(json.dumps(snapshot(), sort_keys=True))
