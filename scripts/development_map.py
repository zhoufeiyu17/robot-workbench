"""Maintain the architecture development map; CPU-only, no robot endpoints."""
import argparse
import ast
import fcntl
import hashlib
import io
import json
import os
from pathlib import Path
import shlex
import subprocess
import tarfile
import tempfile
from datetime import datetime, timezone
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
MAP = Path('web/architecture/development-map.json')
STATES = ('baseline', 'planned', 'active', 'blocked', 'implemented', 'verified')
ASSETS = ('index.html', 'style.css', 'app.mjs', 'usecases.mjs', 'recording.mjs', 'traceability.mjs',
          'catalog.json', 'development-map.html', 'development-map.mjs',
          'development-map.css', 'development-map.json')
SITE = '/home/wujie/services/robot-workbench-architecture'


def now():
    return datetime.now(timezone.utc).isoformat()


def digest(data):
    return hashlib.sha256(data).hexdigest()


def local_path(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('path outside repository')
    return path


def atomic_json(path, data):
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as out:
        json.dump(data, out, ensure_ascii=False, indent=2)
        out.write('\n')
        temporary = out.name
    os.replace(temporary, path)


def source_facts(root, paths, snapshot):
    modules = {p[:-3].replace('/', '.'): p for p in paths if p.endswith('.py')}
    # The recorder adds this existing source root to PYTHONPATH in its CPU child.
    for path in paths:
        if path.startswith('rule_based_workbench/') and path.endswith('.py'):
            modules.setdefault(path[len('rule_based_workbench/'):-3].replace('/', '.'), path)
    result = {}
    for relative in paths:
        path = local_path(root, relative)
        fact = {'exists': path.is_file(), 'sha256': None, 'imports': [], 'deployed_snapshots': []}
        if path.is_file():
            data = path.read_bytes()
            fact['sha256'] = digest(data)
            if path.suffix == '.py':
                tree = ast.parse(data)
                package = relative.rsplit('/', 1)[0].replace('/', '.')
                dependencies = set()
                for node in ast.walk(tree):
                    names = []
                    if isinstance(node, ast.Import):
                        names = [alias.name for alias in node.names]
                    elif isinstance(node, ast.ImportFrom):
                        base = node.module or ''
                        if node.level:
                            parts = package.split('.')
                            prefix = parts[:len(parts) - node.level + 1]
                            base = '.'.join(prefix + ([base] if base else []))
                        names = [base] + [base + '.' + alias.name for alias in node.names]
                    dependencies.update(modules[name] for name in names if name in modules)
                fact['imports'] = sorted(dependencies - {relative})
        for scope in ('', 'model-service'):
            deployed = local_path(root, str(Path(snapshot['root']) / scope / relative))
            if deployed.is_file():
                sha = digest(deployed.read_bytes())
                fact['deployed_snapshots'].append({'scope': scope or 'workbench', 'sha256': sha,
                                                   'matches_local': sha == fact['sha256']})
        result[relative] = fact
    return result


def fingerprint(files, paths):
    # Include mapped transitive imports so a dependency edit invalidates validation.
    pending, seen = list(paths), set()
    while pending:
        path = pending.pop()
        if path in seen:
            continue
        seen.add(path)
        pending.extend(files[path]['source']['imports'])
    return digest(json.dumps([(p, files[p]['source']['sha256']) for p in sorted(seen)]).encode())


def shared_fingerprint(catalog, data, shared):
    ids = sorted({key for step in shared['steps'] + shared['failures'] for key in step['bindings']})
    bindings = [catalog['traceability']['bindings'][key] for key in ids]
    paths = sorted({path for binding in bindings for path in binding['files']})
    contracts = [c for c in catalog['interfaces'] if c['id'] in {b['interface'] for b in bindings}]
    specification = {key: value for key, value in shared.items() if key != 'progress'}
    return digest(json.dumps([specification, bindings, contracts, fingerprint(data['files'], paths)], sort_keys=True).encode())


def refresh_shared(root, data, catalog):
    before = json.dumps(catalog, sort_keys=True)
    for shared in catalog.get('shared_steps', []):
        if 'progress' not in shared:
            continue
        progress = shared['progress']
        progress['fingerprint'] = shared_fingerprint(catalog, data, shared)
        if progress.get('evidence'):
            path = local_path(root, progress['evidence']['path'])
            progress['evidence']['matches_file'] = path.is_file() and digest(path.read_bytes()) == progress['evidence']['sha256']
    if before != json.dumps(catalog, sort_keys=True):
        catalogue_revision(catalog)
        atomic_json(root / 'web/architecture/catalog.json', catalog)
    data['architecture_revision'] = catalog['revision']


def catalogue_revision(catalog):
    suffix = digest(json.dumps(catalog['shared_steps'], sort_keys=True).encode())[:12]
    catalog['revision'] = catalog['revision'].split('+progress.')[0] + '+progress.' + suffix


def update_shared(root, data, catalog, ids, state, note, next_step, owner, evidence):
    if state == 'baseline':
        raise ValueError('SC progress requires an explicit development state')
    evidence_record = None
    receipt = None
    if evidence:
        content = local_path(root, evidence).read_bytes()
        evidence_record = {'path': evidence, 'sha256': digest(content), 'matches_file': True}
        receipt = json.loads(content)
        evidence_record['scope'] = receipt.get('scope', '')
    rows = {sc['id']: sc for sc in catalog['shared_steps']}
    for key in ids:
        shared = rows[key]
        source = shared_fingerprint(catalog, data, shared)
        if state == 'verified':
            ids_used = {b for step in shared['steps'] + shared['failures'] for b in step['bindings']}
            if any(catalog['traceability']['bindings'][b]['status'] != 'implemented' for b in ids_used):
                raise ValueError('SC still has pending implementation bindings')
            if not receipt or receipt.get('source_fingerprint') != source or not receipt.get('scope'):
                raise ValueError('SC verification requires matching source fingerprint and scope')
            outcomes = receipt.get('cases', {})
            if any(outcomes.get(test['id']) != 'passed' for test in shared['acceptance']):
                raise ValueError('SC verification requires every acceptance case to pass')
        shared['progress'].update(state=state, note=note, next=next_step, owner=owner, updated_at=now(),
                                  fingerprint=source, evidence=evidence_record,
                                  verified_fingerprint=source if state == 'verified' else None)
    catalogue_revision(catalog)
    atomic_json(root / 'web/architecture/catalog.json', catalog)
    data['architecture_revision'] = catalog['revision']
    data['changes'].append({'at': now(), 'kind': 'sc', 'ids': ids, 'state': state, 'note': note})
    data['changes'] = data['changes'][-100:]


def refresh(root, data):
    facts = source_facts(root, data['files'], data['deployment_snapshot'])
    for path, fact in facts.items():
        data['files'][path]['source'] = fact
    scanned = {str(p.relative_to(root)) for pattern in data['inventory_globs'] for p in root.glob(pattern)}
    data['unmapped_files'] = sorted(scanned - data['files'].keys())
    for path, row in data['files'].items():
        row['fingerprint'] = fingerprint(data['files'], [path])
    for domain in data['domains']:
        domain['fingerprint'] = fingerprint(data['files'], domain['files'])
    for row in list(data['files'].values()) + data['domains']:
        if row.get('evidence'):
            path = local_path(root, row['evidence']['path'])
            row['evidence']['matches_file'] = path.is_file() and digest(path.read_bytes()) == row['evidence']['sha256']
    data['source_head'] = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    catalog = json.loads((root / 'web/architecture/catalog.json').read_text())
    refresh_shared(root, data, catalog)


def update(root, data, kind, ids, state, note, next_step, owner, evidence):
    evidence_record = None
    if evidence:
        path = local_path(root, evidence)
        evidence_record = {'path': evidence, 'sha256': digest(path.read_bytes())}
    if state == 'verified' and not evidence_record:
        raise ValueError('verified requires a repository evidence file')
    rows = data['files'] if kind == 'file' else {d['id']: d for d in data['domains']}
    for key in ids:
        if key not in rows:
            raise ValueError('unknown map node: ' + key)
        row = rows[key]
        if state == 'verified':
            paths = [key] if kind == 'file' else row['files']
            if not paths or any(not data['files'][p]['source']['exists'] for p in paths):
                raise ValueError('cannot verify absent sources')
        row.update(state=state, note=note, next=next_step, owner=owner, updated_at=now(),
                   evidence=evidence_record, verified_fingerprint=row['fingerprint'] if state == 'verified' else None)
    data['changes'].append({'at': now(), 'kind': kind, 'ids': ids, 'state': state, 'note': note})
    data['changes'] = data['changes'][-100:]


def publish(root, data):
    subprocess.run(['node', str(root / 'scripts/check_architecture.mjs')], check=True)
    subprocess.run(['node', str(root / 'scripts/check_development_map.mjs')], check=True)
    subprocess.run(['node', str(root / 'scripts/check_traceability.mjs')], check=True)
    assets = {name: (root / 'web/architecture' / name).read_bytes() for name in ASSETS}
    identity = digest(b''.join(assets.values()))[:12]
    release = SITE + '/releases/devmap-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f') + '-' + identity
    previous = subprocess.check_output(['ssh', '-o', 'BatchMode=yes', 'lan-services-3090',
                                       'readlink ' + SITE + '/current'], text=True).strip()
    if not previous.startswith(SITE + '/releases/'):
        raise ValueError('unexpected publication owner')
    archive = io.BytesIO()
    with tarfile.open(fileobj=archive, mode='w') as tar:
        for name, content in assets.items():
            item = tarfile.TarInfo(name)
            item.size = len(content)
            item.mode = 0o644
            tar.addfile(item, io.BytesIO(content))
    command = 'mkdir ' + shlex.quote(release) + ' && tar -xf - -C ' + shlex.quote(release)
    subprocess.run(['ssh', '-o', 'BatchMode=yes', 'lan-services-3090', command], input=archive.getvalue(), check=True)
    hashes = {name: digest(content) for name, content in assets.items()}
    remote = """import pathlib,hashlib,os,fcntl
site=pathlib.Path(SITE)
with (site/'.publication.lock').open('a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX)
 assert os.readlink(site/'current') == PREVIOUS, 'publication changed concurrently'
 for name,sha in HASHES.items():
  assert hashlib.sha256((pathlib.Path(RELEASE)/name).read_bytes()).hexdigest()==sha
 temporary=site/('current-'+pathlib.Path(RELEASE).name)
 temporary.symlink_to(RELEASE)
 os.replace(temporary,site/'current')
"""
    bindings = '\n'.join(f'{key}={value!r}' for key, value in
                         [('SITE', SITE), ('PREVIOUS', previous), ('RELEASE', release), ('HASHES', hashes)])
    subprocess.run(['ssh', '-o', 'BatchMode=yes', 'lan-services-3090', 'python3 -'],
                   input=(bindings + '\n' + remote).encode(), check=True)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    for name, sha in hashes.items():
        with opener.open('http://10.17.0.27:18930/' + name, timeout=15) as response:
            if digest(response.read()) != sha:
                raise RuntimeError('served hash mismatch: ' + name)
    receipt = {'at': now(), 'revision': data['revision'], 'url': 'http://10.17.0.27:18930/#progress',
               'standalone_url': 'http://10.17.0.27:18930/development-map.html',
               'release': release, 'previous_release': previous, 'http_verified_sha256': hashes}
    directory = root / '.evidence/development-map-20261105-v1'
    directory.mkdir(parents=True, exist_ok=True)
    atomic_json(directory / ('publication-' + Path(release).name + '.json'), receipt)
    print(json.dumps(receipt, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['refresh', 'set', 'publish'])
    parser.add_argument('--kind', choices=['file', 'domain', 'sc'], default='file')
    parser.add_argument('--id', action='append', default=[])
    parser.add_argument('--state', choices=STATES)
    parser.add_argument('--note')
    parser.add_argument('--next', default='')
    parser.add_argument('--owner', default='本聊天 · 01a16327-7545-7332-a361-0123ddad811d')
    parser.add_argument('--evidence')
    parser.add_argument('--publish', action='store_true', help='Publish to the existing internal architecture site')
    args = parser.parse_args()
    lock_path = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', '--git-path', 'development-map.lock'], text=True).strip()
    with (ROOT / lock_path).open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = json.loads((ROOT / MAP).read_text())
        before = json.dumps(data, sort_keys=True)
        refresh(ROOT, data)
        if args.command == 'set':
            if not args.id or not args.state or not args.note:
                parser.error('set requires --id, --state and --note')
            if args.kind == 'sc':
                catalog = json.loads((ROOT / 'web/architecture/catalog.json').read_text())
                update_shared(ROOT, data, catalog, args.id, args.state, args.note, args.next, args.owner, args.evidence)
            else:
                update(ROOT, data, args.kind, args.id, args.state, args.note, args.next, args.owner, args.evidence)
        if before != json.dumps(data, sort_keys=True):
            data['updated_at'] = now()
            data['revision'] += 1
            atomic_json(ROOT / MAP, data)
        if args.publish or args.command == 'publish':
            publish(ROOT, data)
        else:
            print(json.dumps({'revision': data['revision'], 'files': len(data['files']),
                              'unmapped': data['unmapped_files']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
