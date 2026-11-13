#!/usr/bin/env python3
"""Export completed training releases or install them on a configured inference host."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    export = commands.add_parser('export', help='Training completion hook; produces a portable release manifest')
    export.add_argument('--spec', type=Path, required=True)
    export.add_argument('--source-root', type=Path, required=True)
    export.add_argument('--output', type=Path, required=True)
    publish=commands.add_parser('publish',help='Resolve a committed native training checkpoint and publish its release')
    publish.add_argument('--producer-profile',type=Path,required=True)
    publish.add_argument('--family',choices=('wam','wam-mem'),required=True)
    publish.add_argument('--experiment',type=Path,required=True)
    publish.add_argument('--checkpoint',default='latest')
    publish.add_argument('--name')
    publish.add_argument('--instruction')
    publish.add_argument('--oss-config-stdin',action='store_true')
    install = commands.add_parser('install', help='Publish a completed model into the target catalog')
    origin=install.add_mutually_exclusive_group(required=True)
    origin.add_argument('--source')
    origin.add_argument('--producer')
    install.add_argument('--family',choices=('wam','wam-mem'))
    install.add_argument('--experiment')
    install.add_argument('--checkpoint',default='latest')
    install.add_argument('--name')
    install.add_argument('--instruction')
    install.add_argument('--oss-config-stdin',action='store_true')
    route = install.add_mutually_exclusive_group(required=True)
    route.add_argument('--target')
    route.add_argument('--target-profile', type=Path)
    install.add_argument('--targets', type=Path, default=Path.home()/'.config/robot-workbench/model-targets.json')
    lock=commands.add_parser('lock-runtime',help='Freeze target runtime versions and bind existing model manifests')
    lock_route=lock.add_mutually_exclusive_group(required=True)
    lock_route.add_argument('--target')
    lock_route.add_argument('--target-profile',type=Path)
    lock.add_argument('--targets',type=Path,default=Path.home()/'.config/robot-workbench/model-targets.json')
    args = parser.parse_args()
    if args.command=='lock-runtime' and args.target:
        target=json.loads(args.targets.read_bytes())['targets'][args.target]
        if target['ssh'].startswith('-'):raise ValueError('explicit SSH destination required')
        command=[target['python'],target['installer'],'lock-runtime','--target-profile',target['profile']]
        return subprocess.run(['ssh','-o','BatchMode=yes',*target.get('ssh_args',[]),target['ssh'],shlex.join(command)],check=True).returncode
    if args.command == 'install' and args.target:
        targets = json.loads(args.targets.read_bytes())
        target = targets['targets'][args.target]
        credential=None
        provider=targets.get('credential_provider')
        if provider:
            if provider['ssh'].startswith('-'):raise ValueError('explicit credential provider required')
            fetched=subprocess.run(['ssh','-o','BatchMode=yes',*provider.get('ssh_args',[]),provider['ssh'],
                shlex.join(['cat','--',provider['config_file']])],check=True,capture_output=True,text=True)
            if len(fetched.stdout)>16384:raise ValueError('bounded OSS configuration required')
            credential=fetched.stdout
        source=args.source
        if args.producer:
            if not args.family or not args.experiment:parser.error('--producer requires --family and --experiment')
            producer=targets['producers'][args.producer]
            command=[producer['python'],producer['installer'],'publish','--producer-profile',producer['profile'],
                '--family',args.family,'--experiment',args.experiment,'--checkpoint',args.checkpoint]
            for key in ('name','instruction'):
                if getattr(args,key) is not None:command.extend(['--'+key,getattr(args,key)])
            if credential:command.append('--oss-config-stdin')
            if producer['ssh'].startswith('-'):raise ValueError('explicit producer SSH destination required')
            result=subprocess.run(['ssh','-o','BatchMode=yes',*producer.get('ssh_args',[]),producer['ssh'],shlex.join(command)],
                check=True,capture_output=True,text=True,input=credential)
            source=json.loads(result.stdout)['source']
        command = [target['python'], target['installer'], 'install', '--source', source,
                   '--target-profile', target['profile']]
        if credential:command.append('--oss-config-stdin')
        if target['ssh'].startswith('-'):raise ValueError('explicit SSH destination required')
        return subprocess.run(['ssh', '-o', 'BatchMode=yes',*target.get('ssh_args',[]), target['ssh'], shlex.join(command)],
            check=True,text=True,input=credential).returncode
    if sys.platform == 'darwin':
        raise ValueError('Run asset export on the training host; install using --target from this computer')
    from robot_workbench.model_install import read_json, export_release, OssSource, install_release,resolve_training_release,digest,install_lock,publish_release
    if args.command=='lock-runtime':
        from robot_workbench.model_install import lock_runtimes
        print(json.dumps(lock_runtimes(args.target_profile)))
        return 0
    if args.command == 'export':
        result = export_release(read_json(args.spec), args.source_root, args.output)
        print(json.dumps(dict(status='exported', model_id=result['model_id'], manifest=str(args.output))))
        return 0
    if args.command=='publish':
        profile=read_json(args.producer_profile)
        key=digest(dict(family=args.family,experiment=str(args.experiment),checkpoint=args.checkpoint,name=args.name))
        output=Path(profile['output_root'])/key
        with install_lock(output/'.publish.lock'):
            release,sources=resolve_training_release(args.family,args.experiment,args.checkpoint,profile['source_root'],output,
                runtime_id=profile['runtimes'][args.family],name=args.name,instruction=args.instruction)
            source,release=publish_release(release,sources,dict(profile['oss'],transfer_root=profile['transfer_root']),profile['oss_prefix'],output,
                config_text=sys.stdin.read(16385)if args.oss_config_stdin else None)
        print(json.dumps(dict(status='published',model_id=release['model_id'],source=source)))
        return 0
    if not args.source:parser.error('target-side install requires --source')
    target = read_json(args.target_profile)
    gpu_ids = subprocess.check_output(['nvidia-smi', '--query-gpu=uuid', '--format=csv,noheader'], text=True).splitlines()
    if target['gpu_uuid'] not in gpu_ids:raise ValueError('configured target GPU is absent')
    source = OssSource(args.source, target['oss'], Path(target['asset_root'])/'.transfer',
        config_text=sys.stdin.read(16385)if args.oss_config_stdin else None)
    result = install_release(source.manifest(), target, source.download)
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
