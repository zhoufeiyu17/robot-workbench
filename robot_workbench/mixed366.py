"""Model-only mixed366 lifecycle and conversion. No robot or camera API."""
from copy import deepcopy
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import re
import secrets
import subprocess
import sys
import threading
import time
import numpy as np
from .inference_model import MIXED366_ID,MIXED366_SCHEMA,MIXED366_PINS,NOHISTORY_MODELS,CAMS,worker_identity,model_bundle_digest
from .inference_transport import ModelService,PROTOCOL
from .gpu_admission import acquire as gpu_lock, registered_desktop


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb')as f:
        for b in iter(lambda:f.read(8<<20),b''):h.update(b)
    return h.hexdigest()


def save(path,value):
    path=Path(path);temp=path.with_name(path.name+'.tmp');temp.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False,indent=2)+'\n');temp.chmod(0o600);temp.replace(path)


def descriptor(registry,backend='torch',deployment=None):
    identity=registry.get('model_id',MIXED366_ID);registered=NOHISTORY_MODELS[identity]
    return dict(schema=MIXED366_SCHEMA,model_id=identity,epoch=registered['epoch'],step=registered['step'],backend=backend,
        artifacts={k:dict(sha256=v)for k,v in registered['pins'].items()},model_bundle_sha256=registry['bundle_sha256'],
        runtime=dict(adapter='mixed366_process_model_client.v1',source_manifest_sha256=registry['source_manifest_sha256'],
            frozen_model_source_sha256=registry['runtime_sha256'],triton_deployment_sha256=deployment))


def verify_registry(registry):
    root=Path(registry['root']).resolve(strict=True)
    if registry.get('machine_id_sha256')!=sha('/etc/machine-id')or not registry['gpu_uuid'].startswith('GPU-'):
        raise ValueError('registered host/GPU required')
    for item in registry['files']:
        path=(root/item['path']).resolve(strict=True)
        large_weight=path.suffix.lower()in('.pt','.pth','.ckpt','.safetensors','.bin')and item['bytes']>64*1024**2
        if not path.is_relative_to(root)or path.stat().st_size!=item['bytes']or not large_weight and sha(path)!=item['sha256']:
            raise ValueError('model bundle artifact changed: '+item['path'])
    expected=NOHISTORY_MODELS[registry.get('model_id',MIXED366_ID)]['pins']['checkpoint']
    checkpoints=[row for row in registry['files']if row['path']=='bundle/model/model.pt']
    if len(checkpoints)!=1 or checkpoints[0]['sha256']!=expected:raise ValueError('registered checkpoint mismatch')
    return root


def _worker(connection,root,backend,packed,gpu,log):
    os.environ.update(CUDA_VISIBLE_DEVICES=gpu,STARVLA_PI05_DISABLE_COMPILE='1',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2',TOKENIZERS_PARALLELISM='false')
    with open(log,'a',buffering=1)as stream:
        os.dup2(stream.fileno(),1);os.dup2(stream.fileno(),2)
        try:
            from .mixed366_runtime import Policy
            model=Policy(root,backend,packed);connection.send({'ready':True,'pid':os.getpid()})
            while True:
                request=connection.recv()
                if request is None:return
                started=time.monotonic();result=model.infer(request);result['audit']=dict(result.get('audit',{}),wall_s=time.monotonic()-started,backend=backend,history_steps=0,command_publication=False)
                connection.send({'ok':True,'result':result})
        except BaseException as e:
            import traceback
            traceback.print_exc()
            try:connection.send({'error':str(e)})
            except (OSError,EOFError):pass
        finally:connection.close()


class Client:
    def __init__(self,registry,bundle,packed=None):
        self.registry=registry;self.bundle=deepcopy(bundle);self.model_id=bundle['model_id'];self.packed=packed
        self.process=None;self.connection=None;self.lock=None;self.log_path=str(Path(registry['root'])/'logs'/('worker-'+secrets.token_hex(8)+'.log'))
        Path(self.log_path).parent.mkdir(parents=True,exist_ok=True)
    def worker_status(self):
        p=self.process
        return dict(worker_pid=None if p is None else p.pid,worker_exit_code=None if p is None else p.exitcode,worker_alive=p is not None and p.is_alive())
    def preflight(self):
        verify_registry(self.registry)

    def prepare(self):
        if sys.platform!='linux':raise ValueError('Linux model host required')
        if self.process is None:
            verify_registry(self.registry)
            if self.packed:
                deployment=Path(self.registry['root'])/'triton/deployment.json'
                # Conversion probes are unpublished and use the explicit zero identity.
                expected=self.bundle['runtime']['triton_deployment_sha256']
                if expected!='0'*64:
                    d=json.loads(deployment.read_text())
                    if sha(deployment)!=expected or d.get('status')!='READY'or sha(self.packed)!=d['packed_sha256']:
                        raise ValueError('qualified packed model changed before loading')
            self.lock=gpu_lock(self.registry)
            ctx=multiprocessing.get_context('spawn');parent,child=ctx.Pipe();self.connection=parent
            self.process=ctx.Process(target=_worker,args=(child,self.registry['root'],self.bundle['backend'],self.packed,self.registry['gpu_uuid'],self.log_path))
            self.process.start();child.close()
            if not parent.poll(300):raise TimeoutError('model worker load deadline; inspect its log')
            result=parent.recv()
            if result.get('ready')is not True:raise ValueError(result.get('error','model load failed'))
        result=worker_identity(self.model_id,self.bundle)
        if self.packed:result['packed_sha256']=sha(self.packed)
        return result
    def infer(self,request):
        if not self.process or not self.process.is_alive():raise ValueError('explicit model prepare required')
        if len(request.get('applied_history',[])):raise ValueError('nohistory worker rejects history')
        self.connection.send(request)
        if not self.connection.poll(120):raise TimeoutError('inference timeout; do not replay')
        value=self.connection.recv()
        if value.get('ok')is not True:raise ValueError(value.get('error','model inference failed'))
        return value['result']
    def close(self):
        p=self.process
        if p is not None:
            if p.is_alive():
                try:self.connection.send(None)
                except (OSError,EOFError,BrokenPipeError):pass
                p.join(15)
                if p.is_alive():p.terminate();p.join(15)
                if p.is_alive():raise RuntimeError('model worker exit unconfirmed; GPU lock retained')
            p.close();self.process=None
        if self.connection is not None:self.connection.close();self.connection=None
        if self.lock is not None:self.lock.close();self.lock=None


def fixture(root,task,index=0):
    root=Path(root);name={'coffee':'00-coffee.npz','lid_handoff':'08-lid_handoff.npz'}[task]
    sys.path.insert(0,str(root/'bundle/source/experiments/pi05_recovery_base/base'))
    from encoding import denorm
    stats=json.loads((root/'bundle/sidecars/stats.json').read_text())
    with np.load(root/'bundle/fixtures'/name,allow_pickle=False)as z:
        state=denorm(z['state'][index,:14],stats['state']).astype(np.float32)
        images={c:np.rint((z['images__'+c][index]+1)*127.5).clip(0,255).astype(np.uint8).transpose(1,2,0).copy()for c in CAMS}
    return dict(request_id='probe-'+secrets.token_hex(8),seed=20261102+index,task=task,applied_history=np.empty((0,14),np.float32),observation=dict(model_state14=state,images=images,source_time=0.,frame_ids={}))


class Service(ModelService):
    def __init__(self,registry):
        from .wam import is_model_id
        directory = None
        if 'catalog_root' in registry:
            from .inference_catalog import RegisteredCatalog
            if registry.get('additional_registries'):
                raise ValueError('catalog_root is the single model discovery source')
            directory = RegisteredCatalog(registry['catalog_root'])
            initial_client = directory.initial_client(registry)
            registry = initial_client.registry
        self.registry=registry;self.root=Path(registry['root']);self.jobs={};self.active=None;self.job_lock=threading.Lock();self.convert_thread=None
        self.checkpoints={(registry.get('model_id',MIXED366_ID),'torch'):registry}
        for path in registry.get('additional_registries',[]):
            entry=Path(path)
            if not entry.is_absolute()or entry.is_symlink()or entry.stat().st_mode&0o077:
                raise ValueError('private explicit checkpoint registry required')
            value=json.loads(entry.read_text());identity=value['model_id']
            backend=value.get('backend','torch');key=(identity,backend)
            if (identity not in NOHISTORY_MODELS and not is_model_id(identity)) or key in self.checkpoints:
                raise ValueError('checkpoint registry identity unknown or duplicated')
            self.checkpoints[key]=value
        self.deployment=self.root/'triton/deployment.json'
        self.packed_verification=None
        for p in (self.root/'jobs').glob('*/job.json'):
            d=json.loads(p.read_text());self.jobs[d['job_id']]=d
            if d['status']not in ('READY','SUCCEEDED','FAILED'):d.update(status='FAILED',error={'message':'service restarted; prior outcome unknown, inspect artifacts'})
        def catalog():
            d={'torch':descriptor(registry)}
            if self.deployment.exists():
                v=json.loads(self.deployment.read_text())
                path=(self.root/v['packed']).resolve()
                if not path.is_relative_to(self.root):raise ValueError('packed artifact outside model workspace')
                st=path.stat();signature=(str(path),st.st_size,st.st_mtime_ns,st.st_ctime_ns,v['packed_sha256'])
                if signature!=self.packed_verification:
                    if sha(path)!=v['packed_sha256']:raise ValueError('packed artifact checksum differs')
                    self.packed_verification=signature
                if v.get('status')=='READY'and v['bundle_sha256']==registry['bundle_sha256']:
                    d['triton']=descriptor(registry,'triton',sha(self.deployment))
            for (identity,backend),value in self.checkpoints.items():
                if value is not registry:
                    if is_model_id(identity):
                        from .wam import variants
                        for bundle in variants(value):
                            d[identity+'/'+backend+'/'+str(bundle['inference_options']['denoise_steps'])]=bundle
                    else:d[identity+'/torch']=descriptor(value)
            return d
        def factory(bundle):
            selected=self.checkpoints[(bundle['model_id'],'torch'if bundle['backend']=='triton'else bundle['backend'])]
            if is_model_id(bundle['model_id']):
                from .wam import Client as WamClient
                return WamClient(selected,bundle)
            packed=None if bundle['backend']=='torch'else str(Path(selected['root'])/json.loads((Path(selected['root'])/'triton/deployment.json').read_text())['packed'])
            return Client(selected,bundle,packed)
        if directory is not None:
            super().__init__(initial_client, catalog=directory.catalog,
                             catalog_rows=directory.rows, client_factory=directory.client,
                             client_refresh=directory.refresh_client)
        else:
            super().__init__(factory(catalog()['torch']),catalog=catalog,client_factory=factory)
    def tool_catalog(self):
        ready=any(bundle['model_id']==self.registry.get('model_id',MIXED366_ID) and bundle['backend']=='triton'
                  for bundle in self.catalog().values());busy=self.active is not None or self.status()['busy']
        choices=[dict(id=backend+'-'+task,display_name=backend.title()+' · '+task,status='READY',selectable=not busy,deployment_sha256=sha(self.deployment)if backend=='triton'else self.registry['bundle_sha256'])for backend in ['torch']+(['triton']if ready else[])for task in ['coffee','lid_handoff']]
        return dict(sources=[dict(id='mixed366-e010',display_name='mixed366-nohistory epoch010 / step34250',status='AVAILABLE',selectable=not busy)],models=choices,model_only_probes=True,busy=busy,reason='模型任务正在执行'if busy else None,hardware_control=False,recent_jobs=[{k:d[k]for k in ['job_id','request_id','status','type']}for d in list(self.jobs.values())[-30:]])
    def tool_start(self,kind,body):
        key={'conversions':'source_id','infer':'model_id'}.get(kind)
        if key is None or not isinstance(body,dict)or set(body)!={key,'request_id'}or not re.fullmatch('[A-Za-z0-9_-]{1,128}',str(body.get('request_id'))):raise ValueError('registered ID and request_id required')
        with self.job_lock:
            for d in self.jobs.values():
                if d['request_id']==body['request_id']:
                    if d['input']!=body or d['type']!=kind:raise ValueError('request_id conflict')
                    return deepcopy(d)
            if self.active is not None or self.status()['busy']:raise ValueError('model task busy')
            if kind=='conversions'and body[key]!='mixed366-e010':raise ValueError('unknown conversion source')
            if kind=='infer'and body[key]not in [x['id']for x in self.tool_catalog()['models']]:raise ValueError('unqualified model/task')
            job=secrets.token_hex(12);directory=self.root/'jobs'/job;directory.mkdir(mode=0o700)
            d=dict(job_id=job,request_id=body['request_id'],type=kind,input=body,status='QUEUED',stage='queued',result=None,error=None)
            self.jobs[job]=d;self.active=job;save(directory/'job.json',d)
            self.convert_thread=threading.Thread(target=self._tool,args=(d,),daemon=True);self.convert_thread.start();return deepcopy(d)
    def tool_job(self,key):
        with self.job_lock:
            if key not in self.jobs:raise ValueError('unknown job')
            return deepcopy(self.jobs[key])
    def _tool(self,d):
        directory=self.root/'jobs'/d['job_id']
        def stage(name):d.update(status='RUNNING',stage=name);save(directory/'job.json',d)
        try:
            if d['type']=='conversions':
                if not self.lock.acquire(blocking=False):raise ValueError('model worker busy')
                try:
                    self.client.close();self._set(status='not_loaded',loaded=False,busy=True,error=None)
                    stage('packing');output=directory/'packed.pt'
                    p=subprocess.run([self.registry['python'],'-B','-m','robot_workbench.mixed366','pack',str(self.root),str(output)],env={**os.environ,'CUDA_VISIBLE_DEVICES':'','OPENBLAS_NUM_THREADS':'2','OMP_NUM_THREADS':'2'},capture_output=True,text=True,timeout=600)
                    (directory/'pack.log').write_text(p.stdout+p.stderr)
                    if p.returncode:raise ValueError('CPU conversion failed; inspect pack.log')
                    stage('torch_reference');torch_client=Client(self.registry,descriptor(self.registry))
                    probes=[fixture(self.root,t,i)for t in ['coffee','lid_handoff']for i in [0,1]]
                    try:torch_client.prepare();reference=[torch_client.infer(q)for q in probes]
                    finally:torch_client.close()
                    stage('triton_parity');triton_client=Client(self.registry,descriptor(self.registry,'triton','0'*64),str(output))
                    try:triton_client.prepare();actual=[triton_client.infer(q)for q in probes];repeat=triton_client.infer(probes[0])
                    finally:triton_client.close()
                    metrics=[]
                    for q,a,b in zip(probes,reference,actual):
                        delta=a['normalized'].astype(float)-b['normalized'].astype(float);metrics.append(dict(task=q['task'],max_abs=float(abs(delta).max()),rmse=float(np.sqrt(np.mean(delta*delta)))))
                        np.savez(directory/(q['request_id']+'.npz'),torch_normalized=a['normalized'],triton_normalized=b['normalized'],torch_native14=a['action14'],triton_native14=b['action14'])
                    passed=all(x['max_abs']<=.01 and x['rmse']<=.002 for x in metrics)and np.array_equal(repeat['normalized'],actual[0]['normalized'])
                    receipt=dict(status='READY'if passed else'FAILED',model_id=MIXED366_ID,bundle_sha256=self.registry['bundle_sha256'],packed=str(output.relative_to(self.root)),packed_sha256=sha(output),metrics=metrics,limits=dict(normalized_max_abs=.01,normalized_rmse=.002),same_triton_repeat_exact=bool(np.array_equal(repeat['normalized'],actual[0]['normalized'])),scope='Torch/Triton same-host recorded input comparison; no physical precision qualification')
                    save(directory/'parity.json',receipt)
                    if not passed:raise ValueError('Triton数值对照未通过；原误差保留，不晋级READY')
                    save(self.deployment,receipt);d.update(status='READY',stage='complete',result=receipt)
                finally:self._set(busy=False);self.lock.release()
            else:
                backend,task=d['input']['model_id'].split('-',1)
                bundle=next(value for value in self.catalog().values()
                            if value['model_id']==self.registry.get('model_id',MIXED366_ID) and value['backend']==backend)
                stage('prepare_'+backend)
                metadata=dict(protocol=PROTOCOL,model_id=MIXED366_ID,conversion_id='0'*64,expected_bundle_digest=model_bundle_digest(MIXED366_ID,bundle),backend=backend)
                self.prepare(metadata);stage('infer_'+backend)
                with self.lock:

                    if self.model_bundle!=bundle:raise ValueError('selected model changed before probe')
                    q=fixture(self.root,task);result=self.client.infer(q)
                np.savez(directory/'prediction.npz',action14=result['action14'],normalized=result['normalized'])
                d.update(status='SUCCEEDED',stage='complete',result=dict(model_id=MIXED366_ID,backend=backend,task=task,shape=[60,14],input='已保存验证样本，非实时相机或关节反馈',units='recorded_native_state14',command_publication=False,hardware_control=False,values=result['action14'].tolist(),audit=result['audit']))
        except Exception as error:d.update(status='FAILED',error={'message':str(error)},stage=d.get('stage','failed'))
        finally:
            save(directory/'job.json',d)
            with self.job_lock:self.active=None
    def close(self):
        if self.convert_thread and self.convert_thread.is_alive():raise RuntimeError('conversion still active; wait for its receipt before service shutdown')
        super().close()


if __name__=='__main__':
    if len(sys.argv)==4 and sys.argv[1]=='pack':
        from .mixed366_runtime import pack
        pack(Path(sys.argv[2]),Path(sys.argv[3]))
    else:raise SystemExit('explicit pack ROOT NEW_OUTPUT required')
