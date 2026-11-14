"""Task-owned CPU control plane for fixed ARX13 conversion / preview CLIs."""
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit
from urllib.request import urlopen
import uuid

from .triton_preview import identifier, validate_prediction_result

TERMINAL = {'READY', 'SUCCEEDED', 'FAILED'}


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for data in iter(lambda:f.read(4*1024*1024),b''):h.update(data)
    return h.hexdigest()


def read(path):
    with Path(path).open() as f:return json.load(f)


def save(path, value):
    path=Path(path);tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2));tmp.replace(path)


class Rejected(ValueError):
    def __init__(self,code,message):self.code=code;super().__init__(message)


class ControlPlane:
    def __init__(self,config):
        self.config=config;self.root=Path(config['workspace']).resolve(strict=True)
        self.source=Path(config.get('source_root',self.root/'source')).resolve()
        if not self.source.is_relative_to(self.root):raise ValueError('model source must remain inside the owned workspace')
        self.state=self.root/'triton-control';self.state.mkdir(mode=0o700,exist_ok=True)
        self.jobs_root=self.state/'jobs';self.jobs_root.mkdir(exist_ok=True)
        self.lock=threading.RLock();self.audit_lock=threading.Lock();self.stop=threading.Event()
        self.jobs={};self.active=None;self.worker=None;self.verified={};self.models={}
        self.blocked=False
        self.unconfirmed_gpu_lock=None
        for path in self.jobs_root.glob('*/job.json'):
            job=read(path);self.jobs[job['job_id']]=job
            if job['status'] not in TERMINAL:
                self.blocked=True;job.update(status='UNKNOWN',error=dict(code='SUPERVISOR_RESTARTED',message='旧任务终态未核验，禁止自动重试或启动新任务'))
        self.token_file=self.state/'token'
        if not self.token_file.exists():
            with self.token_file.open('x') as f:os.chmod(self.token_file,0o600);f.write(secrets.token_urlsafe(40))
        if self.token_file.stat().st_mode & 0o077:raise ValueError('private service token required')
        self.token=self.token_file.read_text().strip()

    def allocation(self):
        try:
            a=read(self.root/'gpu-allocation.json');now=datetime.now(timezone.utc)
            updated=datetime.fromisoformat(a['updated_at']);expires=datetime.fromisoformat(a['expires_at'])
            if a.get('session_id')!=self.config['owner_session'] or a.get('gpu_uuid')!=self.config['gpu_uuid'] or not a.get('run_id'):
                raise ValueError('GPU预留身份不匹配')
            if a.get('jobs_enabled') is not True:raise ValueError('GPU负责人尚未开放工作台任务窗口')
            if not 0 <= (now-updated).total_seconds() <= 3600:raise ValueError('GPU预留核验已过期')
            if (expires-now).total_seconds()<1200:raise ValueError('GPU预留剩余不足1200秒')
            return a
        except (OSError,ValueError,TypeError,KeyError) as e:
            raise Rejected('GPU_WINDOW_UNAVAILABLE',str(e)) from None

    def live_input(self):
        try:
            with urlopen(self.config['workbench_url']+'/api/state?environment=physical&business=inference',timeout=3) as response:
                v=json.load(response)
            r=v['runtime'];f=r.get('calibration_feedback') or {}
            stamp=f.get('sample_wall_ns');now=time.time_ns()
            if not(r.get('connected') and r.get('feedback_healthy') and r.get('runtime_nonce')
                   and isinstance(stamp,(int,float)) and 0 <= now-stamp <= 150_000_000
                   and len(v.get('media',[]))==3 and all(x.get('ready') and isinstance(x.get('source_wall_ns'),(int,float))
                       and 0<=now-x['source_wall_ns']<=200_000_000 for x in v['media'])):
                raise ValueError('缺少健康新鲜关节反馈或三路相机；不会接入SDK或启动相机')
            if r.get('automatic_enabled'):raise ValueError('设备动作中，不启动模型预览')
            return dict(runtime_nonce=r['runtime_nonce'],command_epoch=r.get('command_epoch'),sample_wall_ns=stamp)
        except (OSError,ValueError,KeyError,TypeError) as e:
            raise Rejected('INPUT_UNAVAILABLE',str(e)) from None

    def _inside(self,path):
        path=Path(path).resolve(strict=True)
        if not path.is_relative_to(self.root):raise ValueError('artifact outside owned workspace')
        return path

    def _signature(self,path):
        artifact=read(path);paths=[path,self.root/'model/model-bundle.json']
        for key in ('packed','packing_receipt','validation'):paths.append(self._inside(path.parent/artifact[key]['path']))
        for name in artifact['runtime_source']:paths.append(self._inside(self.source/name))
        return tuple((str(p),p.stat().st_size,p.stat().st_mtime_ns,p.stat().st_ctime_ns) for p in paths)

    def _audit_model(self,path,key,signature):
        try:
            sys.path.insert(0,str(self.source))
            from manipulation.projects.arx_base566.deployment import verify_deployment
            _,artifact=verify_deployment(path,self.root/'model/model-bundle.json')
            if self._signature(path)!=signature:raise ValueError('artifact changed during verification')
            value=dict(signature=signature,status='READY',parity=read(path.parent/artifact['validation']['path']))
        except Exception as error:value=dict(signature=signature,status='INVALID',error=str(error)[:512])
        finally:
            with self.lock:self.verified[key]=value
            self.audit_lock.release()

    def catalog(self):
        try:self.allocation();reason=None
        except Rejected as e:reason=str(e)
        with self.lock:
            busy=bool(self.active or self.blocked or self.stop.is_set());models=[];self.models={}
            if self.blocked:reason='旧任务退出未核验，禁止自动重试或启动新任务'
            roots=[p for p in self.root.glob('conversion-*') if p.is_dir()]
            roots += [self.jobs_root/j['job_id']/'output' for j in self.jobs.values() if j['type']=='conversions']
            for output in roots[-100:]:
                run_path=output/'run.json';deployment=output/'deployment.json'
                if not run_path.exists():continue
                run=read(run_path)
                if run.get('schema')!='manipulation.triton_conversion_run.v1':continue
                key='model-'+hashlib.sha256(str(output).encode()).hexdigest()[:24]
                row=dict(id=key,display_name='Base566 e008 · '+output.parent.name if output.name=='output' else 'Base566 e008 · '+output.name,
                         status=run.get('status','UNKNOWN'),selectable=False,error=run.get('error'))
                if run.get('status')=='READY' and deployment.exists():
                    try:
                        signature=self._signature(deployment);cached=self.verified.get(key)
                        if not cached or cached['signature']!=signature:
                            row['status']='VERIFYING'
                            if self.audit_lock.acquire(blocking=False):
                                threading.Thread(target=self._audit_model,args=(deployment,key,signature),daemon=True).start()
                        else:
                            row.update(status=cached['status'],error=cached.get('error'))
                            if cached['status']=='READY':
                                row.update(deployment_sha256=sha(deployment),parity=dict(passed=True,gates=cached['parity']['gates']),selectable=not busy and reason is None)
                                self.models[key]=deployment
                    except (OSError,ValueError,KeyError) as e:row.update(status='INVALID',error=str(e)[:512])
                elif run.get('status')=='READY':row.update(status='INVALID',error='READY缺少部署制品')
                models.append(row)
            return dict(sources=[dict(id='base566-e008',display_name='Base566 epoch008 / step67408',status='AVAILABLE',selectable=not busy and reason is None)],
                models=models,reason=reason,busy=busy,hardware_control=False,
                recent_jobs=[{k:j[k] for k in ('job_id','request_id','status','type')} for j in list(self.jobs.values())[-50:]])

    def start(self,kind,body):
        field={'conversions':'source_id','infer':'model_id'}.get(kind)
        if field is None or not isinstance(body,dict) or set(body)!={field,'request_id'}:raise Rejected('INVALID_REQUEST','只接受固定来源/模型ID及request_id')
        identifier(body[field]);identifier(body['request_id']);fingerprint=json.dumps([kind,body],sort_keys=True)
        with self.lock:
            for job in self.jobs.values():
                if job['request_id']==body['request_id']:
                    if job['fingerprint']!=fingerprint:raise Rejected('REQUEST_CONFLICT','同一request_id不得改变参数')
                    return dict(job_id=job['job_id'],status=job['status'])
            if self.active or self.blocked or self.stop.is_set():raise Rejected('BUSY','已有任务或未确认的旧任务')
            if kind=='conversions' and body[field]!='base566-e008':raise Rejected('UNKNOWN_SOURCE','未登记的源模型')
            deployment=None
            if kind=='infer':
                catalog=self.catalog();model=next((x for x in catalog['models']if x['id']==body[field]),None)
                if not model or model['status']!='READY':raise Rejected('MODEL_NOT_READY','仅READY模型允许推理')
                self.live_input();deployment=self.models[body[field]]
            allocation=self.allocation()
            key=uuid.uuid4().hex;folder=self.jobs_root/key;folder.mkdir(mode=0o700)
            job=dict(job_id=key,request_id=body['request_id'],fingerprint=fingerprint,type=kind,status='QUEUED',stage='queued',
                     created_at=time.time(),source_id=body[field],allocation=allocation,result=None,error=None,hardware_control=False)
            self.jobs[key]=job;self.active=key;save(folder/'job.json',job)
            self.worker=threading.Thread(target=self._run,args=(job,deployment),daemon=True);self.worker.start()
            return dict(job_id=key,status='QUEUED')

    def _run(self,job,deployment):
        folder=self.jobs_root/job['job_id'];output=folder/'output';name='pi05-arx13-'+job['job_id'];process=None;cleanup_ok=True;gpu_lock=None
        try:
            self.allocation()
            from .gpu_admission import acquire
            gpu_lock=acquire(dict(self.config,model_id=job['source_id'],min_free_memory_mib=self.config.get('min_free_memory_mib',0)))
            if job['type']=='infer':self.live_input()
            script=self.source/'scripts'/('deploy_pi05_triton.sh' if job['type']=='conversions' else 'infer_pi05_triton.sh')
            args=['bash',str(script),str(self.root),self.config['gpu_uuid'],self.config['image']]
            container_output='/work/'+str(output.relative_to(self.root))
            if job['type']=='conversions':
                args += ['--bundle','/work/model/model-bundle.json','--output',container_output,'--timeout','540']
                for source in ('/work/recorded-S0-input.npz','/work/nominal-r1/live-input.npz','/work/latency-r1/first-live-input.npz'):args+=['--input',source]
            else:args+=['--deployment','/work/'+str(deployment.relative_to(self.root)),'--output',container_output,'--nominal-live-arx13']
            job.update(status='RUNNING',stage='conversion' if job['type']=='conversions' else 'input_preflight',container_name=name,
                       script_sha256=sha(script),shared_wrapper_sha256=sha(self.source/'scripts/deploy_pi05_triton.sh'));save(folder/'job.json',job)
            with (folder/'process.log').open('xb')as log:
                process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env={**os.environ,
                    'MANIPULATION_JOB_CONTAINER_NAME':name,'MANIPULATION_JOB_CIDFILE':str(folder/'container.cid')})
                job['pid']=process.pid;save(folder/'job.json',job);deadline=time.monotonic()+1100
                while process.poll() is None:
                    if self.stop.wait(.2) or time.monotonic()>deadline:raise Rejected('INTERRUPTED','任务已中断或达到时限')
                    stage=output/'run.json'
                    if stage.exists():job['stage']=read(stage).get('stage',job['stage'])
                if process.returncode!=0:
                    info=read(output/'result.json') if (output/'result.json').exists() else read(output/'run.json') if (output/'run.json').exists() else {}
                    raise Rejected(info.get('error_code','PIPELINE_FAILED'),str(info.get('error','pipeline exit '+str(process.returncode))))
            if job['type']=='conversions':
                run=read(output/'run.json');artifact=output/'deployment.json'
                if run.get('status')!='READY' or not artifact.exists():raise ValueError('pipeline has no READY artifact')
                job.update(status='READY',stage='complete',result=dict(deployment_sha256=sha(artifact),hardware_control=False,command_publication=False))
            else:
                result=read(output/'result.json');result['model_id']=job['source_id']
                validate_prediction_result(result)
                if result.get('deployment_sha256')!=sha(deployment):raise ValueError('prediction deployment identity differs')
                job.update(status='SUCCEEDED',stage='complete',result=result)
        except Exception as e:
            job.update(status='FAILED',error=dict(code=getattr(e,'code','JOB_FAILED'),message=str(e)[:1024]))
        finally:
            if process is not None:
                # Operate only on this unguessable job name, never on a GPU or image-wide process list.
                try:
                    probe=subprocess.run(['docker','inspect','--format','{{.Id}} {{.Name}}',name],capture_output=True,text=True,timeout=5)
                    if probe.returncode==0:
                        cid,actual=probe.stdout.strip().split();assert actual=='/'+name and re.fullmatch('[a-f0-9]{64}',cid)
                        subprocess.run(['docker','stop','--time','8',cid],capture_output=True,timeout=12,check=True)
                    if process.poll() is None:
                        os.killpg(process.pid,signal.SIGTERM)
                        try:process.wait(timeout=5)
                        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
                    verify=subprocess.run(['docker','ps','-q','--filter','name=^/'+name+'$'],capture_output=True,text=True,timeout=5,check=True)
                    if verify.stdout.strip():raise ValueError('owned container still running')
                except Exception as e:cleanup_ok=False;job.update(status='UNKNOWN',error=dict(code='CLEANUP_UNCONFIRMED',message=str(e)[:512]))
            job.update(finished_at=time.time(),cleanup_confirmed=cleanup_ok)
            with self.lock:
                save(folder/'job.json',job);self.active=None;self.blocked=not cleanup_ok
                if gpu_lock is not None:
                    if cleanup_ok:gpu_lock.close()
                    else:self.unconfirmed_gpu_lock=gpu_lock

    def close(self):
        self.stop.set()
        if self.worker:self.worker.join(25)


def serve(plane,port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,code,value):
            raw=json.dumps(value,ensure_ascii=False,allow_nan=False).encode();self.send_response(code)
            self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(raw)));self.send_header('Cache-Control','no-store');self.end_headers()
            try:self.wfile.write(raw)
            except (BrokenPipeError,ConnectionResetError):pass
        def route(self):
            self.close_connection=True
            if self.headers.get('Host') not in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}') or not secrets.compare_digest(self.headers.get('Authorization',''),'Bearer '+plane.token):
                return self.send(403,dict(error=dict(code='UNAUTHORIZED',message='service authorization required')))
            path=urlsplit(self.path)
            try:
                if path.query or path.fragment:raise Rejected('INVALID_REQUEST','无查询参数接口')
                if self.command=='GET':
                    if path.path=='/v1/health':return self.send(200,dict(status='READY',hardware_control=False,busy=bool(plane.active),jobs_enabled=plane.catalog()['sources'][0]['selectable']))
                    if path.path=='/v1/catalog':return self.send(200,plane.catalog())
                    if path.path.startswith('/v1/jobs/'):
                        key=identifier(path.path.rsplit('/',1)[-1])
                        with plane.lock:value=dict(plane.jobs.get(key) or {})
                        value.pop('fingerprint',None);value.pop('allocation',None)
                        return self.send(200,value)if value else self.send(404,dict(error=dict(code='NOT_FOUND',message='任务不存在')))
                if self.command=='POST' and path.path in ('/v1/conversions','/v1/infer'):
                    lengths=self.headers.get_all('Content-Length',[])
                    if self.headers.get('Transfer-Encoding') or len(lengths)!=1 or not lengths[0].isdigit() or not 0<int(lengths[0])<4096:raise Rejected('INVALID_REQUEST','bounded JSON body required')
                    raw=self.rfile.read(int(lengths[0]));body=json.loads(raw)
                    return self.send(202,plane.start(path.path.rsplit('/',1)[-1],body))
                self.send(404,dict(error=dict(code='NOT_FOUND',message='unknown endpoint')))
            except (ValueError,KeyError,TypeError)as e:self.send(409,dict(error=dict(code=getattr(e,'code','INVALID_REQUEST'),message=str(e)[:1024])))
            except Exception as e:self.send(503,dict(error=dict(code='SERVICE_UNAVAILABLE',message=str(e)[:512])))
        do_GET=route
        do_POST=route
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler);server.daemon_threads=True;return server
