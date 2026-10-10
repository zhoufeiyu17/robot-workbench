"""Display-only bridge to the existing C renderer. Never starts a GPU or an Owner.

Actual camera input for candidate mesh is supplied independently by the caller;
target pose edits are sent only to the renderer's separate /Preview scene.
"""
import base64
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from urllib.error import HTTPError
from urllib.parse import urlencode, urlsplit
from urllib.request import Request, build_opener, ProxyHandler, HTTPRedirectHandler


class PendingFrame(ValueError):pass


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self,*args):return None


class RendererClient:
    """Loopback/Bearer transport for display endpoints, with bounded responses."""
    def __init__(self,url,token_file,*,timeout=1.):
        endpoint=urlsplit(url);path=Path(token_file)
        if (endpoint.scheme!='http' or endpoint.hostname not in ('127.0.0.1','localhost')
                or not endpoint.port or endpoint.username or endpoint.password or endpoint.query or endpoint.fragment
                or endpoint.path not in ('','/') or not path.is_absolute() or not 0<timeout<=3):
            raise ValueError('bounded loopback renderer and absolute token file required')
        self.url=url.rstrip('/');self.token=path.read_text().strip();self.timeout=timeout
        if len(self.token)<32:raise ValueError('renderer token too short')
        self.http=build_opener(ProxyHandler({}),NoRedirect())

    def request(self,path,body=None,*,frame=False,binary=False):
        endpoint=urlsplit(path)
        allowed={'/state','/preview','/overview-view','/mirror','/overlay-rendering','/real-overlay','/frame.jpg'}
        if endpoint.path not in allowed or endpoint.scheme or endpoint.netloc or endpoint.fragment:
            raise ValueError('display-only renderer endpoint required')
        raw=None if body is None else json.dumps(body,allow_nan=False).encode()
        if raw is not None and len(raw)>2_000_000:raise ValueError('bounded renderer request required')
        request=Request(self.url+path,data=raw,headers={'Authorization':'Bearer '+self.token,'Content-Type':'application/json'})
        try:
            with self.http.open(request,timeout=self.timeout) as response:
                data=response.read(4*1024*1024+1)
                if len(data)>4*1024*1024:raise ValueError('renderer response exceeds frame budget')
                if frame:return data,json.loads(response.headers['X-Rendered-Frame'])
                return data if binary else json.loads(data)
        except HTTPError as error:
            if error.code==409:raise PendingFrame('renderer has not published this display revision') from None
            raise ValueError('renderer HTTP '+str(error.code)) from None


def vector(value,count,label):
    if (not isinstance(value,(list,tuple)) or len(value)!=count
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in value)):
        raise ValueError(label+' requires finite values')
    return list(value)


def observed_mesh(client,inputs,candidate_id):
    """Reuse C /real-overlay, with the supplied immutable actual frame/readback."""
    from robot_workbench.sample_clock import clock_spec,sample_stamp
    raw,meta,feedback,camera,candidate=inputs
    meta,feedback,camera,candidate=map(deepcopy,(meta,feedback,camera,candidate))
    if not isinstance(raw,bytes) or not raw or len(raw)>1_400_000:raise ValueError('bounded actual JPEG required')
    environment=meta.get('environment','physical')
    clock=clock_spec(environment,clock_domain=meta.get('clock_domain'),source_clock_id=meta.get('source_clock_id'))
    image_stamp=sample_stamp(meta,clock);joint_stamp=sample_stamp(feedback,clock,wall_key='sample_wall_ns')
    if image_stamp!=joint_stamp:
        raise ValueError('display mesh requires the exact sampled frame readback, not a latest target or feedback')
    age=(time.time_ns()-meta['host_receive_ns'])/1e9
    if not 0<=age<2:raise ValueError('actual mesh source image is stale')
    if candidate.get('id')!=candidate_id:raise ValueError('selected candidate changed')
    q=vector(feedback.get('qpos'),14,'actual sampled qpos')
    frame_id=meta.get('pair_id')
    if not isinstance(frame_id,str) or not frame_id:raise ValueError('actual source frame identity required')
    started=time.monotonic()
    image=client.request('/real-overlay',dict(jpeg_base64=base64.b64encode(raw).decode(),qpos=q,
        candidate=candidate,camera=camera),binary=True)
    render_s=time.monotonic()-started
    return image,dict(schema='observed_candidate_mesh.v1',source='actual_frame_candidate_mesh',
        source_frame_id=frame_id,frame_id=frame_id+':'+hashlib.sha256(json.dumps(candidate,sort_keys=True,allow_nan=False).encode()).hexdigest(),
        source_jpeg_sha256=hashlib.sha256(raw).hexdigest(),source_time_ns=image_stamp[0],
        source_wall_ns=meta['host_receive_ns'],
        source_clock_id=clock['source_clock_id'],clock_domain=clock['clock_domain'],environment=environment,
        candidate_id=candidate_id,source_age_s=age,observed_age_s=age+render_s,render_ms=render_s*1000,
        display_only=True,measurement_qualified=False)


class TargetPreview:
    """Two bounded workers: latest target/overview and optional observed mesh.

    Constructing is inert. start() starts CPU HTTP workers only; the caller owns
    an already-admitted renderer process and its exact renderer_instance_id.
    """
    def __init__(self,client,renderer_instance_id,*,validate_pose=None,mesh_inputs=None):
        if not isinstance(renderer_instance_id,str) or not renderer_instance_id:raise ValueError('renderer instance required')
        self.client,self.instance=client,renderer_instance_id
        self.validate_pose,self.mesh_inputs=validate_pose,mesh_inputs
        self.lock=threading.Lock();self.closed=threading.Event();self.changed=threading.Event()
        self.threads=[];self.target=None;self.view=None;self.mirror=None
        self.revisions=dict(target=0,view=0,mirror=0,mesh=0)
        self.requested_wall_ns=dict(target=0,view=0)
        self.frames={};self.error=None;self.mesh_error=None;self.mirror_error=None;self.last_health=0.
        self.mesh_selection=dict(enabled=False,camera=None,candidate_id=None)

    def start(self):
        if self.threads:raise ValueError('preview workers already started')
        for name,fn in (('target-preview',self._preview_loop),('candidate-mesh',self._mesh_loop)):
            thread=threading.Thread(target=fn,name=name,daemon=True);self.threads.append(thread);thread.start()

    def target_pose(self,q12,jaw_gap_mm):
        q=vector(q12,12,'target q12');gaps=vector(jaw_gap_mm,2,'target gaps')
        if self.validate_pose is not None:self.validate_pose(q,gaps)
        with self.lock:
            self.target=dict(q=q,jaw_gap_mm=gaps);self.revisions['target']+=1
            self.requested_wall_ns['target']=time.time_ns()
            revision=self.revisions['target']
        self.changed.set();return dict(queued=True,client_revision=revision,display_only=True,token=None)

    def overview(self,yaw,pitch,zoom,pan_y=0.):
        values=vector([yaw,pitch,zoom,pan_y],4,'overview')
        if not (-math.pi<=yaw<=math.pi and -.15<=pitch<=1.5 and .4<=zoom<=4 and abs(pan_y)<=2):raise ValueError('overview outside bounds')
        with self.lock:
            self.view=dict(zip(('yaw','pitch','zoom','pan_y'),values));self.revisions['view']+=1
            self.requested_wall_ns['view']=time.time_ns()
            self.frames.pop('overview',None);self.frames.pop('target',None)
            revision=self.revisions['view']
        self.changed.set();return {'queued':True,'display_only':True,'client_revision':revision}

    def mirror_feedback(self,body):
        # This only mirrors display kinematics; it never writes to the source plant.
        if set(body)!={'q','jaw_gap_mm','source_id','sequence','source_age_s'}:raise ValueError('renderer mirror fields differ')
        vector(body['q'],12,'feedback q');vector(body['jaw_gap_mm'],2,'feedback gaps')
        with self.lock:self.mirror=deepcopy(body);self.revisions['mirror']+=1
        self.changed.set()

    def mesh(self,enabled,*,camera='head',candidate_id=None):
        if type(enabled) is not bool or camera not in ('head','left','right'):raise ValueError('explicit mesh switch/camera required')
        if enabled and (self.mesh_inputs is None or not isinstance(candidate_id,str) or not candidate_id):
            raise ValueError('actual input reader and selected candidate required')
        with self.lock:
            self.mesh_selection=dict(enabled=enabled,camera=camera,candidate_id=candidate_id)
            self.revisions['mesh']+=1;self.frames.pop('mesh',None);self.mesh_error=None
            revision=self.revisions['mesh']
        return dict(enabled=enabled,client_revision=revision,warning='开启 mesh 叠加会降低姿态预览性能',display_only=True)

    def state(self):
        with self.lock:
            return dict(renderer_instance_id=self.instance,connected=time.monotonic()-self.last_health<3,
                error=self.error,mesh_error=self.mesh_error,mirror_error=self.mirror_error,mesh=deepcopy(self.mesh_selection),
                revisions=dict(self.revisions),frames={key:deepcopy(value[1]) for key,value in self.frames.items()
                    if key!='mesh' or self.mesh_selection['enabled'] and value[1]['observed_age_s']+time.monotonic()-value[2]<2},
                display_only=True,control_authority=False)

    def media(self,view):
        if view not in ('target','overview','mesh'):raise ValueError('unknown preview view')
        with self.lock:
            frame=self.frames.get(view)
            if frame is None:raise PendingFrame('current display frame not ready')
            raw,metadata,received=frame
            if view=='mesh':
                if not self.mesh_selection['enabled'] or metadata['observed_age_s']+time.monotonic()-received>=2:
                    raise PendingFrame('observed mesh is disabled or stale')
            elif time.monotonic()-self.last_health>=3:raise PendingFrame('renderer connection is stale')
            return raw,deepcopy(metadata)

    def close(self):
        self.closed.set();self.changed.set()
        # A blocked renderer request cannot block Owner stop or status readers.
        for thread in self.threads:thread.join(timeout=.05)

    def _preview_loop(self):
        sent=dict(target=-1,view=-1,mirror=-1);target_revision=None;view_revision=None
        target_complete=True;target_requested=0
        while not self.closed.is_set():
            started=time.monotonic()
            with self.lock:
                current=dict(self.revisions);target,view,mirror=deepcopy((self.target,self.view,self.mirror))
                requested=dict(self.requested_wall_ns)
            try:
                remote=self.client.request('/state')
                if remote.get('renderer_instance_id')!=self.instance:
                    with self.lock:
                        self.frames.pop('target',None);self.frames.pop('overview',None);self.last_health=0.
                    raise ValueError('renderer restarted; explicit rebind required')
                # Candidate mesh uses stateless observed pixels only, not duplicate continuous overlays.
                if remote.get('mesh_rendering_enabled') is not False:self.client.request('/overlay-rendering',{'enabled':False})
                if view is not None and sent['view']!=current['view']:
                    view_revision=max(remote.get('overview_revision',0)+1,(view_revision or 0)+1)
                    self.client.request('/overview-view',dict(view,revision=view_revision));sent['view']=current['view']
                    # A new view invalidates the old in-flight target; resend the latest pose.
                    target_complete=True;sent['target']=-1
                if target is not None and target_complete and sent['target']!=current['target']:
                    # Keep one target in flight until its pixels arrive; newer inputs stay coalesced in self.target.
                    target_revision=max(remote.get('preview_revision',-1)+1,(target_revision or 0)+1)
                    self.client.request('/preview',dict(target,revision=target_revision));sent['target']=current['target']
                    target_requested=requested['target'];target_complete=False
                if mirror is not None and sent['mirror']!=current['mirror']:
                    try:
                        self.client.request('/mirror',mirror)
                        with self.lock:self.mirror_error=None
                    except Exception as error:
                        with self.lock:self.mirror_error=str(error)
                    sent['mirror']=current['mirror']
                for name,renderer_view in (('overview','actual'),('target','preview')):
                    if name=='target' and target_revision is None:continue
                    query=dict(view=renderer_view,renderer_instance_id=self.instance,
                               overview_revision=view_revision if view_revision is not None else remote.get('overview_revision',0))
                    if name=='target':query['target_revision']=target_revision
                    stamp=(remote.get('frame_timestamp_s') or {}).get(renderer_view)
                    with self.lock:cached=self.frames.get(name)
                    if (cached and type(stamp) in (int,float) and math.isfinite(stamp)
                            and cached[1]['source_timestamp_ns']==int(stamp*1_000_000_000)
                            and cached[1]['renderer_instance_id']==self.instance
                            and cached[1]['overview_revision']==query['overview_revision']
                            and remote.get('overview_revision',0)==query['overview_revision']
                            and (name!='target' or cached[1]['target_revision']==target_revision==remote.get('preview_revision'))):
                        continue
                    try:raw,metadata=self.client.request('/frame.jpg?'+urlencode(query),frame=True)
                    except PendingFrame:continue
                    if (metadata.get('schema')!='target_preview_frame.v1' or not metadata.get('frame_id')
                            or metadata.get('renderer_instance_id')!=self.instance or metadata.get('view')!=renderer_view
                            or metadata.get('source')!=('kinematic_target' if name=='target' else 'kinematic_overview')
                            or metadata.get('overview_revision')!=query['overview_revision']
                            or name=='target' and metadata.get('target_revision')!=target_revision
                            or type(metadata.get('source_timestamp_ns')) is not int or metadata['source_timestamp_ns']<=0):
                        raise ValueError('renderer pixels have no matching atomic display metadata')
                    with self.lock:
                        if current['view']==self.revisions['view']:
                            previous=self.frames.get(name)
                            if previous and metadata['source_timestamp_ns']<previous[1]['source_timestamp_ns']:
                                raise ValueError('renderer display timestamp moved backwards')
                            stamp=max(target_requested,requested['view']) if name=='target' else requested['view']
                            timing=dict(client_requested_wall_ns=stamp,display_latency_ms=(metadata['source_timestamp_ns']-stamp)/1e6) if stamp else {}
                            self.frames[name]=(raw,dict(metadata,client_revision=sent['target'] if name=='target' else current['target'],
                                client_view_revision=current['view'],**timing),time.monotonic())
                            if name=='target':target_complete=True
                with self.lock:self.last_health=time.monotonic();self.error=None
            except Exception as error:
                with self.lock:self.error=str(error)
            self.changed.wait(max(0.,.025-(time.monotonic()-started)));self.changed.clear()

    def _mesh_loop(self):
        while not self.closed.is_set():
            started=time.monotonic()
            with self.lock:selection=dict(self.mesh_selection);revision=self.revisions['mesh']
            if selection['enabled']:
                try:
                    # Single in-flight job; disabling drops this result and starts no further work.
                    inputs=self.mesh_inputs(selection['camera'],selection['candidate_id'])
                    with self.lock:valid=revision==self.revisions['mesh'] and self.mesh_selection['enabled']
                    if valid:
                        raw,metadata=observed_mesh(self.client,inputs,selection['candidate_id'])
                        with self.lock:
                            if revision==self.revisions['mesh'] and self.mesh_selection['enabled']:
                                self.frames['mesh']=(raw,dict(metadata,renderer_instance_id=self.instance,revision=revision,camera=selection['camera']),time.monotonic());self.mesh_error=None
                except Exception as error:
                    with self.lock:
                        if revision==self.revisions['mesh']:self.mesh_error=str(error)
            self.closed.wait(max(0.,.05-(time.monotonic()-started)))
