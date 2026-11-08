"""Shared model pins on the existing HTTP/NPZ service, without device ownership.

Wire-compatible with prometheus.dagger.model_session's CPU-qualified v1.
The capability prefix is removed before the canonical worker sees a request.
"""
from copy import deepcopy
import json
import math
import re
import secrets
import threading
import time

PROTOCOL = 'prometheus.model-session.v1'
_CAPABILITY = re.compile(r'([a-f0-9]{32})_(.+)\Z')


def _lease_guard(ttl,policy=None,*,identity=None):
    from .execution_settings import Guard,control_parameters,defaults,policy_hash
    if policy is None:
        parameters=control_parameters(defaults('inference'),business='inference')
        policy=dict(revision=None,policy_hash=policy_hash(parameters),parameters=parameters)
    context=dict(pin_ttl_s=ttl)
    if identity is not None:context['expected_identity']=identity
    return Guard.bind(policy,context)


def _session_error(error,token):
    result=RuntimeError(_redact(str(error),token))
    if getattr(error,'guard_decision',None) is not None:result.guard_decision=_redact(deepcopy(error.guard_decision),token)
    return result


def binding(status):
    return {key:deepcopy(status.get(key)) for key in ('model_id','bundle_digest','backend','worker_identity')} | {
        'instance_id':(status.get('fact') or {}).get('instance_id')}


def _redact(value, token):
    if isinstance(value,str):return value.replace(token+'_','').replace(token,'[session]')
    if isinstance(value,dict):return {k:_redact(v,token)for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [_redact(v,token)for v in value]
    return value


class PinnedModelService:
    """One gate for public prepare, unload, predictions and changing diagnostics."""
    def __init__(self,service,*,clock=time.monotonic,ttl_s=30.):
        if type(ttl_s)not in(int,float)or not math.isfinite(ttl_s)or ttl_s<=0 or not callable(clock):
            raise ValueError('positive finite session TTL and clock required')
        self.service,self.clock,self.ttl=service,clock,float(ttl_s)
        self._lease_guard=_lease_guard(self.ttl)
        self._pins={};self._gate=threading.RLock();self._inflight=0;self._closed=False;self._mutating=False

    def __getattr__(self,name):
        value=getattr(self.service,name)
        if name=='tool_catalog':return self._tool_catalog
        return value

    def _expire(self):
        now=self.clock()
        self._pins={token:row for token,row in self._pins.items()if self._lease_guard.evaluate(dict(rule_id='G10',
            signal_id=row['session_id'],observed=dict(current_time=now,renewed_time=row['renewed_at']),
            source_time=row['renewed_at'],checked_time=now),'model_pin_server')['verdict']=='pass'}

    def _check(self,token):
        self._expire()
        row=self._pins.get(token) if isinstance(token,str) else None
        if row is None:raise ValueError('model session missing or expired')
        current=self.service.status()
        now=self.clock()
        decision=row['guard'].evaluate(dict(rule_id='G01',signal_id=row['session_id'],observed=binding(current),
            source_time=now,checked_time=now),'model_pin_server')
        if not current.get('loaded') or decision['verdict']=='reject':
            error=ValueError('pinned model instance changed');error.guard_decision=decision;raise error
        return row

    def _mutation(self,*,allow_queued=False):
        self._expire()
        if self._closed:raise ValueError('model service closed')
        if self._mutating:raise ValueError('model operation busy')
        if self._pins:raise ValueError('model pinned; prepare, unload and diagnostic changes refused')
        if self._inflight:raise ValueError('model request in flight; mutation refused')
        if not allow_queued and getattr(self.service,'active',None)is not None:raise ValueError('model diagnostic task busy')

    def status(self):
        with self._gate:
            self._expire();value=self.service.status()
            for token in self._pins:value=_redact(value,token)
            return dict(value,session_protocol=PROTOCOL,active_sessions=len(self._pins),busy=bool(value.get('busy')or self._mutating))

    def _change(self,call,*,allow_queued=False):
        with self._gate:
            self._mutation(allow_queued=allow_queued);self._mutating=True
        try:return call()
        finally:
            with self._gate:self._mutating=False

    def prepare(self,metadata):
        if not isinstance(metadata,dict):raise ValueError('model session metadata must be an object')
        if 'session_protocol'not in metadata:return self._change(lambda:self.service.prepare(metadata))
        with self._gate:
            self._expire()
            if self._closed:raise ValueError('model service closed')
            if metadata['session_protocol']!=PROTOCOL:raise ValueError('unknown model session protocol')
            operation=metadata.get('operation')
            if operation=='acquire':
                if set(metadata)!={'session_protocol','operation','session_id','binding'}:raise ValueError('invalid acquire fields')
                session=metadata['session_id']
                if not isinstance(session,str)or not 0<len(session)<=128:raise ValueError('bounded client session identity required')
                current=self.service.status()
                if self._mutating or self._inflight or getattr(self.service,'active',None)is not None or current.get('busy'):
                    raise ValueError('model task busy; acquire refused')
                if current.get('status')!='ready'or not current.get('loaded'):raise ValueError('model must already be ready before acquire')
                if metadata['binding']!=binding(current):raise ValueError('model binding mismatch')
                if any(row['session_id']==session for row in self._pins.values()):raise ValueError('client session already acquired')
                if len(self._pins)>=64:raise ValueError('model session capacity reached')
                token=secrets.token_hex(16)
                self._pins[token]=dict(session_id=session,binding=deepcopy(metadata['binding']),renewed_at=self.clock(),
                    guard=_lease_guard(self.ttl,identity=metadata['binding']))
            elif operation=='renew':
                if set(metadata)!={'session_protocol','operation','token'}:raise ValueError('invalid renew fields')
                token=metadata['token'];self._check(token)['renewed_at']=self.clock()
            else:raise ValueError('unknown model session operation')
            return dict(session_protocol=PROTOCOL,token=token,ttl_s=self.ttl)

    def unload(self,metadata):
        if not isinstance(metadata,dict):raise ValueError('model session metadata must be an object')
        if 'session_protocol'not in metadata:return self._change(lambda:self.service.unload(metadata))
        with self._gate:
            self._expire()
            if 'session_protocol'in metadata:
                if (set(metadata)!={'session_protocol','operation','token'}or metadata['session_protocol']!=PROTOCOL
                        or metadata['operation']!='release'):raise ValueError('invalid release fields')
                token=metadata['token']
                if not isinstance(token,str)or not re.fullmatch('[a-f0-9]{32}',token):raise ValueError('invalid release capability')
                absent=self._pins.pop(token,None)is None
                return dict(session_protocol=PROTOCOL,released=True,model_unloaded=False,already_released=absent)

    def infer(self,payload):
        from .inference_transport import CAMS,decode_packet,encode_packet
        model_id=self.service.model_id
        metadata,arrays=decode_packet(payload,{'model_state14','model_history',*CAMS},model_id)
        wire_id=metadata.get('request_id');match=_CAPABILITY.fullmatch(wire_id)if isinstance(wire_id,str)else None
        token=match[1]if match else None
        with self._gate:
            if self._closed:raise ValueError('model service closed')
            if self._mutating:raise ValueError('model operation busy')
            self._expire()
            if token:self._check(token)
            elif self._pins:raise ValueError('model pinned; an acquired session is required')
            if getattr(self.service,'active',None)is not None:raise ValueError('model diagnostic task busy')
            self._inflight+=1
        try:
            if token:
                # No capability in worker logs, status, audit or IPC. Only the
                # authenticated request/response envelope carries the v1 prefix.
                metadata['request_id']=match[2];payload=encode_packet(metadata,arrays)
            result=self.service.infer(payload)
            if token:
                with self._gate:self._check(token)
                response,values=decode_packet(result,{'action14'},model_id)
                response['request_id']=wire_id
                result=encode_packet(response,values)
            return result
        finally:
            with self._gate:self._inflight-=1

    def tool_start(self,*args,**kwargs):
        def start():
            if not callable(getattr(self.service,'tool_start',None)):raise ValueError('diagnostic entry unavailable')
            return self.service.tool_start(*args,**kwargs)
        return self._change(start,allow_queued=True)  # Original tool owns queued-request idempotency.

    def _tool_catalog(self):
        with self._gate:
            self._expire();value=deepcopy(self.service.tool_catalog())
            if self._pins:
                value.update(busy=True,reason='model pinned; diagnostic changes refused')
                for key in ('models','sources'):
                    for row in value.get(key,[]):row['selectable']=False
            return value

    def close(self):
        # An explicit server shutdown invalidates all capabilities. Client release
        # and public /unload are separate operations and never use this path.
        with self._gate:
            self._closed=True;self._pins.clear()
            return self.service.close()


class PinnedModelClient:
    """The same client composition for workbench and DAgger; close only releases."""
    def __init__(self,client,session_id,*,auto_renew=True):
        self.client,self.session_id=client,session_id
        self.token=None;self.auto_renew=auto_renew;self.error=None;self.ttl=0.;self.renewed_at=0.
        self.model_binding=None;self.action_contract=None
        self._stop=threading.Event();self._thread=None

    def __getattr__(self,name):return getattr(self.client,name)

    def _rpc(self,path,body,*,timeout=None):
        try:return json.loads(self.client._request(path,json.dumps(body).encode(),timeout=timeout))
        except Exception as error:
            raise RuntimeError(_redact(str(error),self.token)if self.token else str(error))from None

    def acquire(self):
        if self.token is not None:raise ValueError('client already owns a model session')
        status=self.client.status()
        if status.get('session_protocol')!=PROTOCOL:raise ValueError('model session protocol version mismatch; binding refused')
        from .inference_model import action_contract
        if status.get('action_contract')!=action_contract(self.client.model_id,self.client.model_bundle):
            raise ValueError('server action contract unavailable or incompatible before binding')
        self.client.prepare(ready_only=True)
        started=time.monotonic()
        reply=self._rpc('/prepare',dict(session_protocol=PROTOCOL,operation='acquire',session_id=self.session_id,binding=binding(status)))
        token=reply.get('token');ttl=reply.get('ttl_s')
        if (reply.get('session_protocol')!=PROTOCOL or not isinstance(token,str)or not re.fullmatch('[a-f0-9]{32}',token)
                or type(ttl)not in(int,float)or not math.isfinite(ttl)or ttl<=0):raise ValueError('invalid model pin receipt')
        self.token,self.ttl=token,float(ttl);self.renewed_at=started;self.error=None;self._stop.clear()
        self._lease_guard=_lease_guard(self.ttl)
        self.model_binding=binding(status);self.action_contract=deepcopy(status.get('action_contract'))
        if self.auto_renew:
            self._thread=threading.Thread(target=self._heartbeat,daemon=True,name='model-session-renew')
            self._thread.start()
        return dict(ttl_s=self.ttl,binding=binding(status))

    def healthy(self):
        if self.token is None or self.error is not None:return False
        now=time.monotonic();self.lease_decision=self._lease_guard.evaluate(dict(rule_id='G10',signal_id=self.session_id,
            observed=dict(current_time=now,renewed_time=self.renewed_at),source_time=self.renewed_at,checked_time=now),'model_pin_client')
        return self.lease_decision['verdict']=='pass'

    def bind_control_policy(self,policy):
        self.client.bind_control_policy(policy)
        if self.ttl>0:self._lease_guard=_lease_guard(self.ttl,policy)

    def _heartbeat(self):
        while not self._stop.wait(self.ttl/3):
            try:self.renew()
            except Exception as error:self.error=str(error);return

    def renew(self):
        if self.token is None:raise ValueError('no model session')
        if self.error is not None:raise ValueError('model renewal failed; close and explicitly acquire a new session')
        started=time.monotonic()
        try:
            reply=self._rpc('/prepare',dict(session_protocol=PROTOCOL,operation='renew',token=self.token),
                timeout=min(self.ttl/3,2.,self.client.timeout))
            if reply.get('token')!=self.token or reply.get('session_protocol')!=PROTOCOL or reply.get('ttl_s')!=self.ttl:raise ValueError('model heartbeat receipt mismatch')
            self.renewed_at=started;return reply
        except Exception as error:
            self.error=str(error)
            raise

    def infer(self,request,*,prepared_input=None):
        if not self.healthy():raise self._unhealthy_error('model session is not healthy')
        token=self.token;original=request['request_id']
        if not isinstance(original,str)or not original or len(token+'_'+original)>128:raise ValueError('bounded session request identity required')
        wire_id=token+'_'+original
        try:
            call=dict(request,request_id=wire_id)
            reply=self.client.infer(call) if prepared_input is None else self.client.infer(call,prepared_input=prepared_input)
        except Exception as error:raise _session_error(error,token)from None
        if self.token!=token or not self.healthy():raise self._unhealthy_error('model session expired or replaced while inference was pending')
        if reply.get('request_id')!=wire_id:raise ValueError('model result identity mismatch')
        result=dict(_redact(reply,token),request_id=original)
        if isinstance(result.get('audit'),dict):result['audit']['session_guard']=_redact(deepcopy(self.lease_decision),token)
        return result

    def _unhealthy_error(self,message):
        error=ValueError(message);decision=getattr(self,'lease_decision',None)
        if decision is not None and decision['verdict']=='reject':error.guard_decision=deepcopy(decision)
        return error

    def infer_chunk(self,request,*,epoch,clock_domain,valid_until_source_time,prepared_input=None):
        """Model-domain chunk. The consumer must reject obsolete epochs and schedule it."""
        from .inference_model import action_contract,model_family,model_spec
        if type(epoch)is not int or epoch<0:raise ValueError('nonnegative consumer epoch required')
        if not isinstance(clock_domain,str)or not 0<len(clock_domain)<=128:
            raise ValueError('bounded observation clock domain required')
        expected=action_contract(self.client.model_id,self.client.model_bundle)
        if self.action_contract!=expected:raise ValueError('server action contract unavailable or incompatible')
        # Freeze caller metadata/arrays before a blocking call; a new observation
        # or mode must never relabel an older prediction.
        request=deepcopy(request);model_binding=deepcopy(self.model_binding)
        observed=request['observation']['source_time']
        if (type(observed)not in(int,float)or not math.isfinite(observed)
                or type(valid_until_source_time)not in(int,float)or not math.isfinite(valid_until_source_time)
                or valid_until_source_time<=observed):
            raise ValueError('finite validity deadline after observation in the same source clock required')
        result=self.infer(request) if prepared_input is None else self.infer(request,prepared_input=prepared_input)
        return dict(schema='workbench.action-chunk.v1',session_id=self.session_id,
            request_id=result['request_id'],epoch=epoch,model_binding=model_binding,
            conversion_id=self.client.conversion_id,action_contract=expected,
            observation=dict(source_time=request['observation']['source_time'],clock_domain=clock_domain,
                frame_ids=deepcopy(request['observation'].get('frame_ids',{}))),
            validity=dict(valid_until_source_time=valid_until_source_time,clock_domain=clock_domain,
                enforcement='consumer_checks_current_source_time_and_epoch'),
            task=self.client.task if model_family(self.client.model_id)=='mixed366' else
                model_spec(self.client.model_id,self.client.model_bundle)['prompt'],
            action14=result['action14'],audit=result['audit'],
            execution_authorized=False)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=min(self.ttl/3,2.,self.client.timeout)+1.)
            if self._thread.is_alive():raise RuntimeError('model renewal exit unconfirmed')
        if self.token is None:return
        try:
            reply=self._rpc('/unload',dict(session_protocol=PROTOCOL,operation='release',token=self.token),timeout=min(2.,self.client.timeout))
            if reply.get('session_protocol')!=PROTOCOL or reply.get('released')is not True or reply.get('model_unloaded')is not False:
                raise ValueError('model session release receipt mismatch')
        finally:
            self.token=None;self.client.close()
