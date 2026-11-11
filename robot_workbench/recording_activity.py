"""Recorded MODEL/TEACH intervals and their gap-free derived time axis."""
from bisect import bisect_right
from copy import deepcopy
import math

POLICY='model_teach_compact_v1'
SCHEMA='workbench.recording-activity.v1'
MODES=frozenset(('MODEL','TEACH'))


class ActivityTimeline:
    def __init__(self,run_id,clock_id):
        self.run_id,self.clock_id=run_id,clock_id
        self.intervals=[];self.current=None;self.latest_source_ns=0

    def observe(self,event,data,reference,run_id):
        if reference.source_clock_id!=self.clock_id:raise ValueError('activity control clock changed')
        self.latest_source_ns=max(self.latest_source_ns,reference.source_time_ns)
        if not isinstance(data,dict)or self.run_id not in (run_id,data.get('session_id'),data.get('execution_instance_id')):
            return None
        target=None;request=data.get('request_id','')
        if event=='inference_mode_transition':
            if (data.get('op')=='normal_mode'and data.get('accepted')is True
                    and data.get('status')=='accepted'and not data.get('no_op')):
                target=data.get('requested_mode')
            elif data.get('status')in('failed','rejected','cancelled')and self.current and self.current['request_id']==request:
                target='HOLD'
        elif event in ('inference_finish','inference_episode_stopped','stop','reference_cancelled'):
            target='HOLD'
        if target is None:return None
        if target not in (*MODES,'HOLD','HOME','END','DISCARD'):
            raise ValueError('unknown recording activity transition')
        if target not in MODES and self.current is None:return None
        wall=data.get('capture_wall_ns',data.get('finished_wall_ns'))
        if type(wall)is not int or wall<=0:raise ValueError('activity transition requires original wall/source clock pair')
        if target in MODES:
            if not isinstance(request,str)or not 0<len(request)<=128:raise ValueError('bounded activity request identity required')
            if len(self.intervals)>=256:raise ValueError('recorded activity interval bound reached')
        stamp=reference.source_time_ns
        self.close(stamp,wall)
        if target not in MODES:return None
        start=sum(row['end_source_ns']-row['start_source_ns']for row in self.intervals)
        self.current=dict(mode=target,request_id=request,start_source_ns=stamp,start_wall_ns=wall,
            end_source_ns=None,end_wall_ns=None,effective_start_ns=start)
        self.intervals.append(self.current)
        return deepcopy(self.current)

    def close(self,source_ns,wall_ns):
        if self.current is None:return
        row=self.current
        if source_ns<row['start_source_ns']or wall_ns<row['start_wall_ns']:
            raise ValueError('activity end precedes its start')
        row.update(end_source_ns=source_ns,end_wall_ns=wall_ns)
        if source_ns==row['start_source_ns']:self.intervals.pop()
        self.current=None

    def sample_group(self,clock_id,stamp):
        kind='source_ns'if clock_id==self.clock_id else'wall_ns'if clock_id=='host_wall'else None
        if kind is None:raise ValueError('source clock has no recorded activity mapping')
        for row in reversed(self.intervals):
            if stamp>=row['start_'+kind]:
                end=row['end_'+kind]
                return row['request_id']if end is None or stamp<end else None
        return None

    def finish(self,source_ns):
        if self.current is not None:
            row=self.current
            self.close(source_ns,row['start_wall_ns']+source_ns-row['start_source_ns'])

    def summary(self):
        duration=sum((row['end_source_ns']if row['end_source_ns']is not None else self.latest_source_ns)-row['start_source_ns']
            for row in self.intervals)
        return dict(schema=SCHEMA,active_mode=self.current['mode']if self.current else None,
            effective_duration_s=max(0,duration)/1e9,interval_count=len(self.intervals),
            started_wall_ns=self.intervals[0]['start_wall_ns']if self.intervals else None)

    def manifest(self):
        if self.current is not None:raise ValueError('activity interval still open')
        return dict(schema=SCHEMA,policy=POLICY,control_clock_id=self.clock_id,intervals=deepcopy(self.intervals),
            effective_duration_ns=sum(row['end_source_ns']-row['start_source_ns']for row in self.intervals))


class ActiveTimeAxis:
    """Source timestamps remain immutable; only derived output time is compacted."""
    def __init__(self,value):
        if (not isinstance(value,dict)or set(value)!={'schema','policy','control_clock_id','intervals','effective_duration_ns'}
                or value['schema']!=SCHEMA or value['policy']!=POLICY
                or not isinstance(value['control_clock_id'],str)or not value['control_clock_id']
                or not isinstance(value['intervals'],list)or len(value['intervals'])>256):
            raise ValueError('declared compact activity timeline required')
        self.clock_id=value['control_clock_id'];self.rows=deepcopy(value['intervals']);total=0;previous=None
        fields={'mode','request_id','start_source_ns','end_source_ns','start_wall_ns','end_wall_ns','effective_start_ns'}
        for row in self.rows:
            if (set(row)!=fields or row['mode']not in MODES or not isinstance(row['request_id'],str)
                    or any(type(row[k])is not int or row[k]<0 for k in fields-{'mode','request_id'})
                    or row['end_source_ns']<=row['start_source_ns']or row['end_wall_ns']<=row['start_wall_ns']
                    or row['effective_start_ns']!=total
                    or previous and any(row['start_'+k]<previous['end_'+k]for k in ('source_ns','wall_ns'))):
                raise ValueError('ordered, closed and contiguous effective activity intervals required')
            total+=row['end_source_ns']-row['start_source_ns'];previous=row
        if type(value['effective_duration_ns'])is not int or value['effective_duration_ns']!=total:
            raise ValueError('activity duration differs from recorded intervals')
        self.duration_ns=total
        self.starts={kind:[row['start_'+kind]for row in self.rows]for kind in ('source_ns','wall_ns')}

    def offset_ns(self,clock_id,source_ns):
        kind='source_ns'if clock_id==self.clock_id else'wall_ns'if clock_id=='host_wall'else None
        if kind is None:raise ValueError('source clock has no recorded activity mapping')
        index=bisect_right(self.starts[kind],source_ns)-1
        if index<0:return None
        row=self.rows[index];start,end=row['start_'+kind],row['end_'+kind]
        if not start<=source_ns<end:return None
        return row['effective_start_ns']+(source_ns-start)*(row['end_source_ns']-row['start_source_ns'])//(end-start)


def activity_summary(value):
    if (not isinstance(value,dict)or set(value)!={'schema','active_mode','effective_duration_s','interval_count','started_wall_ns'}
            or value['schema']!=SCHEMA or value['active_mode']not in(None,*MODES)
            or type(value['effective_duration_s'])not in(int,float)or not math.isfinite(value['effective_duration_s'])or value['effective_duration_s']<0
            or type(value['interval_count'])is not int or not 0<=value['interval_count']<=256
            or value['started_wall_ns']is not None and(type(value['started_wall_ns'])is not int or value['started_wall_ns']<=0)):
        raise ValueError('bounded recording activity health required')
    return value


def inference_replay_segments(manifest):
    """Project recorded configuration changes onto the compact video clock."""
    import json
    binding=manifest.get('binding')or{}
    context=binding.get('execution_context')or{}
    saved=context.get('inference_configuration')or{}
    execution=((binding.get('control_policy')or{}).get('parameters')or{}).get('execution')or{}
    base={key:saved.get(key)for key in ('model_id','model_label','backend','denoise_steps','execute_steps')}
    if base['execute_steps']is None:base['execute_steps']=execution.get('execute_steps')
    points=[]
    for payload in (manifest.get('model_inputs')or{}).values():
        metadata=json.loads(payload.get('metadata_json')or'{}')
        model=(metadata.get('binding')or{}).get('model_binding')or{}
        if not model.get('model_id'):continue
        worker=model.get('worker_identity')or{}
        stamp=(payload.get('observation')or{}).get('source_time_ns')
        if type(stamp)is not int:continue
        configuration=dict(base,model_id=model['model_id'],backend=model.get('backend'),
                           denoise_steps=worker.get('inference_steps'))
        if any(configuration[key]!=base[key]for key in ('model_id','backend','denoise_steps')):
            configuration['model_label']=None
        points.append((stamp,configuration))
    points.sort(key=lambda row:row[0])
    activity=manifest.get('recording_activity')
    if activity is None:
        return dict(schema='workbench.inference-replay.v1',configuration=points[0][1]if points else base,
                    segments=[],reason='这条旧录像未记录模式片段时间轴')
    axis=ActiveTimeAxis(activity)
    segments=[]
    for interval in axis.rows:
        start=interval['start_source_ns'];end=interval['end_source_ns']
        prior=[configuration for stamp,configuration in points if stamp<=start]
        inside=[configuration for stamp,configuration in points if start<=stamp<end]
        current=(inside[0]if interval['mode']=='MODEL'and inside else
                 prior[-1]if prior else points[0][1]if points else base)
        changes=[(start,current)]
        if interval['mode']=='MODEL':
            for stamp,configuration in points:
                if start<stamp<end and configuration!=changes[-1][1]:changes.append((stamp,configuration))
        for index,(stamp,configuration)in enumerate(changes):
            stop=changes[index+1][0]if index+1<len(changes)else end
            if len(segments)>=512:raise ValueError('recorded configuration timeline exceeds512 segments')
            segments.append(dict(configuration,mode=interval['mode'],
                start_s=(interval['effective_start_ns']+stamp-start)/1e9,
                end_s=(interval['effective_start_ns']+stop-start)/1e9,
                start_wall_ns=interval['start_wall_ns']+stamp-start,
                end_wall_ns=interval['start_wall_ns']+stop-start))
    return dict(schema='workbench.inference-replay.v1',configuration=points[0][1]if points else base,
                segments=segments,duration_s=axis.duration_ns/1e9)
