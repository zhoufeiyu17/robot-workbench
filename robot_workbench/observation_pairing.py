"""Nearest samples on the training converter's common 30 Hz millisecond grid."""
from collections import deque
from copy import deepcopy
import time


class PairingPending(ValueError):
    pass


class TimestampPairer:
    def __init__(self,guard=None):
        self.identity=None;self.frames={};self.feedback=deque(maxlen=32);self.origin_ns=None
        if guard is None:
            from .execution_settings import Guard,control_parameters,defaults,policy_hash,TRAINING_RGB_PAIR_ERROR_S,TRAINING_STATE_PAIR_ERROR_S
            parameters=control_parameters(defaults('inference'),business='inference')
            guard=Guard.bind(dict(revision=None,policy_hash=policy_hash(parameters),parameters=parameters),
                dict(training_rgb_pair_error_s=TRAINING_RGB_PAIR_ERROR_S,training_state_pair_error_s=TRAINING_STATE_PAIR_ERROR_S))
        self.guard=guard

    def pair(self, pairs, feedback, owner):
        identity=(owner['runtime_nonce'],owner['command_epoch'],tuple(
            (role,row['metadata']['capture_instance_id'],row['metadata']['serial'])for role,row in sorted(pairs.items())))
        if identity!=self.identity:
            self.identity=identity;self.frames={role:deque(maxlen=16)for role in pairs}
            self.feedback.clear();self.origin_ns=None
        stamp=feedback['sample_wall_ns']
        if type(stamp)is not int:raise ValueError('integer feedback wall timestamp required')
        def append(history, when, value):
            if type(when)is not int:raise ValueError('integer frame timestamp required')
            if history and when<history[-1][0]:raise ValueError('observation source clock regressed')
            if not history or when!=history[-1][0]:history.append((when,value))
            while history and history[0][0]<stamp-500_000_000:history.popleft()
        for role,row in pairs.items():
            append(self.frames[role],row['metadata']['host_receive_ns'],dict(row,metadata=deepcopy(row['metadata'])))
        append(self.feedback,stamp,deepcopy(feedback))
        return self.select(self.frames,self.feedback)

    def select(self, frames, feedback, *, target_ns=None):
        """Use the same nearest-grid policy for already bounded source histories."""
        streams=[*frames.values(),feedback]
        if any(not stream for stream in streams):raise PairingPending('等待新鲜完整观测历史')
        first=max(stream[0][0]for stream in streams);last=min(stream[-1][0]for stream in streams)
        if self.origin_ns is None:self.origin_ns=((first+999_999)//1_000_000)*1_000_000
        index=None
        if target_ns is None:
            index=(last-self.origin_ns)*30//1_000_000_000+1
            target=self.origin_ns+round(index*1000/30)*1_000_000
            while target>last:
                index-=1;target=self.origin_ns+round(index*1000/30)*1_000_000
            if index<0 or target<first:raise PairingPending('等待各路共有的30Hz观测时刻')
        else:
            if type(target_ns)is not int or target_ns<0:raise ValueError('nonnegative explicit node timestamp required')
            target=target_ns
            if target<first:raise ValueError('explicit memory observation target expired from source history')
            if target>last:raise PairingPending('waiting for real samples surrounding the explicit memory node')
        def nearest(stream):return min(stream,key=lambda row:(abs(row[0]-target),row[0]))
        selected={role:nearest(stream)for role,stream in frames.items()}
        robot_stamp,robot=nearest(feedback)
        errors={role:when-target for role,(when,_)in selected.items()}
        checked=time.time();decisions=[]
        for role,(when,_)in selected.items():
            decisions.append(self.guard.evaluate(dict(rule_id='G05',signal_id=role,
                observed=errors[role]/1e9,source_time=when/1e9,checked_time=checked),'pairing'))
        decisions.append(self.guard.evaluate(dict(rule_id='G06',signal_id='paired_state',observed=(robot_stamp-target)/1e9,
            source_time=robot_stamp/1e9,checked_time=checked),'pairing'))
        diagnostic=dict(policy=('common_30Hz_grid_nearest_ties_earlier'if target_ns is None else'explicit_node_nearest_ties_earlier'),target_wall_ns=target,
            origin_wall_ns=self.origin_ns,grid_index=index,camera_error_ns=errors,
            feedback_error_ns=robot_stamp-target,
            camera_span_ns=max(row[0]for row in selected.values())-min(row[0]for row in selected.values()),
            training_alignment_within_bounds=all(row['verdict']=='pass'for row in decisions),
            guard_decisions=decisions,decision_clock='host_wall_seconds',
            training_bounds_enforced=False)
        # Training bounds are diagnostic until reviewed; existing live age/span gates remain.
        return {role:row for role,(_,row)in selected.items()},deepcopy(robot),diagnostic
