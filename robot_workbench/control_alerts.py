"""Acknowledged stop events and current heartbeat health are independent facts."""
from copy import deepcopy
import threading
import time


class ControlAlerts:
    def __init__(self, initial, save):
        self.lock=threading.Lock();self.save=save
        self.events=deepcopy(initial.get('events',{}))

    def observe(self, protection):
        event=(protection or {}).get('event')
        if not isinstance(event,dict) or not isinstance(event.get('id'),str):return
        key=event['id']
        if not key or len(key)>128:raise ValueError('invalid control protection event identity')
        with self.lock:
            old=self.events.get(key,{})
            row={**deepcopy(event),'observed_wall_ns':old.get('observed_wall_ns',time.time_ns()),
                 'acknowledged_wall_ns':old.get('acknowledged_wall_ns')}
            if row!=old:
                updated={**self.events,key:row}
                acknowledged=[k for k,v in updated.items() if v.get('acknowledged_wall_ns')]
                for k in acknowledged[:-32]:updated.pop(k)
                self.save(dict(events=updated));self.events=updated

    def acknowledge(self, event_id):
        with self.lock:
            if not isinstance(event_id,str) or event_id not in self.events:raise ValueError('急停事件不存在，请刷新状态')
            row=deepcopy(self.events[event_id])
            if not row.get('acknowledged_wall_ns'):
                row['acknowledged_wall_ns']=time.time_ns()
                updated={**self.events,event_id:row}
                self.save(dict(events=updated));self.events=updated
            return dict(acknowledged=True,event=deepcopy(row),control_recovered=False,motion_resumed=False)

    def snapshot(self, protection, *, reachable, connected=False):
        with self.lock:
            warnings=[deepcopy(v) for v in self.events.values() if not v.get('acknowledged_wall_ns')]
        current=deepcopy(protection) if isinstance(protection,dict) else dict(
            heartbeat=dict(status='not_monitored' if reachable and connected is False else 'unknown',age_s=None,monitored=False),
            active_anomaly=False if reachable and connected is False else None,
            reason=None if reachable and connected is False else '没有取得当前控制心跳观测')
        if not reachable:
            current.update(active_anomaly=None,reason='业务状态不可达，无法核验控制心跳')
            current['heartbeat']=dict(status='unknown',age_s=None,monitored=False)
        return dict(schema='workbench.control_alerts.v1',warnings=warnings,current=current)
