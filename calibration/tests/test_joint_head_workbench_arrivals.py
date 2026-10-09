import unittest
from pathlib import Path
from types import SimpleNamespace
from joint_head_workbench import Workbench


class ArrivalAssociationTest(unittest.TestCase):
    def test_only_current_batch_events_are_recorded_once(self):
        rows=[]
        host=SimpleNamespace(output=Path('/data'),arrival_store=lambda source:SimpleNamespace(record=rows.append))
        pending={'preview':{'owner_pid':10,'command_epoch':2,'plans':[{'sha256':'current'}]},'workflow':{'kind':'head'}}
        event={'event_id':'new','owner_pid':10,'command_epoch':2,'plan_sha256':'current','status':'success'}
        owner={'arrival_events':[dict(event,event_id='old',command_epoch=1),
                                 dict(event,event_id='other',plan_sha256='other'),
                                 dict(event,event_id='failed',status='failed'),event]}
        for _ in range(2):Workbench.record_arrivals(host,owner,pending,Path('/data/batch/capture'),'physical')
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['batch_id'],'batch/capture')
        self.assertEqual(rows[0]['workflow'],'head')
        self.assertEqual(rows[0]['status'],'success')
