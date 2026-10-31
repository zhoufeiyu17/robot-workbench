"""A busy boundary must retain changing-call evidence without admitting an action."""
import threading,time,unittest
from types import SimpleNamespace
from robot_workbench.runtime import Runtime

class PreviewBoundaryEvidenceTest(unittest.TestCase):
    def test_continuous_ticks_are_distinguishable_from_one_stuck_sdk_call(self):
        r=Runtime.__new__(Runtime);r.lock=threading.RLock();r.tick_lock=threading.Lock()
        r.tick_idle=threading.Condition(r.lock);r.calls={1:{'id':1}};r.call_sequence=1;r.last_return=None
        events=[];r.owner=SimpleNamespace(event=lambda kind,**v:events.append((kind,v)))
        done=threading.Event();r.tick_lock.acquire()
        def calls():
            while not done.wait(.002):
                with r.lock:
                    r.call_sequence+=1;r.last_return={'id':r.call_sequence-1}
                    r.calls={r.call_sequence:{'id':r.call_sequence}};r.tick_idle.notify_all()
        worker=threading.Thread(target=calls);worker.start()
        try:
            with self.assertRaisesRegex(ValueError,'preview boundary'):
                with r._preview_boundary():self.fail('busy boundary admitted action')
        finally:done.set();worker.join();r.tick_lock.release()
        self.assertEqual(len(events),1);kind,e=events[0]
        self.assertEqual(kind,'preview_boundary_timeout');self.assertGreater(e['call_sequence_after'],e['call_sequence_before'])
        self.assertNotEqual(e['in_flight_before'],e['in_flight_after']);self.assertTrue(e['tick_locked'])
        r.calls.clear();self.assertTrue(e['in_flight_after'],'evidence must be detached')
        with r._preview_boundary():pass
        self.assertEqual(len(events),1)

if __name__=='__main__':unittest.main()
