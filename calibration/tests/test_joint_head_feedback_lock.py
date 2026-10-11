"""Page lock contention must not pause the capture feedback stream."""
from collections import deque
import threading
import time
import unittest
from types import SimpleNamespace
from joint_head_workbench import Workbench


class FeedbackLockTest(unittest.TestCase):
    def test_monitor_and_capture_snapshot_advance_while_page_lock_is_held(self):
        app=Workbench.__new__(Workbench)
        app.lock=threading.RLock();app.hardware_history_lock=threading.Lock()
        app.hardware_poll_lock=threading.Lock()
        app.hardware_progress=dict(source=None,sdk=None,advanced=[0.,0.])
        app.hardware_history=deque(maxlen=180);app.hardware_error=None
        stop=threading.Event()
        def read():
            if stop.is_set():raise SystemExit
            now=time.monotonic()
            return {'reachable':True,'feedback':{'valid':True,'host_sample_age_s':0},
                    'raw_owner':{'pid':1,'feedback_healthy':True,'calibration_feedback':{
                        'qpos':[0.]*14,'sdk_timestamps':[now,now],'sample_wall_ns':time.time_ns()}}}
        app.hardware_reader=SimpleNamespace(read_state=read)
        worker=threading.Thread(target=app.monitor_hardware,daemon=True)
        try:
            with app.lock:
                worker.start()
                deadline=time.monotonic()+1
                while len(app.feedback_history_snapshot())<5 and time.monotonic()<deadline:time.sleep(.01)
                history=app.feedback_history_snapshot()
                self.assertGreaterEqual(len(history),5)
                self.assertGreater(history[-1]['owner']['calibration_feedback']['sample_wall_ns'],
                                   history[0]['owner']['calibration_feedback']['sample_wall_ns'])
        finally:
            stop.set();worker.join(1)
        self.assertFalse(worker.is_alive())
