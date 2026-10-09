"""Real file concurrency and failed publication at the shared evidence boundary."""
import json
import os
from pathlib import Path
import stat
import sys
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import probe_owner_physx


class AtomicEvidenceSaveTest(unittest.TestCase):
    def test_partial_sibling_never_replaces_complete_json_and_write_failure_keeps_old(self):
        for fail in (False,True):
            with self.subTest(fail=fail),TemporaryDirectory() as temp:
                path=Path(temp)/'collection-setup-status.json'
                old=dict(status='ready',data=['完整状态']*200)
                new=dict(status='initializing_detector',data=['新状态']*500)
                probe_owner_physx.save(path,old);path.chmod(0o640)
                half_written=threading.Event();release=threading.Event();errors=[]
                original_fdopen=os.fdopen
                class SlowWriter:
                    def __init__(self,*args,**kwargs):self.stream=original_fdopen(*args,**kwargs)
                    def __enter__(self):return self
                    def __exit__(self,*args):return self.stream.__exit__(*args)
                    def fileno(self):return self.stream.fileno()
                    def write(self,text):
                        middle=len(text)//2
                        self.stream.write(text[:middle]);self.stream.flush();half_written.set()
                        if not release.wait(5):raise TimeoutError('reader did not inspect paused write')
                        if fail:raise OSError('injected partial write failure')
                        self.stream.write(text[middle:])
                def writer():
                    try:probe_owner_physx.save(path,new)
                    except Exception as error:errors.append(error)
                with patch.object(probe_owner_physx.os,'fdopen',SlowWriter):
                    thread=threading.Thread(target=writer);thread.start()
                    try:
                        self.assertTrue(half_written.wait(5))
                        for _ in range(100):self.assertEqual(json.loads(path.read_bytes()),old)
                    finally:release.set();thread.join(timeout=5)
                self.assertFalse(thread.is_alive())
                self.assertEqual(json.loads(path.read_bytes()),old if fail else new)
                self.assertEqual(stat.S_IMODE(path.stat().st_mode),0o640)
                self.assertEqual(list(Path(temp).iterdir()),[path])
                self.assertEqual(len(errors),int(fail))
                if fail:self.assertIsInstance(errors[0],OSError)

    def test_serialization_and_replace_failure_preserve_file_and_new_mode_respects_umask(self):
        with TemporaryDirectory() as temp:
            path=Path(temp)/'state.json';reference=Path(temp)/'normal-new-file.json'
            reference.write_text('{}')
            probe_owner_physx.save(path,dict(status='ready'))
            self.assertEqual(stat.S_IMODE(path.stat().st_mode),stat.S_IMODE(reference.stat().st_mode))
            before=path.read_bytes()
            with self.assertRaises(ValueError):probe_owner_physx.save(path,dict(value=float('nan')))
            self.assertEqual(path.read_bytes(),before)
            with patch.object(probe_owner_physx.os,'replace',side_effect=OSError('replace failed')):
                with self.assertRaisesRegex(OSError,'replace failed'):probe_owner_physx.save(path,dict(status='changed'))
            self.assertEqual(path.read_bytes(),before)
            self.assertEqual(set(Path(temp).iterdir()),{path,reference})


if __name__=='__main__':unittest.main()
