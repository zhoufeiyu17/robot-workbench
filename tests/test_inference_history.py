"""Bounded disk index, old-data preservation and stable read-only pages; CPU only."""
import gc
import fcntl
import json
from pathlib import Path
import sqlite3
import sys
from tempfile import TemporaryDirectory
import tracemalloc
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_workbench.inference_history import (InferenceHistory, DATABASE, PAGE_RUNS,
    PAGE_BYTES, ENTRY_BYTES, RUN_RESULTS, entries, legacy_entries)


def rows(root, first, count, *, payload=''):
    for run in range(first, first + count):
        for camera in ('head', 'left', 'right'):
            ident = f'run-{run}-{camera}'
            yield (dict(id=ident, run_id=f'run-{run}', kind='recording', label='历史录像',
                        details=dict(evidence=payload)), root / f'run-{run}/{camera}.mp4')


class HistoryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name); self.history = InferenceHistory(self.root)

    def capture(self,run,*,done=True):
        directory=self.root/('inference-'+run)/'capture'/'capture-test';directory.mkdir(parents=True)
        (directory/'capture.json').write_text('{}')
        (directory/'binding.json').write_text(json.dumps(dict(run_id=run,business_kind='inference')))
        (directory/'status.json').write_text(json.dumps(dict(done=done,ingress_closed=done,state='complete'if done else'recording')))
        (directory/'writer.lock').touch();video=directory/'video.mp4';video.write_bytes(b'video')
        (directory/'raw-frame.bin').write_bytes(b'original')
        result=dict(id=run+'-video',run_id=run,kind='replay')
        self.history.append([(result,video)])
        return directory,result,video

    def test_recording_delete_protects_active_writers_and_scopes_the_whole_batch(self):
        first,_,_=self.capture('first');second,_,_=self.capture('second',done=False)
        with self.assertRaises(ValueError):self.history.delete_recordings(['first','second'],active=['second'])
        self.assertTrue(first.is_dir());self.assertEqual(self.history.page()['total_runs'],2)
        with self.assertRaises(ValueError):self.history.delete_recordings(['first'],active=['first'])
        with (first/'writer.lock').open('r+')as writer:
            fcntl.flock(writer,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with self.assertRaisesRegex(ValueError,'写入进程'):self.history.delete_recordings(['first'])
        for run in ('../outside','/absolute'):
            with self.assertRaises(ValueError):self.history.delete_recordings([run])
        outside=self.root/'outside';outside.mkdir();(self.root/'inference-link').symlink_to(outside,target_is_directory=True)
        with self.assertRaises(ValueError):self.history.delete_recordings(['link'])
        self.assertTrue(outside.is_dir());self.assertTrue(second.is_dir())

    def test_abandoned_unsealed_recording_is_deletable_after_writer_exit(self):
        directory,_,_=self.capture('abandoned',done=False)
        self.assertTrue(self.history.recordings()[0]['deletable'])
        self.assertEqual(self.history.delete_recordings(['abandoned'])['deleted_run_ids'],['abandoned'])
        self.assertFalse(directory.exists())

    def test_playback_pages_exclude_incomplete_and_unplayable_runs(self):
        for run,status,ready in [('complete','complete',True),('broken','incomplete',True),('pending','complete',False)]:
            row=dict(id=run,run_id=run,kind='replay',status=status,media=dict(ready=ready))
            self.history.append([(row,self.root/(run+'.mp4'))])
        page=self.history.page(playable_only=True)
        self.assertEqual(page['run_ids'],['complete']);self.assertEqual(page['total_runs'],1)
        self.assertIsNone(page['newer_cursor']);self.assertIsNone(page['older_cursor'])

    def test_deleted_recordings_cannot_return_from_late_index_updates_or_restart(self):
        first,row,video=self.capture('first');second,_,_=self.capture('second')
        result=self.history.delete_recordings(['first','second'])
        self.assertEqual(result['deleted_run_ids'],['first','second']);self.assertEqual(result['failed'],[])
        self.assertFalse(first.parent.parent.exists());self.assertFalse(second.parent.parent.exists())
        self.history.append([(row,video)])
        restored=InferenceHistory(self.root);self.assertTrue(restored.deleted('first'))
        self.assertEqual(restored.page()['total_runs'],0);self.assertIsNone(restored.media(row['id']))
        third,_,_=self.capture('third')
        with patch('robot_workbench.inference_history.shutil.rmtree',side_effect=OSError('disk busy')):
            self.assertEqual(self.history.delete_recordings(['third'])['failed'][0]['run_id'],'third')
        (third/'binding.json').unlink()
        self.assertTrue(self.history.recordings()[0]['deletable'])
        self.assertEqual(self.history.delete_recordings(['third'])['deleted_run_ids'],['third'])

    def test_pages_keep_whole_runs_and_do_not_shift_when_new_results_arrive(self):
        self.history.append(rows(self.root, 1, 30))
        latest = self.history.page()
        self.assertEqual(latest['run_ids'], [f'run-{i}' for i in range(30, 18, -1)])
        self.assertEqual(len(latest['results']), PAGE_RUNS * 3)
        self.assertEqual(latest['total_results'], 90)
        self.history.append(rows(self.root, 31, 1))
        older = self.history.page(before=latest['older_cursor'])
        self.assertEqual(older['run_ids'], [f'run-{i}' for i in range(18, 6, -1)])
        oldest = self.history.page(before=older['older_cursor'])
        self.assertEqual(oldest['run_ids'], [f'run-{i}' for i in range(6, 0, -1)])
        self.assertIsNone(oldest['older_cursor'])
        newer = self.history.page(after=older['newer_cursor'])
        self.assertEqual(newer['run_ids'], latest['run_ids'])
        self.assertEqual(len({row['id'] for p in (latest, older, oldest) for row in p['results']}), 90)
        self.assertEqual(self.history.page()['run_ids'][0], 'run-31')
        for kwargs in (dict(before=0), dict(before=True), dict(after=-1), dict(before=2**63),
                       dict(before=2, after=1), dict(before='1')):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):self.history.page(**kwargs)

    def test_page_bytes_and_per_run_limits_are_enforced_atomically(self):
        # Each task fits; multiple heavy tasks must reduce the page size without
        # cutting off any of the task's three camera results.
        self.history.append(rows(self.root, 1, 6, payload='x' * 140000))
        page = self.history.page()
        self.assertLess(page['result_bytes'], PAGE_BYTES)
        self.assertEqual(len(page['run_ids']), 1)
        self.assertEqual(len(page['results']), 3)
        self.assertEqual(page['total_results'], 18)
        before = list(entries(self.root))
        with self.assertRaisesRegex(ValueError, '256 KiB'):
            self.history.append(rows(self.root, 7, 1, payload='x' * ENTRY_BYTES))
        self.assertEqual(list(entries(self.root)), before)
        with self.assertRaisesRegex(ValueError, '8 results'):
            self.history.append((dict(id=f'too-many-{i}',run_id='too-many'),None) for i in range(RUN_RESULTS+1))
        self.assertEqual(list(entries(self.root)), before)

    def test_old_media_is_resolved_by_id_outside_the_resident_page(self):
        self.history.append(rows(self.root, 1, 50))
        video = self.root / 'run-1/head.mp4'; video.parent.mkdir(); video.write_bytes(b'old video')
        self.assertNotIn('run-1', self.history.page()['run_ids'])
        self.assertEqual(self.history.media('run-1-head').read_bytes(), b'old video')
        self.assertIsNone(self.history.media('../../outside'))
        video.unlink(); video.symlink_to(self.root / 'missing.mp4')
        self.assertIsNone(self.history.media('run-1-head'))
        for value in ('../outside.mp4', '/outside.mp4', 'a/../outside.mp4', 'a\\bad.mp4', 'a.txt'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.history.append([(dict(id=value), self.root / value)])

    def test_streaming_legacy_import_retains_original_and_restart_uses_new_index(self):
        root = self.root / 'legacy'; root.mkdir()
        old = root / 'history.json'
        data = dict(entries=[dict(result=row,path=str(path.relative_to(root))) for row,path in rows(root,1,25)],
                    schema='inference_history.v1')  # Also accept reversed key order.
        original = json.dumps(data, ensure_ascii=False, indent=2).encode(); old.write_bytes(original)
        with patch.object(Path, 'read_text', side_effect=AssertionError('no full JSON read')):
            history = InferenceHistory(root)
        self.assertEqual(history.page()['total_results'], 75)
        self.assertEqual(old.read_bytes(), original)
        history.append(rows(root, 26, 1))
        self.assertEqual(old.read_bytes(), original)
        self.assertEqual(InferenceHistory(root).page()['total_results'], 78)
        self.assertEqual(len(list(entries(root))), 78)
        old.write_bytes(original+b' ')
        with self.assertRaisesRegex(ValueError,'changed after migration'):InferenceHistory(root)
        self.assertEqual(history.page()['total_results'],78,'original DB remains intact; no automatic overwrite')

    def test_duplicate_or_broken_import_rolls_back_and_does_not_hide_original(self):
        root = self.root / 'broken'; root.mkdir(); old = root / 'history.json'
        duplicate = dict(schema='inference_history.v1',entries=[dict(result=dict(id='same'),path=None)]*2)
        old.write_text(json.dumps(duplicate)); original = old.read_bytes()
        with self.assertRaises(sqlite3.IntegrityError):InferenceHistory(root)
        self.assertEqual(old.read_bytes(), original)
        with sqlite3.connect(root / DATABASE) as db:
            self.assertEqual(db.execute('PRAGMA user_version').fetchone()[0],0)
            self.assertIsNone(db.execute("SELECT name FROM sqlite_master WHERE name='results'").fetchone())
        for bad in ('{"schema":"wrong","entries":[]}', '{"schema":"inference_history.v1","entries":[',
                    '{"schema":"inference_history.v1","entries":[],}',
                    '{"schema":"inference_history.v1","entries":[]} trailing'):
            old.write_text(bad)
            with self.subTest(bad=bad), self.assertRaises(ValueError):InferenceHistory(root)
        old.write_text(json.dumps(dict(schema='inference_history.v1',entries=[])))
        self.assertEqual(InferenceHistory(root).page()['total_results'],0)

    def test_interrupted_append_has_no_partial_task_and_connections_close(self):
        def interrupted():
            yield next(rows(self.root,1,1))
            raise OSError('injected disk/write interruption')
        with self.assertRaises(OSError):self.history.append(interrupted())
        self.assertEqual(self.history.page()['total_runs'],0)
        self.history.append(rows(self.root,1,1))
        self.assertEqual(self.history.page()['total_results'],3)
        self.assertFalse((self.root / (DATABASE+'-journal')).exists())

    def test_retained_python_index_memory_stays_flat_as_disk_history_grows(self):
        # Warm CPython's process-global path-component intern table before
        # measuring index retention; other imported suites change its resize point.
        for number in range(1,4201):Path(f'run-{number}/head.mp4').parts
        tracemalloc.start(); self.addCleanup(tracemalloc.stop)
        payload = 'metadata' * 256
        self.history.append(rows(self.root,1,200,payload=payload)); self.history.page(); gc.collect()
        baseline = tracemalloc.get_traced_memory()[0]
        retained_samples = []
        for first in (201,701,1201,1701,2201,2701,3201,3701):
            self.history.append(rows(self.root,first,500,payload=payload))
            page = self.history.page()
            self.assertLessEqual(len(page['results']), PAGE_RUNS * 3)
            del page
            gc.collect(); retained_samples.append(tracemalloc.get_traced_memory()[0] - baseline)
        # CPython pathlib may resize its global string-intern table once. Bound
        # that warm-up too, then require the final 2,000 runs to remain flat.
        self.assertLess(max(retained_samples), 2*1024**2)
        self.assertLess(max(retained_samples[-4:])-min(retained_samples[-4:]), 128*1024)
        self.assertEqual(self.history.page()['total_results'],12600)
        self.assertEqual(set(self.history.__dict__),{'root','path'})
        self.assertGreater((self.root/DATABASE).stat().st_size,10*1024**2)


if __name__ == '__main__':unittest.main()
