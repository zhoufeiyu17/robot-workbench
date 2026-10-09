import unittest
import json
import tempfile
from pathlib import Path
from unittest.mock import Mock, patch
from examples.scripts.debug.joint_head_renderer_service import process_memory, gpu_memory, main


class RendererBudget(unittest.TestCase):
    def test_only_owned_processes_count_and_duplicate_gpu_rows_do_not(self):
        members, rss = process_memory('12 10 100\n13 10 200\n14 20 900', 10)
        self.assertEqual(members, {12:102400, 13:204800})
        self.assertEqual(rss, 307200)
        self.assertEqual(gpu_memory('12, 30\n12, 30\n13, 20\n14, 900', members), 50*1024**2)
        self.assertEqual(gpu_memory('', members), 0)

    def test_memory_budget_and_monitor_failure_stop_only_owned_group(self):
        for failure in (False, True):
            with self.subTest(monitor_failure=failure), tempfile.TemporaryDirectory() as directory:
                root=Path(directory)
                (root/'launch.json').write_text(json.dumps({'cpu_ids':[0], 'env':{'CUDA_VISIBLE_DEVICES':'0'},
                    'argv':['renderer','--output','unused'],'cwd':directory}))
                child=Mock(pid=4242)
                child.poll.return_value=None
                child.wait.return_value=-15
                def query(argv, **kwargs):
                    if argv[0]=='ps':
                        if failure:raise RuntimeError('monitor unavailable')
                        return '4243 4242 35000000\n9999 9999 1'
                    return '' if '--query-compute-apps=pid,used_gpu_memory' in argv else '0'
                with patch('os.sched_setaffinity', create=True), patch('signal.signal'), patch('time.sleep'), \
                     patch('subprocess.Popen',return_value=child) as start, \
                     patch('subprocess.check_output',side_effect=query), patch('os.killpg') as kill:
                    main(root)
                start.assert_called_once()
                kill.assert_called_once_with(4242, 15)
                result=json.loads((root/'status.json').read_text())
                self.assertEqual(result['status'],'recovery_required')
                self.assertEqual(result['reason'],'monitor_failure' if failure else 'memory_budget')


if __name__ == '__main__':
    unittest.main()
