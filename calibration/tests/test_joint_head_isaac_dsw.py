"""Remote worker validation without SSH or GPU calls."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'examples/scripts/debug/joint_head_isaac_dsw.py'
spec = importlib.util.spec_from_file_location('dsw_worker', SCRIPT)
worker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(worker)

class WorkerTest(unittest.TestCase):
    def test_config_validation_and_gpu_guard_before_mutation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            plan = root / 'plan.json'
            plan.write_text('{"stations":[{"id":"target"}]}')
            config = root / 'config.json'
            value = json.loads((SCRIPT.parents[3] / 'configs/dsw.example.json').read_text())
            value.update(ssh_key=str(plan), runner_sha256='a' * 64)
            argv = ['worker', '--config', str(config), '--plan', str(plan), '--output', str(root / 'output')]
            for field, invalid in [('host','-oProxyCommand=bad'), ('port',True), ('root','/srv/task;bad'), ('runner_sha256','bad')]:
                config.write_text(json.dumps({**value,field:invalid}))
                with patch.object(sys,'argv',argv), patch.object(worker.subprocess,'check_output') as call:
                    with self.assertRaises(ValueError): worker.main()
                    call.assert_not_called()
            config.write_text(json.dumps(value))
            with patch.object(sys,'argv',argv), patch.object(worker.subprocess,'check_output',return_value='123\n') as call:
                with self.assertRaisesRegex(RuntimeError,'GPU process'): worker.main()
                self.assertEqual(call.call_count,1)
                self.assertIn('nvidia-smi',call.call_args.args[0][-1])
            self.assertFalse((root/'output').exists())

if __name__ == '__main__':
    unittest.main()
