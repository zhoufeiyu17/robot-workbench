"""Actually fork from an Isaac-like polluted parent; no GPU/runtime installation."""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

from launch_environment import HOST_PYTHON,clean_environment,clean_command,owned_command
from run_pipeline import python_command,transport
from local_execution import TARGET

WB=Path(__file__).resolve().parent


class LaunchEnvironmentTest(unittest.TestCase):
    def test_polluted_parent_forks_clean_supervisor_and_preserves_declared_worker_runtime(self):
        with TemporaryDirectory() as temp:
            root=Path(temp);poison=root/'isaac-parent';poison.mkdir()
            (poison/'re.py').write_text('raise RuntimeError("parent Python path leaked")\n')
            (poison/'python3').write_text('#!/bin/sh\nexit 93\n');(poison/'python3').chmod(0o755)
            declared=root/'declared-deps';declared.mkdir();(declared/'declared_module.py').write_text('value="worker-only"\n')
            worker=root/'worker.py'
            worker.write_text('import re,subprocess,sys,os,json,sysconfig,declared_module\n'
                'print(json.dumps(dict(executable=sys.executable,prefix=sys.prefix,stdlib=sysconfig.get_path("stdlib"),'
                're=re.__file__,subprocess=subprocess.__file__,home=os.environ.get("PYTHONHOME"),'
                'path=os.environ.get("PYTHONPATH"),ld=os.environ.get("LD_LIBRARY_PATH"),declared=declared_module.value)))\n')
            report='import re,subprocess,sys,sysconfig,json,os; print(json.dumps(dict(executable=sys.executable,stdlib=sysconfig.get_path("stdlib"),re=re.__file__,subprocess=subprocess.__file__,home=os.environ.get("PYTHONHOME"),path=os.environ.get("PYTHONPATH"),ld=os.environ.get("LD_LIBRARY_PATH"))))'
            expected=json.loads(subprocess.check_output(clean_command([HOST_PYTHON,'-I','-c',report]),text=True))
            overrides=dict(PYTHONPATH=str(declared),LD_LIBRARY_PATH='/declared/worker/cuda-lib')
            worker_command=clean_command([sys.executable,str(worker)],overrides)
            expected_worker=json.loads(subprocess.check_output(worker_command,text=True))
            script=root/'serve_physx_device.py';script.write_text('# unused CPU contract fixture\n')
            execution=dict(TARGET,session_id='cpu-pollution',max_processes=4,min_free_memory_mib=1,
                plant=dict(session_id='cpu-pollution',pid=1,process_start_ticks='1',scope=str(root),script=str(script),
                    script_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),
                    argv=['python3',str(script),'--output',str(root),'--gpu',str(TARGET['gpu_index'])],max_memory_mib=1))
            commands=[shlex.split(python_command(report)),owned_command(WB,worker_command,5),
                transport({'execution':execution,'workbench_code':str(WB)},'dsw-scene',shlex.join(worker_command),timeout=5)]
            payload=root/'commands.json';payload.write_text(json.dumps(commands))
            dirty=dict(PYTHONHOME=str(poison),PYTHONPATH=str(poison),LD_LIBRARY_PATH=str(poison),
                CARB_APP_PATH=str(poison),ISAAC_PATH=str(poison),PATH=str(poison)+':/usr/bin:/bin')
            # Bootstrap a real parent first, then mutate only its environment just
            # as an initialized Isaac runtime does before launching other workers.
            parent='''import json,os,subprocess,sys
commands=json.load(open(sys.argv[1]));os.environ.update(json.loads(sys.argv[2]));before=dict(os.environ)
bad=subprocess.run([sys.argv[3],'-c','import re,subprocess'],capture_output=True)
assert bad.returncode!=0,'polluted baseline did not exercise a real Python startup failure'
results=[json.loads(subprocess.check_output(command,text=True)) for command in commands]
assert dict(os.environ)==before,'launch helper modified the parent runtime environment'
print(json.dumps(results))
'''
            result=json.loads(subprocess.check_output([sys.executable,'-I','-c',parent,str(payload),json.dumps(dirty),HOST_PYTHON],text=True))
            self.assertEqual(result[0],expected)
            self.assertEqual(result[1],expected_worker);self.assertEqual(result[2],expected_worker)
            self.assertEqual(result[1]['executable'],sys.executable)
            self.assertEqual(result[1]['declared'],'worker-only');self.assertIsNone(result[1]['home'])
            for row in result:
                self.assertTrue(Path(row['re']).is_relative_to(row['stdlib']))
                self.assertTrue(Path(row['subprocess']).is_relative_to(row['stdlib']))

    def test_cleanup_only_removes_parent_runtime_and_reapplies_declared_fields(self):
        parent=dict(PYTHONHOME='/isaac/kit/python',PYTHONPATH='/foreign/stdlib',LD_LIBRARY_PATH='/foreign/lib',
            CUDA_VISIBLE_DEVICES='1',ROBOT_WORKBENCH_SESSION_ID='owned',CUSTOM='preserved',PATH='/foreign/bin')
        original=dict(parent)
        value=clean_environment({'PYTHONPATH':'/declared/deps','LD_LIBRARY_PATH':'/declared/cuda'},parent)
        self.assertEqual(parent,original);self.assertNotIn('PYTHONHOME',value)
        self.assertEqual(value['CUDA_VISIBLE_DEVICES'],'1');self.assertEqual(value['ROBOT_WORKBENCH_SESSION_ID'],'owned')
        self.assertEqual(value['LD_LIBRARY_PATH'],'/declared/cuda');self.assertEqual(value['PYTHONPATH'],'/declared/deps')


if __name__=='__main__':unittest.main()
