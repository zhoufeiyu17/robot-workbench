"""Read-only live-parent identity and deadline checks; no GPU or control requests."""
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import time
import unittest

ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'scripts'),str(ROOT),str(ROOT/'rule_based_workbench')]
from setup_resident_collection import live_parent,verify_parent


class SetupResidentCollectionTest(unittest.TestCase):
    def test_parent_is_old_frozen_code_and_deadline_cannot_extend(self):
        with TemporaryDirectory() as temp:
            root=Path(temp).resolve();proc=root/'proc'/'17';proc.mkdir(parents=True)
            script=root/'old-frozen-code'/'serve_physx_device.py';script.parent.mkdir();script.write_text('# frozen original parent\n')
            base=root/'base.json';base.write_text('{"pipeline_config":{}}')
            original=root/'original-contract.json';original.write_text('{"identity":{"session_id":"same-session"}}')
            argv=['python',str(script),'--output',str(root),'--base',str(base),'--collection-contract',str(original)]
            (proc/'cmdline').write_bytes(b'\0'.join(v.encode() for v in argv)+b'\0')
            (proc/'stat').write_text('17 (parent) '+' '.join(['S']+['0']*18+['123']))
            deadline=time.time_ns()+180_000_000_000
            def save(name,value):
                path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value))
            save('service.json',dict(pid=17,environment='fake',resident_deadline_wall_ns=deadline))
            save('run.json',dict(pid=17,resident_deadline_wall_ns=deadline,base_sha256=hashlib.sha256(base.read_bytes()).hexdigest()))
            save('calibration-config.json',{});save('registration.json',{})
            save('calibrations/hardware/active/manifest.json',dict(simulation_only=True,robot_source='simulated',head={'id':'installed-head'},wrist={'id':'installed-wrist'}))
            contract=dict(identity={'session_id':'same-session'},resources={'max_workers':4},
                collection_setup=dict(min_free_memory_mib=16384,plant_max_memory_mib=12288))
            before={str(path):path.read_bytes() for path in root.rglob('*') if path.is_file()}
            parent=live_parent(root,contract,proc_root=root/'proc');verify_parent(parent,proc_root=root/'proc')
            self.assertEqual(parent['execution']['plant']['script'],str(script))
            self.assertNotEqual(Path(parent['execution']['plant']['script']).parent,ROOT/'scripts')
            self.assertEqual(parent['installed']['head_id'],'installed-head');self.assertEqual(parent['deadline'],deadline)
            self.assertEqual(before,{str(path):path.read_bytes() for path in root.rglob('*') if path.is_file()})
            with self.assertRaises(ValueError):live_parent(root,contract,proc_root=root/'proc',now_ns=deadline-30_000_000_000)
            save('service.json',dict(pid=17,environment='fake',resident_deadline_wall_ns=deadline+600_000_000_000))
            with self.assertRaises(ValueError):live_parent(root,contract,proc_root=root/'proc')
            with self.assertRaises(ValueError):verify_parent(parent,proc_root=root/'proc')


if __name__=='__main__':unittest.main()
