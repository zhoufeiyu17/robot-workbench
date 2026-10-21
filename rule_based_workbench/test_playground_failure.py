"""Reset failure evidence must not advance a simulator or mask the original failure."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import json
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock,patch

sys.path[:0]=[str(Path(__file__).resolve().parent),str(Path(__file__).resolve().parents[1])]
from playground_sim import reset_failure_context,main


class ResetFailureEvidenceTest(unittest.TestCase):
    def test_existing_contact_evidence_is_bounded_and_detached(self):
        contact=SimpleNamespace(actors=('cup','table'),shapes=('cup/collider','table/collider'),
            position=(0.,0.,.78),normal=(0.,0.,1.),impulse=(0.,0.,.1),separation_m=-.001)
        evaluator=SimpleNamespace(last_audit={'physics_step':3,'support_force_n':24.,'table_contact':True},
            history={'table_separated':False})
        runtime=SimpleNamespace(evaluator=evaluator,latest_contacts=[contact]*70,
            read=Mock(),_advance=Mock())
        before=deepcopy(evaluator.last_audit)
        result=reset_failure_context(SimpleNamespace(_m2_runtime=runtime))
        self.assertEqual(result['status'],'recorded');self.assertEqual(result['physics_step'],3)
        self.assertEqual(len(result['contacts']),64);self.assertEqual(result['contact_count'],70)
        self.assertTrue(result['contacts_truncated'])
        result['audit']['support_force_n']=0
        self.assertEqual(evaluator.last_audit,before)
        runtime.read.assert_not_called();runtime._advance.assert_not_called()

    def test_missing_or_nonfinite_diagnostics_remain_unavailable(self):
        self.assertEqual(reset_failure_context(None)['reason'],'runtime_not_created')
        runtime=SimpleNamespace(evaluator=SimpleNamespace(last_audit={'force':float('nan')},history={}),
            latest_contacts=[])
        result=reset_failure_context(SimpleNamespace(_m2_runtime=runtime))
        self.assertEqual(result['status'],'unavailable')
        self.assertEqual(result['reason'],'diagnostic_snapshot_failed')

    def test_outer_failure_does_not_overwrite_setup_evidence(self):
        with TemporaryDirectory() as temporary:
            root=Path(temporary);source=root/'input.json';source.write_text('{}')
            args=SimpleNamespace(source=source,suite=source,hardware_binding=source,plan=None,
                mode='geometry-inputs',finger_envelope=source,finger_envelope_sha256='fixture',output=root/'output')
            saved=dict(status='failed',phase='setup',error='reset rejected',reset_diagnostics={'physics_step':3})
            def fail(*args_):
                (args.output/'result.json').write_text(json.dumps(saved))
                raise ValueError('reset rejected')
            with patch('local_execution.register_auxiliary'),patch('playground_sim.validate_inputs'),\
                    patch('playground_sim.file_sha256',return_value='fixture'),\
                    patch('playground_sim.setup',side_effect=fail),patch('playground_sim.close_runtime'),\
                    patch('playground_sim.traceback.print_exc'):
                with self.assertRaisesRegex(ValueError,'reset rejected'):main(args)
            self.assertEqual(json.loads((args.output/'result.json').read_text()),saved)


if __name__=='__main__':unittest.main()
