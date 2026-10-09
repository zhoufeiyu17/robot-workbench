"""Frozen service adapter GPU boundary; no model imports, CUDA or remote access."""
from pathlib import Path
import os
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'detector_adapter'))
from isaac_detection_service import serve


class AdapterTest(unittest.TestCase):
    def test_explicit_single_mask_still_requires_private_service_configuration(self):
        for gpu in ('0','1','0,1','-1',''):
            with self.subTest(gpu=gpu),patch.dict(os.environ,{'CUDA_VISIBLE_DEVICES':gpu}),patch('sys.platform','linux'):
                error=ValueError if gpu in ('0','1') else RuntimeError
                with self.assertRaises(error):serve(SimpleNamespace(service_root=None),None)


if __name__=='__main__':unittest.main()
