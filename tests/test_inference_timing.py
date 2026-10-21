"""CPU checks for worker measurements and the frozen pipe metadata bridge."""
from contextlib import nullcontext
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
from robot_workbench.inference_model import CAMS, FrozenBaseClient
from test_inference import base566_bundle


class InferenceTimingTests(unittest.TestCase):
    def test_frozen_metadata_reaches_audit_without_changing_actions_or_history(self):
        action=np.arange(60*14,dtype=np.float32).reshape(60,14)
        history=np.arange(28,dtype=np.float32).reshape(2,14)
        worker=dict(make_batch_wall_s=.01,sample_actions_wall_s=.2,output_to_host_wall_s=.03,
            cpu_process_s=.05,gpu_compute_s=None,measurement='worker_host_wall_stages_cuda_async_until_output_to_host')
        frames=[({'status':'ok','infer_wall_s':.25,'timing':worker},{'action14':action}),
                ({'status':'ok','infer_wall_s':.1},{'action14':action}),
                ({'status':'ok'},{'action14':action})]
        pipe=SimpleNamespace(read_frame=lambda *_:frames.pop(0));original_read=pipe.read_frame
        client=FrozenBaseClient(base566_bundle(),log_path='/not-opened',
            adapter=SimpleNamespace(model_input=lambda _:([0.]*14,{})))
        def infer(state,images,*,seed,applied_history):
            self.assertEqual(seed,7);np.testing.assert_array_equal(applied_history,history)
            metadata,arrays=pipe.read_frame(123,456)
            self.assertEqual(metadata['status'],'ok')
            return arrays['action14']
        client.client=SimpleNamespace(infer=infer,history_evidence={'history_actions_physical':history})
        client._pipe_module=pipe
        request=dict(request_id='timing-test',observation={},seed=7,applied_history=history)
        result=client.infer(request);timing=result['audit']['timing']
        np.testing.assert_array_equal(result['action14'],action)
        self.assertEqual(result['audit']['history']['history_actions_physical'],history.tolist())
        self.assertEqual(timing['worker'],dict(worker,infer_wall_s=.25))
        self.assertEqual(client.client.last_timing,timing);self.assertIs(pipe.read_frame,original_read)
        self.assertGreaterEqual(timing['host_total_wall_s'],timing['host_pipe_roundtrip_wall_s'])
        self.assertAlmostEqual(timing['host_total_wall_s'],
            timing['host_model_input_wall_s']+timing['host_pipe_roundtrip_wall_s'])
        self.assertEqual(client.infer(request)['audit']['timing']['worker'],{'infer_wall_s':.1})
        self.assertEqual(client.infer(request)['audit']['timing']['worker'],{})

    def test_invalid_timing_is_rejected_and_private_reader_is_restored(self):
        for bad in (float('nan'),float('inf'),-1.,True,'0.2'):
            with self.subTest(value=bad):
                pipe=SimpleNamespace(read_frame=lambda *_:({'infer_wall_s':bad},{}));reader=pipe.read_frame
                client=FrozenBaseClient(base566_bundle(),log_path='/not-opened',
                    adapter=SimpleNamespace(model_input=lambda _:([0.]*14,{})))
                client._pipe_module=pipe
                client.client=SimpleNamespace(infer=lambda *_,**__:pipe.read_frame(123))
                with self.assertRaisesRegex(ValueError,'invalid worker timing'):
                    client.infer(dict(observation={},seed=0,applied_history=np.zeros((0,14))))
                self.assertIs(pipe.read_frame,reader)

    def test_close_during_query_still_restores_the_original_private_reader(self):
        pipe=SimpleNamespace(read_frame=lambda *_:({'infer_wall_s':.1},{}));reader=pipe.read_frame
        client=FrozenBaseClient(base566_bundle(),log_path='/not-opened',
            adapter=SimpleNamespace(model_input=lambda _:([0.]*14,{})))
        def infer(*args,**kwargs):
            pipe.read_frame(123);client.close()
            raise EOFError('owned worker closed during query')
        client.client=SimpleNamespace(infer=infer,close=lambda:None);client._pipe_module=pipe
        with self.assertRaisesRegex(EOFError,'closed during query'):
            client.infer(dict(observation={},seed=0,applied_history=np.zeros((0,14))))
        self.assertIsNone(client.client);self.assertIs(pipe.read_frame,reader)

    def test_worker_wall_stages_preserve_sampling_and_report_gpu_time_unknown(self):
        spec=importlib.util.spec_from_file_location('base566_timing_cpu_test',ROOT/'scripts/base566_worker.py')
        worker=importlib.util.module_from_spec(spec);spec.loader.exec_module(worker)
        class Tensor:
            def __init__(self,value):self.value=value
            def __getitem__(self,key):return Tensor(self.value[key])
            def float(self):return self
            def cpu(self):return self
            def cuda(self):return self
            def numpy(self):return self.value
        generator=SimpleNamespace(manual_seed=lambda seed:seed)
        torch=SimpleNamespace(Generator=lambda:generator,
            randn=lambda shape,generator:Tensor(np.zeros(shape)),
            no_grad=nullcontext,autocast=lambda *args,**kwargs:nullcontext(),bfloat16='fixture')
        output=np.arange(60*32,dtype=np.float32).reshape(1,60,32)
        calls=[]
        def sample_actions(device,observation,**kwargs):
            calls.append((device,observation,kwargs['num_steps'],kwargs['history_token']))
            return Tensor(output)
        value=worker.sample(np.zeros(14),np.ones((2,14)),
            {name:np.zeros((224,224,3),np.uint8) for name in CAMS})
        runtime=SimpleNamespace(make_batch=lambda *args,**kwargs:('observation',None,None,{'history_token':42},None),
            physical_actions=lambda values,stats:values+1,
            encode_samples=lambda *args,**kwargs:{'history':[value['history']]})
        with patch.dict(sys.modules,{'torch':torch}),patch.object(worker.time,'monotonic',side_effect=[10,11,13,17,21,24,29]),\
                patch.object(worker.time,'process_time',side_effect=[100,101.5]):
            result,timing=worker.predict(SimpleNamespace(sample_actions=sample_actions),value,{},None,runtime,7)
        self.assertEqual(calls,[('cuda','observation',10,42)])
        np.testing.assert_array_equal(result['action14'],output[0,:,:14]+1)
        np.testing.assert_array_equal(result['history_actions_physical'],value['history'])
        np.testing.assert_array_equal(result['history_actions_normalized'],value['history'])
        self.assertEqual([timing[key] for key in ('make_batch_wall_s','noise_wall_s','sample_actions_wall_s',
            'output_to_host_wall_s','physical_actions_wall_s','history_encode_wall_s')],[1,2,4,4,3,5])
        self.assertEqual(timing['cpu_process_s'],1.5);self.assertIsNone(timing['gpu_compute_s'])
        self.assertIn('cuda_async',timing['measurement'])


if __name__=='__main__':unittest.main()
