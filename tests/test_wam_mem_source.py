"""Optional tiny CPU numerical oracle against the frozen training Policy."""
from copy import deepcopy
import os
from pathlib import Path
import sys
import unittest

import numpy as np

from robot_workbench import wam_mem as memory


@unittest.skipUnless(os.environ.get('WAM_MEMORY_ORACLE_SOURCE'), 'frozen training source CPU environment required')
class FrozenPolicyParity(unittest.TestCase):
    def test_real_context_commit_and_readonly_prediction_match_training_policy(self):
        for steps in (16,32):
            with self.subTest(execute_actions=steps):self.check_policy(steps)

    def check_policy(self,steps):
        root=Path(os.environ['WAM_MEMORY_ORACLE_SOURCE'])
        sys.path.insert(0,str(root/'src' if (root/'src').is_dir() else root))
        import torch
        from world_memory.config import Config
        from world_memory.inference import Policy
        from world_memory.models.world import WorldActionModel
        from world_memory.normalization import Normalization

        torch.set_num_threads(2);torch.manual_seed(731)
        config=Config()
        for key,value in dict(width=32,action_width=32,ffn=64,action_ffn=64,layers=2,
                heads=2,head_dim=16,time_dim=32,text_dim=16,memory_tokens=4,
                history_queries=1,recall_tokens=2,recall_query_dim=16,checkpoint_blocks=False).items():
            setattr(config.model,key,value)
        config.data.image_size=64
        class Codec:
            def encode(self,value):
                pooled=torch.nn.functional.interpolate(value.float().mean(1,keepdim=True),
                    size=(value.shape[2],value.shape[3]//16,value.shape[4]//16))
                return pooled.repeat(1,48,1,1,1)/255.
        config.data.views=['main','left','right'];config.data.main_view='main'
        config.data.action_dim=config.data.state_dim=14
        config.data.relative_action_dims=0;config.data.control_frequency=30
        config.data.action_horizon=32;config.data.execute_actions=steps
        config.data.action_normalization='zscore'
        config.model.memory_update='gated_residual';config.model.first_frame_anchor=True
        config.inference.denoising_steps=2;config.validate()
        normalization=Normalization(dict(methods=dict(actions='zscore',state='zscore'),
            action_mean=[.2]*14,action_std=[.7]*14,state_mean=[.1]*14,state_std=[.8]*14)).validate(config)
        model=WorldActionModel(config).eval()
        text={'fixture':(torch.randn(5,config.model.text_dim),torch.ones(5,dtype=torch.bool))}
        original=Policy(model,Codec(),text,normalization,config,'cpu')
        policy=Policy(deepcopy(model),Codec(),text,normalization,config,'cpu')
        candidate=memory.PhaseMemory(policy,'fixture',torch.manual_seed)

        def request(index,operation='infer',mode='MODEL_EXECUTING'):
            stamp=100.+index/30.
            value=dict(request_id=f'{operation}-{mode}-{index}',seed=17,applied_history=[],
                memory=dict(protocol=memory.MEMORY_PROTOCOL,session_id='oracle',clock_domain='source',
                            epoch=1,start='continue',revision=0),
                memory_context=dict(revision=candidate.context_revision,
                    time_s=steps*.03+(index-steps)/30. if index>=steps else index/30.),
                observation=dict(source_time=stamp,state_reference=dict(stream_id='state',source_instance_id='oracle',
                    source_clock_id='source',source_time_ns=round(stamp*1e9),sequence=index),model_state14=np.full(14,index*.001,np.float32),
                    images={name:np.full((64,64,3),30+index%180,np.uint8)for name in memory.wam.CAMS}))
            if operation=='observe':
                value['memory_operation']='observe'
                value['memory_sample']=memory.observation_sample('TEACH'if mode=='TEACH'else'MODEL',index,value['memory_context']['time_s'],stamp)
            return value

        with torch.no_grad():
            for index in (0,steps,steps*2):
                current=request(index)
                rgb={role:current['observation']['images'][name]for role,name in zip(('main','left','right'),memory.wam.CAMS)}
                stamp=current['memory_context']['time_s']
                original.observe(rgb,stamp)
                torch.manual_seed(current['seed']+candidate.calls)
                expected=original.act('fixture',current['observation']['model_state14'])['actions'].numpy()
                candidate.infer(request(index,'observe','TEACH'if index==steps else'MODEL_EXECUTING'))
                current['memory_context']['revision']=candidate.context_revision
                result=candidate.infer(current)
                np.testing.assert_array_equal(result['action14'],expected)
                torch.testing.assert_close(policy.memory,original.memory,rtol=0,atol=0)
                self.assertEqual(policy.times,original.times)
                before=policy.memory.clone();times=list(policy.times);anchor=policy.first_frame.clone()
                candidate.infer(request(index+1))
                torch.testing.assert_close(policy.memory,before,rtol=0,atol=0)
                torch.testing.assert_close(policy.first_frame,anchor,rtol=0,atol=0)
                self.assertEqual(policy.times,times)
                fresh=request(index)
                fresh['request_id']=f'fresh-same-logical-time-{index}'
                fresh['observation']['source_time']+=.1
                fresh['observation']['state_reference']['source_time_ns']+=100000000
                fresh['observation']['model_state14']+=.001
                for image in fresh['observation']['images'].values():image[:]+=1
                answer=candidate.infer(fresh)
                self.assertEqual(answer['action14'].shape,(32,14))
                self.assertEqual(answer['audit']['model_time_s'],stamp)
                torch.testing.assert_close(policy.memory,before,rtol=0,atol=0)
                torch.testing.assert_close(policy.first_frame,anchor,rtol=0,atol=0)
                self.assertEqual(policy.times,times)
        self.assertEqual(candidate.observations,3)
        self.assertEqual(candidate.manual_observations,1)
        self.assertEqual(candidate.calls,9)
        self.assertFalse(torch.cuda.is_initialized())


if __name__=='__main__':unittest.main()
