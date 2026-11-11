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
        root=Path(os.environ['WAM_MEMORY_ORACLE_SOURCE'])
        sys.path[:0]=[str(root/'src'),str(root/'tests')]
        import torch
        from fixtures import Codec,small_config
        from world_memory.inference import Policy
        from world_memory.models.world import WorldActionModel
        from world_memory.normalization import Normalization

        torch.set_num_threads(2);torch.manual_seed(731)
        config=small_config()
        config.data.views=['main','left','right'];config.data.main_view='main'
        config.data.action_dim=config.data.state_dim=14
        config.data.relative_action_dims=0;config.data.control_frequency=30
        config.data.action_horizon=config.data.execute_actions=32
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
                observation=dict(source_time=stamp,model_state14=np.full(14,index*.001,np.float32),
                    images={name:np.full((64,64,3),30+index%180,np.uint8)for name in memory.wam.CAMS}))
            if operation=='observe':
                value['memory_operation']='observe'
                value['memory_sample']=dict(states14=[],actions14=[],state_times=[],action_times=[],mode=mode,
                    effective_index=index,action_semantics='materialized_next_observation',valid_actions=0,
                    executed_action_condition=False)
            return value

        with torch.no_grad():
            for index in (0,32,64):
                current=request(index)
                rgb={role:current['observation']['images'][name]for role,name in zip(('main','left','right'),memory.wam.CAMS)}
                stamp=current['observation']['source_time']-100.
                original.observe(rgb,stamp)
                torch.manual_seed(current['seed']+candidate.calls)
                expected=original.act('fixture',current['observation']['model_state14'])['execute'].numpy()
                if index:candidate.infer(request(index,'observe','TEACH'if index==32 else'MODEL_EXECUTING'))
                result=candidate.infer(current)
                np.testing.assert_array_equal(result['action14'],expected)
                torch.testing.assert_close(policy.memory,original.memory,rtol=0,atol=0)
                self.assertEqual(policy.times,original.times)
                before=policy.memory.clone();times=list(policy.times);anchor=policy.first_frame.clone()
                candidate.infer(request(index+1))
                torch.testing.assert_close(policy.memory,before,rtol=0,atol=0)
                torch.testing.assert_close(policy.first_frame,anchor,rtol=0,atol=0)
                self.assertEqual(policy.times,times)
        self.assertEqual(candidate.observations,3)
        self.assertEqual(candidate.manual_observations,1)
        self.assertEqual(candidate.calls,6)
        self.assertFalse(torch.cuda.is_initialized())


if __name__=='__main__':unittest.main()
