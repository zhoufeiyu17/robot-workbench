"""Frozen nohistory model execution and packing, imported only by a Linux worker."""
import json
from pathlib import Path
import sys


def load_source(root):
    root=Path(root)
    sys.path[:0]=[str(root/'bundle/source'),str(root/'bundle/source/experiments/pi05_recovery_base/base'),str(root)]
    import torch
    torch.set_num_threads(2);torch.set_num_interop_threads(1)
    torch.set_float32_matmul_precision('highest');torch.backends.cuda.matmul.allow_tf32=False
    torch.backends.cudnn.allow_tf32=False;torch.backends.cudnn.benchmark=False
    from experiments.pi05_human_base import model as training
    from experiments.pi05_human_base.data import CAMS,PROMPTS
    from experiments.pi05_recovery_base.runtime import load_tokenizers
    return torch,training,CAMS,PROMPTS,load_tokenizers


def pack(root, output):
    import torch
    sys.path.insert(0,str(root))
    from native_triton.starvla_pi05_loader import pack_starvla_state_dict,packed_manifest,PACKED_FORMAT,_rope_interleaved_linear_weight
    torch.set_num_threads(2)
    source=Path(root)/'bundle/model/model.pt';state=torch.load(source,map_location='cpu',weights_only=True,mmap=True)
    if any('history' in k for k in state):raise ValueError('nohistory checkpoint required')
    packed=pack_starvla_state_dict({'pi05_model.'+k:v for k,v in state.items()})
    encoder='paligemma_with_expert.paligemma.model.language_model.layers.'
    for target,src in [('encoder_attn_norm','input_layernorm'),('encoder_ffn_norm','post_attention_layernorm')]:
        packed[target]=torch.stack([state[f'{encoder}{i}.{src}.weight'].float()for i in range(18)])
    packed['encoder_attn_qkv_w']=torch.stack([torch.cat([
        _rope_interleaved_linear_weight(state[f'{encoder}{i}.self_attn.q_proj.weight'],num_heads=8),
        _rope_interleaved_linear_weight(state[f'{encoder}{i}.self_attn.k_proj.weight'],num_heads=1),
        state[f'{encoder}{i}.self_attn.v_proj.weight'].float().T],dim=1).bfloat16()for i in range(18)])
    for name in ('gate','up'):
        packed[f'encoder_ffn_{name}_w']=torch.stack([state[f'{encoder}{i}.mlp.{name}_proj.weight'].T.contiguous().bfloat16()for i in range(18)])
    packed['vision_position_embedding']=state['paligemma_with_expert.paligemma.model.vision_tower.vision_model.embeddings.position_embedding.weight'].float()
    manifest=packed_manifest(packed,source_path=source)
    manifest.update(prefix_variant='mixed366_nohistory',precision_recipe='source_rounding_nohistory_v1')
    output=Path(output)
    if output.exists():raise ValueError('new packed path required')
    torch.save(dict(format=PACKED_FORMAT,manifest=manifest,weights=packed),output)


def native_model(packed, tokenizer, task_prompts):
    import math,time,torch
    import torch.nn.functional as F
    from native_triton.base566_infer import Base566Inference,norm,rope,attention,base566_euler_update
    class NoHistoryInference(Base566Inference):
        def __init__(self,*args):
            super().__init__(*args)
            self.text_capacity=256
            self.token_storage=torch.zeros((1,self.text_capacity),device='cuda',dtype=torch.long)
            self.images=torch.zeros((3,224,224,3),device='cuda',dtype=torch.bfloat16)
            self.noise=torch.zeros((1,60,32),device='cuda',dtype=torch.float32)
            self.actions=torch.empty_like(self.noise)
            self.graphs={};self.graph_builds=0
            # Fourteen quantized values are -1..255 (1..3 decimal digits).
            # The other eighteen state slots remain 128. Each variable value
            # can shorten the all-three-digit prompt by at most two tokens.
            lengths=set()
            for prompt in task_prompts:
                count=len(self.tokenizer.encode(f"Task: {prompt}, State: {' '.join(['128']*32)};\n",add_bos=True))
                lengths.update(range(count-28,count+1))
            if not lengths or min(lengths)<1 or max(lengths)>self.text_capacity:
                raise ValueError('registered prompts exceed prepared graph capacity')
            self.prepare_started=time.perf_counter()
            stream=torch.cuda.Stream();stream.wait_stream(torch.cuda.current_stream())
            for n in sorted(lengths):
                self.ids=self.token_storage[:,:n]
                with torch.inference_mode(),torch.autocast('cuda',dtype=torch.bfloat16):
                    with torch.cuda.stream(stream):
                        for _ in range(2):self.run()
                    torch.cuda.current_stream().wait_stream(stream)
                    graph=torch.cuda.CUDAGraph()
                    # Each length owns its scratch pool. Only inputs and the
                    # immediately consumed output are shared; replay is serial.
                    with torch.cuda.graph(graph):self.run()
                self.graphs[n]=graph;self.graph_builds+=1
                if torch.cuda.memory_reserved()>22*1024**3:
                    raise ValueError('prepared graph cache exceeds the ARX6 22 GiB budget')
            self.prepare_wall_s=time.perf_counter()-self.prepare_started

        def forward(self,images,noise,*,task_prompt,state_tokens,history_normalized):
            tokens=self.tokenizer.encode(f"Task: {task_prompt}, State: {' '.join(map(str,state_tokens))};\n",add_bos=True)
            n=len(tokens)
            if task_prompt not in task_prompts or len(state_tokens)!=32 or len(history_normalized):
                raise ValueError('registered nohistory prompt and 32 state tokens required')
            if any(int(v)!=v or not -1<=int(v)<=255 for v in state_tokens[:14]) or any(v!=128 for v in state_tokens[14:]):
                raise ValueError('quantized state exceeds prepared graph contract')
            if n not in self.graphs:
                raise ValueError('prompt length was not prepared; no live graph capture allowed')
            self.ids=self.token_storage[:,:n]
            self.ids.copy_(torch.tensor(tokens,device='cuda')[None])
            self.images.copy_(images);self.noise.copy_(noise[None])
            with torch.inference_mode(),torch.autocast('cuda',dtype=torch.bfloat16):self.graphs[n].replay()
            self.last_execution=dict(text_tokens=n,text_capacity=self.text_capacity,
                graph_builds=self.graph_builds,graph_captured_this_call=False,graph_capture_s=0.,
                prepared_token_lengths=sorted(self.graphs),graph_prepare_wall_s=self.prepare_wall_s)
            return self.actions[0]

        def run(self):
            lang=F.embedding(self.ids,self.w['embedding_weight'])*math.sqrt(2048)
            # No history projection, embeddings, masks or zero-filled history slots.
            x=torch.cat([self.vision(self.images[i:i+1])for i in range(3)]+[lang],dim=1)
            length=x.shape[1];positions=torch.arange(length,device='cuda')[None,:]
            cos,sin=self.cos[positions][:,None],self.sin[positions][:,None]
            bias=torch.zeros((1,1,length,length),device='cuda',dtype=torch.float32);kv=[]
            for i in range(18):
                y=norm(x,self.w['encoder_attn_norm'][i])
                q,k,v=[self.project(y,'encoder',name,i).view(1,-1,8 if name=='q'else 1,256).transpose(1,2)for name in ('q','k','v')]
                q,k=rope(q,cos,sin),rope(k,cos,sin);kv.append((k,v))
                if i==17:break
                x=x+self.linear(attention(q,k,v,bias),'encoder_attn_o',i)
                y=norm(x,self.w['encoder_ffn_norm'][i]);x=x+self.linear(F.gelu(self.linear(y,'encoder_ffn_gate',i),approximate='tanh')*self.linear(y,'encoder_ffn_up',i),'encoder_ffn_down',i)
            positions=length+torch.arange(60,device='cuda')[None,:]
            cos,sin=self.cos[positions][:,None],self.sin[positions][:,None]
            bias=torch.zeros((1,1,60,length+60),device='cuda',dtype=torch.float32);self.actions.copy_(self.noise)
            for step in range(10):
                style=self.styles[step];x=self.linear(self.actions,'decoder_action_in_proj')
                for i in range(18):
                    y,gate=norm(x,style=style['attn'][i])
                    q,k,v=[self.project(y,'decoder',name,i).view(1,60,8 if name=='q'else 1,256).transpose(1,2)for name in ('q','k','v')]
                    q,k=rope(q,cos,sin),rope(k,cos,sin);k=torch.cat((kv[i][0],k),dim=2);v=torch.cat((kv[i][1],v),dim=2)
                    x=x+self.linear(attention(q,k,v,bias),'decoder_attn_o',i)*gate
                    y,gate=norm(x,style=style['ffn'][i]);x=x+self.linear(F.gelu(self.linear(y,'decoder_ffn_gate',i),approximate='tanh')*self.linear(y,'decoder_ffn_up',i),'decoder_ffn_down',i)*gate
                y,_=norm(x,style=style['final']);velocity=self.linear(y,'decoder_action_out_proj')
                base566_euler_update[(8,)](self.actions,velocity,size=1920,BLOCK=256,enable_fp_fusion=False)
    return NoHistoryInference(str(packed),str(tokenizer))


class Policy:
    def __init__(self,root,backend,packed=None):
        self.root=Path(root);torch,training,self.cams,self.prompts,load_tokenizers=load_source(root)
        self.torch=torch;self.training=training
        self.stats=json.loads((self.root/'bundle/sidecars/stats.json').read_text())
        self.prompts=json.loads((self.root/'bundle/sidecars/prompts.json').read_text())
        self.tokens=load_tokenizers(dict(paligemma_tokenizer=str(self.root/'bundle/tokenizers/sp/paligemma_tokenizer.model'),fast_tokenizer=str(self.root/'bundle/tokenizers/fast')))
        self.backend=backend
        if backend=='torch':
            model=training.create_model(base=None)
            state=torch.load(self.root/'bundle/model/model.pt',map_location='cpu',weights_only=True,mmap=True)
            model.load_state_dict(state,strict=True);actual=model.state_dict()
            if actual.keys()!=state.keys()or any('history'in k for k in actual)or not all(v.dtype==actual[k].dtype and torch.equal(v,actual[k])for k,v in state.items()):raise ValueError('strict nohistory tensor equality failed')
            del actual,state
            self.model=model.cuda().eval()
        elif backend=='triton':self.model=native_model(packed,self.root/'bundle/tokenizers/sp/paligemma_tokenizer.model',tuple(self.prompts.values()))
        else:raise ValueError('explicit torch/triton required')

    def infer(self,request):
        import time,numpy as np
        from starVLA.model.modules.pi05.observation import Observation
        from encoding import denorm
        started=time.perf_counter();torch=self.torch;obs=request['observation'];task=request['task']
        if task not in self.prompts:raise ValueError('registered task required')
        sample=dict(state=np.asarray(obs['model_state14'],np.float32),images=obs['images'],prompt=self.prompts[task],actions=np.zeros((60,14),np.float32),actions_mask=np.zeros(60,bool))
        with torch.no_grad():o,_,valid,eligible=self.training.batch([sample],self.stats,self.tokens,'cuda')
        if valid.any()or eligible.any()or o.fast_action_loss_mask.any():raise ValueError('supervision leaked into inference')
        o=Observation(**{k:getattr(o,k)for k in ['images','image_masks','state','tokenized_prompt','tokenized_prompt_mask']})
        noise=torch.randn((1,60,32),generator=torch.Generator(device='cpu').manual_seed(request['seed'])).cuda()
        model_started=time.perf_counter()
        with torch.inference_mode(),torch.autocast('cuda',dtype=torch.bfloat16):
            if self.backend=='torch':result=self.model.sample_actions('cuda',o,noise=noise,num_steps=10,action_prefix=None,infer_delay=0)
            else:
                from starVLA.model.modules.pi05 import preprocessing as preprocessing_pytorch
                # Same source preprocessing helper as PI05Pytorch; no uint8 224 roundtrip.
                processed=preprocessing_pytorch.preprocess_observation_pytorch(o,train=False,image_keys=list(self.cams))
                images=torch.stack([processed.images[c][0].permute(1,2,0)for c in self.cams]).bfloat16()
                state_tokens=(np.digitize(o.state[0].float().cpu().numpy(),np.linspace(-1,1,257)[:-1])-1).astype(np.int64)
                result=self.model.forward(images,noise[0],task_prompt=self.prompts[task],state_tokens=state_tokens,history_normalized=torch.empty((0,14),device='cuda')).unsqueeze(0)
            result=result.float().cpu().numpy()
        model_returned=time.perf_counter()
        if result.shape!=(1,60,32)or not np.isfinite(result).all():raise ValueError('finite 60x32 output required')
        action=denorm(result[0,:,:14],self.stats['action']).astype(np.float32)
        audit=dict(preprocessing_wall_s=model_started-started,model_and_transfer_wall_s=model_returned-model_started,
            postprocessing_wall_s=time.perf_counter()-model_returned)
        if self.backend=='triton':audit.update(self.model.last_execution)
        return dict(request_id=request['request_id'],action14=action,normalized=result[0],task=task,command_publication=False,audit=dict(timing=audit))
