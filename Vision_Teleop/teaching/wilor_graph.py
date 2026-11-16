"""Fixed batch CUDA graphs; outputs borrowed until next call of that batch."""
import torch
import inspect,textwrap,types,importlib
import hashlib
from pathlib import Path
from contextlib import contextmanager

@contextmanager
def static_indices(model):
    """Preserve exact tensor math; pre-resolve fixed host index constants."""
    changes=[]
    for obj,name in [(model.backbone,'forward_features'),(model.refine_net,'forward')]:
        old=getattr(obj,name);code=textwrap.dedent(inspect.getsource(old))
        needle='pred_hand_pose[:, [0]]'
        assert code.count(needle)==1
        code=code.replace(needle,'pred_hand_pose[:, :1].clone()')
        env=dict(old.__func__.__globals__);exec(compile(code,'<static pose index>','exec'),env)
        changes.append((obj,name,old));setattr(obj,name,types.MethodType(env[name],obj))
    lbs=importlib.import_module('smplx.lbs');old=lbs.batch_rigid_transform
    parents=tuple(int(x) for x in model.mano.parents.detach().cpu().tolist())
    code=inspect.getsource(old);assert code.count('transform_chain[parents[i]]')==1
    code=code.replace('transform_chain[parents[i]]','transform_chain[static_parents[i]]')
    env=dict(old.__globals__,static_parents=parents);exec(compile(code,'<static MANO parent indices>','exec'),env)
    changes.append((lbs,'batch_rigid_transform',old));lbs.batch_rigid_transform=env['batch_rigid_transform']
    try:yield
    finally:
        for obj,name,old in reversed(changes):setattr(obj,name,old)

class GraphModel:
    def __init__(self, model):
        # Compatibility substitutions apply only to the inspected pinned sources.
        lbs=importlib.import_module('smplx.lbs')
        for fn,expected in [
            (model.backbone.forward_features,'5424ff143d2a8db9f0a722da34857bff2559c4867d9295fb3648e0293f5de013'),
            (model.refine_net.forward,'5878350b52af6c4cd27541ee02579ea7fe30549ef8e101a72fe2cf6a82eef59d'),
            (lbs.batch_rigid_transform,'13e2bfc41be990763487b1c6ed4d01b24fae421d320da1592185bb7b8c725575')]:
            if hashlib.sha256(Path(inspect.getsourcefile(fn)).read_bytes()).hexdigest()!=expected:
                raise ValueError('Graph compatibility source differs from verified version')
        self.model=model
        self.states={}

    def capture(self,n):
        assert n in (1,2) and n not in self.states and not torch.is_grad_enabled()
        x=torch.zeros((n,3,256,256),device='cuda',dtype=torch.float32)
        stream=torch.cuda.Stream()
        stream.wait_stream(torch.cuda.current_stream())
        with static_indices(self.model):
            with torch.cuda.stream(stream):
                for _ in range(3): self.model({'img':x})
            torch.cuda.current_stream().wait_stream(stream)
            torch.cuda.synchronize()
            graph=torch.cuda.CUDAGraph()
            with torch.cuda.graph(graph,stream=stream):
                result=self.model({'img':x})
        torch.cuda.synchronize()
        self.states[n]=(x,result,graph,stream)

    def __call__(self,batch):
        x=batch['img'];n=x.shape[0]
        assert tuple(x.shape)==(n,3,256,256) and x.dtype==torch.float32 and x.is_cuda
        assert n in self.states and not torch.is_grad_enabled()
        storage,result,graph,_=self.states[n]
        storage.copy_(x)
        graph.replay()
        return result
