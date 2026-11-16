"""One pinned local inference backend for the existing native hand worker.

Original weights, FP32 geometry and fixed batch1/2; no automatic fallback or
runtime building. TensorRT/package/model licenses remain separate assets.
"""
from pathlib import Path
import hashlib,os,sys
import torch

ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'results/handheld_teaching/20260921_compiled_backend_01'
ENGINE_SHA256='71c3607e62f482be72866b56b3d21a41514a00edd722c758a50fb43ae6ebfec3'

class CompiledBlocks(torch.nn.Module):
    def __init__(self):
        super().__init__()
        if str(torch.__version__)!='2.8.0+cu128' or torch.cuda.get_device_capability()!=(12,0):
            raise RuntimeError('Compiled WiLoR requires the validated torch/CUDA/GPU environment')
        engine_bytes=(ASSETS/'blocks.engine').read_bytes()
        if hashlib.sha256(engine_bytes).hexdigest()!=ENGINE_SHA256:
            raise ValueError('Compiled WiLoR engine differs from validated artifact')
        sys.path.insert(0,str(ASSETS/'deps'))
        self.dlls=[os.add_dll_directory(str(Path(torch.__file__).parent/'lib')),
                   os.add_dll_directory(str(ASSETS/'deps/tensorrt_libs'))]
        import tensorrt_bindings as trt
        if trt.__version__!='10.13.3.9':raise ValueError('Pinned TensorRT runtime required')
        self.logger=trt.Logger(trt.Logger.WARNING);self.runtime=trt.Runtime(self.logger)
        self.engine=self.runtime.deserialize_cuda_engine(engine_bytes)
        if self.engine is None or self.engine.num_optimization_profiles!=2:
            raise RuntimeError('Two fixed compiled batch profiles required')
        assert self.engine.get_tensor_dtype('tokens')==trt.float32
        assert self.engine.get_tensor_dtype('features')==trt.float32
        self.contexts={n:self.engine.create_execution_context() for n in (1,2)}
        for n,context in self.contexts.items():
            assert context.set_optimization_profile_async(n-1,torch.cuda.current_stream().cuda_stream)
            assert context.set_input_shape('tokens',(n,210,1280))
        torch.cuda.synchronize()

    def forward(self,x):
        n=x.shape[0]
        if (torch.is_grad_enabled() or x.dtype!=torch.float32 or not x.is_cuda or
                n not in self.contexts or tuple(x.shape[1:])!=(210,1280) or not x.is_contiguous()):
            raise ValueError('Frozen inference token contract violated')
        output=torch.empty_like(x);context=self.contexts[n]
        assert context.set_tensor_address('tokens',x.data_ptr())
        assert context.set_tensor_address('features',output.data_ptr())
        if not context.execute_async_v3(torch.cuda.current_stream().cuda_stream):
            raise RuntimeError('Compiled inference enqueue failed')
        return output

def prepare(model):
    from .wilor_graph import GraphModel
    if model.training or len(model.backbone.blocks)!=32:
        raise ValueError('Original frozen eval model required')
    model.backbone.blocks=torch.nn.ModuleList([CompiledBlocks()])
    torch.cuda.empty_cache()
    with torch.inference_mode():
        graph=GraphModel(model)
        for n in (1,2):graph.capture(n)
    return graph
