"""把 YOLOX（mmdeploy 导出、带 NMS）的 onnx 切到 NMS 之前，输出 boxes[1,N,4]、scores[1,C,N]。

为什么：NMS 之后的张量长度随检出框数变化，onnxruntime 的 TensorRT EP 会因形状变化反复重编
那一小段引擎（11-16 Jetson 实测每次卡约 2.9 s），只好整个检测器退到 CUDA EP（17 ms/帧）。
切掉 NMS 后整个检测器是静态形状，可以整个进 TensorRT；NMS 在 gpu_pose.py 里用 cv2 做。

用法（JAKA 上，需要 .venv 里有 onnx）：
  ./.venv/bin/python models/cut_yolox_nms.py models/gpu/yolox_tiny_8xb8-300e_humanart-6f3252f9.onnx
产出同目录 *_nonms.onnx。
"""
import sys

import onnx
from onnx import utils

src = sys.argv[1]
dst = src.replace(".onnx", "_nonms.onnx")
m = onnx.load(src)
nms = [n for n in m.graph.node if n.op_type == "NonMaxSuppression"]
assert len(nms) == 1, f"找到 {len(nms)} 个 NMS 节点"
boxes, scores = nms[0].input[0], nms[0].input[1]
print("NMS 输入:", boxes, scores)
inp = [i.name for i in m.graph.input]
utils.extract_model(src, dst, inp, [boxes, scores])
print("写出", dst)
