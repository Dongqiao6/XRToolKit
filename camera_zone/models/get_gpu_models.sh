#!/usr/bin/env bash
# 下载 GPU 姿态后端（POSE_BACKEND=gpu）要用的模型和 onnxruntime-gpu 轮子。
# 在有外网的机器上跑（Mac 走 127.0.0.1:7897 代理），再 scp 到 JAKA：
#   bash models/get_gpu_models.sh /path/to/staging
#   REMOTE=/path/on/jaka/camera_zone       # JAKA 上本目录的实际位置，按你的部署改
#   scp -r /path/to/staging/gpu  jaka:$REMOTE/models/
#   scp /path/to/staging/wheels/*.whl jaka:$REMOTE/wheels/
#   ssh jaka "cd $REMOTE && .venv/bin/pip install --no-deps --no-index wheels/*.whl"
#   # 切掉 YOLOX 里的 NMS，检测器才能整个进 TensorRT（需要上一步装的 onnx）：
#   ssh jaka "cd $REMOTE && .venv/bin/python models/cut_yolox_nms.py models/gpu/yolox_tiny_8xb8-300e_humanart-6f3252f9.onnx"
# 注意：onnxruntime-gpu 必须用 1.24.0；1.23.0 那个轮子是按 NumPy 1.x 编的，.venv 里是 NumPy 2.2，import 就报错。
# --no-deps：onnxruntime 运行时不需要 sympy/coloredlogs，别让 pip 动 .venv 里现有的 numpy。
# TensorRT 引擎缓存是机器相关的，第一次在 JAKA 上运行 gpu_pose.py 时自动生成在
# <本目录>/models/gpu/trt_cache/，不进交付包。
set -euo pipefail
OUT="${1:?用法: get_gpu_models.sh <staging 目录>}"
mkdir -p "$OUT/gpu" "$OUT/wheels" "$OUT/zip"
MM=https://download.openmmlab.com/mmpose/v1/projects
# 人体检测：YOLOX-tiny（HumanArt 训练，只有 person 一类），416×416，rtmlib 轻量档用的就是它
DET=$MM/rtmposev1/onnx_sdk/yolox_tiny_8xb8-300e_humanart-6f3252f9.zip
DET_M=$MM/rtmposev1/onnx_sdk/yolox_m_8xb8-300e_humanart-c2c7a14a.zip
# 全身 133 点：RTMW-dw-x-l 256×192（rtmlib balanced 档）和 RTMW-dw-l-m 256×192（lightweight 档）
RTMW_L=$MM/rtmw/onnx_sdk/rtmw-dw-x-l_simcc-cocktail14_270e-256x192_20231122.zip
RTMW_M=$MM/rtmw/onnx_sdk/rtmw-dw-l-m_simcc-cocktail14_270e-256x192_20231122.zip
# JetPack 6 / CUDA 12.6 的 onnxruntime-gpu（带 TensorRT EP 和 CUDA EP），cp310 aarch64
ORT=https://pypi.jetson-ai-lab.io/jp6/cu126/+f/d98/0b934b9a29c1a/onnxruntime_gpu-1.24.0-cp310-cp310-linux_aarch64.whl
ORT_SHA=d980b934b9a29c1a9d6f39751edd7662b69fadd75556a10ff363773a58ce0950

get() { [ -s "$2" ] || curl -fL --retry 3 -m 900 -o "$2" "$1"; }
for u in "$DET" "$DET_M" "$RTMW_L" "$RTMW_M"; do
  z="$OUT/zip/$(basename "$u")"; get "$u" "$z"
  d="$OUT/zip/$(basename "$u" .zip)"; mkdir -p "$d"; unzip -o -q "$z" -d "$d"
  f=$(find "$d" -name '*.onnx' | head -1)
  cp "$f" "$OUT/gpu/$(basename "$u" .zip).onnx"
done
W="$OUT/wheels/$(basename "$ORT")"; get "$ORT" "$W"
echo "$ORT_SHA  $W" | shasum -a 256 -c -
# 切 NMS 用的 onnx（及其依赖 protobuf），aarch64 cp310
for spec in onnx==1.17.0 "protobuf>=4.25,<6"; do
  pip3 download --no-deps --only-binary=:all: --platform manylinux2014_aarch64 \
    --platform manylinux_2_17_aarch64 --python-version 3.10 --implementation cp \
    --abi cp310 --abi abi3 --abi none "$spec" -d "$OUT/wheels"
done
ls -la "$OUT/gpu" "$OUT/wheels"
