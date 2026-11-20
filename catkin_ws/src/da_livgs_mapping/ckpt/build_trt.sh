#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIBRARIES_ROOT="${DA_LIVGS_DEPENDENCY_ROOT:-}"
TENSORRT_ROOT="${TENSORRT_ROOT:-${LIBRARIES_ROOT:+$LIBRARIES_ROOT/TensorRT-8.6.1.6}}"
CUDA_ROOT="${CUDA_ROOT:-${LIBRARIES_ROOT:+$LIBRARIES_ROOT/cuda-11.7}}"
CUDNN_ROOT="${CUDNN_ROOT:-${LIBRARIES_ROOT:+$LIBRARIES_ROOT/cudnn-8.9.6-cu11/nvidia/cudnn}}"
TRT_BIN=$TENSORRT_ROOT/bin/trtexec
TRT_LIB=$TENSORRT_ROOT/lib

if [[ ! -x "$TRT_BIN" ]]; then
  echo ">>> TensorRT executable not found: $TRT_BIN" >&2
  exit 1
fi
for model in spnet_512_640.onnx spnet_480_640.onnx; do
  if [[ ! -s "$SCRIPT_DIR/$model" ]]; then
    echo ">>> Missing ONNX model: $SCRIPT_DIR/$model" >&2
    exit 1
  fi
done

echo ">>> Setting TensorRT, CUDA, and cuDNN library paths"
export LD_LIBRARY_PATH="$TRT_LIB:$CUDA_ROOT/lib64:$CUDNN_ROOT/lib:${LD_LIBRARY_PATH:-}"

echo ">>> Building TensorRT engine: 512x640"
"$TRT_BIN" \
  --onnx="$SCRIPT_DIR/spnet_512_640.onnx" \
  --saveEngine="$SCRIPT_DIR/spnet_512_640.engine" \
  --fp16 \
  --optShapes=rgb:1x3x512x640,depth:1x1x512x640,mask:1x1x512x640

echo ">>> Building TensorRT engine: 480x640"
"$TRT_BIN" \
  --onnx="$SCRIPT_DIR/spnet_480_640.onnx" \
  --saveEngine="$SCRIPT_DIR/spnet_480_640.engine" \
  --fp16 \
  --optShapes=rgb:1x3x480x640,depth:1x1x480x640,mask:1x1x480x640

echo ">>> TensorRT engine build finished."
