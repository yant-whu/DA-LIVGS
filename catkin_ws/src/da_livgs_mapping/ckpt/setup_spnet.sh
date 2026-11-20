#!/usr/bin/env bash
set -e

ENV_NAME=da_livgs_mapping2_spnet

if [[ -n "${DA_LIVGS_CONDA_EXE:-}" && -x "${DA_LIVGS_CONDA_EXE}" ]]; then
  CONDA_EXE="${DA_LIVGS_CONDA_EXE}"
elif command -v conda >/dev/null 2>&1; then
  CONDA_EXE="$(command -v conda)"
else
  echo ">>> Conda executable not found." >&2
  exit 1
fi

source "$("$CONDA_EXE" info --base)/etc/profile.d/conda.sh"

if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  echo ">>> Conda env already exists: $ENV_NAME"
else
  echo ">>> Creating conda env: $ENV_NAME"
  conda create -n "$ENV_NAME" python=3.8 pip -y
fi

echo ">>> Activating conda env: $ENV_NAME"
conda activate "$ENV_NAME"

echo ">>> Installing PyTorch"
python -m pip install numpy==1.24.4
python -m pip install \
  torch==2.0.1+cu117 \
  torchvision==0.15.2+cu117 \
  torchaudio==2.0.2+cu117 \
  --index-url https://download.pytorch.org/whl/cu117

echo ">>> Installing ONNX & ONNXRuntime"
python -m pip install onnx==1.14.1 onnxruntime==1.15.1

echo ">>> Done. Environment $ENV_NAME is ready."
