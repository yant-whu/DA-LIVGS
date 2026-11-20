#!/usr/bin/env bash
set -e

ENV_NAME=da_livgs_mapping2_spnet
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo ">>> Activating conda env: $ENV_NAME"
if [[ -n "${DA_LIVGS_CONDA_EXE:-}" && -x "${DA_LIVGS_CONDA_EXE}" ]]; then
  CONDA_EXE="${DA_LIVGS_CONDA_EXE}"
elif command -v conda >/dev/null 2>&1; then
  CONDA_EXE="$(command -v conda)"
else
  echo ">>> Conda executable not found." >&2
  exit 1
fi
source "$("$CONDA_EXE" info --base)/etc/profile.d/conda.sh"
conda activate $ENV_NAME
cd "$SCRIPT_DIR"

if [[ ! -s Large_300.pth ]]; then
  echo ">>> Missing checkpoint: $SCRIPT_DIR/Large_300.pth" >&2
  exit 1
fi

echo ">>> Export ONNX (512x640)"
python export_onnx_512_640.py

echo ">>> Export ONNX (480x640)"
python export_onnx_480_640.py

echo ">>> All ONNX exports finished successfully."
