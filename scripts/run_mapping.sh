#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
CONFIG=config/da_livgs_fastlivo2_balanced.yaml
OUTPUT=${ROOT}/outputs/da_livgs_mapping
LPIPS=${ROOT}/catkin_ws/src/da_livgs_mapping/src/lpips
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      CONFIG=$2
      shift 2
      ;;
    --output)
      OUTPUT=$2
      shift 2
      ;;
    --lpips)
      LPIPS=$2
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done
source /opt/ros/noetic/setup.bash
source "${ROOT}/catkin_ws/devel/setup.bash"
mkdir -p "${OUTPUT}"
roslaunch da_livgs_mapping da_livgs_balanced.launch config_path:="${CONFIG}" result_path:="${OUTPUT}" lpips_path:="${LPIPS}"
