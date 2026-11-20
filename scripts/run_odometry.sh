#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
CONFIG=/config/da_livgs_fastlivo2.yaml
BAG=
RVIZ=false
START=-1
DURATION=-2
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      CONFIG=$2
      shift 2
      ;;
    --bag)
      BAG=$2
      shift 2
      ;;
    --rviz)
      RVIZ=$2
      shift 2
      ;;
    --start)
      START=$2
      shift 2
      ;;
    --duration)
      DURATION=$2
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done
if [[ -z "${BAG}" ]]; then echo "--bag is required" >&2; exit 2; fi
source /opt/ros/noetic/setup.bash
source "${ROOT}/catkin_ws/devel/setup.bash"
roslaunch da_livgs_frontend da_livgs_odometry.launch \
  config_path:="${CONFIG}" \
  bag_path:="${BAG}" \
  bag_start_override:="${START}" \
  bag_durr_override:="${DURATION}" \
  rviz:="${RVIZ}"
