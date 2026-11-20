#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
: "${DA_LIVGS_BUILD_JOBS:=2}"
source /opt/ros/noetic/setup.bash
if [[ -n "${DA_LIVGS_EXTRA_SETUP:-}" ]]; then source "${DA_LIVGS_EXTRA_SETUP}"; fi
CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release)
if [[ -n "${DA_LIVGS_CMAKE_ARGS:-}" ]]; then read -r -a EXTRA_CMAKE_ARGS <<< "${DA_LIVGS_CMAKE_ARGS}"; CMAKE_ARGS+=("${EXTRA_CMAKE_ARGS[@]}"); fi
catkin_make --directory "${ROOT}/catkin_ws" -j"${DA_LIVGS_BUILD_JOBS}" --cmake-args "${CMAKE_ARGS[@]}"
