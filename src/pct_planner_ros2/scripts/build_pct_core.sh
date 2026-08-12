#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PCT_PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PCT_ROS2_WS="${PCT_ROS2_WS:-$(cd "${PCT_PKG_DIR}/../.." && pwd)}"
PCT_PLANNER_ROOT="${PCT_PLANNER_ROOT:-${PCT_PKG_DIR}/PCT_planner}"
GTSAM_VENDOR_PREFIX="${GTSAM_VENDOR_PREFIX:-${PCT_ROS2_WS}/install/gtsam_vendor}"

cd "${PCT_ROS2_WS}"
colcon build --symlink-install --packages-select gtsam_vendor

export GTSAM_DIR="${GTSAM_VENDOR_PREFIX}/lib/cmake/GTSAM"
export LD_LIBRARY_PATH="${GTSAM_VENDOR_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

cd "${PCT_PLANNER_ROOT}/planner"
./build_thirdparty.sh
./build.sh
