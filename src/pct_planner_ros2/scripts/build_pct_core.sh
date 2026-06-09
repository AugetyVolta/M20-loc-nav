#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PCT_PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PCT_ROS2_WS="${PCT_ROS2_WS:-$(cd "${PCT_PKG_DIR}/../.." && pwd)}"
PCT_PLANNER_ROOT="${PCT_PLANNER_ROOT:-${PCT_PKG_DIR}/PCT_planner}"

cd "${PCT_PLANNER_ROOT}/planner"
./build_thirdparty.sh
./build.sh
