#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PCT_PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
export PCT_ROS2_WS="${PCT_ROS2_WS:-$(cd "${PCT_PKG_DIR}/../.." && pwd)}"
export PCT_PLANNER_ROOT="${PCT_PLANNER_ROOT:-${PCT_PKG_DIR}/PCT_planner}"
export PCT_VENV="${PCT_VENV:-${PCT_ROS2_WS}/.venv/m20_nav_jazzy}"
export PCT_VENV_SITE="${PCT_VENV_SITE:-${PCT_VENV}/lib/python3.12/site-packages}"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

export PATH="${PCT_VENV}/bin:${PATH}"
export PYTHONPATH="${PCT_VENV_SITE}:${PCT_PLANNER_ROOT}/planner/lib:${PCT_PLANNER_ROOT}/planner:${PCT_PLANNER_ROOT}/planner/scripts:${PCT_PLANNER_ROOT}/tomography:${PCT_PLANNER_ROOT}/tomography/scripts:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${PCT_PLANNER_ROOT}/planner/lib/3rdparty/gtsam-4.1.1/install/lib:${PCT_PLANNER_ROOT}/planner/lib/3rdparty/osqp/install/lib:${PCT_PLANNER_ROOT}/planner/lib:${PCT_PLANNER_ROOT}/planner/lib/build/src/common/smoothing:${LD_LIBRARY_PATH:-}"
for cuda_lib_dir in "${PCT_VENV_SITE}"/nvidia/*/lib; do
  if [ -d "${cuda_lib_dir}" ]; then
    export LD_LIBRARY_PATH="${cuda_lib_dir}:${LD_LIBRARY_PATH}"
  fi
done

if [ -f "${PCT_ROS2_WS}/install/setup.bash" ]; then
  source "${PCT_ROS2_WS}/install/setup.bash"
fi

if [ -d "${PCT_ROS2_WS}/install/pct_planner_ros2" ]; then
  export AMENT_PREFIX_PATH="${PCT_ROS2_WS}/install/pct_planner_ros2:${AMENT_PREFIX_PATH:-}"
  export COLCON_PREFIX_PATH="${PCT_ROS2_WS}/install:${COLCON_PREFIX_PATH:-}"
  export PATH="${PCT_ROS2_WS}/install/pct_planner_ros2/lib/pct_planner_ros2:${PATH}"
  export PYTHONPATH="${PCT_ROS2_WS}/install/pct_planner_ros2/lib/python3.12/site-packages:${PYTHONPATH:-}"
fi
