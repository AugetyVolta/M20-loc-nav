#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
PCT_ROS2_WS="${PCT_ROS2_WS:-$(cd "${ROOT_DIR}/../../../.." && pwd)}"
GTSAM_VENDOR_PREFIX="${GTSAM_VENDOR_PREFIX:-${PCT_ROS2_WS}/install/gtsam_vendor}"
GTSAM_DIR="${GTSAM_DIR:-${GTSAM_VENDOR_PREFIX}/lib/cmake/GTSAM}"

if [ ! -f "${GTSAM_DIR}/GTSAMConfig.cmake" ]; then
  echo "Missing workspace GTSAM: ${GTSAM_DIR}/GTSAMConfig.cmake" >&2
  echo "Build it first: colcon build --symlink-install --packages-select gtsam_vendor" >&2
  exit 1
fi
# echo "ROOT_DIR: ${ROOT_DIR}"

cd "${ROOT_DIR}/lib"

# rm -rf build
mkdir -p build

cd build
cmake_args=(../ -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5)
cmake_args+=("-DGTSAM_DIR=${GTSAM_DIR}")
cmake "${cmake_args[@]}"
make -j6
cp ./src/a_star/a_star*.so ../
cp ./src/a_star/liba_star_search.so ../
cp ./src/trajectory_optimization/traj_opt*.so ../
cp ./src/trajectory_optimization/libgpmp_optimizer.so ../
cp ./src/ele_planner/ele_planner*.so ../
cp ./src/ele_planner/libele_planner_lib.so ../
cp ./src/map_manager/py_map_manager*.so ../
cp ./src/map_manager/libmap_manager.so ../
cp ./src/common/smoothing/libcommon_smoothing.so ../
cd ..

# # optional
export LD_LIBRARY_PATH="${GTSAM_VENDOR_PREFIX}/lib:${ROOT_DIR}/lib/build/src/common/smoothing:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${ROOT_DIR}/lib:${PYTHONPATH:-}"
# pybind11-stubgen -o ./ a_star
# pybind11-stubgen -o ./ traj_opt
# pybind11-stubgen -o ./ ele_planner
# pybind11-stubgen -o ./ py_map_manager
# cp ./a_star-stubs/__init__.pyi ./a_star.pyi
# cp ./traj_opt-stubs/__init__.pyi ./traj_opt.pyi
# cp ./ele_planner-stubs/__init__.pyi ./ele_planner.pyi
# cp ./py_map_manager-stubs/__init__.pyi ./py_map_manager.pyi
