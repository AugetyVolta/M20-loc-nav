cmake_minimum_required(VERSION 3.8)
project(global_path_planning)

find_package(ament_cmake REQUIRED)

set(PCT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(PCT_STAMP "${CMAKE_CURRENT_BINARY_DIR}/pct_planner_built.stamp")

add_custom_command(
  OUTPUT "${PCT_STAMP}"
  COMMAND
    /bin/bash -lc
    "set -e; cd '${PCT_ROOT}/planner'; \
     if ls lib/a_star*.so lib/ele_planner*.so lib/traj_opt*.so lib/py_map_manager*.so >/dev/null 2>&1; then \
       echo 'PCT planner pybind libraries already exist; skip rebuild.'; \
     else \
       ./build_thirdparty.sh; \
       ./build.sh; \
     fi"
  COMMAND "${CMAKE_COMMAND}" -E touch "${PCT_STAMP}"
  WORKING_DIRECTORY "${PCT_ROOT}"
  VERBATIM
)

add_custom_target(build_pct_planner ALL DEPENDS "${PCT_STAMP}")

install(FILES
  LICENSE
  NOTICE
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
