cmake_minimum_required(VERSION 3.8)
project(gtsam_vendor VERSION 4.1.1)

find_package(ament_cmake REQUIRED)

# Keep one GTSAM ABI for PCT and slam_mapping. These options match the former
# PCT third-party build while omitting development-only targets.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
# Keep GTSAM and its bundled METIS together even when the workspace has not
# been sourced and another GTSAM installation exists under /usr/local.
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "" FORCE)
set(GTSAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_TIMING_ALWAYS OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_UNSTABLE OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(GTSAM_UNSTABLE_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(GTSAM_INSTALL_MATLAB_TOOLBOX OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_WITH_MARCH_NATIVE ON CACHE BOOL "" FORCE)

add_subdirectory(vendor/gtsam gtsam-build)

ament_package(CONFIG_EXTRAS cmake/gtsam_vendor-extras.cmake.in)
