cmake_minimum_required(VERSION 3.8)
project(slam_mapping)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if($ENV{ROS_DISTRO} MATCHES "humble")
  add_compile_definitions(ROS_HUMBLE)
elseif($ENV{ROS_DISTRO} MATCHES "foxy")
  add_compile_definitions(ROS_FOXY)
endif()
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(_OPENMPI_INCLUDE_DIRS "/usr/lib/aarch64-linux-gnu/openmpi/include;/usr/lib/aarch64-linux-gnu/openmpi/include/openmpi")
set(MPI_C_COMPILER /usr/bin/mpicc CACHE FILEPATH "" FORCE)
set(MPI_CXX_COMPILER /usr/bin/mpicxx CACHE FILEPATH "" FORCE)
set(MPI_C_HEADER_DIR /usr/lib/aarch64-linux-gnu/openmpi/include CACHE PATH "" FORCE)
set(MPI_CXX_HEADER_DIR /usr/lib/aarch64-linux-gnu/openmpi/include CACHE PATH "" FORCE)
set(MPI_C_COMPILER_INCLUDE_DIRS "${_OPENMPI_INCLUDE_DIRS}" CACHE STRING "" FORCE)
set(MPI_CXX_COMPILER_INCLUDE_DIRS "${_OPENMPI_INCLUDE_DIRS}" CACHE STRING "" FORCE)
set(MPI_C_COMPILE_OPTIONS "" CACHE STRING "" FORCE)
set(MPI_CXX_COMPILE_OPTIONS "" CACHE STRING "" FORCE)
set(MPI_C_LIB_NAMES mpi CACHE STRING "" FORCE)
set(MPI_CXX_LIB_NAMES mpi CACHE STRING "" FORCE)
set(MPI_mpi_LIBRARY /usr/lib/aarch64-linux-gnu/openmpi/lib/libmpi.so CACHE FILEPATH "" FORCE)
set(MPI_C_WORKS TRUE CACHE BOOL "" FORCE)
set(MPI_CXX_WORKS TRUE CACHE BOOL "" FORCE)
set(MPI_C_VERSION 3.1 CACHE STRING "" FORCE)
set(MPI_CXX_VERSION 3.1 CACHE STRING "" FORCE)
set(MPI_DETERMINE_LIBRARY_VERSION FALSE CACHE BOOL "" FORCE)

# ============== 查找依赖 ==============
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)

set(ros2_dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  cv_bridge
)

# ============== 头文件路径 ==============
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# ==========================================================
# 1. 核心后端建图
# ==========================================================
add_executable(alaserPGO src/laserPosegraphOptimization.cpp include/scancontext/Scancontext.cpp)
target_compile_options(alaserPGO PRIVATE ${OpenMP_CXX_FLAGS})
target_include_directories(alaserPGO PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(alaserPGO ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam -Wl,--no-as-needed ${LIBUSB_LIBRARIES} -Wl,--as-needed)
ament_target_dependencies(alaserPGO ${ros2_dependencies})

add_executable(sc_relocalizer src/sc_relocalizer.cpp include/scancontext/Scancontext.cpp)
target_include_directories(sc_relocalizer PUBLIC ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_include_directories(sc_relocalizer PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(sc_relocalizer ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} -Wl,--no-as-needed ${LIBUSB_LIBRARIES} -Wl,--as-needed)
ament_target_dependencies(sc_relocalizer ${ros2_dependencies})

# ============== 安装 ==============
install(TARGETS 
  alaserPGO
  sc_relocalizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz_cfg DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

unset(_OPENMPI_INCLUDE_DIRS)

# ============== ament 包导出 ==============
ament_package()
