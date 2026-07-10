cmake_minimum_required(VERSION 3.8)
project(fast_lio_map)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

 ADD_COMPILE_OPTIONS(-std=c++17) 
 ADD_COMPILE_OPTIONS(-std=c++17) 
 set(CMAKE_CXX_FLAGS "-std=c++17 -O3") 
  
 add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\") 
  
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions") 
 set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")

  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(amd64)|(AMD64)")
  set(_DEB_HOST_MULTIARCH "x86_64-linux-gnu")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(aarch64)|(arm64)|(ARM64)")
  set(_DEB_HOST_MULTIARCH "aarch64-linux-gnu")
else()
  set(_DEB_HOST_MULTIARCH "${CMAKE_SYSTEM_PROCESSOR}-linux-gnu")
endif()
set(_OPENMPI_BASE "/usr/lib/${_DEB_HOST_MULTIARCH}/openmpi")
set(_OPENMPI_INCLUDE_DIRS "${_OPENMPI_BASE}/include;${_OPENMPI_BASE}/include/openmpi")
set(MPI_C_COMPILER /usr/bin/mpicc CACHE FILEPATH "" FORCE)
set(MPI_CXX_COMPILER /usr/bin/mpicxx CACHE FILEPATH "" FORCE)
set(MPI_C_HEADER_DIR ${_OPENMPI_BASE}/include CACHE PATH "" FORCE)
set(MPI_CXX_HEADER_DIR ${_OPENMPI_BASE}/include CACHE PATH "" FORCE)
set(MPI_C_COMPILER_INCLUDE_DIRS "${_OPENMPI_INCLUDE_DIRS}" CACHE STRING "" FORCE)
set(MPI_CXX_COMPILER_INCLUDE_DIRS "${_OPENMPI_INCLUDE_DIRS}" CACHE STRING "" FORCE)
set(MPI_C_COMPILE_OPTIONS "" CACHE STRING "" FORCE)
set(MPI_CXX_COMPILE_OPTIONS "" CACHE STRING "" FORCE)
set(MPI_C_LIB_NAMES mpi CACHE STRING "" FORCE)
set(MPI_CXX_LIB_NAMES mpi CACHE STRING "" FORCE)
set(MPI_mpi_LIBRARY ${_OPENMPI_BASE}/lib/libmpi.so CACHE FILEPATH "" FORCE)
set(MPI_C_WORKS TRUE CACHE BOOL "" FORCE)
set(MPI_CXX_WORKS TRUE CACHE BOOL "" FORCE)
set(MPI_C_VERSION 3.1 CACHE STRING "" FORCE)
set(MPI_CXX_VERSION 3.1 CACHE STRING "" FORCE)
set(MPI_DETERMINE_LIBRARY_VERSION FALSE CACHE BOOL "" FORCE)

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

# Thirdparty libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}")

set(msg_files
  "msg/Pose6D.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(fastlio_mapping ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlio_mapping OpenMP::OpenMP_CXX)
endif()
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")

if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  # Custommsg to support foxy & galactic
  rosidl_target_interfaces(fastlio_mapping
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastlio_mapping ${cpp_typesupport_target})
endif()

ament_target_dependencies(fastlio_mapping ${dependencies})

# ---------------- Install --------------- #
install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

unset(_OPENMPI_INCLUDE_DIRS)

ament_package()
