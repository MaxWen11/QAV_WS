cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  uav_utils
  mavros
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

# ==========================================
# Find Torch Library (LibTorch)
# ==========================================
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# ==========================================
# Find qpOASES Library and Headers
# ==========================================
find_path(qpOASES_INCLUDE_DIRS NAMES qpOASES.hpp)
find_library(qpOASES_LIBRARIES NAMES qpOASES)

if(qpOASES_INCLUDE_DIRS AND qpOASES_LIBRARIES)
    message(STATUS "Found qpOASES: ${qpOASES_LIBRARIES}")
else()
    message(FATAL_ERROR "qpOASES not found! Please install qpOASES.")
endif()
# ==========================================

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

add_dependencies(px4ctrl_node quadrotor_msgs)

target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
  ${qpOASES_LIBRARIES}
  ${TORCH_LIBRARIES}
)