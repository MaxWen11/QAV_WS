cmake_minimum_required(VERSION 3.12)
project(px4ctrl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(UADL_STANDALONE "Build the numerical core/tests without ROS or Torch" OFF)
option(UADL_ENABLE_TORCH "Enable frozen TorchScript priors for A/B/D" ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_path(qpOASES_INCLUDE_DIR NAMES qpOASES.hpp)
find_library(qpOASES_LIBRARY NAMES qpOASES)
if(NOT qpOASES_INCLUDE_DIR OR NOT qpOASES_LIBRARY)
  message(FATAL_ERROR "qpOASES headers/library are required")
endif()

if(NOT UADL_STANDALONE AND UADL_ENABLE_TORCH)
  find_package(Torch REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
endif()

add_library(uadl_core src/online_gp.cpp src/tube_mpc.cpp src/control_geometry.cpp)
target_include_directories(uadl_core PUBLIC src ${qpOASES_INCLUDE_DIR})
target_link_libraries(uadl_core PUBLIC Eigen3::Eigen ${qpOASES_LIBRARY})
target_compile_options(uadl_core PRIVATE -Wall -Wextra)

if(UADL_STANDALONE)
  enable_testing()
  foreach(name online_gp tube_mpc control_geometry)
    add_executable(test_${name} tests/test_${name}.cpp)
    target_link_libraries(test_${name} PRIVATE uadl_core)
    target_compile_options(test_${name} PRIVATE -UNDEBUG)
    add_test(NAME ${name} COMMAND test_${name})
  endforeach()
  find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter)
  get_target_property(UADL_EIGEN_INCLUDE Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  set(UADL_ADAPTER_TEST_FLAGS)
  if(APPLE AND CMAKE_OSX_SYSROOT)
    list(APPEND UADL_ADAPTER_TEST_FLAGS --cxx-flag=-isysroot --cxx-flag=${CMAKE_OSX_SYSROOT})
  endif()
  add_test(NAME controller_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_controller_integration.py
      --eigen ${UADL_EIGEN_INCLUDE}
      --qpoases-include ${qpOASES_INCLUDE_DIR}
      --qpoases-library ${qpOASES_LIBRARY}
      --core-library $<TARGET_FILE:uadl_core>
      --cxx ${CMAKE_CXX_COMPILER} ${UADL_ADAPTER_TEST_FLAGS})
  return()
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy quadrotor_msgs geometry_msgs nav_msgs sensor_msgs
  std_msgs std_srvs uav_utils mavros_msgs
)
catkin_package(CATKIN_DEPENDS roscpp quadrotor_msgs geometry_msgs nav_msgs
  sensor_msgs std_msgs std_srvs uav_utils mavros_msgs)

add_executable(px4ctrl_node src/px4ctrl_node.cpp src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp src/controller.cpp src/input.cpp)
target_include_directories(px4ctrl_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(px4ctrl_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(px4ctrl_node PRIVATE uadl_core ${catkin_LIBRARIES})
if(UADL_ENABLE_TORCH)
  target_compile_definitions(px4ctrl_node PRIVATE UADL_WITH_TORCH)
  target_include_directories(px4ctrl_node PRIVATE ${TORCH_INCLUDE_DIRS})
  target_link_libraries(px4ctrl_node PRIVATE ${TORCH_LIBRARIES})
endif()

if(CATKIN_ENABLE_TESTING)
  foreach(name online_gp tube_mpc control_geometry)
    add_executable(test_uadl_${name} tests/test_${name}.cpp)
    target_link_libraries(test_uadl_${name} PRIVATE uadl_core)
    target_compile_options(test_uadl_${name} PRIVATE -UNDEBUG)
    add_test(NAME uadl_${name} COMMAND test_uadl_${name})
  endforeach()
endif()
install(TARGETS px4ctrl_node uadl_core
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch config models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
