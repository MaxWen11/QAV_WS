cmake_minimum_required(VERSION 3.0.2)
project(uwb_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nlink_parser
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nlink_parser roscpp sensor_msgs tf2 tf2_geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(uwb_bridge_node src/uwb_bridge_node.cpp)
add_dependencies(uwb_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(uwb_bridge_node ${catkin_LIBRARIES})

install(TARGETS uwb_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
