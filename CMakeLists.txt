cmake_minimum_required(VERSION 2.8.3)
project(twist_tools)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs geometry_msgs)

catkin_package() # INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(twist_multiplier_node src/twist_multiplier_node.cpp)
add_executable(twist_speed_limit_node src/twist_speed_limit_node.cpp)

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(twist_multiplier_node ${catkin_LIBRARIES})

target_link_libraries(twist_speed_limit_node ${catkin_LIBRARIES})

install(
  TARGETS
  twist_multiplier_node
  twist_speed_limit_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

