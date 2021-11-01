cmake_minimum_required(VERSION 3.0.2)
project(pigeon_converter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  nav_msgs
  sensor_msgs
  tf
  laser_geometry
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pigeon_converter
  CATKIN_DEPENDS roscpp message_generation std_msgs nav_msgs sensor_msgs tf laser_geometry
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_to_point_cloud2_node src/scan_to_point_cloud2.cpp)
add_dependencies(scan_to_point_cloud2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_to_point_cloud2_node ${catkin_LIBRARIES})

