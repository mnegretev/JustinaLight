cmake_minimum_required(VERSION 3.0.2)
project(line_finder)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  justina_tools
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  vision_msgs
  pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES line_finder
#  CATKIN_DEPENDS geometry_msgs justina_tools roscpp rospy sensor_msgs std_msgs tf vision_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(line_finder_node src/line_finder_node.cpp)

add_dependencies(line_finder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(line_finder_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
