cmake_minimum_required(VERSION 3.0.2)
project(justina_tools)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  hardware_msgs
  image_transport
  manip_msgs
  message_generation
  nav_msgs
  pcl_conversions
  roscpp
  roslib
  rospy
  sensor_msgs
  sound_play
  std_msgs
  std_srvs
  tf
  tf_conversions
  actionlib_msgs
)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   hardware_msgs#   manip_msgs#   nav_msgs#   sensor_msgs#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES justina_tools
  CATKIN_DEPENDS cv_bridge geometry_msgs hardware_msgs image_transport manip_msgs message_generation nav_msgs pcl_conversions roscpp roslib rospy sensor_msgs sound_play std_msgs std_srvs tf tf_conversions
#  DEPENDS system_lib
)

include_directories(
 include
${catkin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

add_library(justina_tools
  src/JustinaTools.cpp
  src/JustinaVision.cpp
  src/JustinaNavigation.cpp
)

add_dependencies(justina_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(justina_tools
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
