cmake_minimum_required(VERSION 3.0.2)
project(pid_tuner)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
	rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES pid_tuner
  CATKIN_DEPENDS dynamic_reconfigure rospy std_msgs
  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/pid_tuner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
