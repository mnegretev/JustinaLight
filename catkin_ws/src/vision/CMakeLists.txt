cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
	pcl_conversions
	pcl_ros
	roscpp
	rospy
	roslint
	sensor_msgs
	message_generation
)
#
catkin_python_setup()
#
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)


include_directories(
# include segmentation_ud
# Segmentation20
  ${catkin_INCLUDE_DIRS}
)
add_executable(pcl_write src/pcl_write.cpp)
add_executable(clouds_reader src/clouds_reader.cpp)
add_executable(Segmentation_modified src/Segmentation_modified.cpp)
add_executable(Segmentation_OK src/Segmentation_OK.cpp)
target_link_libraries(Segmentation_OK ${catkin_LIBRARIES})
target_link_libraries(Segmentation_modified ${catkin_LIBRARIES})
target_link_libraries(clouds_reader ${catkin_LIBRARIES})
target_link_libraries(pcl_write ${catkin_LIBRARIES})


