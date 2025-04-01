cmake_minimum_required(VERSION 2.8.3)

project(omnifinal)

find_package(catkin REQUIRED
	roscpp
	rospy
##	message_generation
##	std_msgs
##	geometry_msgs
	urdf
	xacro
)

catkin_package(CATKIN_DEPENDS
	roscpp
	rospy
##	std_msgs
##	geometry_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
add_executable(omnifinal_teleop src/keyboard_teleop.cpp)
add_executable(omnifinal_arm_control src/keyboard_arm_control.cpp)
target_link_libraries(omnifinal_teleop ${catkin_LIBRARIES})
target_link_libraries(omnifinal_arm_control ${catkin_LIBRARIES})