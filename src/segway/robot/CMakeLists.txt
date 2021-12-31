cmake_minimum_required(VERSION 2.8.3)

project(robot)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
		tf
		geometry_msgs
		nav_msgs
		roscpp
		rospy
		std_msgs
		roslaunch)

catkin_package()

include_directories(
		#include
		${catkin_INCLUDE_DIRS}
)

add_executable(tf_publisher src/main.cpp)

target_link_libraries(tf_publisher
		${catkin_LIBRARIES}
		)
foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
