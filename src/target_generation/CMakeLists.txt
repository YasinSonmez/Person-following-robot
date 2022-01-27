cmake_minimum_required(VERSION 3.0.2)
project(target_generation)

set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...

include_directories(./src/)

SET(target_generation_HEADERS src/target_generation_node.h )
SET(target_generation_SOURCES src/target_generation_node.cpp )

find_package(catkin REQUIRED COMPONENTS roscpp rospy gazebo_msgs actionlib std_msgs geometry_msgs genmsg sensor_msgs nav_msgs move_base_msgs tf2_ros tf2 tf2_geometry_msgs leg_tracker)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/target_generation_node.cpp)

add_executable(target_generation_node ${target_generation_SOURCES} ${target_generation_HEADERS})

target_link_libraries(target_generation_node ${catkin_LIBRARIES})

add_dependencies(target_generation_node ${catkin_EXPORTED_TARGETS})