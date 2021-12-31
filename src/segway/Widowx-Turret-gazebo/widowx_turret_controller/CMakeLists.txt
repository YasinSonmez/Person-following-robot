cmake_minimum_required(VERSION 2.8.3)
project(widowx_turret_controller)

find_package(catkin REQUIRED COMPONENTS
  arbotix_python
  roscpp
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(pt_cmd src/pt_cmd.cpp)
target_link_libraries(pt_cmd ${catkin_LIBRARIES})
