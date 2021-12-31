cmake_minimum_required(VERSION 2.8.3)
project(segway_navigation_apps)

find_package(catkin REQUIRED)

## Declare a catkin package
catkin_package()

install(DIRECTORY config maps launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif ()
