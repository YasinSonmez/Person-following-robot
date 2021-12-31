cmake_minimum_required(VERSION 2.8.3)
project(segway_v3_config)

find_package(catkin REQUIRED)

catkin_package()

install(FILES segway_config.bash
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_env_hooks(50.segway_config SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
