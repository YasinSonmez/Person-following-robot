cmake_minimum_required(VERSION 2.8.3)
project(segway_ros)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)

## Uncomment if the package has a setup.py
catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/segway.cfg)

catkin_package()

catkin_install_python(PROGRAMS
                        bin/segway_config_eth
                        bin/segway_driver
                        bin/segway_faultlog_parser
                        bin/segway_marker_ctrl
                        bin/segway_move_base
                        bin/segway_teleop
                        bin/segway_wd
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                      
