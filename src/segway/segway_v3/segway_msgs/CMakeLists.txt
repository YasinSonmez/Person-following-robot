cmake_minimum_required(VERSION 2.8.3)
project(segway_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(DIRECTORY msg FILES 
                                AuxPower.msg
                                ConfigCmd.msg
                                Configuration.msg
                                Dynamics.msg
                                Faultlog.msg
                                Propulsion.msg
                                Status.msg)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

                      
