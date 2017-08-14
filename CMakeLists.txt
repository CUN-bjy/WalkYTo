cmake_minimum_required(VERSION 2.8.3)
project(walkyto)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

catkin_python_setup()

add_service_files(
  DIRECTORY srv 
  FILES SimRun.srv
)

## Generate services in the 'srv' folder
# add_service_files(
#  FILES  # e.g. Floats.srv HeaderString.srv
#)

## Generate added messages and services with any dependencies
generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime rospy
)