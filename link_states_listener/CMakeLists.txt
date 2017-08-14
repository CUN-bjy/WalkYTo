cmake_minimum_required(VERSION 2.8.3)
project(link_states_listener)

catkin_python_setup()

find_package(
  catkin REQUIRED COMPONENTS
  gazebo_msgs
  rospy
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES link_states_listener
  CATKIN_DEPENDS gazebo_msgs rospy
  DEPENDS system_lib
)
