cmake_minimum_required(VERSION 2.8.3)
project(walkyto)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


catkin_package(
 # INCLUDE_DIRS include
  LIBRARIES walkyto
  CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
)

add_executable(network_gen src/networks/network_gen.py)
target_link_libraries(network_gen ${catkin_LIBRARIES})

add_executable(simulator src/networks/simulator.py)
target_link_libraries(simulator ${catkin_LIBRARIES})