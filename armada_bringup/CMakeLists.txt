cmake_minimum_required(VERSION 3.0.2)
project(armada_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  genmsg
  actionlib
  actionlib_msgs
  std_msgs
  sensor_msgs
  control_msgs
  message_generation
  robotiq_2f_gripper_control
  tf
)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ActivateGripper.srv
)

## Generate actions in the 'action' folder
#add_action_files(
#  FILES
#)

## Generate added msg and srv with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(r2f85_control_server src/actionserver/r2f85_control_server.cpp)
add_executable(r2f85_joint_state_publisher src/r2f85_joint_state_publisher.cpp)

target_link_libraries(r2f85_control_server ${catkin_LIBRARIES})
target_link_libraries(r2f85_joint_state_publisher ${catkin_LIBRARIES})
