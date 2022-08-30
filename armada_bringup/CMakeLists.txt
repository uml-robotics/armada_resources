cmake_minimum_required(VERSION 3.0.2)
project(armada_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  robotiq_2f_gripper_control
)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

## Generate actions in the 'action' folder
#add_action_files(
#  FILES
#)

## Generate added msg and srv with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#)

#catkin_package(
#  CATKIN_DEPENDS
#)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(r2f85_joint_state_publisher src/r2f85_joint_state_publisher.cpp)

target_link_libraries(r2f85_joint_state_publisher ${catkin_LIBRARIES})
