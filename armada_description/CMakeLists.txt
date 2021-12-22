cmake_minimum_required(VERSION 2.8.3)
project(armada_description)

find_package(catkin REQUIRED)

catkin_package()

#if (CATKIN_ENABLE_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(tests/some_test.xml)
#endif()

install(DIRECTORY armada_test_station armada_tooling robotiq universal_robot
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
