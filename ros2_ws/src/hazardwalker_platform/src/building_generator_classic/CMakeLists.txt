cmake_minimum_required(VERSION 3.0.2)
project(building_generator_classic)

find_package(catkin REQUIRED COMPONENTS
  building_generator_core
  building_generator_interfaces
  gazebo_msgs
  rospy
  tf
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  building_generator_core
  building_generator_interfaces
  gazebo_msgs
  rospy
  tf
)

catkin_install_python(PROGRAMS
  scripts/building_generator_classic_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
