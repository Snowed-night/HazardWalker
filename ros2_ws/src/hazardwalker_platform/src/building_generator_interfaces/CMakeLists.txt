cmake_minimum_required(VERSION 3.0.2)
project(building_generator_interfaces)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_service_files(
  FILES
  CallElevator.srv
  SetDoorState.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
