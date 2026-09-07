cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_control)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    realtime_tools
    unitree_legged_msgs
)

catkin_package(
    CATKIN_DEPENDS
    unitree_legged_msgs
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_library( unitree_legged_control 
    src/joint_controller.cpp
    src/unitree_joint_control_tool.cpp
)
add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)
target_link_libraries(unitree_legged_control ${catkin_LIBRARIES})
# Gazebo/controller_manager 会在控制周期中反复释放再打开控制器插件。
# class_loader 的工厂是进程全局对象，重复装载不仅刷出海量 namespace
# collision 日志，还会不断覆盖并泄漏工厂对象。关节控制器与 Gazebo 同寿命，
# 标记 NODELETE 可让动态库首次加载后常驻，静态注册只执行一次。
set_target_properties(unitree_legged_control PROPERTIES
    LINK_FLAGS "-Wl,-z,nodelete")
