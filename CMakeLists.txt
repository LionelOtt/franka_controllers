cmake_minimum_required(VERSION 2.8.3)
project(franka_controllers)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
    catkin
    REQUIRED
    COMPONENTS    
        controller_interface
        franka_hw
        hardware_interface
        pluginlib
        realtime_tools
        roscpp
        sensor_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES franka_controllers
    CATKIN_DEPENDS
        controller_interface
        franka_hw
        hardware_interface
        pluginlib
        realtime_tools
        roscpp
        sensor_msgs
    #DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_library(
    franka_controllers
    src/joint_position_controller.cpp
)
target_link_libraries(
    franka_controllers
    ${catkin_LIBRARIES}
)

