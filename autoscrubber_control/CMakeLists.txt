cmake_minimum_required(VERSION 2.8.3)
project(autoscrubber_control)

## SET(CMAKE_BUILD_TYPE Debug)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
)

## Declare a catkin package
catkin_package()

## Build global planner using record
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(autoscrubber_control src/autoscrubber_control.cpp)
target_link_libraries(autoscrubber_control ${catkin_LIBRARIES})
