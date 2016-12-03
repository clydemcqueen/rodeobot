cmake_minimum_required(VERSION 2.8.3)
project(rodeobot)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

## Declare a catkin package
catkin_package()

## Require C++14
add_definitions(-std=c++14)

## Build our node
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(battery_monitor src/battery_monitor.cpp)
target_link_libraries(battery_monitor ${catkin_LIBRARIES})
