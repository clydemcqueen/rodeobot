cmake_minimum_required(VERSION 2.8.3)
project(rodeobot)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf)

## Declare a catkin package
catkin_package()

## Require C++14
add_definitions(-std=c++14)

## Build our nodes
include_directories(include ${catkin_INCLUDE_DIRS} ~/catkin_ws/devel_isolated/ca_msgs/include)

add_executable(ns_test src/ns_test.cpp)
target_link_libraries(ns_test ${catkin_LIBRARIES})

add_executable(battery_monitor src/battery_monitor.cpp)
target_link_libraries(battery_monitor ${catkin_LIBRARIES})

add_executable(simple_drive src/simple_drive.cpp)
target_link_libraries(simple_drive ${catkin_LIBRARIES})

add_library(wander src/wander.cpp)
target_link_libraries(wander ${catkin_LIBRARIES})

add_executable(wander_node src/wander_node.cpp)
target_link_libraries(wander_node wander ${catkin_LIBRARIES})

## Build tests; note that we split wander into a library & node because we have tests
catkin_add_gtest(wander_test test/wander_test.cpp)
target_link_libraries(wander_test wander ${catkin_LIBRARIES})
