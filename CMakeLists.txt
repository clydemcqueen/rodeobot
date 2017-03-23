cmake_minimum_required(VERSION 2.8.12)

project(rodeobot)

set(PACKAGE_DEPENDENCIES
  ca_msgs
  geometry_msgs
  roscpp
  roslint
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

## Declare a catkin package
catkin_package()

## Require C++14
add_definitions(-std=c++14)

## Set up include directories
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

## Roslint
roslint_cpp()

## Wanderbot v3 nodelet
add_library(w3 src/w3.cpp)
target_link_libraries(w3 ${catkin_LIBRARIES})

## Wanderbot v3 node
add_executable(w3_node src/w3_node.cpp)
target_link_libraries(w3_node w3 ${catkin_LIBRARIES})

## Battery monitor node
add_executable(battery_monitor src/battery_monitor.cpp)
target_link_libraries(battery_monitor ${catkin_LIBRARIES})

## Wanderbot v2 nodelet: uses camera, costmap_2d and dwa_local_planner
# add_library(wanderbot src/wanderbot.cpp)
# target_link_libraries(wanderbot ${catkin_LIBRARIES})

## Wanderbot v2 node
# add_executable(wanderbot_node src/wanderbot_node.cpp)
# target_link_libraries(wanderbot_node wanderbot ${catkin_LIBRARIES})

## Wanderbot v1 nodelet: uses IR bumbers only
# add_library(wander src/wander.cpp)
# target_link_libraries(wander ${catkin_LIBRARIES})

## Wanderbot v1 node
# add_executable(wander_node src/wander_node.cpp)
# target_link_libraries(wander_node wander ${catkin_LIBRARIES})

## Build tests; note that we split wander into a library & node because we have tests
# catkin_add_gtest(wander_test test/wander_test.cpp)
# target_link_libraries(wander_test wander ${catkin_LIBRARIES})

## Namespace tests I did once
# add_executable(ns_test src/ns_test.cpp)
# target_link_libraries(ns_test ${catkin_LIBRARIES})

## Super simple drive controller, handy for calibrating distance
# add_executable(simple_drive src/simple_drive.cpp)
# target_link_libraries(simple_drive ${catkin_LIBRARIES})
