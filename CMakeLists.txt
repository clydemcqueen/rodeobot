cmake_minimum_required(VERSION 2.8.3)
project(rodeobot)

## Find packages we need
find_package(catkin REQUIRED COMPONENTS roslint roscpp rospy std_msgs tf costmap_2d dwa_local_planner)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

## Declare a catkin package
catkin_package()

## Require C++14
add_definitions(-std=c++14)

## Set up include directories
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ~/catkin_ws/devel_isolated/ca_msgs/include
)

## Roslint
roslint_cpp()

## Wanderbot v2 nodelet: uses camera, costmap_2d and dwa_local_planner
add_library(wanderbot src/wanderbot.cpp)
target_link_libraries(wanderbot ${catkin_LIBRARIES})

## Wanderbot v2 node
add_executable(wanderbot_node src/wanderbot_node.cpp)
target_link_libraries(wanderbot_node wanderbot ${catkin_LIBRARIES})

## Battery monitor node
add_executable(battery_monitor src/battery_monitor.cpp)
target_link_libraries(battery_monitor ${catkin_LIBRARIES})

## Namespace tests
# add_executable(ns_test src/ns_test.cpp)
# target_link_libraries(ns_test ${catkin_LIBRARIES})

## Super simple drive controller, handy for calibrating distance
# add_executable(simple_drive src/simple_drive.cpp)
# target_link_libraries(simple_drive ${catkin_LIBRARIES})

## Wanderbot v1 nodelet: uses IR bumbers only
# add_library(wander src/wander.cpp)
# target_link_libraries(wander ${catkin_LIBRARIES})

## Wanderbot v1 node
# add_executable(wander_node src/wander_node.cpp)
# target_link_libraries(wander_node wander ${catkin_LIBRARIES})

## Build tests; note that we split wander into a library & node because we have tests
# catkin_add_gtest(wander_test test/wander_test.cpp)
# target_link_libraries(wander_test wander ${catkin_LIBRARIES})
