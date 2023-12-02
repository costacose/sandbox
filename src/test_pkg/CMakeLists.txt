cmake_minimum_required(VERSION 3.0.2)
project(test_pkg)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
)

## Catkin specific configuration ##
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
