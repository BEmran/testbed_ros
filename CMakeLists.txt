cmake_minimum_required(VERSION 2.8.3)
project(testbed_up)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/lib/Encoder.cpp
  include/lib/SamplingTime.cpp
)

## Specify additional locations of header files
include_directories(
   include
  ${catkin_INCLUDE_DIRS}
)

add_executable(up_basic src/up_basic.cpp)
target_link_libraries(up_basic  ${catkin_LIBRARIES} ${PROJECT_NAME} usdusb4)

