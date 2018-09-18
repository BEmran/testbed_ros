cmake_minimum_required(VERSION 2.8.3)
project(testbed_up)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

## Specify additional locations of header files
include_directories(
   include
   include/lib
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/lib/Encoder.cpp
  include/lib/TimeSampling.cpp
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  INCLUDE_DIRS include/lib/
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)

add_executable(up_basic src/up_basic.cpp)
target_link_libraries(up_basic ${catkin_LIBRARIES} ${PROJECT_NAME} usdusb4)

#target_link_libraries(demo ${catkin_LIBRARIES} ${PROJECT_NAME}
# ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/lib/Navio/libnavio.a)

