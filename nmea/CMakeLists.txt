cmake_minimum_required(VERSION 2.8.3)
project(mmpug_nmea)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

## Declare a C++ executable
add_executable(nmea_node
  src/lib/udp.cpp
  src/lib/timeoutserial.cpp
  src/nmea_node.cpp)

## Specify libraries to link a library or executable target against
message(INFO "${catkin_LIBRARIES}")
target_link_libraries(nmea_node ${catkin_LIBRARIES})

