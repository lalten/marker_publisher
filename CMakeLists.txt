cmake_minimum_required(VERSION 2.8.3)
project(marker_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1z)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -march=native")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  sensor_msgs
  image_transport
  roscpp
  std_msgs
  message_generation
  tf
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Marker.msg
   MarkerArray.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${aruco_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(markerpub
  src/marker_pub.cpp
  src/marker_pub_main.cpp
)
## Add cmake target dependencies of the executable
add_dependencies(markerpub
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(markerpub
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${aruco_LIBS}
)

#############
## Install ##
#############


#############
## Testing ##
#############
