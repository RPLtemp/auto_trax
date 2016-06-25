cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_tests)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

#################################################################
################### EXECUTABLE SPECIFIC #########################
#################################################################

## Declare a C++ executable
add_executable(wall_following_test_node
        src/wall_following_test_node.cpp
        src/wall_following_test_processor.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(wall_following_test_node
                          ${catkin_LIBRARIES}
                          ${roslib_LIBRARIES}
)
