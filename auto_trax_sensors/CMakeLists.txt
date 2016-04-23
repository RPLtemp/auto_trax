cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_sensors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
  DEPENDS opencv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(dead_reckoning src/dead_reckoning.cpp)
add_executable(line_tracking src/line_tracking.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dead_reckoning ${catkin_LIBRARIES} ${roslib_LIBRARIES})
target_link_libraries(line_tracking ${catkin_LIBRARIES} ${roslib_LIBRARIES} ${OpenCV_LIBRARIES})
