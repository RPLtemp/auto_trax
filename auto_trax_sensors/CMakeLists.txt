cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_sensors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES occ_grid_manager
  CATKIN_DEPENDS cv_bridge image_transport nav_msgs roscpp sensor_msgs
  DEPENDS opencv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

add_library(occ_grid_manager src/library/occ_grid_manager.cpp)

target_link_libraries(occ_grid_manager ${catkin_LIBRARIES})
add_dependencies(occ_grid_manager ${catkin_EXPORTED_TARGETS})

## Declare the C++ executables
add_executable(dead_reckoning src/dead_reckoning.cpp)
add_executable(line_tracking src/line_tracking.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dead_reckoning ${catkin_LIBRARIES} ${roslib_LIBRARIES})
target_link_libraries(line_tracking ${catkin_LIBRARIES} ${roslib_LIBRARIES} ${OpenCV_LIBRARIES} occ_grid_manager)
