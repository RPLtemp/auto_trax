cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_sensors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES auto_trax_image_processing auto_trax_occ_grid_manager
  CATKIN_DEPENDS cv_bridge image_transport nav_msgs roscpp sensor_msgs
  DEPENDS opencv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

include_directories(${Eigen_INCLUDE_DIRS})

add_library(auto_trax_image_processing src/library/image_processing.cpp)
add_library(auto_trax_occ_grid_manager src/library/occ_grid_manager.cpp)

target_link_libraries(auto_trax_image_processing ${catkin_LIBRARIES})
add_dependencies(auto_trax_image_processing ${catkin_EXPORTED_TARGETS})

target_link_libraries(auto_trax_occ_grid_manager ${catkin_LIBRARIES})
add_dependencies(auto_trax_occ_grid_manager ${catkin_EXPORTED_TARGETS})

add_executable(dead_reckoning_node src/dead_reckoning_node.cpp)
add_executable(line_tracking_node src/line_tracking_node.cpp)

target_link_libraries(dead_reckoning_node ${catkin_LIBRARIES} ${roslib_LIBRARIES})
target_link_libraries(line_tracking_node ${catkin_LIBRARIES} ${roslib_LIBRARIES} ${OpenCV_LIBRARIES}
  auto_trax_image_processing auto_trax_occ_grid_manager)
