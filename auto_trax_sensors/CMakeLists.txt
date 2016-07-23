cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_sensors)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  LIBRARIES auto_trax_occ_grid_manager auto_trax_laser_scan_processor auto_trax_image_processing
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport nav_msgs roscpp sensor_msgs std_msgs
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

add_library(auto_trax_image_processing src/library/image_processing.cpp)
target_link_libraries(auto_trax_image_processing ${catkin_LIBRARIES})
add_dependencies(auto_trax_image_processing ${catkin_EXPORTED_TARGETS})

add_library(auto_trax_occ_grid_manager src/library/occ_grid_manager.cpp)
target_link_libraries(auto_trax_occ_grid_manager ${catkin_LIBRARIES})
add_dependencies(auto_trax_occ_grid_manager ${catkin_EXPORTED_TARGETS})

add_library(auto_trax_laser_scan_processor src/library/laser_scan_processor.cpp)
target_link_libraries(auto_trax_laser_scan_processor ${catkin_LIBRARIES})
add_dependencies(auto_trax_laser_scan_processor ${catkin_EXPORTED_TARGETS})

add_executable(dead_reckoning_node src/dead_reckoning_node.cpp)
add_executable(line_tracking_node src/line_tracking_node.cpp)
add_executable(wall_detection_node src/wall_detection_node.cpp)
add_executable(laser_scan_merger_node src/laser_scan_merger_node.cpp)
add_executable(laser_scan_processor_node src/laser_scan_processor_node.cpp)
add_executable(sign_detection_node src/sign_detection_node.cpp)

target_link_libraries(dead_reckoning_node ${catkin_LIBRARIES} ${roslib_LIBRARIES})
target_link_libraries(line_tracking_node ${catkin_LIBRARIES} ${roslib_LIBRARIES} ${OpenCV_LIBRARIES}
  auto_trax_image_processing auto_trax_occ_grid_manager)
target_link_libraries(wall_detection_node ${catkin_LIBRARIES} ${roslib_LIBRARIES})
target_link_libraries(laser_scan_merger_node ${catkin_LIBRARIES} ${roslib_LIBRARIES}
  auto_trax_laser_scan_processor)
target_link_libraries(laser_scan_processor_node ${catkin_LIBRARIES} ${roslib_LIBRARIES}
  auto_trax_laser_scan_processor)
target_link_libraries(sign_detection_node ${catkin_LIBRARIES} ${roslib_LIBRARIES}
  ${OpenCV_LIBRARIES})
