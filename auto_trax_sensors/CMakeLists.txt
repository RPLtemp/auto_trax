cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_sensors)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

link_directories(${CATKIN_DEVEL_PREFIX}/../../lib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

message(${OpenCV_INCLUDE_DIRS} " " ${OpenCV_VERSION})

#cs_add_library(auto_trax_image_processing src/library/image_processing.cpp)
cs_add_library(auto_trax_occ_grid_manager src/library/occ_grid_manager.cpp)
cs_add_library(auto_trax_laser_scan_processor src/library/laser_scan_processor.cpp)

cs_add_executable(dead_reckoning_node src/dead_reckoning_node.cpp)
target_link_libraries(dead_reckoning_node ${catkin_LIBRARIES} ${roslib_LIBRARIES})

#cs_add_executable(line_tracking_node src/line_tracking_node.cpp)
#target_link_libraries(line_tracking_node ${catkin_LIBRARIES} ${roslib_LIBRARIES} ${OpenCV_LIBRARIES}
   #auto_trax_image_processing auto_trax_occ_grid_manager) 

cs_add_executable(wall_detection_node src/wall_detection_node.cpp)
target_link_libraries(wall_detection_node ${catkin_LIBRARIES} ${roslib_LIBRARIES})

cs_add_executable(laser_scan_merger_node src/laser_scan_merger_node.cpp)
target_link_libraries(laser_scan_merger_node ${catkin_LIBRARIES} ${roslib_LIBRARIES}
  auto_trax_laser_scan_processor)

cs_add_executable(laser_scan_processor_node src/laser_scan_processor_node.cpp)
target_link_libraries(laser_scan_processor_node ${catkin_LIBRARIES} ${roslib_LIBRARIES}
  auto_trax_laser_scan_processor)

#cs_add_executable(sign_detection_node src/sign_detection_node.cpp)
#target_link_libraries(sign_detection_node ${catkin_LIBRARIES} ${roslib_LIBRARIES}
#  ${OpenCV_LIBRARIES})

#cs_add_executable(april_tag_detection_node src/april_tag_detection_node.cpp)
#add_dependencies(april_tag_detection_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(april_tag_detection_node ${catkin_LIBRARIES} ${roslib_LIBRARIES} apriltags)

cs_install()

cs_export()
