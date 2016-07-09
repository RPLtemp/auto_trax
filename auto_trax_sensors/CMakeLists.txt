cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_sensors)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(auto_trax_image_processing src/library/image_processing.cpp)
target_link_libraries(auto_trax_image_processing ${catkin_LIBRARIES})

cs_add_library(auto_trax_occ_grid_manager src/library/occ_grid_manager.cpp)
target_link_libraries(auto_trax_occ_grid_manager ${catkin_LIBRARIES})

cs_add_library(auto_trax_laser_scan_processor
        src/library/laser_scan_processor.cpp)
target_link_libraries(auto_trax_laser_scan_processor ${catkin_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(dead_reckoning_node src/dead_reckoning_node.cpp)
target_link_libraries(dead_reckoning_node
        ${catkin_LIBRARIES} ${roslib_LIBRARIES})

cs_add_executable(line_tracking_node src/line_tracking_node.cpp)
target_link_libraries(line_tracking_node
        ${catkin_LIBRARIES} ${roslib_LIBRARIES} ${OpenCV_LIBRARIES}
        auto_trax_image_processing auto_trax_occ_grid_manager)

cs_add_executable(wall_detection_node src/wall_detection_node.cpp)
target_link_libraries(wall_detection_node
        ${catkin_LIBRARIES} ${roslib_LIBRARIES})

cs_add_executable(laser_scan_merger_node src/laser_scan_merger_node.cpp)
target_link_libraries(laser_scan_merger_node
        ${catkin_LIBRARIES} ${roslib_LIBRARIES}
        auto_trax_laser_scan_processor)

cs_add_executable(laser_scan_processor_node src/laser_scan_processor_node.cpp)
target_link_libraries(laser_scan_processor_node
        ${catkin_LIBRARIES} ${roslib_LIBRARIES}
        auto_trax_laser_scan_processor)

##########
# EXPORT #
##########
cs_install()
cs_export()