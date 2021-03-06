cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_path_planning)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/occupancy_grid_processor.cpp
        src/rrt_path_processor.cpp
        src/rrt_path_unit_test.cpp
        include/rrt/2dplane/2dplane.cpp
        include/rrt/2dplane/GridStateSpace.cpp
        include/rrt/2dplane/ObstacleGrid.cpp
        )

############
# BINARIES #
############
cs_add_executable(emergency_stop_node
        src/emergency_stop_node.cpp
        )

cs_add_executable(drive_to_centroid_node
        src/drive_to_centroid.cpp
        )

cs_add_executable(occupancy_grid_node
        src/occupancy_grid_node.cpp
        )

target_link_libraries(occupancy_grid_node ${PROJECT_NAME})

cs_add_executable(rrt_path_node
        src/rrt_path_node.cpp
        )

target_link_libraries(rrt_path_node ${PROJECT_NAME})

cs_add_executable(rrt_unit_test_node
        src/rrt_unit_test_node.cpp
        )

target_link_libraries(rrt_unit_test_node ${PROJECT_NAME})



##########
# EXPORT #
##########
cs_install()
cs_export()