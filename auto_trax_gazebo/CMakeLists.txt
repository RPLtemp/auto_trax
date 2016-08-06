cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_gazebo)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(move_cart
        src/move_cart_plugin.cpp
        )

cs_add_library(occupancy_grid
        src/occupancy_grid_plugin.cpp
        )

target_link_libraries(move_cart
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )

target_link_libraries(occupancy_grid
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )

##########
# EXPORT #
##########
cs_install()
cs_export()
