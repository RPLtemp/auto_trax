cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_gazebo)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(gazebo REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(move_cart
        src/move_cart_plugin.cpp
        )
target_link_libraries(move_cart
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )

##########
# EXPORT #
##########
cs_install()
cs_export()