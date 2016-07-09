cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_msgs)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

add_definitions(-std=c++11)

##########
# EXPORT #
##########
cs_install()
cs_export()
