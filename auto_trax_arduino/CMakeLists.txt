cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_arduino)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############

############
# BINARIES #
############

##########
# EXPORT #
##########
cs_install()
cs_export()
