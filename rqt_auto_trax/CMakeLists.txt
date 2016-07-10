cmake_minimum_required(VERSION 2.8.3)
project(rqt_auto_trax)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

##########
# EXPORT #
##########
cs_install()
cs_export()
