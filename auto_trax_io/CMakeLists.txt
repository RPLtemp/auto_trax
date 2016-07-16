cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_io)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/JHPWMPCA9685.cpp
        )

############
# BINARIES #
############
cs_add_executable(driver_interface
        src/auto_trax_io_node.cpp
        )
target_link_libraries(driver_interface ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
