cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_controller)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/controller_processor.cpp
        src/pid.cpp
        )

############
# BINARIES #
############
cs_add_executable(controller_node
        src/controller_node.cpp
        )
target_link_libraries(controller_node ${PROJECT_NAME})

cs_add_executable(setpoint_plantstate_node
        src/setpoint_plantstate_node.cpp
        )

##########
# EXPORT #
##########
cs_install()
cs_export()
