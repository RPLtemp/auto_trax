cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_localization)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen REQUIRED)
include_directories($(Eigen_INCLUDE_DIRS))

add_definitions(-std=c++11)


cs_add_library(${PROJECT_NAME}
        src/localization_node.cpp
        src/particle_filter.cpp
        src/wheel_bot.cpp)



cs_add_executable(localization_node
        main.cpp)
target_link_libraries(localization_node ${PROJECT_NAME})

cs_install()
cs_export()