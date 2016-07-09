cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_tests)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(wall_following_test
        src/wall_following_test_processor.cpp)

############
# BINARIES #
############
cs_add_executable(wall_following_test_node
        src/wall_following_test_node.cpp)
target_link_libraries(wall_following_test_node wall_following_test)

##########
# EXPORT #
##########
cs_install()
cs_export()