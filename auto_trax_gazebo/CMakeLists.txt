cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_cart
  CATKIN_DEPENDS gazebo_ros roscpp std_msgs
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(move_cart src/move_cart_plugin.cpp)

target_link_libraries(move_cart ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
