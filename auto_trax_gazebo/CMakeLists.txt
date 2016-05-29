cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(auto_trax_gazebo src/${PROJECT_NAME}/auto_trax_gazebo.cpp)

# add_executable(auto_trax_gazebo_node src/auto_trax_gazebo_node.cpp)

# target_link_libraries(auto_trax_gazebo_node ${catkin_LIBRARIES})
