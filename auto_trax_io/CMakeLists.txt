cmake_minimum_required(VERSION 2.8.3)
project(auto_trax_io)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ackermann_msgs
  message_generation
  rtimulib_ros
)

add_service_files(
  FILES
  ApplySteeringAngle.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ackermann_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs ackermann_msgs message_runtime rtimulib_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
include
)

## Declare a C++ executable
add_executable(auto_trax_io
                    src/auto_trax_io_node.cpp
                    src/JHPWMPCA9685.cpp)
                    

add_dependencies(auto_trax_io auto_trax_io_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(auto_trax_io
                          ${catkin_LIBRARIES}
                          ${roslib_LIBRARIES}
 )
