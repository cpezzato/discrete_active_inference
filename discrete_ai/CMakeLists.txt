cmake_minimum_required(VERSION 3.5)
project(behavior_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  move_base_msgs
  behaviortree_cpp_v3
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate services in the srv folder
add_service_files(
  FILES
  symbolicPerception.srv
)

add_action_files(
  DIRECTORY action
  FILES AIPBT.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bt_tutorial
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib_msgs
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS scripts/active_inference_server.py scripts/tiago_perception.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/nodes/AIRLabAction_nodes.cpp
)

add_executable(demo_executeBT src/demoBT.cpp)
#add_dependencies(demo_executeBT ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(demo_executeBT ${catkin_LIBRARIES} ${PROJECT_NAME})
