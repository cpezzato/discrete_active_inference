cmake_minimum_required(VERSION 3.5)
project(behavior_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  mobile_control
  mobile_manipulator
  mobile_moveit
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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib_msgs
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS scripts/AIPBT_server.py scripts/symbolic_perception_server.py scripts/testing/symbolic_perception_client.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/nodes/AIRLabAction_nodes.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(demo_executeBT src/demoBT.cpp)
# add_dependencies(demo_executeBT ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(demo_executeBT ${catkin_LIBRARIES} ${PROJECT_NAME})
