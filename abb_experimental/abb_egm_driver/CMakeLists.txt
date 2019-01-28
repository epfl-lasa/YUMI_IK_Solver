cmake_minimum_required(VERSION 2.8.3)

project(abb_egm_driver)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    actionlib_msgs
    control_msgs
    industrial_msgs
    industrial_robot_client
    industrial_utils
    roscpp
    sensor_msgs
    simple_message
    std_msgs
    trajectory_msgs
    urdf
    abb_egm_interface
    abb_rws_interface
)

find_package(Boost REQUIRED COMPONENTS system thread)



###################################
## Catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
      actionlib
      actionlib_msgs
      control_msgs
      industrial_msgs
      industrial_robot_client
      industrial_utils
      roscpp
      sensor_msgs
      std_msgs
      trajectory_msgs
      urdf
      abb_egm_interface
      abb_rws_interface
    DEPENDS Boost
)



###########
## Build ##
###########
set(SRC_FILES
    include/abb_egm_driver/abb_joint_trajectory_action_abstract.h
    include/abb_egm_driver/abb_joint_trajectory_actions.h
    include/abb_egm_driver/abb_publisher_handler.h

    src/abb_generic_joint_trajectory_action_node.cpp
    src/abb_joint_trajectory_action_abstract.cpp
    src/abb_joint_trajectory_action_dual_armed.cpp
    src/abb_joint_trajectory_action_single_arm.cpp
    src/abb_publisher_handler.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_joint_trajectory_action ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}_joint_trajectory_action
  industrial_robot_client
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_joint_trajectory_action
  industrial_robot_client_gencpp
  ${catkin_EXPORTED_TARGETS}
)
set_target_properties(${PROJECT_NAME}_joint_trajectory_action
  PROPERTIES OUTPUT_NAME joint_trajectory_action
  PREFIX ""
)



#############
## Install ##
############# 
install(TARGETS
  ${PROJECT_NAME}_joint_trajectory_action
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
