cmake_minimum_required(VERSION 3.0.2)
project(mia_hand_ros_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  mia_hand_driver
  mia_hand_description
  std_msgs
  hardware_interface
  controller_manager
  roscpp
  rospy
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
  rqt_joint_trajectory_controller
  )



###################################
## catkin specific configuration ##
###################################

catkin_package(

  INCLUDE_DIRS
    include

  #LIBRARIES ${PROJECT_NAME}

  CATKIN_DEPENDS
    mia_hand_driver
    mia_hand_description
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface
    rqt_joint_trajectory_controller
)


###########
## Build ##
###########

include_directories( include ${catkin_INCLUDE_DIRS}  )


add_library(${PROJECT_NAME}
            src/Mia_hw_node.cpp
            src/mia_mrl_transmission.cpp
            src/mia_thfle_transmission.cpp
            src/mia_index_transmission.cpp
            src/mia_hw_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)



add_executable(Mia_hw_node src/Mia_hw_node.cpp)

add_dependencies(Mia_hw_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(Mia_hw_node ${PROJECT_NAME})




#############
## Install ##
#############

## Useful links:
#
# https://docs.ros.org/en/noetic/api/catkin/html/howto/format2/building_executables.html
# https://docs.ros.org/en/noetic/api/catkin/html/howto/format2/building_libraries.html

# Mark executables for installation
#
install(
  TARGETS Mia_hw_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark libraries for installation
#
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
#
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
