cmake_minimum_required(VERSION 3.0.2)
project(mia_hand_description)

find_package(catkin REQUIRED COMPONENTS roscpp urdf std_msgs genmsg sensor_msgs joint_limits_interface)


###################################
## catkin specific configuration ##
###################################

catkin_package(

  CATKIN_DEPENDS
    roscpp
    urdf
    sensor_msgs
    std_msgs
    joint_limits_interface

  INCLUDE_DIRS
    include


  LIBRARIES ${PROJECT_NAME}

)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} )

add_library(${PROJECT_NAME}
	src/mia_thumb_opp_passivejoints.cpp
	)

add_executable(remap_mia_joint_states
			src/remap_mia_joint_states.cpp
			src/mia_thumb_opp_passivejoints.cpp )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

target_link_libraries(remap_mia_joint_states ${catkin_LIBRARIES} yaml-cpp) 
add_dependencies(remap_mia_joint_states ${catkin_EXPORTED_TARGETS} )



#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/ calibration
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
