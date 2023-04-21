cmake_minimum_required(VERSION 3.6.0)
project(mia_hand_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp mia_hand_msgs std_srvs)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED IMPORTED_TARGET libserial>=1.0.0)


###################################
## catkin specific configuration ##
###################################

catkin_package(
	CATKIN_DEPENDS roscpp mia_hand_msgs std_srvs
  DEPENDS LIBSERIAL
	INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)


###########
## Build ##
###########

include_directories(include ${LIBSERIAL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
	src/serial_port.cpp
	src/cpp_driver.cpp
  src/ros_driver.cpp
)

target_link_libraries(${PROJECT_NAME}
  Threads::Threads
  PkgConfig::LIBSERIAL
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/mia_hand_driver_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})


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
  TARGETS ${PROJECT_NAME}_node
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

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_april.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
