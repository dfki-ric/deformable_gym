cmake_minimum_required(VERSION 3.0.2)
project(mia_hand_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package()


#############
## Install ##
#############

# Mark launch directory for installation
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark config directory for installation
install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
