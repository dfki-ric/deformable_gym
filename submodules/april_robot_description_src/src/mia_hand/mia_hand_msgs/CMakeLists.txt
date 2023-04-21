cmake_minimum_required(VERSION 3.0.2)
project(mia_hand_msgs)


find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)


add_message_files(

	DIRECTORY
		msg

	FILES
    GraspRef.msg
		FingersData.msg
		FingersStrainGauges.msg
		ComponentStatus.msg
)


add_service_files(

	DIRECTORY
		srv

	FILES
		StrReq.srv
		StrResp.srv
    ConnectSerial.srv
)


generate_messages(

	DEPENDENCIES
		std_msgs
)


catkin_package(

#  INCLUDE_DIRS include
#  LIBRARIES april_interfaces

  CATKIN_DEPENDS
  	message_runtime
  	std_msgs

#  DEPENDS system_lib
)


include_directories(${catkin_INCLUDE_DIRS})
