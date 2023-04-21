cmake_minimum_required(VERSION 3.0.2)
project(sr_robot_msgs)

set(MSG_DEPS
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  moveit_msgs
)

find_package(catkin REQUIRED COMPONENTS roslib message_generation ${MSG_DEPS})

add_message_files(
  FILES
  ActuatorStatistics.msg
  AuxSpiData.msg
  BiotacAll.msg
  Biotac.msg
  cartesian_data.msg
  cartesian_position.msg
  command.msg
  config.msg
  contrlr.msg
  ControllerStatistics.msg
  ControlType.msg
  EthercatDebug.msg
  FromMotorDataType.msg
  GraspArray.msg
  JointControllerState.msg
  joint.msg
  JointMusclePositionControllerState.msg
  JointMuscleValveControllerCommand.msg
  JointMuscleValveControllerState.msg
  joints_data.msg
  JointStatistics.msg
  MechanismStatistics.msg
  MidProxDataAll.msg
  MidProxData.msg
  MotorSystemControls.msg
  reverseKinematics.msg
  sendupdate.msg
  ShadowPST.msg
  TactileArray.msg
  Tactile.msg
  UBI0All.msg
  UBI0.msg
  MSTSensor.msg
  MSTFinger.msg
  MSTPalm.msg
  ShadowContactStateStamped.msg
  WrenchWithModelGroupName.msg
  WrenchArray.msg
  HybridControllerStatus.msg
)

add_service_files(
  FILES
  ChangeControlType.srv
  ChangeMotorSystemControls.srv
  ForceController.srv
  GetSegmentedLine.srv
  is_hand_occupied.srv
  ManualSelfTest.srv
  NullifyDemand.srv
  RobotTeachMode.srv
  SetDebugData.srv
  SetEffortControllerGains.srv
  SetMixedPositionVelocityPidGains.srv
  SetPidGains.srv
  SimpleMotorFlasher.srv
  which_fingers_are_touching.srv
  GetFastGraspFromBoundingBox.srv
  PlanTrajectoryFromList.srv
  PlanTrajectoryFromPrefix.srv
  PlanNamedTrajectory.srv
  ExecutePlannedTrajectory.srv
  ListNamedTrajectories.srv
  SetTeachMode.srv
  SetImuScale.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  Grasp.action
  PlanGrasp.action
)

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roslib message_runtime ${MSG_DEPS}
  INCLUDE_DIRS
  LIBRARIES
)
