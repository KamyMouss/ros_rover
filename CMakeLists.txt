cmake_minimum_required(VERSION 2.8.3)
project(yqb_car)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  WheelStatus.msg
  CameraControl.msg
  CameraStatus.msg
  AccelGyroMagMsg.msg
  Barometer.msg
  GPS.msg
  ADC.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)