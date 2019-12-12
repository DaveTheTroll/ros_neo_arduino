cmake_minimum_required(VERSION 2.8.3)
project(ros_neo_arduino)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  NeoLeds.msg
  NeoMode.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

