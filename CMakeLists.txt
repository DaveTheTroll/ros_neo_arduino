cmake_minimum_required(VERSION 2.8.3)
project(ros_neo_arduino)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
find_package(cmake_modules REQUIRED)

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
  INCLUDE_DIRS include
  DEPENDS std_msgs
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(neo_test src/test_main.cpp)
target_link_libraries(neo_test ${catkin_LIBRARIES})
add_dependencies(neo_test ros_neo_arduino_generate_messages_cpp)

