cmake_minimum_required(VERSION 2.8.3)
project(tacotron2_ros)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib_msgs actionlib)
add_action_files(DIRECTORY action FILES TTS.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  INCLUDE_DIRS
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
