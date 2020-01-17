cmake_minimum_required(VERSION 2.8.3)
project(tacotron2_ros)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS
  #  LIBRARIES qt_node_editor
  #  CATKIN_DEPENDS behaviortree_cpp_v3
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
