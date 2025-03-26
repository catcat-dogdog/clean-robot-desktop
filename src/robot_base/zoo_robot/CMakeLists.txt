cmake_minimum_required(VERSION 3.0.2)
project(zoo_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  tf2_ros
  tf2_eigen
  tf_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zoo_robot
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include include/zoo_robot/
)

