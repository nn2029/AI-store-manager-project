cmake_minimum_required(VERSION 2.8.3)
project(rosbot_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
)


catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs cv_bridge std_msgs
)

