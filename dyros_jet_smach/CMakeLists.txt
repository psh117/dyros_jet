cmake_minimum_required(VERSION 2.8.3)
project(dyros_jet_smach)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  std_srvs
  turtlesim
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

