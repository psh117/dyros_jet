cmake_minimum_required(VERSION 2.8.3)
project(dyros_jet_mission_commander)

find_package(catkin REQUIRED COMPONENTS
    dyros_jet_msgs
    )
find_package(Eigen3 REQUIRED)
catkin_package(
    DEPENDS   dyros_jet_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS 
    LIBRARIES # TODO
    )
include_directories()

