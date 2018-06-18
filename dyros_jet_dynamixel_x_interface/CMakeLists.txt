cmake_minimum_required(VERSION 2.8.3)
project(dyros_jet_dynamixel_x_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  std_msgs
  realtime_tools
  tf
  dynamixel_sdk
  dyros_jet_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  DEPENDS   roscpp sensor_msgs std_msgs realtime_tools dynamixel_sdk
  CATKIN_DEPENDS # TODO
  INCLUDE_DIRS include
  LIBRARIES # TODO
  )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(x_interface src/x_interface.cpp src/dxl_handler.cpp)


target_link_libraries(x_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(x_interface ${catkin_EXPORTED_TARGETS})


