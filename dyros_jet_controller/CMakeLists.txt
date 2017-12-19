cmake_minimum_required(VERSION 2.8.3)
project(dyros_jet_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dyros_jet_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  realtime_tools
)
find_package(Eigen3 REQUIRED)
catkin_package(
    DEPENDS   dyros_jet_msgs geometry_msgs roscpp sensor_msgs std_msgs realtime_tools
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


file(GLOB_RECURSE HeaderFiles "include/*.h")
add_custom_target(thormang_ctrl_headers SOURCES ${HeaderFiles})


file(GLOB BRIDGE_SRC
  "src/control_base.cpp"
  "src/simulation_interface.cpp"
#  "src/hardware_interface.cpp"
)
file(GLOB WALKING_SRC
  "src/walking_controller/*.cpp"
)
file(GLOB UPPER_SRC
  "src/upperbody_controller/*.cpp"
)


add_library(dyros_jet_controller_lib
  ${BRIDGE_SRC}
#  ${WALKING_SRC}
#  ${UPPER_SRC}
)

add_executable(dyros_jet_controller src/dyros_jet_controller.cpp)
target_link_libraries(dyros_jet_controller_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
target_link_libraries(dyros_jet_controller
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    dyros_jet_controller_lib
)

add_dependencies(dyros_jet_controller_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(dyros_jet_controller ${catkin_EXPORTED_TARGETS})


SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE Debug)
