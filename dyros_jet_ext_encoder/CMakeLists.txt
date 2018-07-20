cmake_minimum_required(VERSION 2.8.3)
project(dyros_jet_ext_encoder)

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
    INCLUDE_DIRS
    LIBRARIES # TODO
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


add_executable(dyros_jet_ext_encoder src/dyros_jet_ext_encoder.cpp)
add_executable(udp_client src/udp_client.cpp)

target_link_libraries(dyros_jet_ext_encoder
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
target_link_libraries(udp_client
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_dependencies(dyros_jet_ext_encoder ${catkin_EXPORTED_TARGETS})
add_dependencies(udp_client ${catkin_EXPORTED_TARGETS})

SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE Debug)
