cmake_minimum_required(VERSION 2.8.3)
project(dyros_jet_haptic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dyros_jet_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    dyros_jet_msgs
)


link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/release/lin-x86_64-gcc)

add_executable(dyros_jet_haptic src/dyros_haptic.cpp)

target_link_libraries(dyros_jet_haptic 
  #${CMAKE_CURRENT_SOURCE_DIR}/lib/release/lin-x86_64-gcc/libdhd.a 
  #${CMAKE_CURRENT_SOURCE_DIR}/lib/release/lin-x86_64-gcc/libdhd.so.3.6.0
  -lpthread -ldhd -lrt -ldl -lusb-1.0
  ${catkin_LIBRARIES}
)

add_dependencies(dyros_jet_haptic ${catkin_EXPORTED_TARGETS})
