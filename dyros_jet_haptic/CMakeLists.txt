cmake_minimum_required(VERSION 2.8.3)
project(dyros_jet_haptic)

find_package(catkin REQUIRED COMPONENTS
    roscpp
dyros_jet_msgs
)

include_directories(
  ${CMAKE_SOURCE_DIR}/dyros_jet/dyros_jet_haptic/include/sdk-3.6.0/include
${catkin_INCLUDE_DIRS}
)



catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    dyros_jet_msgs
)


link_directories(${CMAKE_SOURCE_DIR}/dyros_jet/dyros_jet_haptic/include/sdk-3.6.0/lib/release/lin-x86_64-gcc)

add_executable(dyros_jet_haptic src/dyros_haptic.cpp)
target_link_libraries(dyros_jet_haptic 
  ${CMAKE_SOURCE_DIR}/dyros_jet/dyros_jet_haptic/include/sdk-3.6.0/lib/release/lin-x86_64-gcc/libdhd.a 
  ${CMAKE_SOURCE_DIR}/dyros_jet/dyros_jet_haptic/include/sdk-3.6.0/lib/release/lin-x86_64-gcc/libdhd.so.3.6.0 
  -lpthread -ldhd -lrt -ldl -lusb-1.0
  ${catkin_LIBRARIES}
)

file(GLOB_RECURSE HeaderFiles "include/*.h")
