cmake_minimum_required(VERSION 2.8.3)
project(dyros_jet_moveit_depend)


find_package(catkin REQUIRED COMPONENTS
  dyros_jet_msgs
  roscpp
  rospy
  std_msgs
  tf
  actionlib_msgs
  actionlib
  control_msgs
  sensor_msgs
  eigen_conversions
)
find_package(Eigen3 REQUIRED)


catkin_package(
    DEPENDS   dyros_jet_msgs eigen_conversions
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
    )


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ./include/dyros_jet_moveit_depend
)

file(GLOB_RECURSE HeaderFiles "include/dyros_jet_moveit_depend/*.h")
add_custom_target(dyros_jet_moveit_depend_headers SOURCES ${HeaderFiles})

add_library(dyros_jet_moveit_depend_lib
    "src/taskserver.cpp"
    )


add_executable(dyros_jet_moveit_depend src/main.cpp)
target_link_libraries(dyros_jet_moveit_depend dyros_jet_moveit_depend_lib ${catkin_LIBRARIES})

add_executable(interactive src/interactive.cpp)
target_link_libraries(interactive ${catkin_LIBRARIES})

add_dependencies(dyros_jet_moveit_depend ${catkin_EXPORTED_TARGETS})
add_dependencies(interactive ${catkin_EXPORTED_TARGETS})
add_dependencies(taskserver ${catkin_EXPORTED_TARGETS})
