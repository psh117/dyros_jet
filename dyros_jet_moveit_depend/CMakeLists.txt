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
)

catkin_package(
    DEPENDS   dyros_jet_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
    )


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ./include/dyros_jet_moveit_depend
)

add_executable(dyros_jet_moveit_depend src/basic_tf.cpp)
target_link_libraries(dyros_jet_moveit_depend ${catkin_LIBRARIES})

add_executable(interactive src/interactive.cpp)
target_link_libraries(interactive ${catkin_LIBRARIES})

add_executable(taskserver src/taskserver.cpp)
target_link_libraries(taskserver ${catkin_LIBRARIES})

add_dependencies(dyros_jet_moveit_depend ${catkin_EXPORTED_TARGETS})
