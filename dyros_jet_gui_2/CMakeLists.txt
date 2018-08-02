cmake_minimum_required(VERSION 2.8.3)
project(dyros_jet_gui_2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  qt_build roscpp rospy sensor_msgs
)
set(QML_IMPORT_PATH "${QML_IMPORT_PATH};${CATKIN_GLOBAL_LIB_DESTINATION}" )
set(QML_IMPORT_PATH2 "${QML_IMPORT_PATH};${CATKIN_GLOBAL_LIB_DESTINATION}" )
include_directories(${catkin_INCLUDE_DIRS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dyros_jet_gui_2
#  CATKIN_DEPENDS qt_build roscpp rospy sensor_msgs
#  DEPENDS system_lib
)


##############################################################################
# Qt Environment
##############################################################################
 
# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency ca ll in package.xml
#rosbuild_prepare_qt4(QtCore QtGui QtQml QtQuick) # Add the appropriate components to the component list here
find_package(Qt5 COMPONENTS Core Gui Qml Quick REQUIRED)


##############################################################################
# Sections
##############################################################################
 
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/dyros_jet_gui2/*.hpp)
 
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})


##############################################################################
# Sources
##############################################################################
 
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
 
##############################################################################
# Binaries
##############################################################################
 
add_executable(gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
qt5_use_modules(gui Quick Core)
target_link_libraries(gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
target_include_directories(gui PUBLIC include)
install(TARGETS gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

