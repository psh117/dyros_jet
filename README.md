# DYROS JET Repository #

* This is a DYROS JET Humanoid Repository

## How do I get set up? ##

```sh
sudo apt-get install ros-kinetic-qt-build ros-kinetic-realtime-tools ros-kinetic-smach-viewer
cd ~/catkin_ws/src
git clone https://github.com/KumarRobotics/imu_3dm_gx4
git clone https://github.com/psh117/rt_dynamixel_msgs
```

### RBDL Setup ###
```sh
wget https://bitbucket.org/rbdl/rbdl/get/default.zip
unzip default.zip
cd rbdl-rbdl-849d2aee8f4c
mkdir build
cd build
cmake -D RBDL_BUILD_ADDON_URDFREADER=ON ..
make all
sudo make install
```
* If an error occurs, open rbdl-rbdl-[commit]/addons/urdfreader/urdfreader.cc
* and remove this line
```cpp
#include <ros.h>
```

### How do I run the simulation? ###

* Launch V-Rep
* Open `dyros_jet_new_api.ttt`
```
dyros_jet_vrep/scene/dyros_jet_new_api.ttt
```
* roslaunch
```sh
roslaunch dyros_jet_launch simulation.launch
```
* GUI
```sh
rosrun dyros_jet_gui dyros_jet_gui
```
* Prepare a simple test (Once)
```sh
chmod +x dyros_jet_mission_commander/src/commander.py
catkin_make
```
* Run the simple test
```sh
rosrun dyros_jet_mission_commander commander.py
```

### About Moveit ###

* Launch moveit
```sh
roslaunch dyros_jet_moveit_config demo.launch
```


### How do I contribuite to this repo? ###
* Read this http://wiki.ros.org/CppStyleGuide
