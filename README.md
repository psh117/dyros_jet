# DYROS JET Repository 

* This is a DYROS JET Humanoid Repository

## How do I get set up? ##

```sh
sudo apt-get install ros-kinetic-qt-build ros-kinetic-realtime-tools ros-kinetic-smach-viewer
cd ~/catkin_ws/src
git clone https://github.com/KumarRobotics/imu_3dm_gx4
git clone https://github.com/psh117/rt_dynamixel_msgs
git clone -b 3.6.0 https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/saga0619/mujoco_ros_sim.git
```

### RBDL Setup ###
```sh
wget https://bitbucket.org/rbdl/rbdl/get/849d2aee8f4c.zip
unzip 849d2aee8f4c.zip
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
* If you get an error about the .so file when launching simulation, add the following command to ~/.bashrc
```sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
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
* Install moveit
```sh
sudo apt install ros-kinetic-moveit
```

* Launch moveit
```sh
roslaunch dyros_jet_moveit dyros_jet_moveit.launch
```

* Get pointcloud to moveit 
```sh
rosrun pcl_ros pcd_to_pointcloud ~/example.pcd _frame_id:=odom
```
if you want new frame of pointcloud,
```sh
rosrun tf static_transform_publisher 0 0 0 -1.5708 0 -1.5708 base_link odom2 100
```
tf static_tranform_publisher x y z α β γ parent_frame child_frame period(milliseconds)

### Mujoco Update (19/01/31) ###
* mujoco_ros - [MUJOCO_ROS_SIM](https://github.com/saga0619/mujoco_ros_sim)

* GUI
```sh
rosrun dyros_jet_gui dyros_jet_gui
```
* roslaunch
```sh
roslaunch dyros_jet_launch mujoco.launch
```

### How do I contribuite to this repo? ###
* Read this http://wiki.ros.org/CppStyleGuide

