#!/bin/bash

echo "HUMANOID2020FALL Dyros Jet Auto Installer"

while true; do
    echo "Select Installation method";
    echo "1 : Install all for UBUNTU16.04";
    echo "2 : Install all for UBUNTU18.04";
    read -p "Select Number : " yn
    case $yn in
        [1]* ) echo "Starting Install ... all prerequistes for UBUNTU16.04";
              mkdir Temp
              cd Temp

              git clone https://github.com/saga0619/rbdl_jet
              cd rbdl_jet
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../..

              git clone https://github.com/saga0619/qpoases
              cd qpoases
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../..

	      echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib'>>~/.bashrc
	      source ~/.bashrc
	      sudo ldconfig
	      

	      rm -rf Temp
              
	      cd ~/catkin_ws/src
              sudo apt-get install ros-kinetic-qt-build ros-kinetic-realtime-tools ros-kinetic-smach-viewer
              git clone https://github.com/myeongju2313/Humanoid2020FallHW.git
              git clone https://github.com/KumarRobotics/imu_3dm_gx4
              git clone https://github.com/psh117/rt_dynamixel_msgs
              git clone -b 3.6.0 https://github.com/ROBOTIS-GIT/DynamixelSDK.git
              git clone https://github.com/saga0619/mujoco_ros_sim.git
	      cd ~/catkin_ws && catkin_make

              exit;;



        [2]* ) echo "Starting Install ... all prerequistes for UBUNTU18.04";
        
              git clone https://github.com/saga0619/rbdl_jet
              cd rbdl_jet
              mkdir build
              cd build
              cmake -D UBUNTU18_04=ON ..
              make all
              sudo make install
              cd ../..

              git clone https://github.com/saga0619/qpoases
              cd qpoases
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../..

	      echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib'>>~/.bashrc
	      source ~/.bashrc
	      sudo ldconfig

              rm -rf Temp

	      cd ~/catkin_ws/src
              sudo apt-get install ros-kinetic-qt-build ros-kinetic-realtime-tools ros-kinetic-smach-viewer
              git clone https://github.com/myeongju2313/Humanoid2020FallHW.git
              git clone https://github.com/KumarRobotics/imu_3dm_gx4
              git clone https://github.com/psh117/rt_dynamixel_msgs
              git clone -b 3.6.0 https://github.com/ROBOTIS-GIT/DynamixelSDK.git
              git clone https://github.com/saga0619/mujoco_ros_sim.git
	      cd ~/catkin_ws && catkin_make
              exit;;

        [Nn]* ) echo "Aborting ...";
                exit;;
        * ) echo "Please select proper number.";;
    esac
done
