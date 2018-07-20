* How to run

 1. haptic calibration

  $ cd ~/(my_catkin_workspace)/src/haptic/sdk-3.6.0/bin
  $ sudo ./HapticInit
  (click Initialize)

 2. roslaunch moveit
  
  $ roslaunch dyros_jet_moveit_config dyros_jet_moveit.launch

 3. rosrun haptic_publisher

  $ cd ~/(your_catkin_workspace)
  $ catkin_make
  $ source devel/setup.bash
  $ rosrun haptic_publisher haptic
 
 4. set zero orientation

  Before clicking the button, put centerline of gripper on x-axis. And button should face +z-direction. Then click the button.

* Error

 1. If haptic device is not found during rosrun-ing haptic_publisher even though it is connected to computer
  $ dmesg
  (Check number of idVender, idProduct of haptic device)
  $ sudo su -
  # cd /etc/udev/rules.d/
  # echo "SUBSYSTEM==\"usb\", ATTRS{idProduct}==\"(your_idProduct_number)\", ATTRS{idVendor}==\"(your_idVendor_number)\", GROUP==\"plugdev\",   MODE==\"666\"" > 10-mydevice.rules
  # udevadm control --reload-rules && udevadm trigger
  # exit

 2. dhdc.h: No such file or directory
  Check include_directories{} in CMakeLists.txt

 3. TODO.....
