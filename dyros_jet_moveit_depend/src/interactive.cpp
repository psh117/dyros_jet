#include <ros/ros.h>


int main(int argc, char **argv){
  ROS_INFO("DYROS Jet Ineractive marker control with haptic device");
  ros::init(argc, argv, "haptic interaction");

  ros::NodeHandle n;
  ros::Rate r(30);
  ros::spin();

}
