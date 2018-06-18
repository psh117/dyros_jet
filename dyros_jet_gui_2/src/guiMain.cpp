#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "guiMain");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
