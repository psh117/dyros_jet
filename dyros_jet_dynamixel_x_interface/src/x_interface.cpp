#include <dyros_jet_dynamixel_x_interface/dxl_handler.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "motion_generator");

  DXLHandler dh("/dev/ttyUSB0", 2.0, 3000000);


  dh.allTorqueOn();
  dh.setInitCommand();

  dh.controlLoop();

  dh.allTorqueOff();

  return 0;
}
