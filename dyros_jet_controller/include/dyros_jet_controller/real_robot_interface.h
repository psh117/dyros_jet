#ifndef REAL_ROBOT_INTERFACE_H
#define REAL_ROBOT_INTERFACE_H

#include <imu_3dm_gx4/FilterOutput.h>
#include <realtime_tools/realtime_publisher.h>

#include <rt_dynamixel_msgs/JointSet.h>
#include <rt_dynamixel_msgs/JointState.h>
#include <rt_dynamixel_msgs/ModeSetting.h>
#include <rt_dynamixel_msgs/MotorSetting.h>

#include "control_base.h"

namespace dyros_jet_controller
{
class RealRobotInterface : public ControlBase
{
public:
  RealRobotInterface(ros::NodeHandle &nh, double Hz);

  virtual void update() override; // update controller based on readdevice
  virtual void writeDevice() override; // publish to actuate devices
  virtual void wait() override;

private:  // CALLBACK
  void jointCallback(const rt_dynamixel_msgs::JointStateConstPtr msg);
  void imuCallback(const sensor_msgs::ImuConstPtr msg);
  void imuFilterCallback(const imu_3dm_gx4::FilterOutputConstPtr msg);

  void leftFootFTCallback(const geometry_msgs::WrenchStampedConstPtr msg);
  void rightFootFTCallback(const geometry_msgs::WrenchStampedConstPtr msg);

private:


private:
  ros::ServiceClient dxl_mode_set_client_; ///< dynamixel mode select service
  ros::ServiceClient dxl_motor_set_client_; ///< dynmamixel motor setting service

  realtime_tools::RealtimePublisher<rt_dynamixel_msgs::JointSet> dxl_joint_set_pub_;

  ros::Subscriber dxl_joint_sub_;

  ros::Subscriber imu_sub_;
  ros::Subscriber imu_filter_sub_;
  ros::Subscriber left_foot_ft_sub_;
  ros::Subscriber right_foot_ft_sub_;


  int dxl_mode_; ///< current dynamixel mode
  int dxl_torque_;    ///< TODO: WHAT?

  ros::Rate rate_;

};

}
#endif // REAL_ROBOT_INTERFACE_H
