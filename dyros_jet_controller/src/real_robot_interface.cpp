#include "dyros_jet_controller/real_robot_interface.h"

namespace dyros_jet_controller
{

RealRobotInterface::RealRobotInterface(ros::NodeHandle &nh, double Hz):
  ControlBase(nh, Hz), rate_(Hz)
{

  dxl_mode_ = rt_dynamixel_msgs::ModeSettingRequest::SETTING;
  dxl_torque_ = 0;

  dxl_mode_set_client_ = nh.serviceClient<rt_dynamixel_msgs::ModeSetting>("/rt_dynamixel/mode");
  dxl_motor_set_client_ = nh.serviceClient<rt_dynamixel_msgs::MotorSetting>("/rt_dynamixel/motor_set");

  dxl_joint_set_pub_.init(nh, "/rt_dynamixel/joint_set", 1);

  dxl_joint_sub_ = nh.subscribe("/rt_dynamixel/joint_state",
                                1, &RealRobotInterface::jointCallback, this,
                                ros::TransportHints().tcpNoDelay(true));
  imu_sub_ = nh.subscribe("/imu/imu",
                          1, &RealRobotInterface::imuCallback, this,
                          ros::TransportHints().tcpNoDelay(true));
  imu_filter_sub_ = nh.subscribe("/imu/filter",
                                 1, &RealRobotInterface::imuFilterCallback, this,
                                 ros::TransportHints().tcpNoDelay(true));

  left_foot_ft_sub_ = nh.subscribe("/ati_ft_sensor/left_foot_ft", 1,
                                   &RealRobotInterface::leftFootFTCallback, this);
  right_foot_ft_sub_ = nh.subscribe("/ati_ft_sensor/right_foot_ft", 1,
                                    &RealRobotInterface::rightFootFTCallback, this);

  dxl_joint_set_pub_.msg_.angle.resize(DyrosJetModel::HW_TOTAL_DOF);
  dxl_joint_set_pub_.msg_.id.resize(DyrosJetModel::HW_TOTAL_DOF);

  for (int i=0; i<DyrosJetModel::HW_TOTAL_DOF; i++)
  {
    dxl_joint_set_pub_.msg_.id[i] = DyrosJetModel::JOINT_ID[i];
  }

}


void RealRobotInterface::update()
{
  ControlBase::update();
}

void RealRobotInterface::writeDevice()
{
  for(int i=0; i< DyrosJetModel::HW_TOTAL_DOF; i++)
  {
    dxl_joint_set_pub_.msg_.angle[i] = desired_q_(i);
  }
  if (dxl_joint_set_pub_.trylock()) {
    dxl_joint_set_pub_.unlockAndPublish();
  }
}

void RealRobotInterface::wait()
{
  rate_.sleep();
}

void RealRobotInterface::jointCallback(const rt_dynamixel_msgs::JointStateConstPtr msg)
{

  for(int i=0; i<DyrosJetModel::HW_TOTAL_DOF; i++)
  {
    for (int j=0; j<msg->id.size(); j++)
    {
      if(DyrosJetModel::JOINT_ID[i] == msg->id[j])
      {
        q_(i) = msg->angle[j];
        if(is_first_boot_)
        {    desired_q_(i) = msg->angle[j]; }

        q_dot_(i) = msg->velocity[j];
        torque_(i) = msg->current[j];
        joint_state_pub_.msg_.error[i] = msg->updated[j];
      }
    }
  }
  if(is_first_boot_)
  {is_first_boot_ = false;}
}


void RealRobotInterface::imuCallback(const sensor_msgs::ImuConstPtr msg)
{
  accelometer_(0) = msg->linear_acceleration.x;
  accelometer_(1) = msg->linear_acceleration.y;
  accelometer_(2) = msg->linear_acceleration.z;

  gyro_(0) = msg->angular_velocity.x;
  gyro_(1) = msg->angular_velocity.y;
  gyro_(2) = msg->angular_velocity.z;
}

void RealRobotInterface::imuFilterCallback(const imu_3dm_gx4::FilterOutputConstPtr msg)
{
  tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  imu_data_ = q;
}

void RealRobotInterface::leftFootFTCallback(const geometry_msgs::WrenchStampedConstPtr msg)
{
  left_foot_ft_(0) = msg->wrench.force.x;
  left_foot_ft_(1) = msg->wrench.force.y;
  left_foot_ft_(2) = msg->wrench.force.z;
  left_foot_ft_(3) = msg->wrench.torque.x;
  left_foot_ft_(4) = msg->wrench.torque.y;
  left_foot_ft_(5) = msg->wrench.torque.z;
}
void RealRobotInterface::rightFootFTCallback(const geometry_msgs::WrenchStampedConstPtr msg)
{
  right_foot_ft_(0) = msg->wrench.force.x;
  right_foot_ft_(1) = msg->wrench.force.y;
  right_foot_ft_(2) = msg->wrench.force.z;
  right_foot_ft_(3) = msg->wrench.torque.x;
  right_foot_ft_(4) = msg->wrench.torque.y;
  right_foot_ft_(5) = msg->wrench.torque.z;
}


}
