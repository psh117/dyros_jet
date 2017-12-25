
#include "dyros_jet_controller/control_base.h"

namespace dyros_jet_controller
{

const string JOINT_NAME[40] = { "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll",
                                "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                                "WaistPitch","WaistYaw",
                                "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                                "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                                "HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"};

/*
const string JOINT_NAME[40] = {"WaistPitch","WaistYaw",
                               "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                               "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                               "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                               "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll",
                               "HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"};
*/
const int JOINT_ID[40] = { 16,18,20,22,24,26,
                           15,17,19,21,23,25,
                           28,27, // waist yaw - roll order
                           2,4,6,8,10,12,14,
                           1,3,5,7,9,11,13,
                           29,30,31,32};


// Constructor
ControlBase::ControlBase(ros::NodeHandle &nh, double Hz) :
  ui_update_count_(0), is_first_boot_(true), Hz_(Hz), total_dof_(DyrosJetModel::HW_TOTAL_DOF),
  task_controller_(model_, q_, Hz, control_time_)
{
  //walking_cmd_sub_ = nh.subscribe

  parameterInitialize();
}

bool ControlBase::checkStateChanged()
{
  if(previous_state_ != current_state_)
  {
    current_state_ = previous_state_;
    return true;
  }
  return false;
}
void ControlBase::makeIDInverseList()
{
  for(int i=0;i<total_dof_; i++)
  {
    joint_id_[i] = JOINT_ID[i];
    joint_id_inversed_[JOINT_ID[i]] = i;
  }
}

void ControlBase::update()
{
  model_.updateKinematics(q_);  // Update end effector positions and Jacobians
  stateChangeEvent();
}

void ControlBase::stateChangeEvent()
{
  if(checkStateChanged())
  {
    if(current_state_ == "move1")
    {
      task_controller_.setEnable(DyrosJetModel::EE_LEFT_HAND, true);
      task_controller_.setEnable(DyrosJetModel::EE_RIGHT_HAND, false);
      task_controller_.setEnable(DyrosJetModel::EE_LEFT_FOOT, false);
      task_controller_.setEnable(DyrosJetModel::EE_RIGHT_FOOT, false);

      Eigen::HTransform target;
      target.linear() = Eigen::Matrix3d::Identity();
      target.translation() << 1.0, 0.0, 1.0;
      task_controller_.setTarget(DyrosJetModel::EE_LEFT_HAND, target, 5.0);
    }
  }
}
void ControlBase::compute()
{
  task_controller_.compute();
  task_controller_.writeDesired(control_mask_, desired_q_);
  // walking_controller.compute();
}

void ControlBase::reflect()
{
}

void ControlBase::parameterInitialize()
{
  q_.setZero();
  q_dot_.setZero();
  torque_.setZero();
  left_foot_ft_.setZero();
  left_foot_ft_.setZero();
  desired_q_.setZero();
}
void ControlBase::readDevice()
{
  ros::spinOnce();
}

}
