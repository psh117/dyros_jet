#include "dyros_jet_controller/joint_controller.h"

namespace dyros_jet_controller
{
 
JointController::JointController(const VectorQd& current_q, const double& control_time) :
  current_q_(current_q), current_time_(control_time), total_dof_(DyrosJetModel::HW_TOTAL_DOF),
  start_time_{}, end_time_{}
{

}

void JointController::compute()
{
  for(unsigned int i=0; i<total_dof_; i++)
  {
    if(joint_enable_[i])
    {
      if (current_time_ >= end_time_[i])
      {
        desired_q_(i) = target_q_(i);
        joint_enable_[i] = false;
      }
      else
      {
        desired_q_(i) = DyrosMath::cubic(current_time_, start_time_[i], end_time_[i], start_q_(i), target_q_(i), 0, 0);
      }
    }
  }
}

void JointController::setTarget(unsigned int joint_number, double target, double start_time, double end_time)
{
  if(joint_number >= total_dof_)
  {
    ROS_ERROR("JointController::setTarget - Out of range. Input = %u", joint_number);
    return ;
  }
  start_time_[joint_number] = start_time;
  end_time_[joint_number] = end_time;
  start_q_(joint_number) = current_q_(joint_number);
  target_q_(joint_number) = target;
}

void JointController::setTarget(unsigned int joint_number, double target, double duration)
{
  setTarget(joint_number, target, current_time_, current_time_ + duration);
}

void JointController::setEnable(unsigned int joint_number, bool enable)
{
  if (joint_number < total_dof_)
  {
    joint_enable_[joint_number] = enable;
  }
  else
  {
    ROS_ERROR("JointController::setEnable - Out of range. Input = %u", joint_number);
  }
}

void JointController::updateControlMask(unsigned int *mask)
{
  for(unsigned int i=0; i<total_dof_; i++)
  {
    if(joint_enable_[i])
    {
      mask[i] = (mask[i] | PRIORITY);
    }
    else
    {
      mask[i] = (mask[i] & ~PRIORITY);
    }
  }
}

void JointController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
{
  for(unsigned int i=0; i<total_dof_; i++)
  {
    if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
    {
      desired_q(i) = desired_q_(i);
    }
  }
}

}
