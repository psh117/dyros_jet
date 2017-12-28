
#include "dyros_jet_controller/control_base.h"

namespace dyros_jet_controller
{

// Constructor
ControlBase::ControlBase(ros::NodeHandle &nh, double Hz) :
  ui_update_count_(0), is_first_boot_(true), Hz_(Hz), control_mask_{}, total_dof_(DyrosJetModel::HW_TOTAL_DOF),
  task_controller_(model_, q_, Hz, control_time_), joint_controller_(q_, control_time_), walking_controller_(q_,control_time_)
{

  smach_sub_ = nh.subscribe("/dyros_jet/smach/container_status", 3, &ControlBase::smachCallback, this);
  task_comamnd_sub_ = nh.subscribe("/dyros_jet/task_command", 3, &ControlBase::taskCommandCallback, this);
  joint_command_sub_ = nh.subscribe("/dyros_jet/joint_command", 3, &ControlBase::jointCommandCallback, this);
  walking_command_sub_ = nh.subscribe("/dyros_jet/walking_command",3, &ControlBase::walkingCommandCallback,this);
  parameterInitialize();
  model_.test();
}

bool ControlBase::checkStateChanged()
{
  if(previous_state_ != current_state_)
  {
    previous_state_ = current_state_;
    return true;
  }
  return false;
}
void ControlBase::makeIDInverseList()
{
  for(int i=0;i<total_dof_; i++)
  {
    joint_id_[i] = DyrosJetModel::JOINT_ID[i];
    joint_id_inversed_[DyrosJetModel::JOINT_ID[i]] = i;
  }
}

void ControlBase::update()
{
  model_.updateKinematics(q_.head<DyrosJetModel::MODEL_DOF>());  // Update end effector positions and Jacobians
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

      Eigen::Isometry3d target;
      target.linear() = Eigen::Matrix3d::Identity();
      target.translation() << 1.0, 0.0, 1.0;
      task_controller_.setTarget(DyrosJetModel::EE_LEFT_HAND, target, 5.0);
    }
  }
}
void ControlBase::compute()
{
  task_controller_.compute();
  joint_controller_.compute();
  // walking_controller.compute();

  task_controller_.updateControlMask(control_mask_);
  joint_controller_.updateControlMask(control_mask_);

  task_controller_.writeDesired(control_mask_, desired_q_);
  joint_controller_.writeDesired(control_mask_, desired_q_);

  tick_ ++;
  control_time_ = tick_ / Hz_;
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

void ControlBase::smachCallback(const smach_msgs::SmachContainerStatusConstPtr& msg)
{
  current_state_ = msg->active_states[0];
}

void ControlBase::taskCommandCallback(const dyros_jet_msgs::TaskCommandConstPtr& msg)
{
  for(unsigned int i=0; i<4; i++)
  {
    if(msg->end_effector[i])
    {
      Eigen::Isometry3d target;
      tf::poseMsgToEigen(msg->pose[i], target);

      if(msg->mode[i] == dyros_jet_msgs::TaskCommand::RELATIVE)
      {
        const auto &current =  model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)i);
        target.translation() = target.translation() + current.translation();
        target.linear() = current.linear() * target.linear();
      }
      task_controller_.setTarget((DyrosJetModel::EndEffector)i, target, msg->duration[i]);
      task_controller_.setEnable((DyrosJetModel::EndEffector)i, true);
    }
  }
}

void ControlBase::jointCommandCallback(const dyros_jet_msgs::JointCommandConstPtr& msg)
{
  for (unsigned int i=0; i<total_dof_; i++)
  {
    if (msg->enable[i])
    {
      joint_controller_.setEnable(i, true);
      joint_controller_.setTarget(i, msg->position[i], msg->duration[i]);
    }
    else
    {
      joint_controller_.setEnable(i, false);
    }
  }
}

void ControlBase::walkingCommandCallback(const dyros_jet_msgs::WalkingCommandConstPtr& msg)
{
  if(msg->walk_mode)
  {
    for (unsigned int i=0; i<total_dof_; i++)
    {
      if (msg->walk_enable[i])
      {
        walking_controller_.setEnable(i, true);
      }
      else
      {
        walking_controller_.setEnable(i, false);
      }
    }
  }
  }
}
