
#include "dyros_jet_controller/control_base.h"

namespace dyros_jet_controller
{

// Constructor
ControlBase::ControlBase(ros::NodeHandle &nh, double Hz) :
  ui_update_count_(0), is_first_boot_(true), Hz_(Hz), control_mask_{}, total_dof_(DyrosJetModel::HW_TOTAL_DOF),
  shutdown_flag_(false),
  joint_controller_(q_, control_time_),
  task_controller_(model_, q_, Hz, control_time_),
  walking_controller_(model_, q_, Hz, control_time_),
  moveit_controller_(model_, q_, Hz),
  joint_control_as_(nh, "/dyros_jet/joint_control", false) // boost::bind(&ControlBase::jointControlActionCallback, this, _1), false
{
  //walking_cmd_sub_ = nh.subscribe
  makeIDInverseList();
  //joint_control_as_.
  joint_control_as_.start();
  joint_state_pub_.init(nh, "/dyros_jet/joint_state", 3);
  joint_state_pub_.msg_.name.resize(DyrosJetModel::HW_TOTAL_DOF);
  joint_state_pub_.msg_.angle.resize(DyrosJetModel::HW_TOTAL_DOF);
  joint_state_pub_.msg_.velocity.resize(DyrosJetModel::HW_TOTAL_DOF);
  joint_state_pub_.msg_.current.resize(DyrosJetModel::HW_TOTAL_DOF);
  joint_state_pub_.msg_.error.resize(DyrosJetModel::HW_TOTAL_DOF);

  joint_robot_state_pub_.init(nh, "/joint_states", 3);
  joint_robot_state_pub_.msg_.name.resize(DyrosJetModel::HW_TOTAL_DOF-4);
  joint_robot_state_pub_.msg_.position.resize(DyrosJetModel::HW_TOTAL_DOF-4);
  //joint_robot_state_pub_.msg_.velocity.resize(DyrosJetModel::HW_TOTAL_DOF-4);
  //joint_robot_state_pub_.msg_.effort.resize(DyrosJetModel::HW_TOTAL_DOF-4);

  for (int i=0; i< DyrosJetModel::HW_TOTAL_DOF; i++)
  {
    joint_state_pub_.msg_.name[i] = DyrosJetModel::JOINT_NAME[i];
  }
  for (int i=0; i< DyrosJetModel::HW_TOTAL_DOF-4; i++)
  {
    joint_robot_state_pub_.msg_.name[i] = DyrosJetModel::JOINT_NAME[i];
  }

  smach_pub_.init(nh, "/dyros_jet/smach/transition", 1);
  smach_sub_ = nh.subscribe("/dyros_jet/smach/container_status", 3, &ControlBase::smachCallback, this);
  task_comamnd_sub_ = nh.subscribe("/dyros_jet/task_command", 3, &ControlBase::taskCommandCallback, this);
  joint_command_sub_ = nh.subscribe("/dyros_jet/joint_command", 3, &ControlBase::jointCommandCallback, this);
  walking_command_sub_ = nh.subscribe("/dyros_jet/walking_command",3, &ControlBase::walkingCommandCallback,this);
  shutdown_command_sub_ = nh.subscribe("/dyros_jet/shutdown_command", 1, &ControlBase::shutdownCommandCallback,this);
  parameterInitialize();
  // model_.test();
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
  for(int i=0;i<DyrosJetModel::HW_TOTAL_DOF; i++)
  {
    joint_id_[i] = DyrosJetModel::JOINT_ID[i];
    joint_id_inversed_[DyrosJetModel::JOINT_ID[i]] = i;
  }
}

void ControlBase::update()
{
  model_.updateKinematics(q_.head<DyrosJetModel::MODEL_DOF>());  // Update end effector positions and Jacobians
  model_.updateSensorData(right_foot_ft_, left_foot_ft_);
  stateChangeEvent();
}

void ControlBase::stateChangeEvent()
{
  if(checkStateChanged())
  {
    if(current_state_ == "move1")
    {
      /*
      task_controller_.setEnable(DyrosJetModel::EE_LEFT_HAND, true);
      task_controller_.setEnable(DyrosJetModel::EE_RIGHT_HAND, false);
      task_controller_.setEnable(DyrosJetModel::EE_LEFT_FOOT, false);
      task_controller_.setEnable(DyrosJetModel::EE_RIGHT_FOOT, false);

      Eigen::Isometry3d target;
      target.linear() = Eigen::Matrix3d::Identity();
      target.translation() << 1.0, 0.0, 1.0;
      task_controller_.setTarget(DyrosJetModel::EE_LEFT_HAND, target, 5.0);
      */
    }
  }
}
void ControlBase::compute()
{

  task_controller_.compute();
  joint_controller_.compute();
  walking_controller_.compute();
  moveit_controller_.compute();

  task_controller_.updateControlMask(control_mask_);
  joint_controller_.updateControlMask(control_mask_);
  walking_controller_.updateControlMask(control_mask_);
  moveit_controller_.updateControlMask(control_mask_);

  task_controller_.writeDesired(control_mask_, desired_q_);
  joint_controller_.writeDesired(control_mask_, desired_q_);
  walking_controller_.writeDesired(control_mask_, desired_q_);
  moveit_controller_.writeDesired(control_mask_, desired_q_);

  tick_ ++;
  control_time_ = tick_ / Hz_;

  /*
  if ((tick_ % 200) == 0 )
  {
    ROS_INFO ("1 sec, %lf sec", control_time_);
  }
  */
}

void ControlBase::reflect()
{
  joint_robot_state_pub_.msg_.header.stamp = ros::Time::now();
  for (int i=0; i<DyrosJetModel::HW_TOTAL_DOF; i++)
  {
    joint_state_pub_.msg_.angle[i] = q_(i);
    joint_state_pub_.msg_.velocity[i] = q_dot_(i);
    joint_state_pub_.msg_.current[i] = torque_(i);
  }

  for (int i=0; i<DyrosJetModel::HW_TOTAL_DOF - 4; i++)
  {
    joint_robot_state_pub_.msg_.position[i] = q_(i);
    //joint_robot_state_pub_.msg_.velocity[i] = q_dot_(i);
    //joint_robot_state_pub_.msg_.effort[i] = torque_(i);
  }

  if(joint_state_pub_.trylock())
  {
    joint_state_pub_.unlockAndPublish();
  }
  if(joint_robot_state_pub_.trylock())
  {
    joint_robot_state_pub_.unlockAndPublish();
  }

  if(joint_control_as_.isActive())
  {
    bool all_disabled = true;
    for (int i=0; i<DyrosJetModel::HW_TOTAL_DOF; i++)
    {
      if (joint_controller_.isEnabled(i))
      {
        all_disabled = false;
      }
    }
    if (all_disabled)
    {
      joint_control_result_.is_reached = true;
      joint_control_as_.setSucceeded(joint_control_result_);
    }
  }
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

  // Action
  if (joint_control_as_.isNewGoalAvailable())
  {
    jointControlActionCallback(joint_control_as_.acceptNewGoal());
  }
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
  for (unsigned int i=0; i<msg->name.size(); i++)
  {
    joint_controller_.setTarget(model_.getIndex(msg->name[i]), msg->position[i], msg->duration[i]);
    joint_controller_.setEnable(model_.getIndex(msg->name[i]), true);
  }
}

void ControlBase::walkingCommandCallback(const dyros_jet_msgs::WalkingCommandConstPtr& msg)
{
  vector<bool> compensate_v;
  compensate_v.reserve(2);

  for (int i =0; i<2; i++)
  {
    compensate_v[i]=msg->compensator_mode[i];
  }


  if(msg->walk_mode == dyros_jet_msgs::WalkingCommand::STATIC_WALKING)
  {
    walking_controller_.setEnable(true);
    walking_controller_.setTarget(msg->walk_mode, msg->compensator_mode[0], msg->compensator_mode[1], msg->ik_mode, msg->heel_toe, msg->first_foot_step,
    msg->x, msg->y, msg->z, msg->height, msg->theta, msg-> step_length_x, msg-> step_length_y);
  }
  else
  {
    cout<<"fffffffffff"<<endl;
    walking_controller_.setEnable(false);
  }
}

void ControlBase::shutdownCommandCallback(const std_msgs::StringConstPtr &msg)
{
  if (msg->data == "Shut up, JET.")
  {
    shutdown_flag_ = true;
  }
}

void ControlBase::jointControlActionCallback(const dyros_jet_msgs::JointControlGoalConstPtr &goal)
{
  for (unsigned int i=0; i<goal->command.name.size(); i++)
  {
    joint_controller_.setTarget(model_.getIndex(goal->command.name[i]), goal->command.position[i], goal->command.duration[i]);
    joint_controller_.setEnable(model_.getIndex(goal->command.name[i]), true);
  }
  joint_control_feedback_.percent_complete = 0.0;
}


}
