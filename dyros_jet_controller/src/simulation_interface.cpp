#include "dyros_jet_controller/simulation_interface.h"

namespace dyros_jet_controller
{

SimulationInterface::SimulationInterface(ros::NodeHandle &nh, double Hz):
  ControlBase(nh, Hz), rate_(Hz), simulation_step_done_(false)
{
  simulation_running_= true;
  simulation_time_ = 0.0f; // set initial simulation time

  vrep_sim_start_pub_ = nh.advertise<std_msgs::Bool>("/startSimulation", 5);
  vrep_sim_stop_pub_ = nh.advertise<std_msgs::Bool>("/stopSimulation", 5);
  vrep_sim_step_trigger_pub_ = nh.advertise<std_msgs::Bool>("/triggerNextStep", 100);
  vrep_sim_enable_syncmode_pub_ = nh.advertise<std_msgs::Bool>("/enableSyncMode", 5);


  vrep_sim_step_done_sub_ = nh.subscribe("/simulationStepDone", 100, &SimulationInterface::simulationStepDoneCallback, this);

  imu_sub_ = nh.subscribe("/vrep_ros_interface/imu", 100, &SimulationInterface::imuCallback, this);
  joint_sub_ = nh.subscribe("/vrep_ros_interface/joint_state", 100, &SimulationInterface::jointCallback, this);
  left_ft_sub_ = nh.subscribe("/vrep_ros_interface/left_foot_ft", 100, &SimulationInterface::leftFTCallback, this);
  right_ft_sub_ = nh.subscribe("/vrep_ros_interface/right_foot_ft", 100, &SimulationInterface::rightFTCallback, this);

  vrep_joint_set_pub_ = nh.advertise<sensor_msgs::JointState>("/vrep_ros_interface/joint_set", 1);

  joint_set_msg_.name.resize(total_dof_);
  joint_set_msg_.position.resize(total_dof_);
  for(int i=0; i<total_dof_; i++)
  {
    joint_set_msg_.name[i] = DyrosJetModel::JOINT_NAME[i];
  }
  ros::Rate poll_rate(100);

  ROS_INFO("Waiting for connection of V-REP ROS Interface");

  while(vrep_sim_enable_syncmode_pub_.getNumSubscribers() == 0 && ros::ok())
    poll_rate.sleep();
  while(vrep_sim_start_pub_.getNumSubscribers() == 0 && ros::ok())
    poll_rate.sleep();

  ROS_INFO(" -- Connected -- ");
  vrepEnableSyncMode();
  vrepStart();

}

void SimulationInterface::vrepStart()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_start_pub_.publish(msg);
}

void SimulationInterface::vrepStop()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_stop_pub_.publish(msg);
}

void SimulationInterface::vrepStepTrigger()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_step_trigger_pub_.publish(msg);
}

void SimulationInterface::vrepEnableSyncMode()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_enable_syncmode_pub_.publish(msg);
}

// Function implement
void SimulationInterface::update()
{
  ControlBase::update();

}
void SimulationInterface::compute()
{
  ControlBase::compute();
}

void SimulationInterface::writeDevice()
{

  for(int i=0;i<total_dof_;i++) {
    joint_set_msg_.position[i] = desired_q_(i);
  }

  if(!is_first_boot_)
    vrep_joint_set_pub_.publish(joint_set_msg_);

  vrepStepTrigger();

}

void SimulationInterface::wait()
{
  // Wait for step done
  while(ros::ok() && !simulation_step_done_)
  {
    ros::spinOnce();
  }
  simulation_step_done_ = false;
  rate_.sleep();
}


// Callback functions



void SimulationInterface::simulationTimeCallback(const std_msgs::Float32ConstPtr& msg)
{
  simulation_time_ = msg->data;
}

void SimulationInterface::simulationStepDoneCallback(const std_msgs::BoolConstPtr &msg)
{
  simulation_step_done_ = msg->data;
}

void SimulationInterface::jointCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  for(int i=0; i<total_dof_; i++)
  {
    for (int j=0; j<msg->name.size(); j++)
    {
      if(DyrosJetModel::JOINT_NAME[i] == msg->name[j].data())
      {
        q_(i) = msg->position[j];
        if(is_first_boot_)
        {
          desired_q_(i) = msg->position[j];
        }

        q_dot_(i) = msg->velocity[j];
        torque_(i) = msg->effort[j];
      }
    }
  }
  if(is_first_boot_)
  {is_first_boot_ = false;}
}

void SimulationInterface::leftFTCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{

}

void SimulationInterface::rightFTCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{

}

void SimulationInterface::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{

}

}

