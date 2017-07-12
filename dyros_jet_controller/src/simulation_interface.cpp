#include "dyros_jet_controller/simulation_interface.h"

namespace dyros_jet_controller
{

SimulationInterface::SimulationInterface(ros::NodeHandle &nh, double Hz): controlBase(nh, Hz), rate_(200)
{
  simulationRunning_ = true;
  simulationTime_ = 0.0f; // set initial simulation time

  vrepSimStartPub_ = nh.advertise<std_msgs::Bool>("/startSimulation", 5);
  vrepSimStopPub_ = nh.advertise<std_msgs::Bool>("/stopSimulation", 5);
  vrepSimStepDonePub_ = nh.advertise<std_msgs::Bool>("/triggerNextStep", 100);
  vrepSimEnableSyncModePub_ = nh.advertise<std_msgs::Bool>("/enableSyncMode", 5);

  imuSub_ = nh.subscribe("/vrep_ros_interface/imu", 100, &SimulationInterface::imuCallback, this);
  jointSub_ = nh.subscribe("/vrep_ros_interface/joint_state", 100, &SimulationInterface::jointCallback, this);
  leftFTSub_ = nh.subscribe("/vrep_ros_interface/left_foot_ft", 100, &SimulationInterface::leftFTCallback, this);
  rightFTSub_ = nh.subscribe("/vrep_ros_interface/right_foot_ft", 100, &SimulationInterface::rightFTCallback, this);

  vrepJointSetPub_ = nh.advertise<sensor_msgs::JointState>("/vrep_ros_interface/joint_set", 1);

  jointSetMsg_.name.resize(total_dof);
  jointSetMsg_.position.resize(total_dof);
  for(int i=0; i<total_dof; i++)
  {
    jointSetMsg_.name[i] = JOINT_NAME[i];
  }
  ros::Rate poll_rate(100);
  ROS_INFO("Wait for connecting");
  while(vrepSimEnableSyncModePub_.getNumSubscribers() == 0)
      poll_rate.sleep();
  while(vrepSimStartPub_.getNumSubscribers() == 0)
      poll_rate.sleep();

  ROS_INFO("Connected");
  vrep_enableSyncMode();
  vrep_start();

}

void SimulationInterface::vrep_start()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrepSimStartPub_.publish(msg);
}

void SimulationInterface::vrep_stop()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrepSimStopPub_.publish(msg);
}

void SimulationInterface::vrep_stepDone()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrepSimStepDonePub_.publish(msg);
}

void SimulationInterface::vrep_enableSyncMode()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrepSimEnableSyncModePub_.publish(msg);
}

// Function implement
void SimulationInterface::update()
{
  controlBase::update();

}
void SimulationInterface::compute()
{
  controlBase::compute();
}

void SimulationInterface::writedevice()
{

  for(int i=0;i<total_dof;i++) {
    jointSetMsg_.position[i] = desired_q_(i);
  }

  vrepJointSetPub_.publish(jointSetMsg_);
  vrep_stepDone();

}

void SimulationInterface::wait()
{
  rate_.sleep();
}




// Callback functions



void SimulationInterface::simulationTimeCallback(const std_msgs::Float32ConstPtr& msg)
{
  simulationTime_ = msg->data;
}

void SimulationInterface::jointCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  for(int i=0; i<total_dof; i++)
  {
      for (int j=0; j<msg->name.size(); j++)
      {
          if(JOINT_NAME[i] == msg->name[j].data())
          {
              q_(i) = msg->position[j];
              q_dot_(i) = msg->velocity[j];
              torque_(i) = msg->effort[j];
          }
      }
  }
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
