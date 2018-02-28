/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "dyros_jet_gui/joint_qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_jet_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

JointQNode::JointQNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv),
  isConnected(false)
{
  joint_cmd_msg_.position.resize(32);
  joint_cmd_msg_.name.resize(32);
  joint_cmd_msg_.duration.resize(32);

  joint_msg_.angle.resize(32);
  joint_msg_.current.resize(32);
  joint_msg_.error.resize(32);
}

JointQNode::~JointQNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  delete nh;  // deallocate ndoe handle
  wait();
}

bool JointQNode::init() {
  ros::init(init_argc,init_argv,"dyros_jet_gui");
  if ( ! ros::master::check() ) {
    return false;
  }
  init_nh();
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  start();
  return true;
}

void JointQNode::init_nh()
{
  nh = new ros::NodeHandle("dyros_jet_ui"); // allocate node handle
  // Add your ros communications here.

  joint_ctrl_publisher = nh->advertise<dyros_jet_msgs::JointCommand>("/dyros_jet/joint_command", 5);
  joint_state_subscirber = nh->subscribe("/dyros_jet/joint_state",1, &JointQNode::jointStateCallback, this);

  isConnected = true;

}

void JointQNode::run() {
  ros::Rate loop_rate(100);
  while ( ros::ok() ) {


    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void JointQNode::jointStateCallback(const dyros_jet_msgs::JointStateConstPtr &msg)
{
  joint_msg_ = *msg;
  jointStateUpdated();
}

void JointQNode::send_joint_ctrl(int id, const char* jointName, double angle)
{
  if(isConnected)
  {
    double rad = angle / 57.295791433;
    double offset = 0.5;
    double current_position;

    current_position = joint_msg_.angle[id-1];


    joint_cmd_msg_.position[id-1] = current_position + rad;
    if(angle > 0) joint_cmd_msg_.duration[id-1] = 0.5 + rad*offset;
    else joint_cmd_msg_.duration[id-1] = 0.5 - rad*offset;
    joint_cmd_msg_.name[id-1] = jointName;

    joint_ctrl_publisher.publish(joint_cmd_msg_);

    joint_cmd_msg_.name[id-1] = " ";
  }
}

void JointQNode::publish_joint_ctrl()
{
  joint_ctrl_publisher.publish(joint_cmd_msg_);
}

}  // namespace dyros_jet_gui
