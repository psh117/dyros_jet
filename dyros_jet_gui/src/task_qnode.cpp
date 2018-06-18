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
#include "dyros_jet_gui/task_qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_jet_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

TaskQNode::TaskQNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv),
  isConnected(false)
{
  joint_cmd_msg_.position.resize(32);
  joint_cmd_msg_.name.resize(32);
  joint_cmd_msg_.duration.resize(32);
  hand_cmd_msg_.position.resize(4);
  hand_cmd_msg_.name.push_back("hand_finger1");
  hand_cmd_msg_.name.push_back("hand_finger2");
  hand_cmd_msg_.name.push_back("hand_thumb_fe");
  hand_cmd_msg_.name.push_back("hand_thumb_aa");
}

TaskQNode::~TaskQNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  delete nh;  // deallocate ndoe handle
  wait();
}

bool TaskQNode::init() {
  ros::init(init_argc,init_argv,"dyros_jet_gui");
  if ( ! ros::master::check() ) {
    return false;
  }
  init_nh();
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  start();
  return true;
}

void TaskQNode::init_nh()
{
  nh = new ros::NodeHandle("dyros_jet_ui"); // allocate node handle
  // Add your ros communications here.

  joint_ctrl_publisher = nh->advertise<dyros_jet_msgs::JointCommand>("/dyros_jet/joint_command", 5);
  task_cmd_publisher = nh->advertise<dyros_jet_msgs::TaskCommand>("/dyros_jet/task_command", 5);
  walking_cmd_publisher = nh->advertise<dyros_jet_msgs::WalkingCommand>("/dyros_jet/walking_command", 5);
  controlbase_bool_publisher = nh->advertise<std_msgs::Bool>("/dyros_jet/controlbase_bool", 5);
  hand_cmd_publisher_ = nh->advertise<sensor_msgs::JointState>("/dyros_jet/hand_command", 5);

  ft_sensor_calib_publisher = nh->advertise<std_msgs::Float32>("/ati_ft_sensor/calibration", 5);
  ft_sensor_lf_state_subscriber = nh->subscribe("/ati_ft_sensor/left_foot_ft", 1, &TaskQNode::left_ftStateCallback, this);
  ft_sensor_rf_state_subscriber = nh->subscribe("/ati_ft_sensor/right_foot_ft", 1, &TaskQNode::right_ftStateCallback, this);

  isConnected = true;

}

void TaskQNode::run() {
  ros::Rate loop_rate(100);
  while ( ros::ok() ) {


    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void TaskQNode::publish_joint_ctrl()
{
  joint_ctrl_publisher.publish(joint_cmd_msg_);
}

void TaskQNode::send_task_ctrl()
{
  task_cmd_publisher.publish(task_cmd_msg_);
}

void TaskQNode::send_walk_ctrl()
{
  walking_cmd_publisher.publish(walk_cmd_msg_);
}

void TaskQNode::send_bool_cb()
{
  controlbase_bool_publisher.publish(controlbase_bool_);
}

void TaskQNode::send_hand_cmd()
{
  hand_cmd_publisher_.publish(hand_cmd_msg_);
}
void TaskQNode::left_ftStateCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  ft_lf_msg_ = *msg;
}

void TaskQNode::right_ftStateCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  ft_rf_msg_ = *msg;
}

void TaskQNode::send_ft_calib(float time)
{
    if(isConnected)
    {
        std_msgs::Float32 msg;
        msg.data = time;

        ft_sensor_calib_publisher.publish(msg);
    }
}
}  // namespace dyros_jet_gui
