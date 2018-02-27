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
#include "dyros_jet_gui/status_qnode.hpp"
#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_jet_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

StatusQNode::StatusQNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv),
  isConnected(false)
{

}

StatusQNode::~StatusQNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  delete nh;  // deallocate ndoe handle
  wait();
}

bool StatusQNode::init() {
  ros::init(init_argc,init_argv,"dyros_jet_gui_status");
  if ( ! ros::master::check() ) {
    return false;
  }
  init_nh();
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  start();
  return true;
}

void StatusQNode::init_nh()
{
  nh = new ros::NodeHandle("dyros_jet_ui"); // allocate node handle
  // Add your ros communications here.

  dxl_mode_set_client_ = nh->serviceClient<rt_dynamixel_msgs::ModeSetting>("/rt_dynamixel/mode");
  dxl_motor_set_client_ = nh->serviceClient<rt_dynamixel_msgs::MotorSetting>("/rt_dynamixel/motor_set");

  shutdown_publisher = nh->advertise<std_msgs::String>("/dyros_jet/shutdown_command", 1);
  smach_publisher = nh->advertise<std_msgs::String>("/dyros_jet/smach/transition", 5);

  smach_subscriber = nh->subscribe("/dyros_jet/smach/container_status",1, &StatusQNode::stateCallback, this);
  hello_cnt_publisher = nh->advertise<std_msgs::Int32>("hello_cnt",5);

  isConnected = true;

}
void StatusQNode::shutdown()
{
  std_msgs::String msg;
  msg.data = "Shut up, JET.";
  shutdown_publisher.publish(msg);
}
void StatusQNode::run() {
  ros::Rate loop_rate(100);
  while ( ros::ok() ) {


    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}






void StatusQNode::changeDxlMode(int mode)
{
  rt_dynamixel_msgs::ModeSettingRequest req;
  rt_dynamixel_msgs::ModeSettingResponse res;

  req.mode = mode;

  dxl_mode_set_client_.call(req,res);
}

void StatusQNode::setAimPosition(int id, double radian)
{
  rt_dynamixel_msgs::MotorSettingRequest req;
  rt_dynamixel_msgs::MotorSettingResponse res;

  req.mode=rt_dynamixel_msgs::MotorSettingRequest::SET_GOAL_POSITION;
  req.id = id;
  req.fvalue = radian;

  dxl_motor_set_client_.call(req,res);
}

void StatusQNode::setTorque(int value)
{

  rt_dynamixel_msgs::MotorSettingRequest req;
  rt_dynamixel_msgs::MotorSettingResponse res;

  req.mode=rt_dynamixel_msgs::MotorSettingRequest::SET_TORQUE_ENABLE;
  req.value = value;

  dxl_motor_set_client_.call(req,res);

}









void StatusQNode::send_transition(std::string str)
{
  if(isConnected)
  {
    std_msgs::String msg;
    msg.data = str;
    smach_publisher.publish(msg);
  }
}

void StatusQNode::stateCallback(const smach_msgs::SmachContainerStatusConstPtr &msg)
{
  current_state = msg->active_states[0];
  stateUpdated();
}


}  // namespace dyros_jet_gui
