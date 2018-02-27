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
  ros::init(init_argc,init_argv,"dyros_jet_gui");
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

  smach_publisher = nh->advertise<std_msgs::String>("/dyros_jet/smach/transition", 5);
  smach_subscriber = nh->subscribe("/dyros_jet/smach/container_status",1, &StatusQNode::stateCallback, this);
  hello_cnt_publisher = nh->advertise<std_msgs::Int32>("hello_cnt",5);

  isConnected = true;

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
