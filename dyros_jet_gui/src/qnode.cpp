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
#include "dyros_jet_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_jet_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
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

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  delete nh;  // deallocate ndoe handle
  wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"dyros_jet_gui");
  if ( ! ros::master::check() ) {
    return false;
  }
  init_nh();
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings,"dyros_jet_gui");
  if ( ! ros::master::check() ) {
    return false;
  }
  init_nh();
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  start();
  return true;
}
void QNode::init_nh()
{
  nh = new ros::NodeHandle("dyros_jet_ui"); // allocate node handle
  // Add your ros communications here.

  smach_publisher = nh->advertise<std_msgs::String>("/transition", 5);
  joint_ctrl_publisher = nh->advertise<dyros_jet_msgs::JointCommand>("/dyros_jet/joint_command", 5);
  task_cmd_publisher = nh->advertise<dyros_jet_msgs::TaskCommand>("/dyros_jet/task_command", 5);
  walking_cmd_publisher = nh->advertise<dyros_jet_msgs::WalkingCommand>("/dyros_jet/walking_command", 5);
  controlbase_publisher = nh->advertise<std_msgs::String>("/dyros_jet/controlbase_bool", 5);
  /*
    joint_ctrl_publisher = nh->advertise<thormang_ctrl_msgs::JointSet>("joint_ctrl",5);
    task_cmd_publisher = nh->advertise<thormang_ctrl_msgs::TaskCmd>("task_cmd",5);
    recog_cmd_publisher = nh->advertise<thormang_ctrl_msgs::RecogCmd>("recog_cmd",5);
    walking_cmd_publisher = nh->advertise<thormang_ctrl_msgs::WalkingCmd>("walking_cmd",5);
    */

  ft_sensor_calib_publisher = nh->advertise<std_msgs::Float32>("/ati_ft_sensor/calibration", 5);

  hello_cnt_publisher = nh->advertise<std_msgs::Int32>("hello_cnt",5);
  joint_state_subscirber = nh->subscribe("/dyros_jet/joint_state",1, &QNode::jointStateCallback, this);
  //joint_state_subscirber = nh->subscribe("joint_state",1,&QNode::jointStateCallback,this);
  //recog_point_subscriber = nh->subscribe("/custom_recog_point",1, &QNode::recogInfoCallback, this);

  isConnected = true;

}

void QNode::run() {
  ros::Rate loop_rate(100);
  while ( ros::ok() ) {


    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::send_transition(std::string str)
{
  if(isConnected)
  {
    std_msgs::String msg;
    msg.data = str;
    smach_publisher.publish(msg);
  }
}


void QNode::jointStateCallback(const dyros_jet_msgs::JointStateConstPtr &msg)
{
  joint_msg_ = *msg;
  jointStateUpdated();
}

void QNode::send_joint_ctrl(int id, const char* jointName, double angle)
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

void QNode::publish_joint_ctrl()
{
  joint_ctrl_publisher.publish(joint_cmd_msg_);
}

void QNode::send_task_ctrl()
{
  task_cmd_publisher.publish(task_cmd_msg_);
}

void QNode::send_walk_ctrl()
{
  walking_cmd_publisher.publish(walk_cmd_msg_);
}

void QNode::send_data_cb()
{
  controlbase_publisher.publish(controlbase_data_);
}
/*
void QNode::send_ft_calib(float time)
{
    if(isConnected)
    {
        std_msgs::Float32 msg;
        msg.data = time;

        ft_sensor_calib_publisher.publish(msg);
    }
}

void QNode::send_joint_ctrl(int id, double angle)
{
    if(isConnected)
    {
        thormang_ctrl_msgs::JointSet msg;
        msg.angle = angle;
        msg.id = id;

        joint_ctrl_publisher.publish(msg);
    }
}

void QNode::send_walking_cmd(thormang_ctrl_msgs::WalkingCmd& walking_msg)
{
    walking_cmd_publisher.publish(walking_msg);
}

void QNode::send_recog_cmd(thormang_ctrl_msgs::RecogCmd& recog_msg)
{
    recog_cmd_publisher.publish(recog_msg);
}

void QNode::send_task_cmd(thormang_ctrl_msgs::TaskCmd& task_msg)
{    
    task_cmd_publisher.publish(task_msg);
}

void QNode::send_hello_count(const int count)
{
    std_msgs::Int32 msg;
    msg.data = count;
    hello_cnt_publisher.publish(msg);
}
void QNode::jointStateCallback(const thormang_ctrl_msgs::JointStateConstPtr &msg)
{
    joint_msg = *msg;
    jointStateUpdated();
}
void QNode::recogInfoCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    recog_info_msg = *msg;
    recogInfoUpdated();
}

*/
void QNode::log( const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  switch ( level ) {
  case(Debug) : {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Info) : {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Warn) : {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Error) : {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Fatal) : {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace dyros_jet_gui
