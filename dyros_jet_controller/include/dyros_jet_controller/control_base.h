#ifndef _CONTROL_BASE_H
#define _CONTROL_BASE_H

// STD Library
#include <array>
#include <vector>

// System Library
#include <termios.h>

// ROS Library
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_datatypes.h>

// ROS Messages
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <smach_msgs/SmachContainerStatus.h>

#include "dyros_jet_msgs/JointSet.h"
#include "dyros_jet_msgs/JointState.h"
#include "dyros_jet_msgs/WalkingCmd.h"
#include "dyros_jet_msgs/RecogCmd.h"
#include "dyros_jet_msgs/TaskCmd.h"
#include "dyros_jet_msgs/TaskCmdboth.h"

// User Library
#include "math_type_define.h"
#include "dyros_jet_controller/dyros_jet_model.h"
#include "task_controller.h"
// #include "Walking_Controller.h"
// #include "Upperbody_Controller.h"


namespace dyros_jet_controller
{

using namespace Eigen;
using namespace std;

class ControlBase
{

public:
  ControlBase(ros::NodeHandle &nh, double Hz);
  virtual ~ControlBase(){}
  // Default User Call function
  void parameterInitialize(); // initialize all parameter function(q,qdot,force else...)
  virtual void readDevice(); // read device means update all subscribed sensor data and user command
  virtual void update(); // update controller based on readdevice
  virtual void compute(); // compute algorithm and update all class object
  virtual void reflect(); // reflect next step actuation such as motor angle else
  virtual void writeDevice()=0; // publish to actuate devices
  virtual void wait()=0;  // wait

  bool checkStateChanged();

  void stateChangeEvent();



  const double getHz() { return Hz_; }
protected:

  unsigned int joint_id_[DyrosJetModel::HW_TOTAL_DOF];
  unsigned int joint_id_inversed_[DyrosJetModel::HW_TOTAL_DOF];
  unsigned int control_mask_[DyrosJetModel::HW_TOTAL_DOF];

  int ui_update_count_;
  bool is_first_boot_;

  VectorQd q_; // current q
  VectorQd q_dot_; // current qdot
  VectorQd torque_; // current joint toruqe

  Vector6d left_foot_ft_; // current left ft sensor values
  Vector6d right_foot_ft_; // current right ft sensor values

  Vector3d gyro_; // current gyro sensor values
  Vector3d accelometer_; // current accelometer values

  Matrix3d pelvis_orientation_;

  VectorQd desired_q_; // current desired joint values

  int total_dof_;

  DyrosJetModel model_;
  TaskController task_controller_;

private:
  double Hz_; ///< control
  unsigned long tick_;
  double control_time_;

  string current_state_;
  string previous_state_;


  // ROS
  ros::Subscriber walking_cmd_sub_;
  ros::Subscriber task_cmd_sub_;
  ros::Subscriber joint_cmd_sub_;
  //ros::Subscriber recog_point_sub_;
  // ros::Subscriber recog_cmd_sub_;

  // State Machine (SMACH)
  realtime_tools::RealtimePublisher<std_msgs::String> smach_pub_;
  ros::Subscriber smach_sub_;

  // realtime_tools::RealtimePublisher<thormang_ctrl_msgs::JointState> joint_state_pub_-;

  void smachCallback(const smach_msgs::SmachContainerStatusConstPtr& msg);
private:

  void makeIDInverseList();

};

}

#endif
