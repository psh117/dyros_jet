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
#include <eigen_conversions/eigen_msg.h>

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
#include <actionlib/server/simple_action_server.h>

#include <dyros_jet_msgs/JointSet.h>
#include <dyros_jet_msgs/JointState.h>
#include <dyros_jet_msgs/TaskCommand.h>
#include <dyros_jet_msgs/JointCommand.h>
#include <dyros_jet_msgs/WalkingCommand.h>
#include <dyros_jet_msgs/WalkingState.h>
#include <dyros_jet_msgs/JointControlAction.h>
//#include "dyros_jet_msgs/RecogCmd.h"
//#include "dyros_jet_msgs/TaskCmdboth.h"

// User Library
#include "math_type_define.h"
#include "dyros_jet_controller/controller.h"
#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/task_controller.h"
#include "dyros_jet_controller/haptic_controller.h"
#include "dyros_jet_controller/joint_controller.h"
#include "dyros_jet_controller/walking_controller.h"
#include "dyros_jet_controller/moveit_controller.h"
#include "dyros_jet_controller/haptic_controller.h"

// #include "Upperbody_Controller.h"



namespace dyros_jet_controller
{

using namespace Eigen;
using namespace std;

class ControlBase
{
  //typedef actionlib::SimpleActionServer<dyros_jet_msgs::JointControlAction> JointServer;

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
  bool isShuttingDown() const {return shutdown_flag_;}

  bool checkStateChanged();

  void stateChangeEvent();

  double currentTime(){return control_time_;};
  void syncSimControlTime(double time){control_time_ = time; std::cout <<"control time "<< (double)control_time_ <<std::endl;};

  const double getHz() { return Hz_; }
protected:

  unsigned int joint_id_[DyrosJetModel::HW_TOTAL_DOF];
  unsigned int joint_id_inversed_[DyrosJetModel::HW_TOTAL_DOF];
  unsigned int control_mask_[DyrosJetModel::HW_TOTAL_DOF];

  int ui_update_count_;
  bool is_first_boot_;
  bool extencoder_init_flag_;

  VectorQd q_; ///< current q
  VectorQd q_dot_; ///< current qdot
  VectorQd q_dot_filtered_; ///< current qdot with filter
  VectorQd torque_; // current joint toruqe
  Eigen::Vector12d q_ext_;
  Eigen::Vector12d q_ext_dot_;
  Eigen::Vector12d q_ext_offset_;


  Vector6d left_foot_ft_; // current left ft sensor values
  Vector6d right_foot_ft_; // current right ft sensor values

  tf::Quaternion imu_data_; ///< IMU data with filter
  Vector3d gyro_; // current gyro sensor values
  Vector3d accelometer_; // current accelometer values
  Vector3d imu_grav_rpy_;
  Matrix3d pelvis_orientation_;

  Vector3d com_sim_; //com position from simulation COMvisualziefunction
  Eigen::Isometry3d lfoot_global_;
  Eigen::Isometry3d rfoot_global_;
  Eigen::Isometry3d base_global_;

  VectorQd desired_q_; // current desired joint values
  Eigen::Vector12d extencoder_offset_;

  Eigen::Vector6d mujoco_virtual_;
  Eigen::Vector6d mujoco_virtual_dot_;


  int total_dof_;

  DyrosJetModel model_;
  TaskController task_controller_;
  HapticController haptic_controller_;
  JointController joint_controller_;
  WalkingController walking_controller_;
  MoveitController moveit_controller_;

protected:
  string current_state_;
  realtime_tools::RealtimePublisher<dyros_jet_msgs::JointState> joint_state_pub_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> joint_robot_state_pub_;

private:
  double Hz_; ///< control
  unsigned long tick_;
  double control_time_;

  double r_prev;
  double p_prev;
  double y_prev;
  double r_prev1;
  double p_prev1;
  double y_prev1;

  string previous_state_;

  bool shutdown_flag_;

  // ROS
  ros::Subscriber task_cmd_sub_;
  ros::Subscriber joint_cmd_sub_;
  ros::Subscriber task_comamnd_sub_;
  ros::Subscriber haptic_command_sub_;
  ros::Subscriber joint_command_sub_;
  ros::Subscriber walking_command_sub_;
  ros::Subscriber shutdown_command_sub_;
  ros::Publisher walkingstate_command_pub_;
  std_msgs::Bool walkingState_msg;


  // TODO: realtime_tools
  dyros_jet_msgs::JointControlFeedback joint_control_feedback_;
  dyros_jet_msgs::JointControlResult joint_control_result_;
  actionlib::SimpleActionServer<dyros_jet_msgs::JointControlAction>  joint_control_as_;  // Action Server

  // State Machine (SMACH)
  realtime_tools::RealtimePublisher<std_msgs::String> smach_pub_;
  ros::Subscriber smach_sub_;




  void smachCallback(const smach_msgs::SmachContainerStatusConstPtr& msg);
  void taskCommandCallback(const dyros_jet_msgs::TaskCommandConstPtr& msg);
  void hapticCommandCallback(const dyros_jet_msgs::TaskCommandConstPtr& msg);
  void jointCommandCallback(const dyros_jet_msgs::JointCommandConstPtr& msg);
  void walkingCommandCallback(const dyros_jet_msgs::WalkingCommandConstPtr& msg);
  void shutdownCommandCallback(const std_msgs::StringConstPtr& msg);
  void jointControlActionCallback(const dyros_jet_msgs::JointControlGoalConstPtr &goal);
private:

  void makeIDInverseList();

};

}

#endif
