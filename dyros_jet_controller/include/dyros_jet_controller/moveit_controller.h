#ifndef MOVEIT_CONTROLLER_H
#define MOVEIT_CONTROLLER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <dyros_jet_msgs/JointState.h>
#include <Eigen/Dense>


#include "dyros_jet_controller/dyros_jet_model.h"
#include "math_type_define.h"

namespace dyros_jet_controller
{



class MoveitController
{
public:
  static constexpr unsigned int PRIORITY = 4;  ///< Joint priority

  MoveitController(DyrosJetModel& model, const VectorQd& current_q, const double& control_time);
  void compute();
  void setEnable(DyrosJetModel::EndEffector ee, bool enable);
  void updateControlMask(unsigned int *mask);
  void writeDesired(const unsigned int *mask, VectorQd& desired_q);
private:
  bool ee_enabled_[4];  ///< End effector enable

  const unsigned int total_dof_;
  VectorQd start_q_;
  VectorQd desired_q_;
  VectorQd target_q_;
  const VectorQd& current_q_;
  const double &current_time_;
  double start_time_[DyrosJetModel::HW_TOTAL_DOF];
  double end_time_[DyrosJetModel::HW_TOTAL_DOF];

  const DyrosJetModel& model_;

  int moveit_controller_joint_size = 14;

  // For action server
  ros::NodeHandle nh_;
  std::string action_name_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectory_;

  ros::Time goal_start_time_;
  ros::Duration goal_last_time_;

  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryGoalConstPtr goal_;


  void moveitGoalCB();
  void moveitPreemptCB();
  void moveitExecuteCB();
  void waitForClient();

  int feedback_header_stamp_;
};




} // namespace dyros_jet_controller


#endif  // MOVEIT_CONTROLLER_H
