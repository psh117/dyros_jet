#ifndef TASK_SERVER_H
#define TASK_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <dyros_jet_msgs/JointState.h>
#include <eigen3/Eigen/Dense>




class FollowJointTrajectoryActionServer {
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::vector<trajectory_msgs::JointTrajectoryPoint> JointTrajectory;


  ros::Time goalStartTime;
  ros::Duration goalLastTime;

  std::string action_name_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryGoalConstPtr goal_;

  ros::Subscriber jointStateSubscriber;


  //ros::Subscriber followJointTrajectoryResultSubscriber;
  //std::vector<std::string> joint_names_;


  dyros_jet_msgs::JointState jointState;


public:
    FollowJointTrajectoryActionServer(std::string name);
  ~FollowJointTrajectoryActionServer(void);


  // receive action goal function
  void goalCB();

  // cancel the action

  void preemptCB();


  void executeCB();

  // get current state of the action
  void jointCB(const dyros_jet_msgs::JointState &msg);


  void waitForClient();

  int randTest;

private:
  int feedbackHeaderStamp;


};








#endif
