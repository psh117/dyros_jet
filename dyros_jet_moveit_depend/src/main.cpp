//world frame tf broadcaster, simulation robot status subsribe.
// JH Ahn
//

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <dyros_jet_msgs/JointState.h>
#include "taskserver.h"

ros::Publisher rviz_joint_pub;


sensor_msgs::JointState JointState;


const std::string JointName[40] = {
                              "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                              "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                              "WaistPitch","WaistYaw",
                              "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                              "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"};

float pos_x=0.0;
float pos_y=0.0;
float pos_z=1.0;
float rot_x=0.0;
float rot_y=0.0;
float rot_z=0.0;
float rot_w=1.0;


Eigen::Vector3d QuinticSpline(
                   double time,       ///< Current time
                   double time_0,     ///< Start time
                   double time_f,     ///< End time
                   double x_0,        ///< Start state
                   double x_dot_0,    ///< Start state dot
                   double x_ddot_0,   ///< Start state ddot
                   double x_f,        ///< End state
                   double x_dot_f,    ///< End state
                   double x_ddot_f )  ///< End state ddot
{
  double a1,a2,a3,a4,a5,a6;

  double time_s;
  time_s = time_f - time_0;
  a1=x_0;
  a2=x_dot_0;
  a3=x_ddot_0/2.0;

  Eigen::Matrix3d Temp;
  Temp<<pow(time_s, 3), pow(time_s, 4), pow(time_s, 5),
        3.0 * pow(time_s, 2), 4.0 * pow(time_s, 3), 5.0 * pow(time_s, 4),
        6.0 * time_s, 12.0 * pow(time_s, 2), 20.0 * pow(time_s, 3);

  Eigen::Vector3d R_temp;
  R_temp<<x_f-x_0-x_dot_0*time_s-x_ddot_0*pow(time_s,2)/2.0,
        x_dot_f-x_dot_0-x_ddot_0*time_s,
        x_ddot_f-x_ddot_0;

  Eigen::Vector3d RES;

  RES = Temp.inverse()*R_temp;

  a4=RES(0);
  a5=RES(1);
  a6=RES(2);

  double time_fs = time - time_0;

  double position = a1+a2*pow(time_fs,1)+a3*pow(time_fs,2)+a4*pow(time_fs,3)+a5*pow(time_fs,4)+a6*pow(time_fs,5);
  double velocity = a2+2.0*a3*pow(time_fs,1)+3.0*a4*pow(time_fs,2)+4.0*a5*pow(time_fs,3)+5.0*a6*pow(time_fs,4);
  double acceleration =2.0*a3+6.0*a4*pow(time_fs,1)+12.0*a5*pow(time_fs,2)+20.0*a6*pow(time_fs,3);

  Eigen::Vector3d RESULT;

  RESULT<<position,velocity,acceleration;

  return RESULT;
}

Eigen::Vector3d QuinticSpline_Trajectory(){


}






void sub_joint_cb(const dyros_jet_msgs::JointStateConstPtr &joint_value)
{


  int joint_size = joint_value->name.size();
  joint_size=joint_size-4;
  sensor_msgs::JointState view_model;
  view_model.header.stamp = ros::Time::now();
  view_model.name.resize(joint_size);
  view_model.position.resize(joint_size);
  for(int i=0; i< joint_size;i++)
  {
      view_model.name[i] = joint_value->name[i];
      view_model.position[i] = joint_value->angle[i];
  }
  rviz_joint_pub.publish(view_model);
}

int main(int argc, char **argv)
{
   ROS_INFO("::::::::DYROS_JET_MOVEIT_DEPEND::ACTIVATE");
   ros::init(argc,argv,"basic_tf");
   ros::NodeHandle nh;
   tf::TransformBroadcaster br;
   tf::Transform tr;
   ros::Rate r(30);
   rviz_joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states",100);
   ros::Subscriber Joint_Status_subscriber = nh.subscribe<dyros_jet_msgs::JointState>("/dyros_jet/joint_state",1,sub_joint_cb);
   FollowJointTrajectoryActionServer server("right_arm_controller/joint_action_server");

   while(ros::ok()){
     /*while (sub2.getNumPublishers() < 1)
     {
       ROS_WARN_ONCE("Please create a subscriber to the marker");
       sleep(1);
     }

     while (sub2.getNumPublishers()<1){

     }*/


     tr.setOrigin(tf::Vector3(pos_x,pos_y,pos_z));
     tr.setRotation(tf::Quaternion(rot_x,rot_y,rot_z,rot_w));
     //tr.setOrigin(tf::Vector3(0,0,1));
     //tr.setRotation(tf::Quaternion(0,0,0,1));
     br.sendTransform(tf::StampedTransform(tr,ros::Time::now(),"world","base_link"));

     ros::spinOnce();
     r.sleep();
   }
  return 0;
}
