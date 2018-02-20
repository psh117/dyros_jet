//world frame tf broadcaster, simulation robot status subsribe.

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

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


/*
void real_joint_cb(const dyros_jet_msgs::JointState::ConstPtr &joint_value)
{
  int joint_size = 28;
  sensor_msgs::JointState view_model;
  view_model.header.stamp = ros::Time::now();
  view_model.name.resize(joint_size);
  view_model.position.resize(joint_size);
  for(int i=0; i< joint_size;i++)
  {
      view_model.name[i] = JointName[i];
      view_model.position[i] = joint_value->angle[i];
  }
  rviz_joint_pub.publish(view_model);
}*/

void simul_joint_cb(const sensor_msgs::JointState::ConstPtr &joint_value)
{
  int joint_size = 28;
  sensor_msgs::JointState view_model;
  view_model.header.stamp = ros::Time::now();
  view_model.name.resize(joint_size);
  view_model.position.resize(joint_size);
  for(int i=0; i< joint_size;i++)
  {
      view_model.name[i] = JointName[i];
      view_model.position[i] = joint_value->position[i];
  }
  rviz_joint_pub.publish(view_model);
}

/*
void chatterCallback2(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& msg)
{
  ROS_INFO("x,y,z : %f, %f, %f", msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

  pos_x = msg->pose.position.x;
  pos_y = msg->pose.position.y;
  pos_z = msg->pose.position.z;
  rot_x = msg->pose.orientation.x;
  rot_y = msg->pose.orientation.y;
  rot_z = msg->pose.orientation.z;
  rot_w = msg->pose.orientation.w;
}*/


int main(int argc, char **argv)
{
   ROS_INFO("DYROS_JET_MOVEIT_DEPEND::ACTIVATE");
   ros::init(argc,argv,"basic_tf");
   ros::NodeHandle nh;
   tf::TransformBroadcaster br;
   tf::Transform tr;
   ros::Rate r(30);
   rviz_joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states",1);
   //ros::Subscriber jet_joint_sub = nh.subscribe<dyros_jet_msgs::JointState>("joint_state",1,real_joint_cb);
   ros::Subscriber jet_simul_sub = nh.subscribe<sensor_msgs::JointState>("vrep_ros_interface/joint_state",1,simul_joint_cb);
   //ros::Subscriber sub2 = nh.subscribe("marker/feedback",1,chatterCallback2);
   ros::Publisher controller_pub = nh.advertise<control_msgs::FollowJointTrajectoryAction>("jet_control_test/follow_joint_trajectory", 1);
   
   while(ros::ok()){
     /*while (sub2.getNumPublishers() < 1)
     {
       if (!ros::ok())
       {
         //tr.setOrigin(tf::Vector3(0,0,1));
         //tr.setRotation(tf::Quaternion(0,0,0,1));
         //br.sendTransform(tf::StampedTransform(tr,ros::Time::now(),"world","base_link"));
         return 0;
       }
       ROS_WARN_ONCE("Please create a subscriber to the marker");
       sleep(1);
     }

     while (sub2.getNumPublishers()<1){

     }*/

     tr.setOrigin(tf::Vector3(pos_x,pos_y,pos_z));
     tr.setRotation(tf::Quaternion(rot_x,rot_y,rot_z,rot_w));
     //tr.setOrigin(tf::Vector3(0,0,1));
     //tr.setRotation(tf::Quaternion(0,0,0,1));
     br.sendTransform(tf::StampedTransform(tr,ros::Time::now(),"odom_combined","world"));

     ros::spinOnce();
     r.sleep();
   }

}
