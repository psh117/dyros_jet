//world frame tf broadcaster, simulation robot status subsribe.
// JH Ahn
//

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <dyros_jet_msgs/JointState.h>
#include <taskserver.h>

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



int randTest;

FollowJointTrajectoryActionServer::FollowJointTrajectoryActionServer(std::string name) :
   // as_(nh_, name, boost::bind(&FollowJointTrajectoryActionServer::executeCB, this, _1),false),
  as_(nh_, name, false),
  action_name_(name)
{
  ROS_INFO("::::::::DYROS_JET_ACTION_SERVER::ACTIVATE");

  //register the goal and feedback callbacks
  as_.registerGoalCallback(boost::bind(&FollowJointTrajectoryActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&FollowJointTrajectoryActionServer::preemptCB, this));


  // subscribe current state of the action
  jointStateSubscriber = nh_.subscribe("dyros_jet/joint_state",1,&FollowJointTrajectoryActionServer::jointCB, this);

  as_.start();
}
FollowJointTrajectoryActionServer::~FollowJointTrajectoryActionServer(void) {}







// receive action goal function
void FollowJointTrajectoryActionServer::goalCB() {
  feedbackHeaderStamp=0;
  goal_ = as_.acceptNewGoal();
  ROS_INFO("Goal received :::: ");
  goalStartTime = ros::Time::now();


  randTest = rand()%3;
  int pointSize;
  pointSize = static_cast<int>(goal_->trajectory.points.size());
  std::cout<<"Total size of point : "<< pointSize <<std::endl;

  for(int i=0;i<pointSize;i++){
    std::cout<<"time: "<<goal_->trajectory.points[i].time_from_start<<"\t "<<goal_->trajectory.joint_names[0]<<"\t position : "
            <<goal_->trajectory.points[i].positions[0]<<"\t velocity : "
           <<goal_->trajectory.points[i].velocities[0]<<"\t acceleration : "
          <<goal_->trajectory.points[i].accelerations[0]<<std::endl;



  }



  goalLastTime = goal_->trajectory.points[pointSize-1].time_from_start;
  std::cout<<"Last time of point :: \n \t sec :"<< goalLastTime.sec << "\t nsec : " <<goalLastTime.nsec<<std::endl;

}





// cancel the action
void FollowJointTrajectoryActionServer::preemptCB() {
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}





// get current state of the action
void FollowJointTrajectoryActionServer::jointCB(const dyros_jet_msgs::JointState &msg) {
  if (!as_.isActive())
    return;


  jointState=msg;
  ros::Duration passedTime = ros::Time::now() - goalStartTime;


  if(goalLastTime<passedTime){

    if(randTest==0){
      as_.setSucceeded();
      ROS_INFO("Random TEST :: Succeeded");
    }
    else if(randTest == 1){
      as_.setAborted();
      ROS_INFO("Random TEST :: Aborted");
    }
    else {
      ROS_INFO_ONCE("Random TEST :: DO nothing");
    }
  }

  //feedback publisher
  feedback_.actual.time_from_start=ros::Time::now()-goalStartTime;
  feedback_.header.seq=feedbackHeaderStamp;
  feedbackHeaderStamp++;
  as_.publishFeedback(feedback_);


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
