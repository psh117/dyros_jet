#include "taskserver.h"

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





int main(int argc, char **argv) {
  ros::init(argc, argv, "joint_action_server");
  FollowJointTrajectoryActionServer server("joint_action_server");

  while(ros::ok()){
    ros::spinOnce();
  }

  return 0;
}
