/**
  Dyros Jet moveit trajectory action server.
  receive goal from moveit.

  Status is randomly selected.
  Report Succeeded  / Report Abort / Report nothing/

  by JH Ahn


*/



#include "taskserver.h"



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

void QuinticSpline_Trajectory(ros::Time current_time, ros::Time start_time, control_msgs::FollowJointTrajectoryGoalConstPtr goal_info){
  /*int pointSize;
  pointSize = static_cast<int>(goal_info->trajectory.points.size());

  ros::Duration passed_time;
  passed_time = current_time - start_time;
  int j=0;
  Eigen::Vector3d position_now;
  for(int i=0;i<pointSize-1;i++){
    if((passed_time>=goal_info->trajectory.points[i].time_from_start)&&(passed_time<goal_info->trajectory.points[i+1].time_from_start)){
      position_now=QuinticSpline(passed_time.toSec(),0,goal_info->trajectory.points[i+1].time_from_start.toSec(),
          goal_info->trajectory.points[i].positions[j],goal_info->trajectory.points[i].velocities[j],goal_info->trajectory.points[i].accelerations[j],
          goal_info->trajectory.points[i+1].positions[j],goal_info->trajectory.points[i+1].velocities[j],goal_info->trajectory.points[i+1].accelerations[j]);
    }
  }

*/



}



FollowJointTrajectoryActionServer::FollowJointTrajectoryActionServer(std::string name) :
   // as_(nh_, name, boost::bind(&FollowJointTrajectoryActionServer::executeCB, this, _1),false),
  as_(nh_, name, false),
  action_name_(name)
{
  ROS_INFO("::::::::DYROS_JET_ACTION_SERVER::ACTIVATE");

  //register the goal and feedback callbacks
  as_.registerGoalCallback(boost::bind(&FollowJointTrajectoryActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&FollowJointTrajectoryActionServer::preemptCB, this));

  feedback_.joint_names.resize(7);
  feedback_.actual.positions.resize(7);
  feedback_.actual.velocities.resize(7);
  feedback_.actual.accelerations.resize(7);


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

  srand(ros::Time::now().toNSec());
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
  ros::Duration passedTime = ros::Time::now() - goalStartTime;  //goal 시작시간부터 현재까지 경과된 시간
  if((ros::Time::now() >=goalStartTime)&&(passedTime<goalLastTime)){ //If, current time is between start time of trajectory and end time of trajectory


    int pointSize;
    pointSize = static_cast<int>(goal_->trajectory.points.size());


    int j=0;
    Eigen::Vector3d position_now;
    feedback_.joint_names=goal_->trajectory.joint_names;
    for(int j=0;j<7;j++){

      for(int i=0;i<pointSize-1;i++){
        if((passedTime>=goal_->trajectory.points[i].time_from_start)&&(passedTime<goal_->trajectory.points[i+1].time_from_start)){
          position_now=QuinticSpline(passedTime.toSec(),goal_->trajectory.points[i].time_from_start.toSec(),goal_->trajectory.points[i+1].time_from_start.toSec(),
              goal_->trajectory.points[i].positions[j],goal_->trajectory.points[i].velocities[j],goal_->trajectory.points[i].accelerations[j],
              goal_->trajectory.points[i+1].positions[j],goal_->trajectory.points[i+1].velocities[j],goal_->trajectory.points[i+1].accelerations[j]);
        }
      }
      feedback_.actual.positions[j]=position_now(0);
      feedback_.actual.velocities[j] = position_now(1);
      feedback_.actual.accelerations[j] = position_now(2);

    }
    ROS_INFO_ONCE("LoopEND");








    feedback_.actual.time_from_start=ros::Time::now()-goalStartTime;
    feedback_.header.seq=feedbackHeaderStamp;


    feedbackHeaderStamp++;





  }
















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



  as_.publishFeedback(feedback_);


}


