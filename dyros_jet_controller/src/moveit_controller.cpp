#include <dyros_jet_controller/moveit_controller.h>



namespace dyros_jet_controller
{

MoveitController::MoveitController(DyrosJetModel& model, const VectorQd& current_q, const double& control_time) :
  current_q_(current_q), current_time_(control_time), model_(model),
  total_dof_(DyrosJetModel::HW_TOTAL_DOF), feedback_header_stamp_(0),
  start_time_{}, end_time_{},
  action_name_("arm_controller/joint_action_server"),
  as_(nh_, action_name_, false)
{
  as_.registerGoalCallback(boost::bind(&MoveitController::moveitGoalCB, this));
  as_.registerPreemptCallback(boost::bind(&MoveitController::moveitPreemptCB, this));


  // subscribe current state of the action

  as_.start();
}

void MoveitController::setEnable(DyrosJetModel::EndEffector ee, bool enable)
{
  ee_enabled_[ee] = enable;
}


void MoveitController::updateControlMask(unsigned int *mask)
{
  unsigned int index = 0;
  for(int i=0; i<total_dof_; i++)
  {
    if(i < 6)
    {
      index = 0;
    }
    else if (i < 6 + 6)
    {
      index = 1;
    }
    else if (i < 6 + 6 + 2)
    {
      continue; // waist
    }
    else if (i < 6 + 6 + 2 + 7)
    {
      index = 2;
    }
    else if (i < 6 + 6 + 2 + 7 + 7)
    {
      index = 3;
    }

    if(ee_enabled_[index])
    {
      if (mask[i] >= PRIORITY * 2)
      {
        // Higher priority task detected
        ee_enabled_[index] = false;
        end_time_[index] = current_time_;
        as_.setAborted();
        ROS_INFO("Moveit Controller Aborted");
        if (index < 2)  // Legs
        {
          desired_q_.segment<6>(model_.joint_start_index_[index]) = current_q_.segment<6>(model_.joint_start_index_[index]);
        }
        else
        {
          desired_q_.segment<7>(model_.joint_start_index_[index]) = current_q_.segment<7>(model_.joint_start_index_[index]);
        }
      }
      mask[i] = (mask[i] | PRIORITY);
    }
    else
    {
      mask[i] = (mask[i] & ~PRIORITY);
    }
  }
}

void MoveitController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
{
  for(unsigned int i=0; i<total_dof_; i++)
  {
    if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
    {
      desired_q(i) = desired_q_(i);
    }
  }
}

void MoveitController::compute()
{
  if (!as_.isActive())
    return;


  ros::Duration passedTime = ros::Time::now() - goal_start_time_;  //goal 시작시간부터 현재까지 경과된 시간
  if((ros::Time::now() >=goal_start_time_)&&(passedTime<goal_last_time_)){ //If, current time is between start time of trajectory and end time of trajectory

    int point_size;
    point_size = static_cast<int>(goal_->trajectory.points.size());

    int j=0;
    Eigen::Vector3d position_now;
    feedback_.joint_names=goal_->trajectory.joint_names;
    for(int j=0;j<moveit_controller_joint_size;j++){ // j = joint number

      for(int i=0;i<point_size-1;i++){
        if((passedTime>=goal_->trajectory.points[i].time_from_start)&&(passedTime<goal_->trajectory.points[i+1].time_from_start)){
          position_now=DyrosMath::QuinticSpline(passedTime.toSec(),goal_->trajectory.points[i].time_from_start.toSec(),goal_->trajectory.points[i+1].time_from_start.toSec(),
              goal_->trajectory.points[i].positions[j],goal_->trajectory.points[i].velocities[j],goal_->trajectory.points[i].accelerations[j],
              goal_->trajectory.points[i+1].positions[j],goal_->trajectory.points[i+1].velocities[j],goal_->trajectory.points[i+1].accelerations[j]);
        }
      }
      feedback_.actual.positions[j] = position_now(0);
      feedback_.actual.velocities[j] = position_now(1);
      feedback_.actual.accelerations[j] = position_now(2);
      desired_q_(model_.getIndex(goal_->trajectory.joint_names[j])) = position_now(0);

    }
    ROS_INFO_ONCE("LoopEND");

    feedback_.actual.time_from_start=ros::Time::now()-goal_start_time_;
    feedback_.header.seq=feedback_header_stamp_;


    feedback_header_stamp_++;
    as_.publishFeedback(feedback_);
  }
  if(ros::Time::now() > goal_start_time_ +  goal_last_time_)
  {
    as_.setSucceeded();
    setEnable(DyrosJetModel::EE_RIGHT_HAND, false);
    setEnable(DyrosJetModel::EE_LEFT_HAND,false);
    ROS_INFO("BothHand Disabled ");
  }

  //feedback publisher

}

void MoveitController::moveitGoalCB()
{

  feedback_header_stamp_=0;
  goal_ = as_.acceptNewGoal();
  ROS_INFO("Goal received :::: ");

  goal_start_time_ = ros::Time::now();
  goal_last_time_ = goal_->trajectory.points.back().time_from_start;


  setEnable(DyrosJetModel::EE_RIGHT_HAND, false);
  setEnable(DyrosJetModel::EE_LEFT_HAND,false);

  moveit_controller_joint_size = goal_->trajectory.points[0].positions.size();

  for(int i=0;i<moveit_controller_joint_size;i++){
      if(goal_->trajectory.joint_names[i]=="R_ElbowRoll"){
          setEnable(DyrosJetModel::EE_RIGHT_HAND, true);
          ROS_INFO("RightArm Enabled");}
  }
  for(int i=0;i<moveit_controller_joint_size;i++){
      if(goal_->trajectory.joint_names[i]=="L_ElbowRoll"){
          setEnable(DyrosJetModel::EE_LEFT_HAND, true);
          ROS_INFO("LeftArm Enabled");}

  }




  feedback_.joint_names.resize(moveit_controller_joint_size);
  feedback_.actual.positions.resize(moveit_controller_joint_size);
  feedback_.actual.velocities.resize(moveit_controller_joint_size);
  feedback_.actual.accelerations.resize(moveit_controller_joint_size);

  //std::cout<<"Last time of point :: \n \t sec :"<< goal_last_time_.sec << "\t nsec : " <<goal_last_time_.nsec<<std::endl;
}

void MoveitController::moveitPreemptCB()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();

}
void MoveitController::moveitExecuteCB()
{

  ROS_INFO("moveitExecuteCB??? :::: ");
}
void MoveitController::waitForClient()
{

}

}
