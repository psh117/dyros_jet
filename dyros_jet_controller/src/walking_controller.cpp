#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller.h"
#include "cvxgen_6_8_0/cvxgen/solver.h"


Vars vars;
Params params;
Workspace work;
Settings settings;

namespace dyros_jet_controller
{



void WalkingController::compute()
{
  //std::cout<<"enable"<<walking_enable_<<endl;
  if(walking_enable_)
  {

    std::cout<<"walking_tick_:"<<walking_tick_<<endl;
    //std::cout<<"current_step_num_:"<<current_step_num_<<endl;
    //std::cout<<"total_step_num:"<<total_step_num_<<endl;
    //std::cout<<"t_last_:"<<t_last_<<endl;

    updateInitialState();

    getRobotState();

    floatToSupportFootstep();

    /////state estimation///////////
    getEstimationInputMatrix();

    solve();
    ////////////////////////////////

    if(current_step_num_< total_step_num_)
    {
      getZmpTrajectory();

      getComTrajectory();

      getPelvTrajectory();

      getFootTrajectory();

      supportToFloatPattern();



      //std::cout<<"work.converged:"<<work.converged<<endl;

      //std::cout<<"vars.x[0]:"<<vars.x[0]<<endl;
      //std::cout<<"vars.x[2]:"<<vars.x[2]<<endl;
      //std::cout<<"vars.x[4]:"<<vars.x[4]<<endl;
      ///////////////0/////////////////


      /////////////////////compute/////////////////////////
      if (ik_mode_ == 0)
      {
        //pelv_trajectory_float_ = pelv_float_init_;
        //pelv_trajectory_float_.translation()(2) = pelv_float_init_.translation()(2);
        //pelv_trajectory_float_.translation()(0) = DyrosMath::cubic(walking_tick_, 0, 200, pelv_float_init_.translation()(0), 0, 0, 0);
        //pelv_trajectory_float_.translation()(1) = 0.10*sin(2*M_PI*walking_tick_/(hz_*30.0));
        //pelv_trajectory_float_.translation()(1) = DyrosMath::cubic(walking_tick_, 0, 200, pelv_float_init_.translation()(0), 0.10, 0, 0);
        //lfoot_trajectory_float_ = lfoot_float_init_;
        //rfoot_trajectory_float_ = rfoot_float_init_;



        computeIkControl(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, desired_leg_q_);
        for(int i=0; i<12; i++)
        {
          desired_q_(i) = desired_leg_q_(i);
        }
      }
      else if (ik_mode_ == 1)
      {
        computeJacobianControl(lfoot_trajectory_float_, rfoot_trajectory_float_, lfoot_trajectory_euler_float_, rfoot_trajectory_euler_float_, desired_leg_q_dot_);
        for(int i=0; i<6; i++)
        {
          if(walking_tick_ == 0)
          {
            desired_q_(i) = q_init_(i);
            desired_q_(i+6) = q_init_(i+6);
          }
          desired_q_(i) = desired_leg_q_dot_(i)/hz_+current_q_(i);
          desired_q_(i+6) = desired_leg_q_dot_(i+6)/hz_+current_q_(i+6);
        }
      }
      //////////////////////////////////////////////////////


      compensator();

      //std::cout<<"com_support_current_:"<<com_support_current_<<endl;
      //std::cout<<"zmp_measured_:"<<zmp_measured_<<endl;


      //////////////data saving in text files///////////
      file[0]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<zmp_desired_(0)<<"\t"<<zmp_desired_(1)<<"\t"<<foot_step_(current_step_num_, 0)<<"\t"<<foot_step_(current_step_num_, 1)<<"\t"<<
               foot_step_support_frame_(current_step_num_, 0)<<"\t"<<foot_step_support_frame_(current_step_num_, 1)<<"\t"<<foot_step_support_frame_(current_step_num_, 2)<<endl;
      file[1]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<com_desired_(0)<<"\t"<<com_desired_(1)<<"\t"<<com_desired_(2)<<"\t"<<com_dot_desired_(0)<<"\t"<<com_dot_desired_(1)<<"\t"<<
               com_dot_desired_(2)<<"\t"<<com_support_init_(0)<<"\t"<<com_support_init_(0)<<"\t"<<com_support_init_(0)<<endl;
      file[2]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<desired_leg_q_(0)<<"\t"<<desired_leg_q_(1)<<"\t"<<desired_leg_q_(2)<<"\t"<<desired_leg_q_(3)<<"\t"<<desired_leg_q_(4)<<"\t"<<
               desired_leg_q_(5)<<"\t"<<desired_leg_q_(6)<<"\t"<<desired_leg_q_(7)<<"\t"<<desired_leg_q_(8)<<"\t"<<desired_leg_q_(9)<<"\t"<<desired_leg_q_(10)<<"\t"<<desired_leg_q_(11)<<endl;
      file[3]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<current_q_(0)<<"\t"<<current_q_(1)<<"\t"<<current_q_(2)<<"\t"<<current_q_(3)<<"\t"<<current_q_(4)<<"\t"<<current_q_(5)<<"\t"<<
               current_q_(6)<<"\t"<<current_q_(7)<<"\t"<<current_q_(8)<<"\t"<<current_q_(9)<<"\t"<<current_q_(10)<<"\t"<<current_q_(11)<<endl;
      file[4]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<rfoot_trajectory_support_.translation()(0)<<"\t"<<rfoot_trajectory_support_.translation()(1)<<"\t"<<
               rfoot_trajectory_support_.translation()(2)<<"\t"<<lfoot_trajectory_support_.translation()(0)<<"\t"<<lfoot_trajectory_support_.translation()(1)<<"\t"<<lfoot_trajectory_support_.translation()(2)<<"\t"<<
               rfoot_support_init_.translation()(0)<<"\t"<<rfoot_support_init_.translation()(1)<<"\t"<<rfoot_support_init_.translation()(2)<<endl;
      file[5]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<pelv_trajectory_support_.translation()(0)<<"\t"<<pelv_trajectory_support_.translation()(1)<<"\t"<<pelv_trajectory_support_.translation()(2)
            <<endl;
      file[6]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<com_support_current_(0)<<"\t"<<com_support_current_(1)<<"\t"<<com_support_current_(2)
            <<"\t"<<pelv_support_current_.translation()(0)<<"\t"<<pelv_support_current_.translation()(1)<<"\t"<<pelv_support_current_.translation()(2)<<"\t"<<com_support_dot_current_(0)<<"\t"<<com_support_dot_current_(1)<<"\t"<<com_support_dot_current_(2)
           <<"\t"<<com_sim_current_(0)<<"\t"<<com_sim_current_(1)<<"\t"<<com_sim_current_(2)<<endl;
      file[7]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<rfoot_support_current_.translation()(0)<<"\t"<<rfoot_support_current_.translation()(1)<<"\t"<<rfoot_support_current_.translation()(2)
            <<"\t"<<lfoot_support_current_.translation()(0)<<"\t"<<lfoot_support_current_.translation()(1)<<"\t"<<lfoot_support_current_.translation()(2)<<endl;
      file[8]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<vars.x[0]<<"\t"<<vars.x[1]<<"\t"<<vars.x[2]<<"\t"<<vars.x[3]<<"\t"<<vars.x[4]<<"\t"<<vars.x[5]<<"\t"<<zmp_measured_(0)<<"\t"<<zmp_measured_(1)<<"\t"<<zmp_r_(0)<<"\t"<<zmp_r_(1)<<"\t"<<zmp_l_(0)<<"\t"<<zmp_l_(1)<<endl;
      file[9]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<r_ft_(0)<<"\t"<<r_ft_(1)<<"\t"<<r_ft_(2)<<"\t"<<r_ft_(3)<<"\t"<<r_ft_(4)<<"\t"<<r_ft_(5)<<"\t"<<l_ft_(0)<<"\t"<<l_ft_(1)<<"\t"<<l_ft_(2)<<"\t"<<l_ft_(3)<<"\t"<<l_ft_(4)<<"\t"<<l_ft_(5)<<endl;
      ///////////////////////////////////////////////

      updateNextStepTime();

    }
    else
    {
      desired_q_ = current_q_;
    }
  }
}

void WalkingController::setTarget(int walk_mode, bool hip_compensation, bool lqr, int ik_mode, bool heel_toe,
                                  bool is_right_foot_swing, double x, double y, double z, double height, double theta,
                                  double step_length_x, double step_length_y)
{
  target_x_ = x;
  target_y_ = y;
  target_z_ = z;
  com_height_ = height;
  target_theta_ = theta;
  step_length_x_ = step_length_x;
  step_length_y_ = step_length_y;
  ik_mode_ = ik_mode;
  walk_mode_ = walk_mode;
  hip_compensator_mode_ = hip_compensation; //uint32 compensator_mode[0] : HIP_COMPENSTOR    uint32 compensator_mode[1] : EXTERNAL_ENCODER
  lqr_compensator_mode_ = lqr;
  heel_toe_mode_ = heel_toe;
  is_right_foot_swing_ = is_right_foot_swing;

  parameterSetting();
}


void WalkingController::setEnable(bool enable)
{
  walking_enable_=enable;
  desired_q_ = current_q_;
}

void WalkingController::updateControlMask(unsigned int *mask)
{
  if(walking_enable_)
  {
    for (int i=0; i<total_dof_-18; i++) //control only leg
    {
      mask[i] = (mask[i] | PRIORITY);
    }
    mask[total_dof_-1] = (mask[total_dof_-1] & ~PRIORITY); //Gripper
    mask[total_dof_-2] = (mask[total_dof_-2] & ~PRIORITY); //Gripper
    mask[total_dof_-3] = (mask[total_dof_-3] & ~PRIORITY); //Head
    mask[total_dof_-4] = (mask[total_dof_-4] & ~PRIORITY); //Head
  }
  else
  {
    for (int i=0; i<total_dof_; i++)
    {
      mask[i] = (mask[i] & ~PRIORITY);
    }
  }
}

void WalkingController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
{
  for(unsigned int i=0; i<total_dof_; i++)
  {
    if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
    {
      desired_q(i) = desired_q_(i);
      WalkingController::target_x_ =1;
    }
  }
}

void WalkingController::parameterSetting()
{
  /*
  t_last_ = 0.1*hz_;
  t_start_= 0.1*hz_;
  t_temp_= 0.1*hz_; /
  t_rest_init_ = 0.1*hz_; /
  t_rest_last_= 0.1*hz_; /
  t_double1_= 0.1*hz_; /
  t_double2_= 0.1*hz_; /
  t_total_= 1.3*hz_; /
  t_temp_ = 3.0*hz_; /
  */

  t_double1_= 0.1*hz_;
  t_double2_= 0.1*hz_;
  t_rest_init_ = 2.0*hz_;
  t_rest_last_= 2.0*hz_;
  t_total_= 5.2*hz_;
  t_temp_ = 3.0*hz_;
  t_last_ = t_total_ + t_temp_;
  t_start_ = t_temp_+1;

  //initialize (should revise)


  t_start_real_ = t_start_ + t_rest_init_;

  current_step_num_ = 0;
  walking_tick_ = 0;
  walking_time_ = 0;

  foot_height_ = 0.05;
  //com_update_flag_ = true; // frome A to B1
  gyro_frame_flag_ = false;
  com_control_mode_ = true;

  //zc_ = 0.75;

  ///////////cvxgen setting////////////
  set_defaults();
  setup_indexing();
}

/**Foot step related fuctions
 */

void WalkingController::getRobotState()
{
  lfoot_float_current_ = model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)0);
  rfoot_float_current_ = model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)1);
  com_float_current_ = model_.getCurrentCom();
  com_sim_current_ = model_.getSimulationCom();
  r_ft_ = model_.getRightFootForce();
  l_ft_ = model_.getLeftFootForce();

  std::cout<<com_sim_current_<<endl;

  pelv_float_current_.setIdentity();

  Eigen::Isometry3d ref_frame;

  if(foot_step_(current_step_num_, 6) == 0)  //right foot support
  {
    ref_frame = rfoot_float_current_;
  }
  else if(foot_step_(current_step_num_, 6) == 1)
  {
    ref_frame = lfoot_float_current_;
  }

  const int com_size = com_float_old_.cols();

  for(int i =0; i<com_size-1; i++)
  {
    com_float_old_.col(com_size-1-i) = com_float_old_.col(com_size-2-i);
  }
  com_float_old_.col(0) = com_float_current_;


  lfoot_support_current_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_current_);
  rfoot_support_current_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_current_);
  pelv_support_current_ = DyrosMath::inverseIsometry3d(ref_frame);
  com_support_current_ = pelv_support_current_.linear()*com_float_current_ + pelv_support_current_.translation();
  com_sim_current_ = pelv_support_current_.linear()*com_sim_current_ + pelv_support_current_.translation();
  for(int i=0; i<com_size; i++)
  {
    com_support_old_.col(i) = pelv_support_current_.linear()*com_float_old_.col(i) + pelv_support_current_.translation();
  }


  current_leg_jacobian_l_=model_.getLegJacobian((DyrosJetModel::EndEffector) 0);
  current_leg_jacobian_r_=model_.getLegJacobian((DyrosJetModel::EndEffector) 1);

  thread_q_ = current_q_;
  current_motor_q_leg_ = current_q_.segment<12>(0);
  current_link_q_leg_ = current_q_ext_;
}


void WalkingController::calculateFootStepTotal()
{
  /***
   * this function calculate foot steps which the robot should put on
   * algorith: set robot orientation to the destination -> go straight -> set target orientation on the destination
   *
   * foot_step_(current_step_num_, i) is the foot step where the robot will step right after
   * foot_step_(crrennt_step_num_, 6) = 0 means swingfoot is left(support foot is right)
   */


  double initial_rot;
  double final_rot = 0.0;
  double initial_drot = 0.0;
  double final_drot = 0.0;

  initial_rot= atan2(target_y_, target_x_);

  if(initial_rot > 0.0)
    initial_drot = 10*DEG2RAD;
  else
    initial_drot = -10*DEG2RAD;

  unsigned int initial_total_step_number = initial_rot/initial_drot;
  double initial_residual_angle = initial_rot-initial_total_step_number*initial_drot;

  final_rot = target_theta_-initial_rot;
  if(final_rot > 0.0)
    final_drot = 10*DEG2RAD;
  else
    final_drot = -10*DEG2RAD;

  unsigned int final_total_step_number = final_rot/final_drot;
  double final_residual_angle = final_rot-final_total_step_number*final_drot;
  double length_to_target = sqrt(target_x_*target_x_+target_y_*target_y_);
  double dlength = step_length_x_; //footstep length;
  unsigned int middle_total_step_number = length_to_target/(dlength);
  double middle_residual_length = length_to_target-middle_total_step_number*(dlength);

  if(length_to_target == 0)
  {
    middle_total_step_number = 5; //walking on the spot 10 times
    dlength = 0;
  }


  unsigned int number_of_foot_step;


  int del_size;

  del_size = 1;
  number_of_foot_step = initial_total_step_number*del_size+middle_total_step_number *del_size+final_total_step_number*del_size;

  if(initial_total_step_number!=0 || abs(initial_residual_angle)>=0.0001)
  {
    if(initial_total_step_number%2==0)
      number_of_foot_step = number_of_foot_step+2*del_size;
    else
    {
      if(abs(initial_residual_angle)>= 0.0001)
        number_of_foot_step = number_of_foot_step+3*del_size;
      else
        number_of_foot_step = number_of_foot_step+del_size;
    }
  }

  if(middle_total_step_number!=0 || abs(middle_residual_length)>=0.0001)
  {
    if(middle_total_step_number%2==0)
      number_of_foot_step = number_of_foot_step+2*del_size;
    else
    {
      if(abs(middle_residual_length)>= 0.0001)
        number_of_foot_step = number_of_foot_step+3*del_size;
      else
        number_of_foot_step = number_of_foot_step+del_size;
    }
  }

  if(final_total_step_number!=0 || abs(final_residual_angle)>= 0.0001)
  {
    if(abs(final_residual_angle)>= 0.0001)
      number_of_foot_step = number_of_foot_step+2*del_size;
    else
      number_of_foot_step = number_of_foot_step+del_size;
  }



  foot_step_.resize(number_of_foot_step, 7);
  foot_step_.setZero();
  foot_step_support_frame_.resize(number_of_foot_step, 7);
  foot_step_support_frame_.setZero();
  foot_step_support_frame_offset_.resize(number_of_foot_step, 7);
  foot_step_support_frame_offset_.setZero();

  int index = 0;
  int temp, temp2, temp3, is_right;

  if(is_right_foot_swing_ == true)
    is_right = 1;
  else
    is_right = -1;


  temp = -is_right; //right foot will be first swingfoot
  temp2 = -is_right;
  temp3 = -is_right;


  if(initial_total_step_number!=0 || abs(initial_residual_angle)>=0.0001)
  {
    for (int i =0 ; i<initial_total_step_number; i++)
    {
      temp *= -1;
      foot_step_(index,0) = temp*0.127794*sin((i+1)*initial_drot);
      foot_step_(index,1) = -temp*0.127794*cos((i+1)*initial_drot);
      foot_step_(index,5) = (i+1)*initial_drot;
      foot_step_(index,6) = 0.5+0.5*temp;
      index++;
    }

    if(temp==is_right)
    {
      if(abs(initial_residual_angle) >= 0.0001)
      {
        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
        foot_step_(index,6) = 0.5+0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
        foot_step_(index,6) = 0.5+0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
        foot_step_(index,6) = 0.5+0.5*temp;
        index++;

      }
      else
      {
        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
        foot_step_(index,6) = 0.5+0.5*temp;
        index++;
      }
    }
    else if(temp==-is_right)
    {
      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
      foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
      foot_step_(index,6) = 0.5+0.5*temp;
      index++;

      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
      foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
      foot_step_(index,6) = 0.5+0.5*temp;
      index++;
    }
  }



  if(middle_total_step_number!=0 || abs(middle_residual_length)>=0.0001)
  {
    for (int i =0 ; i<middle_total_step_number; i++)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(i+1))+temp2*sin(initial_rot)*(0.127794);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(i+1))-temp2*cos(initial_rot)*(0.127794);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5+0.5*temp2;
      index++;
    }

    if(temp2==is_right)
    {
      if(abs(middle_residual_length) >= 0.0001)
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5+0.5*temp2;
        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5+0.5*temp2;
        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5+0.5*temp2;
        index++;
      }
      else
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5+0.5*temp2;
        index++;
      }
    }
    else if(temp2==-is_right)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5+0.5*temp2;
      index++;

      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5+0.5*temp2;
      index++;
    }
  }




  double final_position_x = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length);
  double final_position_y = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length);


  if(final_total_step_number!=0 || abs(final_residual_angle)>= 0.0001)
  {
    for (int i =0 ; i<final_total_step_number; i++)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x+temp3*0.127794*sin((i+1)*final_drot+initial_rot);
      foot_step_(index,1) = final_position_y-temp3*0.127794*cos((i+1)*final_drot+initial_rot);
      foot_step_(index,5) = (i+1)*final_drot+initial_rot;
      foot_step_(index,6) = 0.5+0.5*temp3;
      index++;
    }

    if(abs(final_residual_angle) >= 0.0001)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x+temp3*0.127794*sin(target_theta_);
      foot_step_(index,1) = final_position_y-temp3*0.127794*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5+0.5*temp3;
      index++;

      temp3 *= -1;

      foot_step_(index,0) = final_position_x+temp3*0.127794*sin(target_theta_);
      foot_step_(index,1) = final_position_y-temp3*0.127794*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5+0.5*temp3;
      index++;
    }
    else
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x+temp3*0.127794*sin(target_theta_);
      foot_step_(index,1) = final_position_y-temp3*0.127794*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5+0.5*temp3;
      index++;
    }
  }
  //foot_step_(0,1) = +0.127794;
  //foot_step_(0,6) = 0;
  //
  //foot_step_(1,1) = -0.127794;
  //foot_step_(1,6) = 1;
  //
  //foot_step_(2,1) = +0.127794;
  //foot_step_(2,6) = 0;
  //
  //foot_step_(3,1) = -0.127794;
  //foot_step_(3,6) = 1;
  //
  //foot_step_(4,1) = +0.127794;
  //foot_step_(4,6) = 0;
}

void WalkingController::calculateFootStepSeparate()
{
  /***
   * this function calculate foot steps which the robot should put on
   * algorith: go straight to X direction -> side walk to Y direction -> set target orientation on the destination
   *
   * foot_step_(current_step_num_, i) is the foot step where the robot will step right after
   * foot_step_(crrennt_step_num_, 6) = 0 means swingfoot is left(support foot is right)
   */

  double x = target_x_;
  double y = target_y_;
  double alpha = target_theta_;

  const double dx = step_length_x_;
  const double dy = step_length_y_;
  const double dtheta = 10.0*DEG2RAD;
  if(x<0.0)
    const double dx = -step_length_x_;
  if(y<0.0)
    const double dy = -step_length_y_;
  if(alpha<0.0)
    const double dtheta = -10.0*DEG2RAD;

  int x_number = x/dx;
  int y_number = y/dy;
  int theta_number = alpha/dtheta;

  double x_residual = x-x_number*dx;
  double y_residual = y-y_number*dy;
  double theta_residual = alpha-theta_number*dtheta;
  unsigned int number_of_foot_step = 0;
  int temp = -1;

  if(x_number!=0 || abs(x_residual)>=0.001)
  {
    temp *= -1;
    number_of_foot_step += 1;

    for(int i=0;i<x_number;i++)
    {
      temp *= -1;
    }
    number_of_foot_step += x_number;

    if(abs(x_residual)>=0.001)
    {
      temp *= -1;
      temp *= -1;
      number_of_foot_step += 2;
    }
    else
    {
      temp *= -1;
      number_of_foot_step += 1;
    }
  }

  if(y_number!=0 || abs(y_residual)>=0.001)
  {
    if(x==0)
    {
      if(y>=0)
        temp = -1;
      else
        temp = 1;
      temp *= -1;
      number_of_foot_step += 1;
    }

    if(y>=0 && temp==-1)
    {
      number_of_foot_step += 1;
    }
    else if(y<0 && temp==1)
    {
      number_of_foot_step += 1;
    }

    number_of_foot_step += 2*y_number;

    if(abs(y_residual)>=0.001)
    {
      number_of_foot_step += 2;
    }
  }

  if(theta_number!=0 || abs(theta_residual)>= 0.02)
  {
    number_of_foot_step += theta_number;

    if(abs(theta_residual) >= 0.02)
    {
      number_of_foot_step += 2;
    }
    else
    {
      number_of_foot_step += 1;
    }
  }


  foot_step_.resize(number_of_foot_step, 7);
  foot_step_.setZero();
  foot_step_support_frame_.resize(number_of_foot_step, 7);
  foot_step_support_frame_.setZero();
  foot_step_support_frame_offset_.resize(number_of_foot_step, 7);
  foot_step_support_frame_offset_.setZero();

  //int temp = -1;
  temp = 1; //fisrt support step is left foot
  int index = 0;

  if(x_number!=0 || abs(x_residual)>=0.001)
  {
    temp *= -1;

    foot_step_(index,0) = 0;
    foot_step_(index,1) = -temp*0.127794;
    foot_step_(index,6) = 0.5+temp*0.5;
    index++;

    for(int i=0;i<x_number;i++)
    {
      temp *= -1;

      foot_step_(index,0) = (i+1)*dx;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }

    if(abs(x_residual)>=0.001)
    {
      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;

      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }
    else
    {
      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }
  }

  if(y_number!=0 || abs(y_residual)>=0.001)
  {
    if(x==0)
    {
      if(y>=0)
        temp = -1;
      else
        temp = 1;

      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }

    if(y>=0 && temp==-1)
    {
      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }
    else if(y<0 && temp==1)
    {
      temp *= -1;


      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }

    for(int i=0;i<y_number;i++)
    {
      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794+(i+1)*dy;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;

      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794+(i+1)*dy;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }

    if(abs(y_residual)>=0.001)
    {
      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794+y;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;

      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794+y;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }
  }


  if(theta_number!=0 || abs(theta_residual)>= 0.02)
  {
    for (int i =0 ; i<theta_number; i++)
    {
      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin((i+1)*dtheta)+x;
      foot_step_(index,1) = -temp*0.127794*cos((i+1)*dtheta)+y;
      foot_step_(index,5) = (i+1)*dtheta;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }

    if(abs(theta_residual) >= 0.02)
    {
      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin(alpha)+x;
      foot_step_(index,1) = -temp*0.127794*cos(alpha)+y;
      foot_step_(index,5) = alpha;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;

      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin(alpha)+x;
      foot_step_(index,1) = -temp*0.127794*cos(alpha)+y;
      foot_step_(index,5) = alpha;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }
    else
    {
      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin(alpha)+x;
      foot_step_(index,1) = -temp*0.127794*cos(alpha)+y;
      foot_step_(index,5) = alpha;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }
  }





}

void WalkingController::getZmpTrajectory()
{

  unsigned int planning_step_number  = 3;

  unsigned int norm_size = 0;

  if(current_step_num_ >= total_step_num_ - planning_step_number)
    norm_size = (t_last_-t_start_+1)*(total_step_num_-current_step_num_)+20*hz_;
  else
    norm_size = (t_last_-t_start_+1)*(planning_step_number);
  if(current_step_num_ == 0)
    norm_size = norm_size + t_temp_+1;
  addZmpOffset();
  zmpGenerator(norm_size, planning_step_number);
}

void WalkingController::floatToSupportFootstep()
{
  Eigen::Isometry3d reference;
  //foot_step_support_frame_.resize(total_step_num_, 6);

  if(current_step_num_ == 0)
  {
    if(foot_step_(0,6) == 0) //right support
    {
      reference.translation() = rfoot_float_init_.translation();
      reference.translation()(2) = 0.0;
      reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rfoot_float_init_.linear())(2));
      reference.translation()(0) = 0.0;
    }
    else  //left support
    {
      reference.translation() = lfoot_float_init_.translation();
      reference.translation()(2) = 0.0;
      reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(lfoot_float_init_.linear())(2));
      reference.translation()(0) = 0.0;
    }
  }
  else
  {
    reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_-1,5));
    for(int i=0 ;i<3; i++)
      reference.translation()(i) = foot_step_(current_step_num_-1,i);
  }

  Eigen::Vector3d temp_local_position;
  Eigen::Vector3d temp_global_position;

  if(current_step_num_ == 0)
  {
    for(int i=0; i<total_step_num_; i++)
    {
      for(int j=0; j<3; j++)
        temp_global_position(j)  = foot_step_(i,j);

      temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

      for(int j=0; j<3; j++)
        foot_step_support_frame_(i,j) = temp_local_position(j);

      foot_step_support_frame_(i,3) = foot_step_(i,3);
      foot_step_support_frame_(i,4) = foot_step_(i,4);
      foot_step_support_frame_(i,5) = foot_step_(i,5) - supportfoot_float_init_(5);

    }
  }
  else
  {
    for(int i=0; i<total_step_num_; i++)
    {
      for(int j=0; j<3; j++)
        temp_global_position(j)  = foot_step_(i,j);

      temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

      for(int j=0; j<3; j++)
        foot_step_support_frame_(i,j) = temp_local_position(j);

      foot_step_support_frame_(i,3) = foot_step_(i,3);
      foot_step_support_frame_(i,4) = foot_step_(i,4);
      foot_step_support_frame_(i,5) = foot_step_(i,5) - foot_step_(current_step_num_-1,5);

    }
  }

  for(int j=0;j<3;j++)
    temp_global_position(j) = swingfoot_float_init_(j);

  temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

  for(int j=0;j<3;j++)
    swingfoot_support_init_(j) = temp_local_position(j);

  swingfoot_support_init_(3) = swingfoot_float_init_(3);
  swingfoot_support_init_(4) = swingfoot_float_init_(4);

  if(current_step_num_ == 0)
    swingfoot_support_init_(5) = swingfoot_float_init_(5) - supportfoot_float_init_(5);
  else
    swingfoot_support_init_(5) = swingfoot_float_init_(5) - foot_step_(current_step_num_-1,5);



  for(int j=0;j<3;j++)
    temp_global_position(j) = supportfoot_float_init_(j);

  temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

  for(int j=0;j<3;j++)
    supportfoot_support_init_(j) = temp_local_position(j);

  supportfoot_support_init_(3) = supportfoot_float_init_(3);
  supportfoot_support_init_(4) = supportfoot_float_init_(4);

  if(current_step_num_ == 0)
    supportfoot_support_init_(5) = 0;
  else
    supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_(current_step_num_-1,5);

}

void WalkingController::updateInitialState()
{  
  if( walking_tick_ ==0)
  {
    thread_tick_ = 0;

    calculateFootStepTotal();

    q_init_ = current_q_;
    lfoot_float_init_ = model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)(0));
    rfoot_float_init_ = model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)(1));
    com_float_init_ = model_.getCurrentCom();


    pelv_float_init_.setIdentity();

    Eigen::Isometry3d ref_frame;

    if(foot_step_(0, 6) == 0)  //right foot support
    {
      ref_frame = rfoot_float_init_;
    }
    else if(foot_step_(0, 6) == 1)
    {
      ref_frame = lfoot_float_init_;
    }

    lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
    rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
    pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame);
    com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation();

    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

    supportfoot_float_init_.setZero();
    swingfoot_float_init_.setZero();


    if(foot_step_(0,6) == 1)  //left suppport foot
    {
      for(int i=0; i<2; i++)
        supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

      for(int i=0; i<2; i++)
        swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

      supportfoot_float_init_(0) = 0.0;
      swingfoot_float_init_(0) = 0.0;
    }
    else
    {
      for(int i=0; i<2; i++)
        supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

      for(int i=0; i<2; i++)
        swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

      supportfoot_float_init_(0) = 0.0;
      swingfoot_float_init_(0) = 0.0;
    }

    zc_ = com_support_init_(2);
    pelv_suppprt_start_ = pelv_support_init_;


    total_step_num_ = foot_step_.col(1).size();

    ///preview control variable///
    if(com_control_mode_ == true)
    {
      xi_ = com_support_init_(0);
      yi_ = com_support_init_(1);
    }
    else
    {
      xi_ = pelv_support_init_.translation()(0)+com_offset_(0);
      yi_ = pelv_support_init_.translation()(1)+com_offset_(1);
    }

  }
  else if(current_step_num_!=0 && walking_tick_ == t_start_)
  {
    //model_.updateKinematics(current_q_);
    q_init_ = current_q_;
    lfoot_float_init_ = model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)(0));
    rfoot_float_init_ = model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)(1));
    com_float_init_ = model_.getCurrentCom();


    pelv_float_init_.setIdentity();

    Eigen::Isometry3d ref_frame;

    if(foot_step_(current_step_num_, 6) == 0)  //right foot support
    {
      ref_frame = rfoot_float_init_;
    }
    else if(foot_step_(current_step_num_, 6) == 1)
    {
      ref_frame = lfoot_float_init_;
    }

    lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
    rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
    pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame);
    com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation();

    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

    //supportfoot_float_init_.setZero();
    //swingfoot_float_init_.setZero();

    //if(foot_step_(0,6) == 1)  //left suppport foot
    //{
    //  for(int i=0; i<2; i++)
    //    supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
    //  for(int i=0; i<3; i++)
    //    supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);
    //
    //  for(int i=0; i<2; i++)
    //    swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
    //  for(int i=0; i<3; i++)
    //    swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);
    //
    //  //supportfoot_float_init_(0) = 0.0;
    //  //swingfoot_float_init_(0) = 0.0;
    //}
    //else
    //{
    //  for(int i=0; i<2; i++)
    //    supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
    //  for(int i=0; i<3; i++)
    //    supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);
    //
    //  for(int i=0; i<2; i++)
    //    swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
    //  for(int i=0; i<3; i++)
    //    swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);
    //
    //  //supportfoot_float_init_(0) = 0.0;
    //  //swingfoot_float_init_(0) = 0.0;
    //}


  }
}

void WalkingController::updateNextStepTime()
{
  if(walking_tick_ == t_last_)
  {
    if(current_step_num_ != total_step_num_-1)
    {
      t_start_ = t_last_ +1;
      t_start_real_ = t_start_ + t_rest_init_;
      t_last_ = t_start_ + t_total_ -1;

      current_step_num_ ++;
    }
  }

  if(current_step_num_ == total_step_num_-1 && walking_tick_ >= t_last_ +5.0*hz_)
  {
    walking_enable_ = false;
  }

  /*
  if(walking_tick_ >= 30*hz_)
  {
     walking_enable_ = false;
  }
  */
  walking_tick_ ++;

}

void WalkingController::addZmpOffset()
{
  lfoot_zmp_offset_ = -0.02;
  rfoot_zmp_offset_ = 0.02;

  foot_step_support_frame_offset_ = foot_step_support_frame_;

  if(foot_step_(0,6) == 0) //right support foot
  {
    //  supportfoot_support_init_offset_(0) = supportfoot_support_init_(0)/2;
    supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
    swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + lfoot_zmp_offset_;
  }
  else
  {
    //  supportfoot_support_init_offset_(0) = supportfoot_support_init_(0);
    supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_;
    swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + rfoot_zmp_offset_;
  }

  for(int i=0; i<total_step_num_; i++)
  {
    if(foot_step_(i,6) == 0)//right support, left swing
    {
      foot_step_support_frame_offset_(i,1) += lfoot_zmp_offset_;
    }
    else
    {
      foot_step_support_frame_offset_(i,1) += rfoot_zmp_offset_;
    }
  }
}

void WalkingController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{

  ref_zmp_.resize(norm_size, 2);
  com_offset_.setZero();

  Eigen::VectorXd temp_px;
  Eigen::VectorXd temp_py;

  unsigned int index =0;

  if(current_step_num_ ==0)
  {
    for (int i=0; i<= t_temp_; i++) //200 tick
    {
      if(i <= 0.5*hz_)
      {
        ref_zmp_(i,0) = com_support_init_(0)+com_offset_(0);
        ref_zmp_(i,1) = com_support_init_(1)+com_offset_(1);
      }
      else if(i < 1.5*hz_)
      {
        double del_x = i-0.5*hz_;
        ref_zmp_(i,0) = com_support_init_(0)+com_offset_(0)-del_x*(com_support_init_(0)+com_offset_(0))/(1.0*hz_);
        ref_zmp_(i,1) = com_support_init_(1)+com_offset_(1);
      }
      else
      {
        ref_zmp_(i,0) = 0.0;
        ref_zmp_(i,1) = com_support_init_(1)+com_offset_(1);
      }

      index++;
    }
  }

  if(current_step_num_ >= total_step_num_-planning_step_num)
  {
    for(unsigned int i = current_step_num_; i<total_step_num_ ; i++)
    {
      onestepZmp(i,temp_px,temp_py);

      for (unsigned int j=0; j<t_total_; j++)
      {
        ref_zmp_(index+j,0) = temp_px(j);
        ref_zmp_(index+j,1) = temp_py(j);
      }
      index = index+t_total_;
    }

    for (unsigned int j=0; j<20*hz_; j++)
    {
      ref_zmp_(index+j,0) = ref_zmp_(index-1,0);
      ref_zmp_(index+j,1) = ref_zmp_(index-1,1);
    }
    index = index+20*hz_;
  }
  else
  {
    for(unsigned int i=current_step_num_; i < current_step_num_+planning_step_num; i++)
    {
      onestepZmp(i,temp_px,temp_py);
      for (unsigned int j=0; j<t_total_; j++)
      {
        ref_zmp_(index+j,0) = temp_px(j);
        ref_zmp_(index+j,1) = temp_py(j);
      }
      index = index+t_total_;
    }
  }

}

void WalkingController::onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py)
{
  temp_px.resize(t_total_);
  temp_py.resize(t_total_);
  temp_px.setZero();
  temp_py.setZero();

  double Kx = 0.0;
  double Kx2 = 0.0;
  double Ky = 0.0;
  double Ky2 = 0.0;

  if(current_step_number == 0)
  {
    Kx = supportfoot_support_init_offset_(0);
    Kx2 = (foot_step_support_frame_(current_step_number,0)- supportfoot_support_init_offset_(0))/2.0;
    //    Kx2 = (foot_step_support_frame_(current_step_number,0))/2.0;


    Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
    Ky2 = (foot_step_support_frame_(current_step_number,1)- supportfoot_support_init_offset_(1))/2.0;
    //    Ky2 = (foot_step_support_frame_(current_step_number,1))/2.0;

    for(int i=0; i<t_total_; i++)
    {
      if(i < t_rest_init_)
      {
        temp_px(i) = 0.0;
        temp_py(i) = com_support_init_(1)+com_offset_(1);
      }
      else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
      {
        temp_px(i) = Kx/t_double1_*(i+1-t_rest_init_);
        temp_py(i) = com_support_init_(1)+com_offset_(1) + Ky/t_double1_*(i+1-t_rest_init_);
      }
      else if(i>= t_rest_init_+t_double1_ && i< t_total_-t_rest_last_-t_double2_)
      {
        temp_px(i) = supportfoot_support_init_offset_(0);
        temp_py(i) = supportfoot_support_init_offset_(1);
      }
      else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
      {
        temp_px(i) = supportfoot_support_init_offset_(0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
        temp_py(i) = supportfoot_support_init_offset_(1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
      }
      else
      {
        temp_px(i) = temp_px(i-1);
        temp_py(i) = temp_py(i-1);
      }
    }
  }
  else if(current_step_number == 1)
  {
    Kx = foot_step_support_frame_offset_(current_step_number-1,0) - (foot_step_support_frame_(current_step_number-1,0) + supportfoot_support_init_(0))/2.0;
    Kx2 = (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0 - foot_step_support_frame_offset_(current_step_number-1,0);

    Ky =  foot_step_support_frame_offset_(current_step_number-1,1) - (foot_step_support_frame_(current_step_number-1,1) + supportfoot_support_init_(1))/2.0;
    Ky2 = (foot_step_support_frame_(current_step_number,1)+foot_step_support_frame_(current_step_number-1,1))/2.0 - foot_step_support_frame_offset_(current_step_number-1,1);
    for(int i=0; i<t_total_; i++)
    {
      if(i < t_rest_init_)
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+supportfoot_support_init_(0))/2.0;
        temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+supportfoot_support_init_(1))/2.0;

      }
      else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+supportfoot_support_init_(0))/2.0 + Kx/t_double1_*(i+1-t_rest_init_);
        temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+supportfoot_support_init_(1))/2.0 + Ky/t_double1_*(i+1-t_rest_init_);
      }
      else if(i>= t_rest_init_+t_double1_ & i< t_total_-t_rest_last_-t_double2_)
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0);
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1);
      }
      else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
      }
      else
      {
        temp_px(i) = temp_px(i-1);
        temp_py(i) = temp_py(i-1);
      }
    }
  }
  else
  {
    Kx = foot_step_support_frame_offset_(current_step_number-1,0) - (foot_step_support_frame_(current_step_number-1,0) + foot_step_support_frame_(current_step_number-2,0))/2.0;
    Kx2 = (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0 - foot_step_support_frame_offset_(current_step_number-1,0);

    Ky =  foot_step_support_frame_offset_(current_step_number-1,1) - (foot_step_support_frame_(current_step_number-1,1) + foot_step_support_frame_(current_step_number-2,1))/2.0;
    Ky2 = (foot_step_support_frame_(current_step_number,1)+foot_step_support_frame_(current_step_number-1,1))/2.0 -  foot_step_support_frame_offset_(current_step_number-1,1);

    for(int i=0; i<t_total_; i++)
    {
      if(i < t_rest_init_)
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0;
        temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+foot_step_support_frame_(current_step_number-2,1))/2.0;
      }
      else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0 + Kx/t_double1_*(i+1-t_rest_init_);
        temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+foot_step_support_frame_(current_step_number-2,1))/2.0 + Ky/t_double1_*(i+1-t_rest_init_);
      }
      else if(i>= t_rest_init_+t_double1_ & i< t_total_-t_rest_last_-t_double2_)
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0);
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1);
      }
      else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
      }
      else
      {
        temp_px(i) = temp_px(i-1);
        temp_py(i) = temp_py(i-1);
      }
    }
  }
}

void WalkingController::getComTrajectory()
{

  modifiedPreviewControl();

  xs_ = xd_;
  ys_ = yd_;

  if (walking_tick_ == t_start_+t_total_-1 && current_step_num_ != total_step_num_-1)
  {
    Eigen::Vector3d com_pos_prev;
    Eigen::Vector3d com_pos;
    Eigen::Vector3d com_vel_prev;
    Eigen::Vector3d com_vel;
    Eigen::Vector3d com_acc_prev;
    Eigen::Vector3d com_acc;

    Eigen::Matrix3d temp_rot;
    Eigen::Vector3d temp_pos;

    temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_,5));
    for(int i=0; i<3; i++)
      temp_pos(i) = foot_step_support_frame_(current_step_num_,i);

    com_pos_prev(0) = xs_(0);
    com_pos_prev(1) = ys_(0);
    com_pos = temp_rot*(com_pos_prev - temp_pos);

    com_vel_prev(0) = xs_(1);
    com_vel_prev(1) = ys_(1);
    com_vel_prev(2) = 0.0;
    com_vel = temp_rot*com_vel_prev;

    com_acc_prev(0) = xs_(2);
    com_acc_prev(1) = ys_(2);
    com_acc_prev(2) = 0.0;
    com_acc = temp_rot*com_acc_prev;

    xs_(0) = com_pos(0);
    ys_(0) = com_pos(1);
    xs_(1) = com_vel(0);
    ys_(1) = com_vel(1);
    xs_(2) = com_acc(0);
    ys_(2) = com_acc(1);
  }


  double start_time;

  if(current_step_num_ == 0)
    start_time = 0;
  else
    start_time = t_start_;

  zmp_desired_(0) = ref_zmp_(walking_tick_-start_time,0);
  zmp_desired_(1) = ref_zmp_(walking_tick_-start_time,1);

  if(com_control_mode_ == true)
  {
    com_desired_(0) = xd_(0);
    com_desired_(1) = yd_(0);
    com_desired_(2) = DyrosMath::cubic(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0);

    com_dot_desired_(0) = xd_(1);
    com_dot_desired_(1) = yd_(1);
    com_dot_desired_(2) = DyrosMath::cubicDot(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0, hz_);

    double k= 100.0;
    p_ref_(0) = xd_(1)+k*(xd_(0)-com_support_current_(0));
    p_ref_(1) = yd_(1)+k*(yd_(0)-com_support_current_(1));
    p_ref_(2) = k*(com_desired_(2)-com_support_current_(2));
    l_ref_.setZero();
  }
  else
  {
    com_desired_(0) = xd_(0);
    com_desired_(1) = yd_(0);
    com_desired_(2) = DyrosMath::cubic(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0);

    com_dot_desired_(0) = xd_(1);
    com_dot_desired_(1) = yd_(1);
    com_dot_desired_(2) = DyrosMath::cubicDot(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0, hz_);

    double k= 100.0;
    p_ref_(0) = xd_(1)+k*(xd_(0)-com_support_current_(0));
    p_ref_(1) = yd_(1)+k*(yd_(0)-com_support_current_(1));
    p_ref_(2) = k*(com_desired_(2)-com_support_current_(2));
    l_ref_.setZero();
  }
}

void WalkingController::getPelvTrajectory()
{
  double z_rot = foot_step_support_frame_(current_step_num_,5);

  //Trunk Position
  if(com_control_mode_ == true)
  {
    double kp = 0.9;

    // kp = Cubic(abs(_COM_desired(0)-_COM_real_support(0)),0.0,0.05,1.0,0.0,3.0,0.0);
    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + kp*(com_desired_(0) - com_support_current_(0));
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + kp*(com_desired_(1) - com_support_current_(1));

    //pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + kp*(com_desired_(0) - vars.x[2]);
    //pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + kp*(com_desired_(1) - vars.x[3]);
    pelv_trajectory_support_.translation()(2) = com_desired_(2); //_T_Trunk_support.translation()(2) + kp*(_COM_desired(2) - _COM_real_support(2));
  }
  else
  {
    double kp = 3.0;
    double d = 0.5;

    if(walking_tick_ >= t_start_ && walking_tick_ < t_start_+0.3*hz_)
    {
      kp = 0+ 3.0*(walking_tick_-t_start_)/(0.3*hz_);
      d = 0+ 0.5*(walking_tick_-t_start_)/(0.3*hz_);
    }

    //if(walking_tick_ ==0)
    // COM_pd = 0.0;
    //Trunk_trajectory.translation()(0) = _T_Trunk_support.translation()(0)+kp*(_COM_desired(0) - _COM_real_support(0) + 0.06) + d*xd_(1)- d*(_COM_real_support(0)-COM_prev(0))/Hz  ;


    double offset_x = 0.0;
    if(foot_step_(current_step_num_,6) == 1) //right foot swing(left foot support)
    {
      double temp_time = 0.1*hz_;
      if(walking_tick_ < t_start_real_)
        offset_x = DyrosMath::cubic(walking_tick_, t_start_+temp_time,t_start_real_-temp_time,0.0,0.02,0.0,0.0);
      else
        offset_x = DyrosMath::cubic(walking_tick_, t_start_+t_total_-t_rest_last_+temp_time,t_start_+t_total_-temp_time,0.02,0.0,0.0,0.0);
    }

    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 1.0*(com_desired_(0)-com_support_current_(0));
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 1.0*(com_desired_(1)-com_support_current_(1));
    pelv_trajectory_support_.translation()(2) = com_desired_(2);

    double dt = 1.0/hz_;
    kp = 100.0;
    d = 2000.0;
  }

  //Trunk orientation
  Eigen::Vector3d Trunk_trajectory_euler;

  if(walking_tick_ < t_start_real_+t_double1_)
  {
    for(int i=0; i<2; i++)
      Trunk_trajectory_euler(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_+t_double1_,pelv_support_euler_init_(i),0.0,0.0,0.0);;
    Trunk_trajectory_euler(2) = pelv_support_euler_init_(2);
  }
  else if(walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_)
  {
    for(int i=0; i<2; i++)
      Trunk_trajectory_euler(i) = 0.0;

    if(foot_step_(current_step_num_,6) == 2)
      Trunk_trajectory_euler(2) = pelv_support_euler_init_(2);
    else
      Trunk_trajectory_euler(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_double2_-t_rest_last_, pelv_support_euler_init_(2),z_rot/2.0,0.0,0.0);
  }
  else
  {
    for(int i=0; i<2; i++)
      Trunk_trajectory_euler(i) = 0.0;

    if(foot_step_(current_step_num_,6) == 2)
      Trunk_trajectory_euler(2) = pelv_support_euler_init_(2);
    else
      Trunk_trajectory_euler(2) = z_rot/2.0;
  }

  pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2))*DyrosMath::rotateWithY(Trunk_trajectory_euler(1))*DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
}

void WalkingController::getFootTrajectory()
{
  Eigen::Vector6d target_swing_foot;

  for(int i=0; i<6; i++)
    target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i);


  if(walking_tick_ < t_start_real_+t_double1_)
  {
    lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
    lfoot_trajectory_dot_support_.setZero();


    if(foot_step_(current_step_num_,6) == 1) //left foot support
      lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_init_.translation()(2),0.0,0.0,0.0);
    else // swing foot (right foot support)
    {
      if(current_step_num_ == 0)
        lfoot_trajectory_support_.translation()(2) = lfoot_support_init_.translation()(2);
      else
      {
        if(walking_tick_ < t_start_)
          lfoot_trajectory_support_.translation()(2) = lfoot_support_init_.translation()(2);
        else if(walking_tick_ >= t_start_ && walking_tick_ < t_start_real_)
          lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_init_.translation()(2),0.0,0.0,0.0);
        else
          lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_,t_start_real_+t_double1_,0.0,0.0,0.0,0.0);
      }
    }


    lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;

    for(int i=0; i<2; i++)
      lfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_euler_init_(i),0.0,0.0,0.0);

    lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));


    rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
    rfoot_trajectory_dot_support_.setZero();
    //rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,_T_Start_real,rfoot_trajectory_init_.translation()(2),0.0,0.0,0.0);


    if(foot_step_(current_step_num_,6) == 0) //right foot support
      rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_support_init_.translation()(2),0.0,0.0,0.0);
    else // swing foot (left foot support)
    {
      if(current_step_num_ == 0)
        rfoot_trajectory_support_.translation()(2) = rfoot_support_init_.translation()(2);
      else
      {
        if(walking_tick_ < t_start_)
          rfoot_trajectory_support_.translation()(2) = rfoot_support_init_.translation()(2);
        else if(walking_tick_ >= t_start_ && walking_tick_ < t_start_real_)
          rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_support_init_.translation()(2),0.0,0.0,0.0);
        else
          rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_,t_start_real_+t_double1_,0.0,0.0,0.0,0.0);
      }
    }


    rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;

    for(int i=0; i<2; i++)
      rfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_support_euler_init_(i),0.0,0.0,0.0);

    rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

  }
  else if(walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_)
  {
    double t_rest_temp = 0.1*hz_;
    double ankle_temp;
    ankle_temp = 0*DEG2RAD;

    if(foot_step_(current_step_num_,6) == 1) //Left foot support : Left foot is fixed at initial values, and Right foot is set to go target position
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
      lfoot_trajectory_euler_support_.setZero();

      lfoot_trajectory_dot_support_.setZero();
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

      // setting for Left supporting foot

      if(walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0) // the period for lifting the right foot
      {
        rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0);
        rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0,hz_);

        rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0);
        rfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0,hz_);
      } // the period for lifting the right foot
      else
      {
        rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0);
        rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0,hz_);
        
        rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0);
        rfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0,hz_);
      } // the period for putting the right foot

      rfoot_trajectory_euler_support_(0) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,target_swing_foot(0+3),0.0,0.0);
      rfoot_trajectory_dot_support_(0+3) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,target_swing_foot(0+3),0.0,0.0,hz_);

      for(int i=0; i<2; i++)
      {
        rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
        rfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0,hz_);
      }

      rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      rfoot_trajectory_dot_support_(5) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0,hz_);

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if(foot_step_(current_step_num_,6) == 0) // Right foot support : Right foot is fixed at initial values, and Left foot is set to go target position
    {
      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.translation()(2) = 0.0;
      //rfoot_trajectory_support_.linear() = rfoot_trajectory_init_.linear();
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
      rfoot_trajectory_euler_support_(0) = 0.0;
      rfoot_trajectory_euler_support_(1) = 0.0;
      rfoot_trajectory_dot_support_.setZero();

      double ankle_temp;
      ankle_temp = 0*DEG2RAD;
      //ankle_temp = -15*DEG2RAD;

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

      if(walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0)
      {

        lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0);
        lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0,hz_);

        lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0);
        lfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0,hz_);

      }
      else
      {
        lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0);
        lfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0,hz_);


        lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0);
        lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0,hz_);
      }

      lfoot_trajectory_euler_support_(0) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,target_swing_foot(0+3),0.0,0.0);
      lfoot_trajectory_dot_support_(0+3) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,target_swing_foot(0+3),0.0,0.0,hz_);

      for(int i=0; i<2; i++)
      {
        lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
        lfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0,hz_);
      }

      //  for(int i=0; i<3; i++)
      //  {
      //      lfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,_T_Start+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,0.0,target_swing_foot(i+3),0.0);
      //      lfoot_trajectory_dot_support_(i+3) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,_T_Start+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,0.0,target_swing_foot(i+3),0.0,hz_);
      //  }


      lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      lfoot_trajectory_dot_support_(5) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0,hz_);

      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    }
    else
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_support_.linear() = lfoot_support_init_.linear();
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
      lfoot_trajectory_dot_support_.setZero();

      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.linear() = rfoot_support_init_.linear();
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
      rfoot_trajectory_dot_support_.setZero();
    }
  }
  else
  {
    if(foot_step_(current_step_num_,6) == 1)
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_support_.translation()(2) = 0.0;
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
      lfoot_trajectory_euler_support_(0) = 0.0;
      lfoot_trajectory_euler_support_(1) = 0.0;
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
      lfoot_trajectory_dot_support_.setZero();

      for(int i=0; i<3; i++)
      {
        rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
        rfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
      rfoot_trajectory_dot_support_.setZero();

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if (foot_step_(current_step_num_,6) == 0)
    {
      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.translation()(2) = 0.0;
      //rfoot_trajectory_support_.linear() = rfoot_trajectory_init_.linear();
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
      rfoot_trajectory_euler_support_(0) = 0.0;
      rfoot_trajectory_euler_support_(1) = 0.0;
      rfoot_trajectory_dot_support_.setZero();

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));


      for(int i=0; i<3; i++)
      {
        lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
        lfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
      lfoot_trajectory_dot_support_.setZero();
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    }
    else
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_support_.linear() = lfoot_support_init_.linear();
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
      lfoot_trajectory_dot_support_.setZero();

      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.linear() = rfoot_support_init_.linear();
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
      rfoot_trajectory_dot_support_.setZero();
    }
  }
}

void WalkingController::supportToFloatPattern()
{
  if(gyro_frame_flag_ == true)
  {
    Eigen::Isometry3d reference = pelv_trajectory_float_;
    DyrosMath::floatGyroframe(pelv_trajectory_support_,reference,pelv_trajectory_float_);
    DyrosMath::floatGyroframe(lfoot_trajectory_support_,reference,lfoot_trajectory_float_);
    DyrosMath::floatGyroframe(rfoot_trajectory_support_,reference,rfoot_trajectory_float_);
    lfoot_trajectory_euler_float_ = DyrosMath::rot2Euler(lfoot_trajectory_float_.linear());
    rfoot_trajectory_euler_float_ = DyrosMath::rot2Euler(rfoot_trajectory_float_.linear());
  }
  else
  {
    pelv_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*pelv_trajectory_support_;
    lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*lfoot_trajectory_support_;
    rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*rfoot_trajectory_support_;
    lfoot_trajectory_euler_float_ = DyrosMath::rot2Euler(lfoot_trajectory_float_.linear());
    rfoot_trajectory_euler_float_ = DyrosMath::rot2Euler(rfoot_trajectory_float_.linear());
  }
}


void WalkingController::computeIkControl(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& desired_leg_q)
{
  Eigen::Vector3d lp, rp;
  //Should revise by dg, Trunk_trajectory_global.translation()
  lp = float_lleg_transform.linear().transpose()*(float_trunk_transform.translation()-float_lleg_transform.translation());
  rp = float_rleg_transform.linear().transpose()*(float_trunk_transform.translation()-float_rleg_transform.translation());

  Eigen::Matrix3d trunk_lleg_rotation,trunk_rleg_rotation;
  trunk_lleg_rotation = float_trunk_transform.linear().transpose()*float_lleg_transform.linear();
  trunk_rleg_rotation = float_trunk_transform.linear().transpose()*float_rleg_transform.linear();

  Eigen::Vector3d ld, rd;
  ld.setZero(); rd.setZero();
  ld(0) = 0;
  ld(1) = 0.105;
  ld(2) = -0.1119;
  rd(0) = 0;
  rd(1) = -0.105;
  rd(2) = -0.1119;

  ld = trunk_lleg_rotation.transpose() * ld;
  rd = trunk_rleg_rotation.transpose() * rd;

  Eigen::Vector3d lr, rr;
  lr = lp + ld;
  rr = rp + rd;

  double l_upper = 0.3713; //direct length from hip to knee
  double l_lower = 0.3728; //direct length from knee to ankle

  double offset_hip_pitch = 24.0799945102*DEG2RAD;
  double offset_knee_pitch = 14.8197729791*DEG2RAD;
  double offset_ankle_pitch = 9.2602215311*DEG2RAD;

  //////////////////////////// LEFT LEG INVERSE KINEMATICS ////////////////////////////

  double lc = sqrt(lr(0)*lr(0)+lr(1)*lr(1)+lr(2)*lr(2));
  desired_leg_q(3) = (- acos((l_upper*l_upper + l_lower*l_lower - lc*lc) / (2*l_upper*l_lower))+ M_PI); // - offset_knee_pitch //+ alpha_lower

  double l_ankle_pitch = asin((l_upper*sin(M_PI-desired_leg_q(3)))/lc);
  desired_leg_q(4) = -atan2(lr(0), sqrt(lr(1)*lr(1)+lr(2)*lr(2))) - l_ankle_pitch;// - offset_ankle_pitch ;
  desired_leg_q(5) = atan2(lr(1), lr(2));

  Eigen::Matrix3d r_tl2;
  Eigen::Matrix3d r_l2l3;
  Eigen::Matrix3d r_l3l4;
  Eigen::Matrix3d r_l4l5;

  r_tl2.setZero();
  r_l2l3.setZero();
  r_l3l4.setZero();
  r_l4l5.setZero();

  r_l2l3 = DyrosMath::rotateWithY(desired_leg_q(3));
  r_l3l4 = DyrosMath::rotateWithY(desired_leg_q(4));
  r_l4l5 = DyrosMath::rotateWithX(desired_leg_q(5));

  r_tl2 = trunk_lleg_rotation * r_l4l5.transpose() * r_l3l4.transpose() * r_l2l3.transpose();

  desired_leg_q(1) = asin(r_tl2(2,1));

  double c_lq5 = -r_tl2(0,1)/cos(desired_leg_q(1));
  if (c_lq5 > 1.0)
  {
    c_lq5 =1.0;
  }
  else if (c_lq5 < -1.0)
  {
    c_lq5 = -1.0;
  }

  desired_leg_q(0) = -asin(c_lq5);
  desired_leg_q(2) = -asin(r_tl2(2,0)/cos(desired_leg_q(1))) + offset_hip_pitch;
  desired_leg_q(3) = desired_leg_q(3)- offset_knee_pitch;
  desired_leg_q(4) = desired_leg_q(4)- offset_ankle_pitch;

  //////////////////////////// RIGHT LEG INVERSE KINEMATICS ////////////////////////////

  double rc = sqrt(rr(0)*rr(0)+rr(1)*rr(1)+rr(2)*rr(2));
  desired_leg_q(9) = (-acos((l_upper*l_upper + l_lower*l_lower - rc*rc) / (2*l_upper*l_lower))+ M_PI); // - offset_knee_pitch //+ alpha_lower

  double r_ankle_pitch = asin((l_upper*sin(M_PI-desired_leg_q(9)))/rc);
  desired_leg_q(10) = -atan2(rr(0), sqrt(rr(1)*rr(1)+rr(2)*rr(2)))-r_ankle_pitch;
  desired_leg_q(11) = atan2(rr(1),rr(2));

  Eigen::Matrix3d r_tr2;
  Eigen::Matrix3d r_r2r3;
  Eigen::Matrix3d r_r3r4;
  Eigen::Matrix3d r_r4r5;

  r_tr2.setZero();
  r_r2r3.setZero();
  r_r3r4.setZero();
  r_r4r5.setZero();

  r_r2r3 = DyrosMath::rotateWithY(desired_leg_q(9));
  r_r3r4 = DyrosMath::rotateWithY(desired_leg_q(10));
  r_r4r5 = DyrosMath::rotateWithX(desired_leg_q(11));

  r_tr2 = trunk_rleg_rotation * r_r4r5.transpose() * r_r3r4.transpose() * r_r2r3.transpose();

  desired_leg_q(7) = asin(r_tr2(2,1));

  double c_rq5 = -r_tr2(0,1)/cos(desired_leg_q(7));
  if (c_rq5 > 1.0)
  {
    c_rq5 =1.0;
  }
  else if (c_rq5 < -1.0)
  {
    c_rq5 = -1.0;
  }

  desired_leg_q(6) = -asin(c_rq5);
  desired_leg_q(8) = asin(r_tr2(2,0)/cos(desired_leg_q(7))) - offset_hip_pitch;
  desired_leg_q(9) = -desired_leg_q(9) + offset_knee_pitch;
  desired_leg_q(10) = -desired_leg_q(10) + offset_ankle_pitch;

  //desired_leg_q(8) = desired_leg_q(8);  //motor axis direction
  //desired_leg_q(9) = desired_leg_q(9);
  //desired_leg_q(10) =desired_leg_q(10);

}


void WalkingController::computeJacobianControl(Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector3d float_lleg_transform_euler, Eigen::Vector3d float_rleg_transform_euler, Eigen::Vector12d& desired_leg_q_dot)
{


  Eigen::Matrix6d jacobian_temp_l, jacobian_temp_r, current_leg_jacobian_l_inv, current_leg_jacobian_r_inv,
      J_damped, I_matrix;
  double wl, wr, w0, lambda, a;
  w0 = 0.001;
  lambda = 0.05;
  jacobian_temp_l=current_leg_jacobian_l_*current_leg_jacobian_l_.transpose();
  jacobian_temp_r=current_leg_jacobian_r_*current_leg_jacobian_r_.transpose();
  wr = sqrt(jacobian_temp_r.determinant());
  wl = sqrt(jacobian_temp_l.determinant());

  if (wr<=w0)
  { //Right Jacobi
    a = lambda * pow(1-wr/w0,2);
    J_damped = current_leg_jacobian_r_.transpose()*current_leg_jacobian_r_+a*Eigen::Matrix6d::Identity();
    J_damped = J_damped.inverse();

    cout << "Singularity Region of right leg: " << wr << endl;
    current_leg_jacobian_r_inv = J_damped*current_leg_jacobian_r_.transpose();
  }
  else
  {
    //current_leg_jacobian_r_inv = DyrosMath::pinv(current_leg_jacobian_r_);
    current_leg_jacobian_r_inv = (current_leg_jacobian_r_.transpose()*current_leg_jacobian_r_).inverse()*current_leg_jacobian_r_.transpose();
  }

  if (wl<=w0)
  {
    a = lambda*pow(1-wl/w0,2);
    J_damped = current_leg_jacobian_l_.transpose()*current_leg_jacobian_l_+a*Eigen::Matrix6d::Identity();
    J_damped = J_damped.inverse();

    cout << "Singularity Region of right leg: " << wr << endl;
    current_leg_jacobian_l_inv = J_damped*current_leg_jacobian_l_.transpose();
  }
  else
  {
    current_leg_jacobian_l_inv = (current_leg_jacobian_l_.transpose()*current_leg_jacobian_l_).inverse()*current_leg_jacobian_l_.transpose();
    //current_leg_jacobian_l_inv = DyrosMath::pinv(current_leg_jacobian_r_);
  }

  Eigen::Matrix6d kp; // for setting CLIK gains
  kp.setZero();
  kp(0,0) = 160;
  kp(1,1) = 160;
  kp(2,2) = 160;
  kp(3,3) = 100;
  kp(4,4) = 100;
  kp(5,5) = 100;



  Eigen::Vector6d lp, rp, cubic_xr, cubic_xl;
  lp.setZero(); rp.setZero(), cubic_xr.setZero(), cubic_xl.setZero();
  lp.topRows<3>() = (-lfoot_float_current_.translation()+float_lleg_transform.translation());
  rp.topRows<3>() = (-rfoot_float_current_.translation()+float_rleg_transform.translation());

  for(int i=0;i<3;i++)
  {
    cubic_xl(i) = float_lleg_transform.translation()(i);
    cubic_xl(i+3) = float_lleg_transform_euler(i);
  }

  for(int i=0;i<3;i++)
  {
    cubic_xr(i) = float_rleg_transform.translation()(i);
    cubic_xr(i+3) = float_rleg_transform_euler(i);
  }
  Eigen::Vector3d r_leg_phi, l_leg_phi;
  l_leg_phi = DyrosMath::legGetPhi(lfoot_float_current_, lfoot_float_init_, cubic_xl);
  r_leg_phi = DyrosMath::legGetPhi(rfoot_float_current_, rfoot_float_init_, cubic_xr);
  //l_leg_phi = DyrosMath::getPhi(lfoot_float_current_.linear(), lfoot_float_init_.linear());
  //r_leg_phi = DyrosMath::getPhi(rfoot_float_current_.linear(), rfoot_float_init_.linear());

  lp.bottomRows<3>() = - l_leg_phi;
  rp.bottomRows<3>() = - r_leg_phi;

  Eigen::Vector6d q_lfoot_dot,q_rfoot_dot;
  q_lfoot_dot=current_leg_jacobian_l_inv*kp*lp;
  q_rfoot_dot=current_leg_jacobian_r_inv*kp*rp;

  for (int i=0; i<6; i++)
  {
    desired_leg_q_dot(i+6) = q_rfoot_dot(i);
    desired_leg_q_dot(i) = q_lfoot_dot(i);
  }
}

void WalkingController::modifiedPreviewControl()
{
  /////reference: http://www.tandfonline.com/doi/pdf/10.1080/0020718508961156?needAccess=true/////////////

  if(walking_tick_==0)
  {
    previewControlParameter(1.0/hz_, 16*hz_/10, k_ ,com_support_init_, gi_, gp_l_, gx_, a_, b_, c_);
  }

  if(current_step_num_ == 0)
    zmp_start_time_ = 0.0;
  else
    zmp_start_time_ = t_start_;

  ux_1_ = 0.0;
  uy_1_ = 0.0;

  previewControl(1.0/hz_, 16*hz_/10, walking_tick_-zmp_start_time_, xi_, yi_, xs_, ys_, ux_1_, uy_1_, ux_, uy_, gi_, gp_l_, gx_, a_, b_, c_, xd_, yd_);
  Eigen::Vector3d xs_matrix, ys_matrix, xs, ys;
  for (int i=0; i<3; i++)
    xs_matrix(i) = xd_(i);
  for (int i=0; i<3; i++)
    ys_matrix(i) = yd_(i);

  //double est_zmp_error_x, est_zmp_error_y, est_zmp;
  //est_zmp_error_x = c_*xs_matrix;
  //est_zmp_error_y = c_*ys_matrix;

  previewControl(1.0/hz_, 16*hz_/10, walking_tick_-zmp_start_time_, xi_, yi_, xs_, ys_, ux_1_, uy_1_, ux_, uy_, gi_, gp_l_, gx_, a_, b_, c_, xd_, yd_);

  ux_1_ = ux_;
  uy_1_ = uy_;

}

void WalkingController::previewControl(
    double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys,
    double ux_1, double uy_1 , double& ux, double& uy, double gi, Eigen::VectorXd gp_l,
    Eigen::Matrix1x3d gx, Eigen::Matrix3d a, Eigen::Vector3d b, Eigen::Matrix1x3d c,
    Eigen::Vector3d &xd, Eigen::Vector3d &yd)
{
  int zmp_size;
  zmp_size = ref_zmp_.col(1).size();

  Eigen::VectorXd px_ref, py_ref;
  px_ref.resize(zmp_size);
  py_ref.resize(zmp_size);

  for (int i=0; i<zmp_size; i++)
  {
    px_ref(i) = ref_zmp_(i,0);
    py_ref(i) = ref_zmp_(i,1);
  }

  Eigen::Vector3d x, y, x_1, y_1;
  x.setZero();
  y.setZero();
  x_1.setZero();
  y_1.setZero();



  if(tick==0 && current_step_num_ == 0)
  {
    x(0) = x_i;
    y(0) = y_i;
  }
  else
  {
    x = xs;
    y = ys;
  }

  x_1(0) = x(0)-x(1)*dt;
  x_1(1) = x(1)-x(2)*dt;
  x_1(2) = x(2);
  y_1(0) = y(0)-y(1)*dt;
  y_1(1) = y(1)-y(2)*dt;
  y_1(2) = y(2);

  double xzmp_err =0.0, yzmp_err = 0.0;

  Eigen::Matrix<double, 1, 1> px, py;
  px = (c*x);
  py = (c*y);
  xzmp_err = px(0) - px_ref(tick);
  yzmp_err = py(0) - py_ref(tick);

  double sum_gp_px_ref = 0.0, sum_gp_py_ref =0.0;
  for(int i = 0; i < NL; i++)
  {
    sum_gp_px_ref = sum_gp_px_ref + gp_l(i)*(px_ref(tick+1+i)-px_ref(tick+i));
    sum_gp_py_ref = sum_gp_py_ref + gp_l(i)*(py_ref(tick+1+i)-py_ref(tick+i));
  }
  double gx_x, gx_y, del_ux, del_uy;
  gx_x = gx*(x-x_1);
  gx_y = gx*(y-y_1);

  del_ux = -(xzmp_err*gi)-gx_x-sum_gp_px_ref;
  del_uy = -(yzmp_err*gi)-gx_y-sum_gp_py_ref;

  ux = ux_1 + del_ux;
  uy = uy_1 + del_uy;

  xd = a*x + b*ux;
  yd = a*y + b*uy;

}

void WalkingController::previewControlParameter(
    double dt, int NL, Eigen::Matrix4d& k, Eigen::Vector3d com_support_init_,
    double& gi, Eigen::VectorXd& gp_l, Eigen::Matrix1x3d& gx,
    Eigen::Matrix3d& a, Eigen::Vector3d& b, Eigen::Matrix1x3d& c)
{
  a.setIdentity();
  a(0,1) = dt;
  a(0,2) = dt*dt/2.0;
  a(1,2) = dt;

  b.setZero();
  b(0) =dt*dt*dt/6.0;
  b(1) =dt*dt/2.0;
  b(2) =dt;

  c(0,0) = 1;
  c(0,1) = 0;
  c(0,2) = -zc_/GRAVITY;


  Eigen::Vector4d b_bar;
  b_bar(0) = c*b;
  b_bar.segment(1,3) = b;

  Eigen::Matrix1x4d b_bar_tran;
  b_bar_tran = b_bar.transpose();

  Eigen::Vector4d i_p;
  i_p.setZero();
  i_p(0) = 1;

  Eigen::Matrix4x3d f_bar;
  f_bar.setZero();
  f_bar.block<1,3>(0,0) = c*a;
  f_bar.block<3,3>(1,0) = a;

  Eigen::Matrix4d a_bar;
  a_bar.block<4,1>(0,0) = i_p;
  a_bar.block<4,3>(0,1) = f_bar;

  double qe;
  qe = 1.0;
  Eigen::Matrix<double, 1,1> r;
  r(0,0) = 0.000001;

  Eigen::Matrix3d qx;
  qx.setZero();
  Eigen::Matrix4d q_bar;
  q_bar.setZero();
  q_bar(0,0) = qe;
  q_bar.block<3,3>(1,1) = qx;

  k=DyrosMath::discreteRiccatiEquation(a_bar, b_bar, r, q_bar);

  double temp_mat;
  temp_mat = r(0)+b_bar_tran*k*b_bar;

  Eigen::Matrix4d ac_bar;
  ac_bar.setZero();
  ac_bar = a_bar - b_bar*b_bar_tran*k*a_bar/temp_mat;

  gi = b_bar_tran*k*i_p;
  gi *= 1/temp_mat;
  gx = b_bar_tran*k*f_bar/temp_mat;

  Eigen::MatrixXd x_l(4, NL);
  Eigen::Vector4d x_l_column;
  x_l.setZero();
  x_l_column.setZero();
  x_l_column = -ac_bar.transpose()*k*i_p;
  for(int i=0; i<NL; i++)
  {
    x_l.col(i) = x_l_column;
    x_l_column = ac_bar.transpose()*x_l_column;
  }
  gp_l.resize(NL);
  double gp_l_column;
  gp_l_column = -gi;
  for(int i=0; i<NL; i++)
  {
    gp_l(i) = gp_l_column;
    gp_l_column = b_bar_tran*x_l.col(i);
    gp_l_column = gp_l_column/temp_mat;
  }

}

void WalkingController::compensator()
{
  if(hip_compensator_mode_ == true)
  {
    hipCompensation();
    hipCompensator();
  }

  if(lqr_compensator_mode_ == true)
  {
    Eigen::Vector12d d_q;
    d_q.setZero();
    for (int i=0; i<12; i++)
      d_q(i) = desired_q_(i);

    Eigen::Vector12d lqr_joint_input;
    lqr_joint_input.setZero();

    slowcalc_mutex_.lock();

    ad_copy_ = ad_right_;
    bd_copy_ = bd_right_;
    ad_total_copy_ = ad_total_right_;
    bd_total_copy_ = bd_total_right_;
    kkk_copy_ = kkk_motor_right_;
    slowcalc_mutex_.unlock();


    vibrationControl(d_q,lqr_joint_input);


    double _grav_gain_timing;
    _grav_gain_timing = 1.0;
    if(foot_step_(current_step_num_,6) == 0) // right foot (left foot gain)
    {
      if(walking_tick_ > t_start_+t_rest_init_+t_double1_ && walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_)
        _grav_gain_timing = DyrosMath::cubic(walking_tick_,t_start_+t_rest_init_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,1.0,0.3,0.0,0.0);
      else
        _grav_gain_timing = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_,t_start_+t_total_-t_rest_last_,0.3,1.0,0.0,0.0);
    }




    for (int n = 7 ; n < 12; n++) // left foot
    {
      if(abs(lqr_joint_input(n)-desired_q_(n)) > 20.0*DEG2RAD )
      {
      }
      else
      {
        if(n == 7 || n == 8)
          desired_q_(n) = lqr_joint_input(n) - 0.0021*_grav_gain_timing*grav_ground_torque_[n];
        else if (n == 9)
          desired_q_(n) = lqr_joint_input(n) - 0.0003*_grav_gain_timing*grav_ground_torque_[n];
        else
          desired_q_(n) = lqr_joint_input(n);
      }
    }


    _grav_gain_timing = 1.0;
    if(foot_step_(current_step_num_,6) == 1) // left foot (right foot gain)
    {
      if(walking_tick_ > t_start_+t_rest_init_+t_double1_ && walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_)
        _grav_gain_timing = DyrosMath::cubic(walking_tick_,t_start_+t_rest_init_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,1.0,0.3,0.0,0.0);
      else
        _grav_gain_timing = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_,t_start_+t_total_-t_rest_last_,0.3,1.0,0.0,0.0);
    }


    for (int n = 1 ; n < 6; n++)
    {
      if(abs(lqr_joint_input(n)-desired_q_(n)) > 20.0*DEG2RAD )
      {
      }
      else
      {
        if ( n == 1 || n == 2)
          desired_q_(n) = lqr_joint_input(n) - 0.0021*_grav_gain_timing*grav_ground_torque_[n];
        else if (n == 3)
          desired_q_(n) = lqr_joint_input(n) - 0.0003*_grav_gain_timing*grav_ground_torque_[n];
        else
          desired_q_(n) = lqr_joint_input(n);
      }
    }

  }

}

void WalkingController::hipCompensator()
{
  double left_hip_angle = 3.6*DEG2RAD, right_hip_angle = 4.7*DEG2RAD, left_hip_angle_first_step = 3.5*DEG2RAD, right_hip_angle_first_step = 4.7*DEG2RAD,
      left_hip_angle_temp = 00, right_hip_angle_temp = 0.0, temp_time = 0.1*hz_, left_pitch_angle = 0.0*DEG2RAD;

  if (current_step_num_ == 0)
  {
    if(foot_step_(current_step_num_, 6) == 1) //left support foot
    {
      if(walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+temp_time,0.0*DEG2RAD, left_hip_angle_first_step, 0.0, 0.0);
      else if(walking_tick_ >= t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-temp_time,t_start_+t_total_-t_rest_last_,left_hip_angle_first_step, 0.0, 0.0, 0.0);
      else
        left_hip_angle_temp = 0.0*DEG2RAD;
    }
    else if (foot_step_(current_step_num_, 6) == 0) // right support foot
    {
      if(walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+temp_time,0.0*DEG2RAD, right_hip_angle_first_step, 0.0, 0.0);
      else if(walking_tick_ >= t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-temp_time,t_start_+t_total_-t_rest_last_,right_hip_angle_first_step, 0.0, 0.0, 0.0);
      else
        right_hip_angle_temp = 0.0*DEG2RAD;
    }
    else
    {
      left_hip_angle_temp = 0.0*DEG2RAD;
      right_hip_angle_temp = 0.0*DEG2RAD;
    }
  }
  else
  {
    if(foot_step_(current_step_num_, 6) == 1)
    {
      if(walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+temp_time,0.0*DEG2RAD,left_hip_angle,0.0,0.0);
      else if (walking_tick_ >= t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-temp_time,t_start_+t_total_-t_rest_last_,left_hip_angle,0.0,0.0,0.0);
      else
        left_hip_angle_temp = 0.0*DEG2RAD;

    }
    else if(foot_step_(current_step_num_,6) == 0)
    {
      if(walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+temp_time,0.0*DEG2RAD,right_hip_angle,0.0,0.0);
      else if(walking_tick_ >= t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-temp_time,t_start_+t_total_-t_rest_last_,left_hip_angle,0.0,0.0,0.0);
      else
        right_hip_angle_temp = 0.0*DEG2RAD;
    }
    else
    {
      left_hip_angle_temp = 0.0*DEG2RAD;
      right_hip_angle_temp = 0.0*DEG2RAD;
    }
  }
  desired_q_(1) = desired_q_(1) + left_hip_angle_temp;
  desired_q_(7) = desired_q_(7) - right_hip_angle_temp;
  joint_offset_angle_(1) = left_hip_angle_temp;
  joint_offset_angle_(7) = -right_hip_angle_temp;
}

void WalkingController::hipCompensation()
{
  double a_total, b_total, alpha_l, alpha_r, rq0, rq1, rq2, rq3, rq4, rq5, lq0, lq1, lq2, lq3, lq4, lq5, robotweight, fromright, fromleft, alpha, alpha_1, f_r, f_l; //alpha is weighting factor

  a_total = -0.0012;
  b_total = 0.00087420;
  robotweight = 46.892*9.81;

  lq0= desired_q_(0);
  lq1= desired_q_(1);
  lq2= desired_q_(2);
  lq3= desired_q_(3);
  lq4= desired_q_(4);
  lq5= desired_q_(5);
  rq0= desired_q_(6);
  rq1= desired_q_(7);
  rq2= desired_q_(8);
  rq3= desired_q_(9);
  rq4= desired_q_(10);
  rq5= desired_q_(11);

  /*  fromright = cos(rq2)*sin(rq0)*(-1.454E-1)-sin(rq0)*sin(rq2)*(3.39E2/1.0E3)-cos(rq3)*(cos(rq2)*sin(rq0)+cos(rq0)*sin(rq1)*sin(rq2))*(3.0/5.0E1)-cos(rq3)*(sin(rq0)*sin(rq2)-cos(rq0)*cos(rq2)*sin(rq1))*(4.6E1/1.25E2)-sin(rq3)*(cos(rq2)*sin(rq0)+cos(rq0)*sin(rq1)*sin(rq2))*(4.6E1/1.25E2)+sin(rq3)*(sin(rq0)*sin(rq2)-cos(rq0)*cos(rq2)*sin(rq1))*(3.0/5.0E1)+cos(rq0)*cos(rq2)*sin(rq1)*(3.39E2/1.0E3)-cos(rq0)*sin(rq1)*sin(rq2)*1.454E-1-2.1E1/2.0E2;
  fromleft = cos(lq2)*sin(lq0)*(-1.454E-1)+sin(lq0)*sin(lq2)*(3.39E2/1.0E3)+cos(lq3)*(sin(lq0)*sin(lq2)+cos(lq0)*cos(lq2)*sin(lq1))*(4.6E1/1.25E2)-cos(lq3)*(cos(lq2)*sin(lq0)-cos(lq0)*sin(lq1)*sin(lq2))*(3.0/5.0E1)+sin(lq3)*(sin(lq0)*sin(lq2)+cos(lq0)*cos(lq2)*sin(lq1))*(3.0/5.0E1)+sin(lq3)*(cos(lq2)*sin(lq0)-cos(lq0)*sin(lq1)*sin(lq2))*(4.6E1/1.25E2)+cos(lq0)*cos(lq2)*sin(lq1)*(3.39E2/1.0E3)+cos(lq0)*sin(lq1)*sin(lq2)*1.454E-1+2.1E1/2.0E2;
*/
  /*
  fromright = com_float_current_(1)-rfoot_float_current_.translation()(1);
  fromleft = com_float_current_(1)-lfoot_float_current_.translation()(1);
*/

  fromright = -rfoot_float_current_.translation()(1);
  fromleft = -lfoot_float_current_.translation()(1);


  alpha = -fromleft/(fromright-fromleft);

  if(fromright>=0)
  {
    alpha=1;
  }
  if(fromleft<=0)
  {
    alpha=0;
  }

  alpha_1 = 1-alpha;

  f_r = robotweight*alpha;
  f_l = robotweight*alpha_1;

  Eigen::Vector6d lTau, rTau, Jc2, Jc8;
  lTau.setZero();
  rTau.setZero();
  Jc2.setZero();
  Jc8.setZero();
  /*
  Jc2(0) = 0;
  Jc2(1) = cos(rq2)*sin(rq1)*(3.39E2/1.0E3)-sin(rq1)*sin(rq2)*1.454E-1-sin(rq1)*sin(rq2)*sin(rq3)*(4.6E1/1.25E2)+cos(rq2)*cos(rq3)*sin(rq1)*(4.6E1/1.25E2)-cos(rq2)*sin(rq1)*sin(rq3)*(3.0/5.0E1)-cos(rq3)*sin(rq1)*sin(rq2)*(3.0/5.0E1);
  Jc2(2) = cos(rq1)*cos(rq2)*1.454E-1+cos(rq1)*sin(rq2)*(3.39E2/1.0E3)+cos(rq1)*cos(rq2)*cos(rq3)*(3.0/5.0E1)+cos(rq1)*cos(rq2)*sin(rq3)*(4.6E1/1.25E2)+cos(rq1)*cos(rq3)*sin(rq2)*(4.6E1/1.25E2)-cos(rq1)*sin(rq2)*sin(rq3)*(3.0/5.0E1);
  Jc2(3) = cos(rq1)*cos(rq2)*cos(rq3)*(3.0/5.0E1)+cos(rq1)*cos(rq2)*sin(rq3)*(4.6E1/1.25E2)+cos(rq1)*cos(rq3)*sin(rq2)*(4.6E1/1.25E2)-cos(rq1)*sin(rq2)*sin(rq3)*(3.0/5.0E1);
  Jc2(4) = 0;
  Jc2(5) = 0;

  Jc8(0) = 0;
  Jc8(1) = cos(lq2)*sin(lq1)*(3.39E2/1.0E3)+sin(lq1)*sin(lq2)*1.454E-1-sin(lq1)*sin(lq2)*sin(lq3)*(4.6E1/1.25E2)+cos(lq2)*cos(lq3)*sin(lq1)*(4.6E1/1.25E2)+cos(lq2)*sin(lq1)*sin(lq3)*(3.0/5.0E1)+cos(lq3)*sin(lq1)*sin(lq2)*(3.0/5.0E1);
  Jc8(2) = cos(lq1)*cos(lq2)*(-1.454E-1)+cos(lq1)*sin(lq2)*(3.39E2/1.0E3)-cos(lq1)*cos(lq2)*cos(lq3)*(3.0/5.0E1)+cos(lq1)*cos(lq2)*sin(lq3)*(4.6E1/1.25E2)+cos(lq1)*cos(lq3)*sin(lq2)*(4.6E1/1.25E2)+cos(lq1)*sin(lq2)*sin(lq3)*(3.0/5.0E1);
  Jc8(3) = cos(lq1)*cos(lq2)*cos(lq3)*(-3.0/5.0E1)+cos(lq1)*cos(lq2)*sin(lq3)*(4.6E1/1.25E2)+cos(lq1)*cos(lq3)*sin(lq2)*(4.6E1/1.25E2)+cos(lq1)*sin(lq2)*sin(lq3)*(3.0/5.0E1);
  Jc8(4) = 0;
  Jc8(5) = 0;
*/
  Jc8=current_leg_jacobian_l_.transpose().col(3);
  Jc2=current_leg_jacobian_r_.transpose().col(3);


  for(int i=0; i<6; i++)
  {
    rTau(i)=Jc2(i)*f_r;
    lTau(i)=Jc8(i)*f_l;
  }

  double rising = 1.0, timingtiming = 1.0, k = 0.2, k1 = 0.2;
  /*
  if(walking_tick_ > t_start_+ t_rest_init_ && walking_tick_ <= t_start_+t_rest_init_+t_double1_*timingtiming)
  {
      rising = (walking_tick_-t_start_-t_rest_init_)/(t_double1_*timingtiming);
  }
  else if(walking_tick_ > t_start_+t_rest_init_+t_double1_*timingtiming && walking_tick_<= t_start_+t_total_-t_rest_last_-t_double2_*timingtiming)
  {
      rising =1;
  }
  else if(walking_tick_ > t_start_+t_total_-t_rest_last_-t_double2_*timingtiming &&  walking_tick_<= t_start_+t_total_-t_rest_last_)
  {
      rising = -(walking_tick_- (t_start_+t_total_-t_rest_last_))/(t_double2_*timingtiming);
  }*/

  joint_offset_angle_.setZero();
  grav_ground_torque_.setZero();

  if (lqr_compensator_mode_ == false)
  {
    desired_q_(8)=desired_q_(8)+(a_total*rTau(2)+b_total)*rising*k1;//offwhenslow
    desired_q_(9)=desired_q_(9)+(a_total*rTau(3)+b_total)*rising*0.3;//offwhenslow
    desired_q_(10)=desired_q_(10)+(a_total*rTau(4)+b_total)*rising*k1;//offwhenslow
  }
  // _desired_q(23-2)=_desired_q(23-2)+(a_total*rTau(5)+b_total)*rising;

  joint_offset_angle_(8) = (a_total*rTau(2)+b_total)*rising*k;
  joint_offset_angle_(9) = (a_total*rTau(3)+b_total)*rising*0.2;
  joint_offset_angle_(10) = (a_total*rTau(4)+b_total)*rising*k;

  if (lqr_compensator_mode_  == false)
  {
    desired_q_(2)=desired_q_(2)+(a_total*lTau(2)+b_total)*rising*k;//offwhenslow
    desired_q_(3)=desired_q_(3)+(a_total*lTau(3)+b_total)*rising*0.3;//offwhenslow
    desired_q_(4)=desired_q_(4)+(a_total*lTau(4)+b_total)*rising*k;//offwhenslow
    //  _desired_q(29-2)=_desired_q(29-2)+(a_total*lTau(5)+b_total)*rising*k;
  }

  joint_offset_angle_(2) = (a_total*lTau(2)+b_total)*rising*k1;
  joint_offset_angle_(3) = (a_total*lTau(3)+b_total)*rising*0.2;
  joint_offset_angle_(4) = (a_total*lTau(4)+b_total)*rising*k1;


  for (int i=0; i<6; i++)
  {
    grav_ground_torque_(i) = lTau[i];
    grav_ground_torque_(i+6) = rTau[i];
  }

}


void WalkingController::vibrationControl(const Eigen::Vector12d desired_leg_q, Eigen::Vector12d &output)
{
  if(walking_tick_ == 0)
  {
    pre_motor_q_leg_ = current_motor_q_leg_;
    pre_link_q_leg_ = current_link_q_leg_;

    for (int i=0; i<6; i++)
    {
      lqr_output_pre_(i) = current_motor_q_leg_(i+6); // left
      lqr_output_pre_(i+6) = current_motor_q_leg_(i); //right
    }
  }

  // right foot X_bar
  for (int i = 0; i < 6; i++)
  {
    x_bar_right_(i, 0) = current_link_q_leg_(i+6) - desired_q_(i+6); //left
    x_bar_right_(i+6,0) = current_link_q_leg_(i) - desired_q_(i); // right

    x_bar_right_(i+12,0) = current_motor_q_leg_(i+6) - pre_motor_q_leg_(i+6); //left
    x_bar_right_(i+18,0) = current_motor_q_leg_(i) - pre_motor_q_leg_(i); //left

    x_bar_right_(i+24,0) = current_link_q_leg_(i+6) - pre_link_q_leg_(i+6); //left
    x_bar_right_(i+30,0) = current_link_q_leg_(i) - pre_link_q_leg_(i); //left

  }


  Eigen::Vector12d del_u_right;

  del_u_right = -kkk_copy_*x_bar_right_;

  // if(_cnt == 0)
  //  cout << "uuu" << del_u_right << endl;

  x_bar_right_ = ad_total_copy_*x_bar_right_ + bd_total_copy_*del_u_right;


  Eigen::Vector12d current_u;
  current_u.setZero(); // left right order

  for (int i = 0; i < 6; i++)
  {
    //left
    current_u(i) = (current_motor_q_leg_(i+6) - ad_copy_(i, i)*pre_motor_q_leg_(i+6)) / bd_copy_(i, i);
    //right
    current_u(i+6) = (current_motor_q_leg_(i) - ad_copy_(i, i)*pre_motor_q_leg_(i)) / bd_copy_(i, i);
  }



  Eigen::Vector12d dist;
  dist.setZero(); //left right order

  for (int i=0; i<6; i++)
  {
    //left
    dist(i) = lqr_output_pre_(i) - current_u(i);
    //right
    dist(i+6) = lqr_output_pre_(i+6) - current_u(i+6);
  }


  if(walking_tick_ == 0)
    dist_prev_ = dist;

  dist = 0.7*dist_prev_+0.3*dist;

  // Mingon's LQR contorller gain(using external encoder)
  double default_gain = 0.2;

  double compliant_gain = 1.0;


  for (int i = 0; i < 12; i++)
  {
    if(i > 6) //left foot
    {

      double gain_temp = default_gain;

      if(walking_enable_ == true)
      {
        if (foot_step_(current_step_num_,6) == 0) // right support foot
        {
          if(walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_-0.1*hz_) // gain_temp -> 0.2
          {
            gain_temp = default_gain;
          }
          else if(walking_tick_ >= t_start_+t_total_-t_rest_last_-t_double2_-0.1*hz_ && walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_)
          {
            gain_temp = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-0.1*hz_,t_start_+t_total_-t_rest_last_-t_double2_,default_gain,compliant_gain,0.0,0.0);
          }
          else
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_+t_total_-t_rest_last_,t_start_+t_total_,compliant_gain,default_gain,0.0,0.0);
          }
        }
        else //left support foot
        {
          gain_temp = default_gain;

        }
      }


      lqr_output_(i-6) = lqr_output_pre_(i-6) + del_u_right(i-6, 0) - gain_temp*dist(i-6);
    }
    else // right foot
    {

      double gain_temp = default_gain;

      if(walking_enable_ == true)
      {
        if (foot_step_(current_step_num_,6) == 1) // left suppor foot
        {
          if(walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_-0.1*hz_) // gain_temp -> 0.2
          {
            gain_temp = default_gain;
          }
          else if(walking_tick_ >= t_start_+t_total_-t_rest_last_-t_double2_-0.1*hz_ && walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_)
          {
            gain_temp = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-0.1*hz_,t_start_+t_total_-t_rest_last_-t_double2_,default_gain,compliant_gain,0.0,0.0);
          }
          else
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_+t_total_-t_rest_last_,t_start_+t_total_,compliant_gain,default_gain,0.0,0.0);
          }
        }
        else //left foot support
        {
          gain_temp = default_gain;

        }
      }
      lqr_output_(i+6) = lqr_output_pre_(i+6) + del_u_right(i+6, 0) - gain_temp*dist(i+6);
    }
  }

  lqr_output_pre_ = lqr_output_;
  pre_motor_q_leg_ = current_motor_q_leg_;
  pre_link_q_leg_ = current_link_q_leg_;
  dist_prev_ = dist;

  for (int i=0; i<6; i++)
  {
    desired_q_(i) = lqr_output_(i+6); //right foot
    desired_q_(i+6) = lqr_output_(i); //left foot
  }

}


void WalkingController::massSpringMotorModel(double spring_k, double damping_d, double motor_k, Eigen::MatrixXd & mass, Eigen::MatrixXd& a, Eigen::MatrixXd& b, Eigen::MatrixXd& c)
{
  int dof = 12;
  Eigen::Matrix12d spring_k_mat;
  Eigen::Matrix12d damping_d_mat;
  Eigen::Matrix12d motor_k_mat;

  spring_k_mat.setZero();
  damping_d_mat.setZero();
  motor_k_mat.setZero();

  for (int i = 0; i < dof; i++)
  {
    spring_k_mat(i, i) = spring_k;
    damping_d_mat(i, i) = damping_d;
    motor_k_mat(i, i) = motor_k;
  }

  // knee joint
  motor_k_mat(3,3) =  18.0; //18.0
  motor_k_mat(9,9) =  18.0;

  Eigen::Matrix12d inv_mass;
  inv_mass = mass;

  Eigen::Matrix12d zero_12;
  zero_12.setZero();

  Eigen::Matrix12d eye_12;
  eye_12.setIdentity();

  Eigen::Matrix12d mass_temp;
  mass_temp = inv_mass;

  Eigen::Matrix12d a1;
  a1 = mass_temp*(spring_k_mat - damping_d_mat*motor_k_mat);

  Eigen::Matrix12d a2;
  a2 = -mass_temp*(spring_k_mat);

  Eigen::Matrix12d a3;
  a3 = -mass_temp*(damping_d_mat);

  a.resize(dof*3, dof*3); a.setZero();
  a << -motor_k_mat, zero_12, zero_12, zero_12, zero_12, eye_12, a1, a2, a3;

  Eigen::Matrix12d b1;
  b1 = mass_temp*damping_d_mat*motor_k_mat;

  b.resize(dof*3, dof); b.setZero();
  b << motor_k_mat, zero_12, b1;

  c.resize(dof, dof*3); c.setZero();
  c << zero_12, eye_12, zero_12;
}

void WalkingController::discreteModel(Eigen::MatrixXd& a, Eigen::MatrixXd& b, Eigen::MatrixXd& c, int np, double dt,
                                      Eigen::MatrixXd& ad, Eigen::MatrixXd& bd, Eigen::MatrixXd& cd, Eigen::MatrixXd& ad_total, Eigen::MatrixXd& bd_total)
{
  int n = a.rows(); //state \B0\B9\BC\F6
  int r = b.cols(); // Input \B0\B9\BC\F6
  int p = c.rows(); // output \B0\B9\BC\F6

  ad.resize(n, n); ad.setZero();
  // ad = A*dt;
  // ad = ad.exp();


  Eigen::MatrixXd inv_a;
  inv_a.resize(n, n); inv_a = a.inverse();

  Eigen::MatrixXd eye6;
  eye6.resize(n, n); eye6.setIdentity();

  bd.resize(n, r);
  // bd = inv_a*(ad - eye6)*B;

  ad = eye6 +a*dt;
  bd = b*dt;

  cd.resize(p, n);
  cd = c;

  //std::cout << "AAA_bar" << AAA_bar << std::endl;
  //std::cout << "BBB_bar" << BBB_bar << std::endl;

  Eigen::MatrixXd ca;
  ca.resize(p, n);
  ca = cd*ad;

  Eigen::MatrixXd eye_p;
  eye_p.resize(p, p); eye_p.setIdentity();

  Eigen::MatrixXd zero_n_p;
  zero_n_p.resize(n, p); zero_n_p.setZero();

  Eigen::MatrixXd cb;
  cb.resize(p, r); cb.setZero();
  cb = cd*bd;

  if (np < 1)
  {
    ad_total.resize(n + p, n + p);
    bd_total.resize(n + p, r);

    ad_total << eye_p, ca, zero_n_p, ad;
    bd_total << cb, bd;
  }
  else
  {
    ad_total.resize(n + p + np, n + p + np);
    bd_total.resize(n + p + np, r);

    Eigen::MatrixXd zero_temp1;
    zero_temp1.resize(p, (np - 1)*p); zero_temp1.setZero();

    Eigen::MatrixXd zero_temp2;
    zero_temp2.resize(n, np*p); zero_temp2.setZero();

    Eigen::MatrixXd zero_temp3;
    zero_temp3.resize(np*p, p + n); zero_temp3.setZero();

    Eigen::MatrixXd shift_register;
    shift_register.resize(np*p, np*p); shift_register.setZero();

    Eigen::MatrixXd zero_temp4;
    zero_temp4.resize(np*p, r); zero_temp4.setZero();
    for (int i = 0; i<p*(np - 1); i++)
      shift_register(i, i + p) = 1;

    ad_total << eye_p, ca, -eye_p, zero_temp1, zero_n_p, ad, zero_temp2, zero_temp3, shift_register;
    bd_total << cb, bd, zero_temp4;
  }
}

void WalkingController::riccatiGain(Eigen::MatrixXd& ad_total, Eigen::MatrixXd& bd_total, Eigen::Matrix<double, 12*4, 12*4>& q, Eigen::Matrix12d& r, Eigen::Matrix<double, 12, 12*4>& k)
{

  const int n = ad_total.rows(); //state \B0\B9\BC\F6
  const int m = bd_total.cols(); // Input \B0\B9\BC\F6


  kkk_.resize(n, n);

  kkk_ = DyrosMath::discreteRiccatiEquation(ad_total, bd_total, r, q);

  //KKK.resize(8,8); KKK.setIdentity();

  Eigen::MatrixXd trans_bd;
  trans_bd.resize(m, n); trans_bd = bd_total.transpose();

  k.resize(m, n);
  k = ((r + trans_bd*kkk_*bd_total).inverse())*trans_bd*kkk_*ad_total;
  //k = k.inverse();
  //k = k*trans_bd*kkk_*ad_total;
}



void WalkingController::slowCalc()
{
  while(true)
  {
    if(calc_start_flag_)
    {
      Eigen::Vector28d qqq;
      qqq.setZero();
      slowcalc_mutex_.lock();

      for(int i =0; i<28; i++)
      {
        qqq(i) = thread_q_(i);
      }
      slowcalc_mutex_.unlock();

      Eigen::Matrix<double, 6, 18> contact_j;
      Eigen::Matrix6d lamda;
      Eigen::Matrix<double, 6, 18> j_c_bar;
      Eigen::Matrix<double, 18, 18> pc;
      Eigen::Matrix<double, 18, 18> eye_18;
      Eigen::Matrix<double, 18, 18> mass_inverse;
      Eigen::Matrix<double, 12, 12> temp22;
      Eigen::Matrix<double, 12*4, 12*4> q_mat;
      Eigen::Matrix12d r_mat;

      Eigen::Matrix<double, 12, 12*4> lqr_k; //lqr_k.resize(12, 12*4);

      if(thread_tick_ == 0)
      {
        mass_matrix_.resize(18, 18);     mass_matrix_.setZero();
        mass_matrix_pc_.resize(18, 18);  mass_matrix_pc_.setZero();
        mass_matrix_sel_.resize(12, 12); mass_matrix_sel_.setZero();
      }





      mass_matrix_ = model_.getLegInertia();

      contact_j.setZero();

      if(walking_enable_ == true)
      {
        if(foot_step_(current_step_num_,6) == 0) //right foot
        {
          contact_j = model_.getLegWithVLinkJacobian((DyrosJetModel::EndEffector)(1));
        }
        else if(foot_step_(current_step_num_,6) == 1)
        {
          contact_j = model_.getLegWithVLinkJacobian((DyrosJetModel::EndEffector)(0));
        }




        lamda.setZero();
        lamda = (contact_j*mass_matrix_.inverse()*contact_j.transpose());
        lamda = lamda.inverse();

        j_c_bar.setZero();

        j_c_bar = lamda*contact_j*mass_matrix_.inverse();

        pc.setZero();

        pc = contact_j.transpose()*j_c_bar;

        eye_18.setIdentity();


        mass_inverse = mass_matrix_.inverse();

        mass_matrix_pc_ = mass_inverse*(eye_18-pc);

        for (int i=0; i<12; i++)
        {
          for (int j=0; j<12; j++)
            mass_matrix_sel_(i,j) = mass_matrix_pc_(i+6,j+6);
        }

      }
      else
      {
        mass_inverse = mass_matrix_.inverse();

        for (int i=0; i<12; i++)
        {
          for (int j=0; j<12; j++)
            mass_matrix_sel_(i,j) = mass_inverse(i+6,j+6);
        }
      }

      temp22.setZero();
      temp22 = mass_matrix_sel_;

      mass_matrix_sel_.setZero();
      for (int i=0; i<12; i++)
      {
        mass_matrix_sel_(i,i) = temp22(i,i);
      }


      if(thread_tick_ == 0)
        std::cout << "masssel" << mass_matrix_sel_ << endl;

      double spring_k = 2000.0;
      double damping_d = 100.0;
      double motor_k = 10.0;

      massSpringMotorModel(spring_k, damping_d, motor_k, mass_matrix_sel_, a_right_mat_, b_right_mat_, c_right_mat_);

      discreteModel(a_right_mat_, b_right_mat_, c_right_mat_, 0, 1.0/hz_, a_disc_, b_disc_, c_right_mat_, a_disc_total_, b_disc_total_);



      q_mat.setIdentity();
      q_mat = q_mat*1.0;

      for (int i=0; i<12; i++)
      {
        q_mat(i,i) = 10.0;

      }

      r_mat.setIdentity();
      r_mat = r_mat * 1.0;



      riccatiGain(a_disc_total_, b_disc_total_, q_mat, r_mat, lqr_k);


      calc_update_flag_ = true;
      thread_tick_++;

      slowcalc_mutex_.lock();

      ad_right_.resize(a_disc_.rows(),a_disc_.cols());
      ad_right_ = a_disc_;
      bd_right_.resize(b_disc_.rows(),b_disc_.cols());
      bd_right_ = b_disc_;
      ad_total_right_.resize(a_disc_total_.rows(),a_disc_total_.cols());
      ad_total_right_ = a_disc_total_;
      bd_total_right_.resize(b_disc_total_.rows(),b_disc_total_.cols());
      bd_total_right_ = b_disc_total_;
      kkk_motor_right_.resize(lqr_k.rows(),lqr_k.cols());
      kkk_motor_right_ = lqr_k;
      slowcalc_mutex_.unlock();
    }

    this_thread::sleep_for(chrono::milliseconds(100));

  }

}



void WalkingController::getEstimationInputMatrix()
{
  double mass_total = 51.315;

  if(walking_tick_ == 0)
    solve();

  for (int i = 0; i < 6; i++)
    x_estimation_(i) = vars.x[i];





  Eigen::Vector2d FT_xy;
  
  zmp_r_(0) = (-r_ft_(4) - r_ft_(0)*0.0062) / r_ft_(2);
  zmp_r_(1) = (r_ft_(3) - r_ft_(1)*0.0062) / r_ft_(2);
  zmp_l_(0) = (-l_ft_(4) - l_ft_(0)*0.0062) / l_ft_(2);
  zmp_l_(1) = (l_ft_(3) - l_ft_(1)*0.0062) / l_ft_(2);




  if (foot_step_(current_step_num_,6)==1) //left foot support
  {
    if ( (walking_tick_ > t_start_ + t_rest_init_+ 2*t_double1_ && walking_tick_ < t_start_+t_total_ - t_rest_last_ - 2*t_double2_)) //ssp
    {

      //std::cout<<"left_ssp"<<endl;
      zmp_measured_ = zmp_l_;
      //FT_xy = l_ft_.topRows(2);
    }
    else //dsp
    {
      //std::cout<<"left_dsp"<<endl;
      zmp_measured_(0) = ((((rfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_r_)(0) + (rfoot_support_current_.translation())(0))*r_ft_(2) + zmp_l_(0)*l_ft_(2)) / (r_ft_(2) + l_ft_(2));
      zmp_measured_(1) = ((((rfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_r_)(1) + (rfoot_support_current_.translation())(1))*r_ft_(2) + zmp_l_(1)*l_ft_(2)) / (r_ft_(2) + l_ft_(2));
      //FT_xy = r_ft_.topRows(2)+l_ft_.topRows(2);
    }

    //COM_m = COM_m_l;
    //com_dot_measured_ = (com_measured_l_ - com_old_measured_l_)*hz_;
    //COM_dot_m = J_COM_PSEM*PseudoInverse((L_foot_P[6].rotation().transpose())*(J_LL[6].topLeftCorner<3,6>()), 0)*_Gyro_LFoot_Rot.transpose()*(_Position_Base-_Position_Base_old)*Hz;
    //COM_dot_m = _Gyro_RFoot_Rot.transpose()*(_Position_Base - _Position_Base_old)*Hz;

  }
  else
  {
    if ( (walking_tick_ > t_start_ + t_rest_init_+ 2*t_double1_ && walking_tick_ < t_start_+t_total_ - t_rest_last_ - 2*t_double2_) ) //ssp
    {
      //std::cout<<"right_ssp"<<endl;
      zmp_measured_ = zmp_r_;
      //FT_xy = r_ft_.topRows(2);
    }
    else //dsp
    {
      //std::cout<<"right_dsp"<<endl;
      zmp_measured_(0) = ((((lfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_l_)(0) + (lfoot_support_current_.translation())(0))*l_ft_(2) + zmp_r_(0)*r_ft_(2)) / (r_ft_(2) + l_ft_(2));
      zmp_measured_(1) = ((((lfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_l_)(1) + (lfoot_support_current_.translation())(1))*l_ft_(2) + zmp_r_(1)*r_ft_(2)) / (r_ft_(2) + l_ft_(2));
      //FT_xy = r_ft_.topRows(2) + l_ft_.topRows(2);
    }
    //COM_m = COM_m_r;
    //com_dot_measured_ = (com_measured_r_ - com_old_measured_r_)*hz_;
    //COM_dot_m = J_COM_PSEM*PseudoInverse((R_foot_P[6].rotation().transpose())*(J_RL[6].topLeftCorner<3, 6>()), 0)*_Gyro_RFoot_Rot.transpose()*(_Position_Base - _Position_Base_old)*Hz;
    //COM_dot_m = _Gyro_LFoot_Rot.transpose()*(_Position_Base - _Position_Base_old)*Hz;

  }
  FT_xy = r_ft_.topRows(2) + l_ft_.topRows(2);


  if(walking_tick_ == 0)
  {
    com_support_dot_current_.setZero();
    com_support_dot_old_estimation_.setZero();
    com_support_old_estimation_(0) = com_support_current_(0);
    com_support_old_estimation_(1) = com_support_current_(0);
    zmp_old_estimation_ = zmp_measured_;
  }
  else if(walking_tick_ == t_start_ &&  current_step_num_ != 0)
  {
    Eigen::Vector3d com_se_support_old;
    Eigen::Vector3d com_dot_se_support_old;
    Eigen::Vector3d zmp_se_support_old;

    com_se_support_old.setZero();
    com_dot_se_support_old.setZero();
    zmp_se_support_old.setZero();
    com_dot_se_support_old(0) = vars.x[0];
    com_dot_se_support_old(1) = vars.x[1];
    com_se_support_old(0) = vars.x[2];
    com_se_support_old(1) = vars.x[3];
    zmp_se_support_old(0) = vars.x[4];
    zmp_se_support_old(1) = vars.x[5];

    if(foot_step_(current_step_num_, 6) == 0)  //right foot support
    {

      //com_support_old_.col(1) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, com_support_old_.col(1));

      com_support_dot_old_estimation_(0) = (lfoot_support_current_.linear()*com_dot_se_support_old)(0);
      com_support_dot_old_estimation_(1) = (lfoot_support_current_.linear()*com_dot_se_support_old)(1);
      com_support_old_estimation_(0) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, com_se_support_old)(0);
      com_support_old_estimation_(1) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, com_se_support_old)(1);
      zmp_old_estimation_(0) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, zmp_se_support_old)(0);
      zmp_old_estimation_(1) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, zmp_se_support_old)(1);
    }
    else if(foot_step_(current_step_num_, 6) == 1)
    {
      //com_support_old_.col(1) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_,com_support_old_.col(1));

      com_support_dot_old_estimation_(0) = (rfoot_support_current_.linear()*com_dot_se_support_old)(0);
      com_support_dot_old_estimation_(1) = (rfoot_support_current_.linear()*com_dot_se_support_old)(1);
      com_support_old_estimation_(0) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, com_se_support_old)(0);
      com_support_old_estimation_(1) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, com_se_support_old)(1);
      zmp_old_estimation_(0) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, zmp_se_support_old)(0);
      zmp_old_estimation_(1) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, zmp_se_support_old)(1);
    }

  }
  else if(walking_tick_ != 0)
  {
    //if(walking_tick_ < 10)
    //{
    //  com_support_dot_current_.setZero();
    //}
    //com_support_dot_current_ = (-2*com_support_old_.col(3)+9*com_support_old_.col(2)-18*com_support_old_.col(1)+11*com_support_old_.col(0))*hz_/6;
    com_support_dot_current_ =(com_support_old_.col(0)-com_support_old_.col(1))*hz_;
    com_support_dot_old_estimation_(0) = vars.x[0];
    com_support_dot_old_estimation_(1) = vars.x[1];
    com_support_old_estimation_(0) = vars.x[2];
    com_support_old_estimation_(1) = vars.x[3];
    zmp_old_estimation_(0) = vars.x[4];
    zmp_old_estimation_(1) = vars.x[5];
  }



  /*std::cout<<"r_ft: "<<r_ft_<<endl;
  std::cout<<"l_ft: "<<l_ft_<<endl;

  std::cout<<"zmp_r_"<<zmp_r_<<endl;
  std::cout<<"zmp_l_"<<zmp_l_<<endl;
  */

  ///////////////////////////////////////Make Matirx//////////////////////////////
  a_kin_.setZero();
  a_kin_.block<2, 2>(0, 0).setIdentity();
  a_kin_(0, 2) = -GRAVITY / (zc_*hz_) ;
  a_kin_(1, 3) = -GRAVITY / (zc_*hz_);
  a_kin_(0, 4) = GRAVITY / (zc_*hz_);
  a_kin_(1, 5) = GRAVITY / (zc_*hz_);
  b_kin_ = com_support_dot_old_estimation_;


  a_c_dot_.setZero();
  a_c_dot_.block<2, 2>(0, 0).setIdentity();
  b_c_dot_ = com_support_dot_current_.topRows(2);

  /*
  A_c_dot.setZero();
  A_c_dot(0, 2) = -(Hz);
  A_c_dot(1, 3) = -(Hz);
  A_c_dot.block<2, 2>(0, 0).setIdentity();
  b_c_dot = -COM_old_m.segment<2>(0)*Hz;
  */

  a_c_.setZero();
  a_c_.block<2, 2>(0, 2).setIdentity();
  b_c_ = com_support_current_.topRows(2);
  /*
  A_c.setZero();
  A_c(0, 2) = -(Hz);
  A_c(1, 3) = -(Hz);
  A_c.block<2, 2>(0, 0).setIdentity();
  b_c= -COM_old_m.segment<2>(0)*Hz;
  */

  a_zmp_.setZero();
  a_zmp_.block<2, 2>(0, 4).setIdentity();
  b_zmp_ = zmp_measured_;

  a_c_c_dot_.setZero();
  a_c_c_dot_.block<2, 2>(0, 2).setIdentity();
  a_c_c_dot_(0, 0) = -1 / hz_;
  a_c_c_dot_(1, 1) = -1 / hz_;
  b_c_c_dot_ = com_support_old_estimation_;

  a_f_.setZero();
  a_f_(0, 2) = mass_total*GRAVITY / (zc_);
  a_f_(1, 3) = mass_total*GRAVITY / (zc_);
  a_f_(0, 4) = -mass_total*GRAVITY / (zc_);
  a_f_(1, 5) = -mass_total*GRAVITY / (zc_);
  b_f_ = FT_xy;

  a_noise_.setIdentity();

  if(walking_tick_ == 0 )
  {
    b_noise_(0) = com_support_dot_current_(0);
    b_noise_(1) = com_support_dot_current_(1);
    b_noise_(2) = com_support_current_(0);
    b_noise_(3) = com_support_current_(1);
    b_noise_(4) = zmp_measured_(0);
    b_noise_(5) = zmp_measured_(1);
  }
  else
  {
    b_noise_(0) = com_support_dot_old_estimation_(0);
    b_noise_(1) = com_support_dot_old_estimation_(1);
    b_noise_(2) = com_support_old_estimation_(0);
    b_noise_(3) = com_support_old_estimation_(1);
    b_noise_(4) = zmp_old_estimation_(0);
    b_noise_(5) = zmp_old_estimation_(1);
  }


  /////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix<double, 10, 1> coeff;
  coeff.setIdentity();
  /*
  coeff(0) = 5e2;		//kinematic
  coeff(1) = 1e3;		//c_dot_x
  coeff(2) = 1e3;		//c_dot_y
  coeff(3) = 1e1;		//c
  coeff(4) = 1e4;		//zmp
  coeff(5) = 8e1;		//approximation
  coeff(6) = 1e-3;	//FT-accer
  coeff(7) = 7e3;	    //noise of c_dot
  coeff(8) = 1.5e1;	    //noise of c
  coeff(9) = 5e4;	    //noise of zmp
  */


  coeff(0) = 1e3;		//kinematic
  coeff(1) = 1e2;		//c_dot_x
  coeff(2) = 1e2;		//c_dot_y
  coeff(3) = 3e2;		//c
  coeff(4) = 9e3;		//zmp
  coeff(5) = 6e2;		//approximation
  coeff(6) = 2e-1;	//FT-accer
  coeff(7) = 0e2;	    //noise of c_dot
  coeff(8) = 4e0;	    //noise of c
  coeff(9) = 2e4;	    //noise of zmp


  /* for simluation*/
  /*
  coeff(0) = 1e2;		//kinematic
  coeff(1) = 3e1;		//c_dot_x
  coeff(2) = 3e1;		//c_dot_y
  coeff(3) = 7e2;		//c
  coeff(4) = 8e3;		//zmp
  coeff(5) = 6e2;		//approximation
  coeff(6) = 2e0;	//FT-accer
  coeff(7) = 1e2;	    //noise of c_dot
  coeff(8) = 0e0;	    //noise of c
  coeff(9) = 3e4;	    //noise of zmp
  */

  /*
  coeff(0) = 1e0;		//kinematic
  coeff(1) = 1e0;		//c_dot_x
  coeff(2) = 1e0;		//c_dot_y
  coeff(3) = 1e0;		//c
  coeff(4) = 1e0;		//zmp
  coeff(5) = 1e0;		//approximation
  coeff(6) = 1e0;	//FT-accer
  coeff(7) = 1e0;	    //noise of c_dot
  coeff(8) = 1e0;	    //noise of c
  coeff(9) = 1e0;	    //noise of zmp
  */


  /*
  coeff(0) = 0e0;		//kinematic
  coeff(1) = 1e0;		//c_dot_x
  coeff(2) = 1e0;		//c_dot_y
  coeff(3) = 3e0;		//c
  coeff(4) = 9e0;		//zmp
  coeff(5) = 1e0;		//approximation
  coeff(6) = 0e0;	//FT-accer
  coeff(7) = 0e-1;	    //noise of c_dot
  coeff(8) = 0e-1;	    //noise of c
  coeff(9) = 0e0;	    //noise of zmp
  */


  a_total_.block<2, 6>(0, 0) = coeff(0)*a_kin_;
  a_total_.block<1, 6>(2, 0) = coeff(1)*a_c_dot_.block<1, 6>(0, 0);
  a_total_.block<1, 6>(3, 0) = coeff(2)*a_c_dot_.block<1, 6>(1, 0);
  a_total_.block<2, 6>(4, 0) = coeff(3)*a_c_;
  a_total_.block<2, 6>(6, 0) = coeff(4)*a_zmp_;
  a_total_.block<2, 6>(8, 0) = coeff(5)*a_c_c_dot_;
  a_total_.block<2, 6>(10, 0) = coeff(6)*a_f_;
  a_total_.block<2, 6>(12, 0) = coeff(7)*a_noise_.block<2, 6>(0, 0);
  a_total_.block<2, 6>(14, 0) = coeff(8)*a_noise_.block<2, 6>(2, 0);
  a_total_.block<2, 6>(16, 0) = coeff(9)*a_noise_.block<2, 6>(4, 0);

  b_total_.segment<2>(0) = coeff(0)*b_kin_;
  b_total_(2) = coeff(1)*b_c_dot_(0);
  b_total_(3) = coeff(2)*b_c_dot_(1);
  b_total_.segment<2>(4) = coeff(3)*b_c_;
  b_total_.segment<2>(6) = coeff(4)*b_zmp_;
  b_total_.segment<2>(8) = coeff(5)*b_c_c_dot_;
  b_total_.segment<2>(10) = coeff(6)*b_f_;
  b_total_.segment<2>(12) = coeff(7)*b_noise_.segment<2>(0);
  b_total_.segment<2>(14) = coeff(8)*b_noise_.segment<2>(2);
  b_total_.segment<2>(16) = coeff(9)*b_noise_.segment<2>(4);

  /*
  for (int i = 0; i < 6; i++)
      for (int j = 0; j < 12; j++)
      {
          params.A[j + i * 12] = A_total(j, i);
      }

  for (int j = 0; j < 12; j++)
      params.b[j] = b_total(j);
  */
  Eigen::Matrix<double, 6, 6> q;

  q = a_total_.transpose()*a_total_;

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
    {
      params.Q[j + i * 6] = q(j, i);
    }

  for (int i = 0; i < 6; i++)
    params.c[i] = -2*(b_total_.transpose()*a_total_)(i);

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 8; j++)
    {
      params.Ai[j + i * 8] = 0;
    }


  params.Ai[20] = 1;
  params.Ai[22] = -1;
  params.Ai[29] = 1;
  params.Ai[31] = -1;
  params.Ai[32] = 1;
  params.Ai[34] = -1;
  params.Ai[41] = 1;
  params.Ai[43] = -1;


  //if (walking_tick_ > t_start_ + t_rest_init_+ 2*t_double1_ && walking_tick_ < t_start_+t_total_ - t_rest_last_ - 2*t_double2_)
  //{
  //  params.bi[0] = 0.15;  //zmp
  //  params.bi[1] = 0.0825;
  //  params.bi[2] = 0.15;
  //  params.bi[3] = 0.0825;
  //  params.bi[4] = 0.15;   //com
  //  params.bi[5] = 0.0825;
  //  params.bi[6] = 0.15;
  //  params.bi[7] = 0.0825;
  //}
  //else
  //{
  //
  //  params.bi[0] = 0.25;
  //  params.bi[1] = 0.3;
  //  params.bi[2] = 0.25;
  //  params.bi[3] = 0.3;
  //  params.bi[4] = 0.5;
  //  params.bi[5] = 0.5;
  //  params.bi[6] = 0.5;
  //  params.bi[7] = 0.5;
  //}

  //params.bi[0] = 0.15;     //zmp x <= 0.15
  //params.bi[1] = 0.0825;   //zmp y <= 0.21
  //params.bi[2] = 0.15;     //zmp x >= -0.15
  //params.bi[3] = 0.3293;   //zmp y >= -0.21
  //params.bi[4] = 0.15;     //com x <= 0.15
  //params.bi[5] = 0.0825;   //com y <= 0.21
  //params.bi[6] = 0.15;     //com x >= -0.15
  //params.bi[7] = 0.3293;   //com y >= -0.21

  params.bi[0] = 0.9;
  params.bi[1] = 0.9;
  params.bi[2] = 0.9;
  params.bi[3] = 0.9;
  params.bi[4] = 0.9;
  params.bi[5] = 0.9;
  params.bi[6] = 0.9;
  params.bi[7] = 0.9;

  /*
  for (int i = 0; i < 6; i++)
      for (int j = 0; j < 2; j++)
          params.Ae[j + 2 * i] = 0;

  for (int i = 0; i < 8; i++)
      params.be[i] = 0;

  params.Ae[0] = 1;
  params.Ae[3] = 1;
  params.Ae[4] = -Hz;
  params.Ae[7] = -Hz;

  params.be[0] = -COM_dot_old_SE(0)*Hz;
  params.be[1] = -COM_dot_old_SE(1)*Hz;



  */

  if(walking_tick_ ==0)
  {
    //std::cout << "a_total: \n" << a_total_ << endl;
    //std::cout << "a_kin: \n" << a_kin_ << endl;
    //std::cout << "a_c_dot: \n" << a_c_dot_ << endl;
    //std::cout << "a_c: \n" << a_c_ << endl;
    //std::cout << "a_zmp: \n" << a_zmp_ << endl;
    //std::cout << "a_c_c_dot: \n" << a_c_c_dot_ << endl;
    //std::cout << "a_f: \n" << a_f_ << endl;
    //std::cout << "b_total: \n" << b_total_ << endl;
    //std::cout << "b_kin: \n" << b_kin_ << endl;
    //std::cout << "b_c_dot: \n" << b_c_dot_ << endl;
    //std::cout << "b_c: \n" << b_c_ << endl;
    //std::cout << "b_zmp: \n" << b_zmp_ << endl;
    //std::cout << "b_c_c_dot: \n" << b_c_c_dot_ << endl;
    //std::cout << "b_f: \n" << b_f_ << endl;
  }


}

}


