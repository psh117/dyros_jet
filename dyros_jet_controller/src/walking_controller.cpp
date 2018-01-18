#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller.h"

namespace dyros_jet_controller
{



void WalkingController::compute(VectorQd* desired_q)
{
  if(walking_enable_)
  {

    updateInitialState();
  /*getFootStep();
    getComTrajectory();
    getZmpTrajectory();
  */

    //////compute//////////////////
    if (ik_mode_ == 0)
    {
     // computeIkControl(desired_leg_q_);
      for(int i=0; i<6; i++)
      {
        desired_q_(i) = desired_leg_q_(i);
        desired_q_(i+6) = desired_leg_q_(i+6);
      }
    }
    else if (ik_mode_ == 1)
    {
//      computeJacobianControl(desired_leg_q_dot_);
      for(int i=0; i<6; i++)
      {
//        if(_cnt == 0){
/*          desired_q_(i) = _init_q_(i);
          desired_q_(i+6) = _init_q_(i+6);*/
        }
 /*       desired_q_(i) = desired_leg_q_dot_(i)/hz_+_real_q(i);
        desired_q_(i+6) = desired_leg_q_dot_(i+6)/hz_+_real_q(i+6);*/ //_cnt, real_q_와 _init_q_에 대한 정의 필요
      }
    }
    ///////////////////////////////////
  //compensator();
  walking_tick_ ++;
}

void WalkingController::setTarget(int walk_mode, std::vector<bool> compensator_mode, int ik_mode, bool heel_toe,
                                  bool is_right_foot_swing, double x, double y, double z, double theta,
                                  double step_length_x, double step_length_y)
{

  target_x_ = x;
  target_y_ = y;
  target_z_ = z;
  target_theta_ = theta;
  step_length_x_ = step_length_x;
  step_length_y_ = step_length_y;
  ik_mode_ = ik_mode;
  walk_mode_ = walk_mode;
  compensator_mode_ = compensator_mode;
  heel_toe_mode_ = heel_toe;
  is_right_foot_swing_ = is_right_foot_swing;

  parameterSetting();
}


void WalkingController::setEnable(bool enable)
{
  walking_enable_=enable;
}

void WalkingController::updateControlMask(unsigned int *mask)
{
  if(walking_enable_)
  {
    for (int i=0; i<total_dof_-4; i++)
    {
      mask[i] = (mask[i] | PRIORITY);
    }
    mask[total_dof_-1] = (mask[total_dof_-1] & ~PRIORITY); //Gripper
    mask[total_dof_-2] = (mask[total_dof_-2] & ~PRIORITY); //Gripper
    mask[total_dof_-3] = (mask[total_dof_-2] & ~PRIORITY); //Head
    mask[total_dof_-4] = (mask[total_dof_-2] & ~PRIORITY); //Head
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
  t_last_ = 0.1*hz_;
  t_start_= 0.1*hz_;
  t_temp_= 0.1*hz_;
  t_rest_init_ = 0.1*hz_;
  t_rest_last_= 0.1*hz_;
  t_double1_= 0.1*hz_;
  t_double2_= 0.1*hz_;
  t_total_= 1.3*hz_;
  t_temp_ = 3.0*hz_;

  t_start_real_ = t_start_ + t_rest_init_;

  foot_height_ = 0.05;
}

/**Foot step related fuctions
 */

void WalkingController::getFootStep()
{
  calculateFootStepTotal();

  total_step_num_ = foot_step_.col(1).size();

  floatToSupportFootstep();
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
  const double &dlength = step_length_x_; //footstep length;
  unsigned int middle_total_step_number = length_to_target/(dlength);
  double middle_residual_length = length_to_target-middle_total_step_number*(dlength);



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


  foot_step_.resize(number_of_foot_step,7);
  foot_step_.setZero();

  int index = 0;
  int temp = -1; //right foot will take off first

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

    if(temp==1)
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
    else if(temp==-1)
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


  int temp2 = -1;

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

    if(temp2==1)
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
    else if(temp2==-1)
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
/*
  cout << "middle total number" << middle_total_step_number << endl;
  cout << "middle residual length" << middle_residual_length << endl;
  cout << "total foot step1" << foot_step_ << endl;*/



  double final_position_x = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length);
  double final_position_y = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length);

  int temp3 = -1;

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
  int number_of_foot_step = 0;
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
  floatToSupportFootstep();

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

  if(current_step_num_ == 0)
  {
    if(foot_step_(current_step_num_,6) == 0) //right support
    {
      reference.translation() = rfoot_float_init_.translation();
      reference.translation()(2) = 0;
      reference.linear() = rfoot_float_init_.linear();
      reference.translation()(0) = 0.0;
    }
    else  //left support
    {
      reference.translation() = lfoot_float_init_.translation();
      reference.translation()(2) = 0;
      reference.linear() = lfoot_float_init_.linear();
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
  if(walking_tick_ ==0)
  {
    //model_.updateKinematics(current_q_);
    q_init_ = current_q_;
    lfoot_float_init_ = model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)0);
    rfoot_float_init_ = model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)1);
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
  }
  else if(walking_tick_ == t_start_)
  {

  }

}


void WalkingController::addZmpOffset()
{
  foot_step_support_frame_offset_ = foot_step_support_frame_;

  if(foot_step_(0,6) == 0) //left support foot
  {
    supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_;
    swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + rfoot_zmp_offset_;
  }
  else
  {
    supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
    swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + lfoot_zmp_offset_;
  }

  for(int i=0; i<total_step_num_; i++)
  {
    if(foot_step_(i,6) == 0)
    {
      foot_step_support_frame_offset_(i,1) += rfoot_zmp_offset_;
    }
    else
    {
      foot_step_support_frame_offset_(i,1) += lfoot_zmp_offset_;
    }
  }
}

void WalkingController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{
  ref_zmp_.resize(norm_size, 2);

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
    Kx2 = (foot_step_support_frame_(current_step_number,0)+supportfoot_support_init_(0))/2.0 - supportfoot_support_init_offset_(0);

    Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
    Ky2 = (foot_step_support_frame_(current_step_number,1)+supportfoot_support_init_(1))/2.0 - supportfoot_support_init_offset_(1);



    for(int i=0; i<t_total_; i++)
    {
      if(i < t_rest_init_)
      {
        temp_px(i) = 0.0;
        temp_py(i) = com_support_init_(1)+com_offset_(1);
      }
      else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
      {
        temp_py(i) = com_support_init_(1)+com_offset_(1) + Ky/t_double1_*(i+1-t_rest_init_);
        temp_px(i) = Kx/t_double1_*(i+1-t_rest_init_);
      }
      else if(i>= t_rest_init_+t_double1_ & i< t_total_-t_rest_last_-t_double2_)
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
  com_offset_.setZero();
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

  if (walking_tick_ == t_start_ && current_step_num_ != 0)
  {
    Eigen::Vector3d COB_vel_prev;
    Eigen::Vector3d COB_vel;
    Eigen::Vector3d COB_acc_prev;
    Eigen::Vector3d COB_acc;

    Eigen::Matrix3d temp;

    if(current_step_num_ == 1)
    {
      temp = DyrosMath::rotateWithZ(-supportfoot_float_init_(5));

      COB_vel_prev(0) = xs_(1);
      COB_vel_prev(1) = ys_(1);
      COB_vel_prev(2) = 0.0;
      COB_vel = temp*COB_vel_prev;

      COB_acc_prev(0) = xs_(2);
      COB_acc_prev(1) = ys_(2);
      COB_acc_prev(2) = 0.0;
      COB_acc = temp*COB_acc_prev;
    }
    else
    {
      temp = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_-1,5)); ////////////////�ٽ� �����غ��� ��ǥ��

      COB_vel_prev(0) = xs_(1);
      COB_vel_prev(1) = ys_(1);
      COB_vel_prev(2) = 0.0;
      COB_vel = temp*COB_vel_prev;

      COB_acc_prev(0) = xs_(2);
      COB_acc_prev(1) = ys_(2);
      COB_acc_prev(2) = 0.0;
      COB_acc = temp*COB_acc_prev;
    }

    xs_(1) = COB_vel(0);
    ys_(1) = COB_vel(1);
    xs_(2) = COB_acc(0);
    ys_(2) = COB_acc(1);
  }

  //COM �ġ ���
  if(com_update_flag_ == true)
  {
    if(com_control_mode_ == true)
    {
      xs_(0) = com_support_init_(0);//+xs_(1)*1.0/Hz;
      ys_(0) = com_support_init_(1);//+ys_(1)*1.0/Hz;
    }
    else
    {
      xs_(0) = pelv_support_init_.translation()(0)+ com_offset_(0);// + xs_(1)*1.0/Hz;
      ys_(0) = pelv_support_init_.translation()(1)+ com_offset_(1);// + ys_(1)*1.0/Hz;
    }
  }

  double x_err;
  double y_err;

  ////PreviewControl_basic(_cnt-_init_info.t, _K, Frequency2,  16*Hz2/10, 0.6802, _Gx, _Gi, _Gp_I, _A, _BBB, &_sum_px_err, &_sum_py_err, _px_ref, _py_ref, _xi, _yi, xs_, ys_, xd_, yd_, x_err, y_err);
  modifiedPreviewControl();
  xs_=xd_;
  ys_=yd_;

  //Reference ZMP �� ���� �ð�� ������� �������� ZMP�� �����ϱ� ������ start_time��� ���ߵ�.
  double start_time = 0;

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
    //com_desired_(2) = _init_info._COM_support_init(2);
    //com_desired_(2) = Cubic(_cnt,_T_Start,_T_Start+_T_Double1,_init_COM._COM(2),0.0,_init_info._COM_support_init(2),0.0);
    com_desired_(2) = pelv_support_init_.translation()(2);

    com_dot_desired_(0) = xd_(1);
    com_dot_desired_(1) = yd_(1);
    com_dot_desired_(2) = 0;

    double k= 100.0;
    p_ref_(0) = xd_(1)+k*(xd_(0)-com_support_cuurent_(0));
    p_ref_(1) = yd_(1)+k*(yd_(0)-com_support_cuurent_(1));
    p_ref_(2) = k*(com_desired_(2)-com_support_cuurent_(2));
    l_ref_.setZero();
  }
  else
  {
    com_desired_(0) = xd_(0);
    com_desired_(1) = yd_(0);
    com_desired_(2) = pelv_support_init_.translation()(2);

    com_dot_desired_(0) = xd_(1);
    com_dot_desired_(1) = yd_(1);
    com_dot_desired_(2) = 0;

    double k= 100.0;
    p_ref_(0) = xd_(1)+k*(xd_(0)-com_support_cuurent_(0));
    p_ref_(1) = yd_(1)+k*(yd_(0)-com_support_cuurent_(1));
    p_ref_(2) = k*(com_desired_(2)-com_support_cuurent_(2));
    l_ref_.setZero();
  }
}

void WalkingController::getPelvTrajectory()
{

}

void WalkingController::getFootTrajectory()
{
    Eigen::Vector6d target_swing_foot;

    for(int i=0; i<6; i++)
        target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i);


    if(walking_tick_ < t_start_real_+t_double1_)
    {
        lfoot_trajectory_current_.translation() = lfoot_trajectory_init_.translation();
        lfoot_trajectory_dot_current_.setZero();


        if(foot_step_(current_step_num_,6) == 1) //left foot support
            lfoot_trajectory_current_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_trajectory_init_.translation()(2),0.0,0.0,0.0);
        else // swing foot (right foot support)
        {
            if(current_step_num_ == 0)
                  lfoot_trajectory_current_.translation()(2) = lfoot_trajectory_init_.translation()(2);
            else
            {
                if (walking_tick_ < t_start_)
                    lfoot_trajectory_current_.translation()(2) = lfoot_trajectory_init_.translation()(2);
                else if(walking_tick_ >= t_start_ &&walking_tick_ < t_start_real_)
                    lfoot_trajectory_current_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_trajectory_init_.translation()(2),0.0,0.0,0.0);
                else
                    lfoot_trajectory_current_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_,t_start_real_+t_double1_,0.0,0.0,0.0,0.0);
            }
        }


        lfoot_trajectory_euler_current_ = lfoot_trajectory_euler_init_;

        for(int i=0; i<2; i++)
            lfoot_trajectory_euler_current_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_trajectory_euler_init_(i),0.0,0.0,0.0);

        lfoot_trajectory_current_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_current_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_current_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_current_(0));


        rfoot_trajectory_current_.translation() = rfoot_trajectory_init_.translation();
      rfoot_trajectory_dot_current_.setZero();
        //rfoot_trajectory_current_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,_T_Start_real,rfoot_trajectory_init_.translation()(2),0.0,0.0,0.0);


        if(foot_step_(current_step_num_,6) == 0) //right foot support
            rfoot_trajectory_current_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_trajectory_init_.translation()(2),0.0,0.0,0.0);
        else // swing foot (left foot support)
        {
            if(current_step_num_ == 0)
                 rfoot_trajectory_current_.translation()(2) = rfoot_trajectory_init_.translation()(2);
            else
            {
                if(walking_tick_ < t_start_)
                    rfoot_trajectory_current_.translation()(2) = rfoot_trajectory_init_.translation()(2);
                else if(walking_tick_ >= t_start_ && walking_tick_ < t_start_real_)
                    rfoot_trajectory_current_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_trajectory_init_.translation()(2),0.0,0.0,0.0);
                else
                    rfoot_trajectory_current_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_,t_start_real_+t_double1_,0.0,0.0,0.0,0.0);
            }
        }


        rfoot_trajectory_euler_current_ = rfoot_trajectory_euler_init_;
        for(int i=0; i<2; i++)
            rfoot_trajectory_euler_current_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_trajectory_euler_init_(i),0.0,0.0,0.0);

        rfoot_trajectory_current_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_current_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_current_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_current_(0));

    }
    else if(walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_)
    {
        double t_rest_temp = 0.05*hz_;
        double ankle_temp;
        ankle_temp = 0*DEG2RAD;

        if(foot_step_(current_step_num_,6) == 1) //Left foot support : Left foot is fixed at initial values, and Right foot is set to go target position
        {
            lfoot_trajectory_current_.translation() = lfoot_trajectory_init_.translation();
            lfoot_trajectory_euler_current_ = lfoot_trajectory_euler_init_;
            lfoot_trajectory_euler_current_.setZero();

            lfoot_trajectory_dot_current_.setZero();
            lfoot_trajectory_current_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_current_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_current_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_current_(0));

            // setting for Left supporting foot

            if(walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0) // the period for lifting the right foot
            {

                rfoot_trajectory_current_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,0.0,foot_height_,0.0);
                rfoot_trajectory_dot_current_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,0.0,foot_height_,0.0,hz_);

                rfoot_trajectory_euler_current_(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,0.0,ankle_temp,0.0);
                rfoot_trajectory_dot_current_(4) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,0.0,ankle_temp,0.0,hz_);
            } // the period for lifting the right foot

            else // the period for putting the right foot
            {
                rfoot_trajectory_euler_current_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0);
                rfoot_trajectory_dot_current_(4) = DyrosMath::cubicDot(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0,hz_);

                rfoot_trajectory_current_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,0.0,target_swing_foot(2),0.0);
                rfoot_trajectory_dot_current_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,0.0,target_swing_foot(2),0.0,hz_);
            }  // the period for putting the right foot

            for(int i=0; i<2; i++)
            {
                rfoot_trajectory_euler_current_(0) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,0.0,target_swing_foot(0+3),0.0);
                rfoot_trajectory_dot_current_(0+3) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,0.0,target_swing_foot(0+3),0.0,hz_);

                rfoot_trajectory_current_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp+0.05*hz_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp-0.05*hz_,rfoot_trajectory_init_.translation()(i),0.0,target_swing_foot(i),0.0);
                rfoot_trajectory_dot_current_(i) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,rfoot_trajectory_init_.translation()(i),0.0,target_swing_foot(i),0.0,hz_);
            }

            rfoot_trajectory_euler_current_(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,rfoot_trajectory_euler_init_(2),0.0,target_swing_foot(5),0.0);
            rfoot_trajectory_dot_current_(5) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,rfoot_trajectory_euler_init_(2),0.0,target_swing_foot(5),0.0,hz_);

            rfoot_trajectory_current_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_current_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_current_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_current_(0));
        }
        else if(foot_step_(current_step_num_,6) == 0) // Right foot support : Right foot is fixed at initial values, and Left foot is set to go target position
        {
            rfoot_trajectory_current_.translation() = rfoot_trajectory_init_.translation();
          rfoot_trajectory_current_.translation()(2) = 0.0;
           //rfoot_trajectory_current_.linear() = rfoot_trajectory_init_.linear();
            rfoot_trajectory_euler_current_ = rfoot_trajectory_euler_init_;
            rfoot_trajectory_euler_current_(0) = 0.0;
          rfoot_trajectory_euler_current_(1) = 0.0;
            rfoot_trajectory_dot_current_.setZero();

            double ankle_temp;
            ankle_temp = 0*DEG2RAD;
            //ankle_temp = -15*DEG2RAD;

            rfoot_trajectory_current_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_current_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_current_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_current_(0));

            if(walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0)
            {

                lfoot_trajectory_current_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,0.0,foot_height_,0.0);
                lfoot_trajectory_dot_current_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,0.0,foot_height_,0.0,hz_);

                lfoot_trajectory_euler_current_(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,0.0,ankle_temp,0.0);
                lfoot_trajectory_dot_current_(4) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,0.0,ankle_temp,0.0,hz_);

            }
            else
            {
                lfoot_trajectory_euler_current_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0);
                lfoot_trajectory_dot_current_(4) = DyrosMath::cubicDot(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0,hz_);


                lfoot_trajectory_current_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,0.0,target_swing_foot(2),0.0);
                lfoot_trajectory_dot_current_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,foot_height_-t_rest_temp,0.0,target_swing_foot(2),0.0,hz_);
            }

            for(int i=0; i<2; i++)
            {
                lfoot_trajectory_euler_current_(0) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,0.0,target_swing_foot(0+3),0.0);
                lfoot_trajectory_dot_current_(0+3) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,0.0,target_swing_foot(0+3),0.0,hz_);


                lfoot_trajectory_current_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp+0.05*hz_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp-0.05*hz_,lfoot_trajectory_init_.translation()(i),0.0,target_swing_foot(i),0.0);
                lfoot_trajectory_dot_current_(i) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp-0.05*hz_,lfoot_trajectory_init_.translation()(i),0.0,target_swing_foot(i),0.0,hz_);
            }

          //  for(int i=0; i<3; i++)
          //  {
          //      lfoot_trajectory_euler_current_(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,_T_Start+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,0.0,target_swing_foot(i+3),0.0);
          //      lfoot_trajectory_dot_current_(i+3) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,_T_Start+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,0.0,target_swing_foot(i+3),0.0,hz_);
          //  }


                 lfoot_trajectory_euler_current_(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,lfoot_trajectory_euler_init_(2),0.0,target_swing_foot(5),0.0);
                lfoot_trajectory_dot_current_(5) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,lfoot_trajectory_euler_init_(2),0.0,target_swing_foot(5),0.0,hz_);

            lfoot_trajectory_current_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_current_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_current_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_current_(0));
        }
        else
        {
            lfoot_trajectory_current_.translation() = lfoot_trajectory_init_.translation();
            lfoot_trajectory_current_.linear() = lfoot_trajectory_init_.linear();
            lfoot_trajectory_euler_current_ = lfoot_trajectory_euler_init_;
            lfoot_trajectory_dot_current_.setZero();

            rfoot_trajectory_current_.translation() = rfoot_trajectory_init_.translation();
            rfoot_trajectory_current_.linear() = rfoot_trajectory_init_.linear();
            rfoot_trajectory_euler_current_ = rfoot_trajectory_euler_init_;
            rfoot_trajectory_dot_current_.setZero();
        }
    }
    else
    {
        if(foot_step_(current_step_num_,6) == 1) // �޹� ����
        {
            lfoot_trajectory_current_.translation() = lfoot_trajectory_init_.translation();
            lfoot_trajectory_current_.translation()(2) = 0.0;
            lfoot_trajectory_euler_current_ = lfoot_trajectory_euler_init_;
            lfoot_trajectory_euler_current_(0) = 0.0;
            lfoot_trajectory_euler_current_(1) = 0.0;
            lfoot_trajectory_current_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_current_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_current_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_current_(0));
            lfoot_trajectory_dot_current_.setZero();

            for(int i=0; i<3; i++)
            {
                rfoot_trajectory_current_.translation()(i) = target_swing_foot(i);
                rfoot_trajectory_euler_current_(i) = target_swing_foot(i+3);
            }
            rfoot_trajectory_dot_current_.setZero();

            rfoot_trajectory_current_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_current_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_current_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_current_(0));
        }
        else if (foot_step_(current_step_num_,6) == 0)
        {
            rfoot_trajectory_current_.translation() = rfoot_trajectory_init_.translation();
            rfoot_trajectory_current_.translation()(2) = 0.0;
            //rfoot_trajectory_current_.linear() = rfoot_trajectory_init_.linear();
            rfoot_trajectory_euler_current_ = rfoot_trajectory_euler_init_;
            rfoot_trajectory_euler_current_(0) = 0.0;
            rfoot_trajectory_euler_current_(1) = 0.0;
            rfoot_trajectory_dot_current_.setZero();

            rfoot_trajectory_current_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_current_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_current_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_current_(0));


            for(int i=0; i<3; i++)
            {
                lfoot_trajectory_current_.translation()(i) = target_swing_foot(i);
                lfoot_trajectory_euler_current_(i) = target_swing_foot(i+3);
            }
            lfoot_trajectory_dot_current_.setZero();
            lfoot_trajectory_current_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_current_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_current_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_current_(0));
        }
        else
        {
            lfoot_trajectory_current_.translation() = lfoot_trajectory_init_.translation();
            lfoot_trajectory_current_.linear() = lfoot_trajectory_init_.linear();
            lfoot_trajectory_euler_current_ = lfoot_trajectory_euler_init_;
            lfoot_trajectory_dot_current_.setZero();

            rfoot_trajectory_current_.translation() = rfoot_trajectory_init_.translation();
            rfoot_trajectory_current_.linear() = rfoot_trajectory_init_.linear();
            rfoot_trajectory_euler_current_ = rfoot_trajectory_euler_init_;
            rfoot_trajectory_dot_current_.setZero();
        }
    }
}

void WalkingController::computeIkControl(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::VectorLXd& desired_leg_q)
{
 /* for (int i=2; i<4; i++)
  {
    currnet_leg_transform_[i-2]=model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)i);
  }
  currnet_leg_transform_l_=currnet_leg_transform_[0];
  currnet_leg_transform_r_=currnet_leg_transform_[1];*/

  Eigen::Vector3d lp, rp;
  //Should revise by dg, Trunk_trajectory_global.translation()
  lp = float_lleg_transform.linear().transpose()*(float_trunk_transform.translation()-float_lleg_transform.translation());
  rp = float_rleg_transform.linear().transpose()*(float_trunk_transform.translation()-float_rleg_transform.translation());

  Eigen::Matrix3d trunk_lleg_rotation,trunk_rleg_rotation;
  trunk_lleg_rotation = float_trunk_transform.linear().transpose()*float_lleg_transform.linear();
  trunk_rleg_rotation = float_trunk_transform.linear().transpose()*float_rleg_transform.linear();

  Eigen::Vector3d ld, rd;
  ld.setZero(); rd.setZero();
  ld(1) = 0.105;
  ld(2) = -0.1829;
  rd(1) = -0.105;
  rd(2) = -0.1829;
  ld = trunk_lleg_rotation.transpose() * ld;
  rd = trunk_rleg_rotation.transpose() * rd;

  Eigen::Vector3d lr, rr;
  lr = lp + ld;
  rr = rp + rd;

  double l_upper = 0.3729; // direct length from hip to knee
  double l_lower = 0.3728; //direct length from knee to ankle

  double offset_hip_pitch = 24.6271*DEG2RAD;
  double offset_knee_pitch = 15.3655*DEG2RAD;
  double offset_ankle_pitch = 9.2602*DEG2RAD;

  //////////////////////////// LEFT LEG INVERSE KINEMATICS ////////////////////////////

  double lc = lr.norm();
  desired_leg_q(3) = (- acos((l_upper*l_upper + l_lower*l_lower - lc*lc) / (2*l_upper*l_lower))+ 3.141592); // - offset_knee_pitch //+ alpha_lower

  double l_ankle_pitch = asin((l_upper*sin(3.141592-desired_leg_q(3)))/lc);
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
  desired_leg_q(2) = -asin(r_tl2(2,0)/cos(desired_leg_q(7)))+offset_hip_pitch;
  desired_leg_q(3) = desired_leg_q(9)- offset_knee_pitch;
  desired_leg_q(4) = desired_leg_q(10)- offset_ankle_pitch;

  //////////////////////////// RIGHT LEG INVERSE KINEMATICS ////////////////////////////

  double rc = rr.norm();
  desired_leg_q(9) = (- acos((l_upper*l_upper + l_lower*l_lower - rc*rc) / (2*l_upper*l_lower))+ 3.141592); // - offset_knee_pitch //+ alpha_lower

  double r_ankle_pitch = asin((l_upper*sin(3.141592-desired_leg_q(3)))/rc);
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
  desired_leg_q(8) = -asin(r_tr2(2,0)/cos(desired_leg_q(7)))+offset_hip_pitch;
  desired_leg_q(9) = desired_leg_q(9)- offset_knee_pitch;
  desired_leg_q(10) = desired_leg_q(10)- offset_ankle_pitch;

}


void WalkingController::computeJacobianControl(Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::VectorLXd& desired_leg_q_dot)
{
  for (int i=0; i<2; i++)
  {
    current_leg_jacobian_[i] = model_.getLegJacobian((DyrosJetModel::EndEffector) i);
  }
  current_leg_jacobian_l_=current_leg_jacobian_[0];
  current_leg_jacobian_r_=current_leg_jacobian_[1];

  Eigen::Matrix6d jacobian_temp_l, jacobian_temp_r, current_leg_jacobian_l_inv, current_leg_jacobian_r_inv,
      J_Damped;
  double wl, wr, w0, lambda, a;
  w0 = 0.001;
  lambda = 0.05;
  jacobian_temp_l=current_leg_jacobian_l_*current_leg_jacobian_l_.transpose();
  jacobian_temp_r=current_leg_jacobian_r_*current_leg_jacobian_r_.transpose();
  wr = sqrt(jacobian_temp_l.determinant());
  wl = sqrt(jacobian_temp_r.determinant());

  if (wr<=w0)
  { //Right Jacobi
    a = lambda * pow(1-wr/w0,2);
    J_Damped = current_leg_jacobian_r_*current_leg_jacobian_r_.transpose()+a*Eigen::Matrix6d::Identity();
    J_Damped = J_Damped.inverse();

    cout << "Singularity Region of right leg: " << wr << endl;
    current_leg_jacobian_r_inv = current_leg_jacobian_r_.transpose()*J_Damped;
  }
  else
  {
    current_leg_jacobian_r_inv = DyrosMath::pinv(current_leg_jacobian_r_);
  }

  if (wl<=w0)
  {
    a = lambda*pow(1-wl/w0,2);
    J_Damped = current_leg_jacobian_l_*current_leg_jacobian_l_.transpose()+a*Eigen::Matrix6d::Identity();
    J_Damped = J_Damped.inverse();

    cout << "Singularity Region of right leg: " << wr << endl;
    current_leg_jacobian_l_inv = current_leg_jacobian_l_.transpose()*J_Damped;
  }
  else
  {
    current_leg_jacobian_l_inv = DyrosMath::pinv(current_leg_jacobian_r_);
  }

  Eigen::Matrix6d kp; // for setting CLIK gains
  kp.setZero();
  kp(0,0) = 100;
  kp(1,1) = 100;
  kp(2,2) = 100;
  kp(3,3) = 150;
  kp(4,4) = 150;
  kp(5,5) = 150;

  for (int i=0; i<2; i++)
  {
    currnet_leg_transform_[i] = model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)i);
  }
  currnet_leg_transform_l_=currnet_leg_transform_[0];
  currnet_leg_transform_r_=currnet_leg_transform_[1];

  Eigen::Vector6d lp, rp;
  lp.setZero(); rp.setZero();
  lp.topRows<3>() = float_lleg_transform.linear().transpose()*(-currnet_leg_transform_l_.translation()+float_lleg_transform.translation()); //Foot_Trajectory should revise
  rp.topRows<3>() = float_rleg_transform.linear().transpose()*(-currnet_leg_transform_r_.translation()+float_rleg_transform.translation());

  Eigen::Vector3d r_leg_phi, l_leg_phi;
  /*r_leg_phi_ = DyrosMath::getPhi(currnet_leg_transform_r_.linear(),float_rleg_transform,translation());
  l_leg_phi_ = DyrosMath::getPhi(currnet_leg_transform_l_.linear(),float_lleg_transform,translation());*/
  //1.15, Getphi의 phi 값 부호가 반대가 되야 할수도 있음

  lp.bottomRows<3>() = - l_leg_phi;
  rp.bottomRows<3>() = - r_leg_phi;

  Eigen::Vector6d q_lfoot_dot,q_rfoot_dot;
  q_lfoot_dot=current_leg_jacobian_l_inv*kp*lp;
  q_rfoot_dot=current_leg_jacobian_r_inv*kp*rp;

  for (int i=0; i<6; i++)
  {
    desired_leg_q_dot(i+6) = q_rfoot_dot(i+6);
    desired_leg_q_dot(i) = q_lfoot_dot(i);
  }

/*  if(_cnt == 4.5*hz_ || _cnt == 7.5*hz_)
  {
  cout << "RFOOT J " << _RFoot_J_inv << endl;
  cout << "LFOOT J " << _LFoot_J_inv << endl;
  }*/ // _cnt수정해야해
//  if (_foot_step(_step_number,6) == 0){ _step_number, _foot_step 수정
      // right foot single support
     //write the com trajectory basd on the right foot and left foot trajectory based on the com
    // cout<<"right SSP"<<endl;

//  }
//  else{
      //left foot single support
      //write the com trajectory based on the left foot and right foot trajectory based on the com
      //cout<<"Left SSP"<<endl;
//  }

}

void WalkingController::modifiedPreviewControl()
{
  if(walking_tick_==0)
    previewControlParameter(1.0/hz_, 16*hz_/10, _k ,com_support_init_, _gi, _gp_l, _gx, _a, _b, _c);
  if(current_step_num_ == 0)
    zmp_start_time_ = 0.0;
  else
    zmp_start_time_ = t_start_;
  int norm_size;
  norm_size = ref_zmp_.col(1).size();

  Eigen::VectorXd px_ref, py_ref;
  px_ref.resize(norm_size);
  py_ref.resize(norm_size);

  for (int i=0; i<norm_size; i++)
  {
      px_ref(i) = ref_zmp_(i,0);
      py_ref(i) = ref_zmp_(i,1);
  }
  double ux, uy, ux_1 = 0.0, uy_1 = 0.0;
  previewControl(1.0/hz_, 16*hz_/10, walking_tick_-zmp_start_time_, _k, xi_, yi_, xs_, ys_, px_ref, py_ref, ux_1, uy_1, ux, uy, _gi, _gp_l, _gx, _a, _b, _c, _xd, _yd);

  Eigen::Vector3d xs_matrix, ys_matrix;
  for (int i=0; i<3; i++)
      xs_matrix(i) = _xd(i);
  for (int i=0; i<3; i++)
      ys_matrix(i) = _yd(i);

  double est_zmp_error_x, est_zmp_error_y, rx, ry;
  est_zmp_error_x = _c*xs_matrix;
  est_zmp_error_y = _c*ys_matrix;

  rx = 0.0;
  ry = 0.0;

  previewControl(1.0/hz_, 16*hz_/10, walking_tick_-zmp_start_time_, _k, xi_, yi_, xs_, ys_, px_ref, py_ref, ux_1, uy_1, ux, uy, _gi, _gp_l, _gx, _a, _b, _c, _xd, _yd);

  ux_1 = ux;
  uy_1 = uy;

  Eigen::Vector3d _xs, _ys;
  _xs = _xd;
  _ys = _yd;

  double est_zmp;
  est_zmp = _c(0)*_xs(0)+_c(1)*_xs(1)+_c(2)*_xs(2);

}

void WalkingController::previewControl(
    double dt, int NL, int k_, Eigen::Matrix4d k, double x_i,
    double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys,
    Eigen::VectorXd px_ref, Eigen::VectorXd py_ref, double ux_1 ,
    double uy_1 , double ux, double uy, double gi, Eigen::VectorXd gp_l,
    Eigen::Matrix1x3d gx, Eigen::Matrix3d a, Eigen::Vector3d b,
    Eigen::Matrix1x3d c, Eigen::Vector3d &xd, Eigen::Vector3d &yd)
{ //Preview와 prameter에서 VectorXD로 되있는거 수정해야함
  Eigen::Vector3d x, y, x_1, y_1;
  x.setZero();
  y.setZero();
  x_1.setZero();
  y_1.setZero();

  if(k_==0 && current_step_num_ == 0)
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

  double px, py;
  px = c*x;
  py = c*y;
  xzmp_err = px - px_ref(k_);
  yzmp_err = py - py_ref(k_);

  double sum_gp_px_ref = 0.0, sum_gp_py_ref =0.0;
  for(int i = 0; i < NL; i++)
  {
      sum_gp_px_ref = sum_gp_px_ref + gp_l(i)*(px_ref(k_+1+i)-px_ref(k_+i));
      sum_gp_py_ref = sum_gp_py_ref + gp_l(i)*(py_ref(k_+1+i)-py_ref(k_+i));
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
  zc = com_support_init_(2);
  a.setIdentity();
  a(0,1) = dt;
  a(0,2) = dt*dt/2.0;
  a(1,2) = dt;
  b(0) =dt*dt*dt/6.0;
  b(1) =dt*dt/2.0;
  b(2) =dt;
  c(0,0) = 1;
  c(0,1) = 0;
  c(0,2) = zc/GRAVITY;

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

  double qe, r;
  qe = 1.0;
  r = 0.000001;

  Eigen::Matrix3d qx;
  qx.setZero();
  Eigen::Matrix4d q_bar;
  q_bar.setZero();
  q_bar(0,0) = qe;
  q_bar.block<3,3>(1,1) = qx;

  k=discreteRiccatiEquation(a_bar, b_bar, r, q_bar);

  double temp_mat;
  temp_mat = r+b_bar_tran*k*b_bar;

  Eigen::Matrix4d ac_bar;
  ac_bar.setZero();
  ac_bar = a_bar - b_bar*b_bar_tran*k*a_bar/temp_mat;

  gi = b_bar_tran*k*i_p;
  gi = gi/temp_mat;
  gx = b_bar_tran*k*f_bar/temp_mat;

  Eigen::MatrixXd x_l(4, NL);
  Eigen::Vector4d x_l_column;
  x_l.setZero();
  x_l_column.setZero();
  x_l_column = -ac_bar.transpose()*k*i_p;
  for(int i=0; i<NL; i++)
  {
      x_l.block<4,1>(0,i) = x_l_column;
      x_l_column = ac_bar.transpose()*x_l_column;
  }
  double gp_l_column;
  gp_l_column = -gi;
  for(int i=0; i<NL; i++)
  {
      gp_l(i) = gp_l_column;
      gp_l_column = b_bar_tran*x_l.col(i);
      gp_l_column = gp_l_column/temp_mat;
  }

}


Eigen::Matrix4d WalkingController::discreteRiccatiEquation(Eigen::Matrix4d a, Eigen::Vector4d b, double r, Eigen::Matrix4d q)
{
  Eigen::Matrix4d z11, z12, z21, z22;
  z11 = a.inverse();
  z12 = a.inverse()*b*b.transpose()/r;
  z21 = q*a.inverse();
  z22 = a.transpose() + q*a.inverse()*b*b.transpose()/r;

  Eigen::Matrix8d z;
  z.setZero();
  z.topLeftCorner(4,4) = z11;
  z.topRightCorner(4,4) = z12;
  z.bottomLeftCorner(4,4) = z21;
  z.bottomRightCorner(4,4) = z22;

  std::vector<double> eigVal_real(8);
  std::vector<double> eigVal_img(8);
  std::vector<Eigen::Vector8d> eigVec_real(8);
  std::vector<Eigen::Vector8d> eigVec_img(8);

  for(int i=0; i<8; i++)
  {
    eigVec_real[i].setZero();
    eigVec_img[i].setZero();
  }

  Eigen::Vector8d deigVal_real, deigVal_img;
  Eigen::Matrix8d deigVec_real, deigVec_img;
  deigVal_real.setZero();
  deigVal_img.setZero();
  deigVec_real.setZero();
  deigVec_img.setZero();
  deigVal_real = z.eigenvalues().real();
  deigVal_img = z.eigenvalues().imag();

  Eigen::EigenSolver<Eigen::Matrix8d> es(z);
  //EigenVector Solver
  //Matrix3D ones = Matrix3D::Ones(3,3);
  //EigenSolver<Matrix3D> es(ones);
  //cout << "The first eigenvector of the 3x3 matrix of ones is:" << endl << es.eigenvectors().col(1) << endl;

  for(int i=0;i<8; i++)
  {
      for(int j=0; j<8; j++)
      {
          deigVec_real(j,i) = es.eigenvectors().col(i)(j).real();
          deigVec_img(j,i) = es.eigenvectors().col(i)(j).imag();
      }
  }

  //Order the eigenvectors
  //move e-vectors correspnding to e-value outside the unite circle to the left

  Eigen::Matrix8x4d tempZ_real, tempZ_img;
  tempZ_real.setZero();
  tempZ_img.setZero();
  int c=0;

  for (int i=0;i<8;i++)
  {
      if ((deigVal_real(i)*deigVal_real(i)+deigVal_img(i)*deigVal_img(i))>1.0) //outside the unit cycle
      {
          for(int j=0; j<8; j++)
          {
              tempZ_real(j,c) = deigVec_real(j,i);
              tempZ_img(j,c) = deigVec_img(j,i);
          }
          c++;
      }
  }

  Eigen::Matrix8x4cd tempZ_comp;
  for(int i=0;i<8;i++)
  {
      for(int j=0;j<4;j++)
      {
          tempZ_comp.real()(i,j) = tempZ_real(i,j);
          tempZ_comp.imag()(i,j) = tempZ_img(i,j);
      }
  }

  Eigen::Matrix4cd U11, U21, X;
  for(int i=0;i<4;i++)
  {
      for(int j=0;j<4;j++)
      {
          U11(i,j) = tempZ_comp(i,j);
          U21(i,j) = tempZ_comp(i+4,j);
      }
  }
  X = U21*(U11.inverse());
  Eigen::Matrix4d X_sol;
  for(int i=0;i<4;i++)
  {
      for(int j=0;j<4;j++)
      {
          X_sol(i,j) = X.real()(i,j);
      }
  }

  return X_sol;

}



}


