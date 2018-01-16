#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller.h"

namespace dyros_jet_controller
{



void WalkingController::compute(VectorQd* desired_q)
{
  if(walking_enable_)
  {
    if(walking_tick_ ==0)
    {
      updateInitialState();
    }
    /*getFootStep();
    getComTrajectory();
    getZmpTrajectory();

  */

    //////compute//////////////////
    if (ik_mode_ == 0)
    {
      computeIkControl(desired_leg_q_);
      for(int i=0; i<6; i++)
      {
        desired_q_(i) = desired_leg_q_(i);
        desired_q_(i+6) = desired_leg_q_(i+6);
      }
    }
    else if (ik_mode_ == 1)
    {
      computeJacobianControl(desired_leg_q_dot_);
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
  int temp = -1;

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
  temp = -1; //fisrt step is right foot
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
  zmpGenerator();

}

void WalkingController::floatToSupportFootstep()
{
  Eigen::Isometry3d reference;

  if(current_step_num_ == 0)
  {
    if(foot_step_(current_step_num_,6) == 1) //right support
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
  //model_.updateKinematics(current_q_);
  q_init_ = current_q_;
  lfoot_float_init_ = model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)0);
  rfoot_float_init_ = model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)1);
  com_float_init_ = model_.getCurrentCom();

  pelv_float_init_.setIdentity();

  Eigen::Isometry3d ref_frame;

  if(foot_step_(current_step_num_, 6) == 0)  //left foot support
  {
    ref_frame = lfoot_float_init_;
  }
  else if(foot_step_(current_step_num_, 6) == 1)
  {
    ref_frame = rfoot_float_init_;
  }

  lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
  rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
  pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame);
  com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation();

  supportfoot_float_init_.setZero();
  swingfoot_float_init_.setZero();

  if(foot_step_(0,6) == 0)  //left suppport foot
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

void WalkingController::computeIkControl(Eigen::VectorLXd& desired_leg_q)
{
  for (int i=2; i<4; i++)
  {
    currnet_leg_transform_[i-2]=model_.getCurrentTrasmfrom((DyrosJetModel::EndEffector)i);
  }
  currnet_leg_transform_l_=currnet_leg_transform_[0];
  currnet_leg_transform_r_=currnet_leg_transform_[1];

  Eigen::Vector3d lp, rp;
  //Should revise by dg, Trunk_trajectory_global.translation()
  lp = currnet_leg_transform_l_.linear().transpose()*(Trunk_trajectory_global.translation()-currnet_leg_transform_l_.translation());
  rp = currnet_leg_transform_r_.linear().transpose()*(Trunk_trajectory_global.translation()-currnet_leg_transform_r_.translation());

  Eigen::Matrix3d trunk_leg_transform_l_, trunk_leg_transform_r_;
  trunk_leg_transform_l_ = Trunk_trajectory_global.linear().transpose()*currnet_leg_transform_l_.linear();
  trunk_leg_transform_r_ = Trunk_trajectory_global.linear().transpose()*currnet_leg_transform_r_.linear();

  Eigen::Vector3d ld, rd;
  ld.setZero(); rd.setZero();
  ld(1) = 0.105;
  ld(2) = -0.1829;
  rd(1) = -0.105;
  rd(2) = -0.1829;
  ld = trunk_leg_transform_l_.transpose() * ld;
  rd = trunk_leg_transform_r_.transpose() * rd;

  Eigen::Vector3d lr, rr;
  lr = lp + ld;
  rr = rp + rd;

  double l_upper_ = 0.3729; // direct length from hip to knee
  double l_lower_ = 0.3728; //direct length from knee to ankle

  double offset_hip_pitch_ = 24.6271*DEG2RAD;
  double offset_knee_pitch_ = 15.3655*DEG2RAD;
  double offset_ankle_pitch_ = 9.2602*DEG2RAD;

  //////////////////////////// LEFT LEG INVERSE KINEMATICS ////////////////////////////

  double lc = lr.norm();
  desired_leg_q(3) = (- acos((l_upper_*l_upper_ + l_lower_*l_lower_ - lc*lc) / (2*l_upper_*l_lower_))+ 3.141592); // - offset_knee_pitch //+ alpha_lower

  double l_ankle_pitch_ = asin((l_upper_*sin(3.141592-desired_leg_q(3)))/lc);
  desired_leg_q(4) = -atan2(lr(0), sqrt(lr(1)*lr(1)+lr(2)*lr(2))) - l_ankle_pitch_;// - offset_ankle_pitch ;
  desired_leg_q(5) = atan2(lr(1), lr(2));

  Eigen::Matrix3d r_tl2_;
  Eigen::Matrix3d r_l2l3_;
  Eigen::Matrix3d r_l3l4_;
  Eigen::Matrix3d r_l4l5_;

  r_tl2_.setZero();
  r_l2l3_.setZero();
  r_l3l4_.setZero();
  r_l4l5_.setZero();

  r_l2l3_ = DyrosMath::rotateWithY(desired_leg_q(3));
  r_l3l4_ = DyrosMath::rotateWithY(desired_leg_q(4));
  r_l4l5_ = DyrosMath::rotateWithX(desired_leg_q(5));

  r_tl2_ = trunk_leg_transform_l_ * r_l4l5_.transpose() * r_l3l4_.transpose() * r_l2l3_.transpose();

  desired_leg_q(1) = asin(r_tl2_(2,1));

  double c_lq5_ = -r_tl2_(0,1)/cos(desired_leg_q(1));
  if (c_lq5_ > 1.0)
  {
    c_lq5_ =1.0;
  }
  else if (c_lq5_ < -1.0)
  {
    c_lq5_ = -1.0;
  }

  desired_leg_q(0) = -asin(c_lq5_);
  desired_leg_q(2) = -asin(r_tl2_(2,0)/cos(desired_leg_q(7)))+offset_hip_pitch_;
  desired_leg_q(3) = desired_leg_q(9)- offset_knee_pitch_;
  desired_leg_q(4) = desired_leg_q(10)- offset_ankle_pitch_;

  //////////////////////////// RIGHT LEG INVERSE KINEMATICS ////////////////////////////

  double rc = rr.norm();
  desired_leg_q(9) = (- acos((l_upper_*l_upper_ + l_lower_*l_lower_ - rc*rc) / (2*l_upper_*l_lower_))+ 3.141592); // - offset_knee_pitch //+ alpha_lower

  double r_ankle_pitch_ = asin((l_upper_*sin(3.141592-desired_leg_q(3)))/rc);
  desired_leg_q(10) = -atan2(rr(0), sqrt(rr(1)*rr(1)+rr(2)*rr(2)))-r_ankle_pitch_;
  desired_leg_q(11) = atan2(rr(1),rr(2));

  Eigen::Matrix3d r_tr2_;
  Eigen::Matrix3d r_r2r3_;
  Eigen::Matrix3d r_r3r4_;
  Eigen::Matrix3d r_r4r5_;

  r_tr2_.setZero();
  r_r2r3_.setZero();
  r_r3r4_.setZero();
  r_r4r5_.setZero();

  r_r2r3_ = DyrosMath::rotateWithY(desired_leg_q(9));
  r_r3r4_ = DyrosMath::rotateWithY(desired_leg_q(10));
  r_r4r5_ = DyrosMath::rotateWithX(desired_leg_q(11));

  r_tr2_ = trunk_leg_transform_r_ * r_r4r5_.transpose() * r_r3r4_.transpose() * r_r2r3_.transpose();

  desired_leg_q(7) = asin(r_tr2_(2,1));

  double c_rq5_ = -r_tr2_(0,1)/cos(desired_leg_q(7));
  if (c_rq5_ > 1.0)
  {
    c_rq5_ =1.0;
  }
  else if (c_rq5_ < -1.0)
  {
    c_rq5_ = -1.0;
  }

  desired_leg_q(6) = -asin(c_rq5_);
  desired_leg_q(8) = -asin(r_tr2_(2,0)/cos(desired_leg_q(7)))+offset_hip_pitch_;
  desired_leg_q(9) = desired_leg_q(9)- offset_knee_pitch_;
  desired_leg_q(10) = desired_leg_q(10)- offset_ankle_pitch_;

}


void WalkingController::computeJacobianControl(Eigen::VectorLXd& desired_leg_q_dot)
{
  for (int i=0; i<2; i++)
  {
    current_leg_jacobian_[i] = model_.getLegJacobian((DyrosJetModel::EndEffector) i);
  }
  current_leg_jacobian_l_=current_leg_jacobian_[0];
  current_leg_jacobian_r_=current_leg_jacobian_[1];

  Eigen::Matrix6d jacobian_temp_l_, jacobian_temp_r_, current_leg_jacobian_l_inv_, current_leg_jacobian_r_inv_,
      J_Damped;
  double wl, wr, w0, lambda, a;
  w0 = 0.001;
  lambda = 0.05;
  jacobian_temp_l_=current_leg_jacobian_l_*current_leg_jacobian_l_.transpose();
  jacobian_temp_r_=current_leg_jacobian_r_*current_leg_jacobian_r_.transpose();
  wr = sqrt(jacobian_temp_l_.determinant());
  wl = sqrt(jacobian_temp_r_.determinant());

  if (wr<=w0)
  { //Right Jacobi
    a = lambda * pow(1-wr/w0,2);
    J_Damped = current_leg_jacobian_r_*current_leg_jacobian_r_.transpose()+a*Eigen::Matrix6d::Identity();
    J_Damped = J_Damped.inverse();

    cout << "Singularity Region of right leg: " << wr << endl;
    current_leg_jacobian_r_inv_ = current_leg_jacobian_r_.transpose()*J_Damped;
  }
  else
  {
    current_leg_jacobian_r_inv_ = DyrosMath::pinv(current_leg_jacobian_r_);
  }

  if (wl<=w0)
  {
    a = lambda*pow(1-wl/w0,2);
    J_Damped = current_leg_jacobian_l_*current_leg_jacobian_l_.transpose()+a*Eigen::Matrix6d::Identity();
    J_Damped = J_Damped.inverse();

    cout << "Singularity Region of right leg: " << wr << endl;
    current_leg_jacobian_l_inv_ = current_leg_jacobian_l_.transpose()*J_Damped;
  }
  else
  {
    current_leg_jacobian_l_inv_ = DyrosMath::pinv(current_leg_jacobian_r_);
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
  lp.topRows<3>() = Foot_trajectory_global.LFoot.linear().transpose()*(-currnet_leg_transform_l_.translation()+Foot_trajectory_global.LFoot.translation()); //Foot_Trajectory should revise
  rp.topRows<3>() = Foot_trajectory_global.RFoot.linear().transpose()*(-currnet_leg_transform_r_.translation()+Foot_trajectory_global.RFoot.translation());

  Eigen::Vector3d r_leg_phi_, l_leg_phi_;
  r_leg_phi_ = DyrosMath::getPhi(currnet_leg_transform_r_.linear(),Foot_trajectory_global.RFoot_euler);
  l_leg_phi_ = DyrosMath::getPhi(currnet_leg_transform_l_.linear(),Foot_trajectory_global.lFoot_euler);
  //1.15, Getphi의 phi 값 부호가 반대가 되야 할수도 있음

  lp.bottomRows<3>() = - l_leg_phi_;
  rp.bottomRows<3>() = - r_leg_phi_;

  Eigen::Vector6d q_lfoot_dot_,q_rfoot_dot_;
  q_lfoot_dot_=current_leg_jacobian_l_inv_*kp*lp;
  q_rfoot_dot_=current_leg_jacobian_r_inv_*kp*rp;

  for (int i=0; i<6; i++)
  {
    desired_leg_q_dot(i+6) = q_rfoot_dot_(i+6);
    desired_leg_q_dot(i) = q_lfoot_dot_(i);
  }

/*  if(_cnt == 4.5*hz_ || _cnt == 7.5*hz)
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

}


