#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller.h"

namespace dyros_jet_controller
{



void WalkingController::compute(VectorQd* desired_q)
{
  /*getFootStep();
  getCOMTrajectory();
  getZMPTrajectory();
  computeIKControl();
  computeJacobianControl();
  compensator();
*/
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
    }
  }
}

/**Foot step related fuctions
 */

void WalkingController::getFootStep()
{


}
  
void turnandGoandTurn()
{
  double initial_rot;
  double final_rot = 0.0;
  double initial_drot = 0.0;
  double final_drot = 0.0;

  initial_rot= atan2(target_x_, target_y_);

  if(initial_rot > 0.0)
    initial_drot = 10*DEG2RAD;
  else
      initial_drot = -10*DEG2RAD;

  int initial_total_step_number = initial_rot/initial_drot;
  double initial_residual_angle = initial_rot-initial_total_step_number*initial_drot;

  final_rot = target_theta_-initial_rot;
  if(final_rot > 0.0)
    final_drot = 10*DEG2RAD;
  else
    final_drot = -10*DEG2RAD;

  int final_total_step_number = final_rot/final_drot;
  double final_residual_angle = final_rot-final_total_step_number*final_drot;
  double length_to_target = sqrt(target_x_*target_x_+target_y_*target_y_);
  double dlength = step_length_x_; //footstep length;
  int middle_total_step_number = length_to_target/(dlength);
  double middle_residual_length = length_to_target-middle_total_step_number*(dlength);



  int temp_size;


  int del_size;

  del_size = 1;
  temp_size = initial_total_step_number*del_size+middle_total_step_number *del_size+final_total_step_number*del_size;

  if(initial_total_step_number!=0 || abs(initial_residual_angle)>=0.0001)
  {
    if(initial_total_step_number%2==0)
        temp_size = temp_size+2*del_size;
    else
    {
      if(abs(initial_residual_angle)>= 0.0001)
          temp_size = temp_size+3*del_size;
      else
          temp_size = temp_size+del_size;
    }
  }

  if(middle_total_step_number!=0 || abs(middle_residual_length)>=0.0001)
  {
    if(middle_total_step_number%2==0)
      temp_size = temp_size+2*del_size;
    else
    {
      if(abs(middle_residual_length)>= 0.0001)
        temp_size = temp_size+3*del_size;
      else
        temp_size = temp_size+del_size;
    }
  }

  if(final_total_step_number!=0 || abs(final_residual_angle)>= 0.0001)
  {
    if(abs(final_residual_angle)>= 0.0001)
      temp_size = temp_size+2*del_size;
    else
      temp_size = temp_size+del_size;
  }


  foot_step_.resize(temp_size,7);
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

