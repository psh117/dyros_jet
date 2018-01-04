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
  
void WalkingController::turnandGoandTurn()
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

void WalkingController::computeIKControl(Eigen::VectorLXd *desired_leg_q)
{
  for (int i=0; i<2; i++)
  {
    model_.getTransformEndEffector((DyrosJetModel::EndEffector)i, &currnet_leg_transform_[i]);
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
  desired_leg_q(9) = (- acos((l_upper_*l_upper_ + l_lower_*l_lower_ - lc*lc) / (2*l_upper_*l_lower_))+ 3.141592); // - offset_knee_pitch //+ alpha_lower

  double l_ankle_pitch_ = asin((l_upper_*sin(3.141592-qd(9)))/lc);
  desired_leg_q(10) = -atan2(lr(0), sqrt(lr(1)*lr(1)+lr(2)*lr(2))) - l_ankle_pitch_;// - offset_ankle_pitch ;
  desired_leg_q(11) = atan2(lr(1), lr(2));

  Eigen::Matrix3d r_tl2_;
  Eigen::Matrix3d r_l2l3_;
  Eigen::Matrix3d r_l3l4_;
  Eigen::Matrix3d r_l4l5_;

  r_tl2_.setZero();
  r_l2l3_.setZero();
  r_l3l4_.setZero();
  r_l4l5_.setZero();

  r_l2l3_(0,0) = cos(desired_leg_q(9));
  r_l2l3_(0,2) = sin(desired_leg_q(9));
  r_l2l3_(1,1) = 1.0;
  r_l2l3_(2,0) = -sin(desired_leg_q(9));
  r_l2l3_(2,2) = cos(desired_leg_q(9));

  r_l3l4_(0,0) = cos(desired_leg_q(10));
  r_l3l4_(0,2) = sin(desired_leg_q(10));
  r_l3l4_(1,1) = 1.0;
  r_l3l4_(2,0) = -sin(desired_leg_q(10));
  r_l3l4_(2,2) = cos(desired_leg_q(10));

  r_l4l5_(0,0) = 1.0;
  r_l4l5_(1,1) = cos(desired_leg_q(11));
  r_l4l5_(1,2) = -sin(desired_leg_q(11));
  r_l4l5_(2,1) = sin(desired_leg_q(11));
  r_l4l5_(2,2) = cos(desired_leg_q(11));

  r_tl2_ = trunk_leg_transform_l_ * r_l4l5_.transpose() * r_l3l4_.transpose() * r_l2l3_.transpose();

  desired_leg_q(7) = asin(r_tl2_(2,1));

  double c_lq5_ = -R_tl2(0,1)/cos(qd(7));
  if (c_lq5_ > 1.0)
  {
    c_lq5_ =1.0;
  }
  else if (c_lq5_ < -1.0)
  {
    c_lq5_ = -1.0;
  }

  desired_leg_q(6) = -asin(c_lq5_);
  desired_leg_q(8) = -asin(r_tl2_(2,0)/cos(desired_leg_q(7)))+offset_hip_pitch_;
  desired_leg_q(9) = desired_leg_q(9)- offset_knee_pitch_;
  desired_leg_q(10) = desired_leg_q(10)- offset_ankle_pitch_;

  //////////////////////////// RIGHT LEG INVERSE KINEMATICS ////////////////////////////

  double rc = rr.norm();
  desired_leg_q(3) = (- acos((l_upper_*l_upper_ + l_lower_*l_lower_ - rc*rc) / (2*l_upper_*l_lower_))+ 3.141592); // - offset_knee_pitch //+ alpha_lower

  double r_ankle_pitch_ = asin((l_upper_*sin(3.141592-desired_leg_q(3)))/rc);
  desired_leg_q(4) = -atan2(rr(0), sqrt(rr(1)*rr(1)+rr(2)*rr(2)))-r_ankle_pitch_;
  desired_leg_q(5) = atan2(rr(1),rr(2));

  Eigen::Matrix3d r_tr2_;
  Eigen::Matrix3d r_r2r3_;
  Eigen::Matrix3d r_r3r4_;
  Eigen::Matrix3d r_r4r5_;

  r_tr2_.setZero();
  r_r2r3_.setZero();
  r_r3r4_.setZero();
  r_r4r5_.setZero();

  r_r2r3_(0,0) = cos(desired_leg_q(3));
  r_r2r3_(0,2) = sin(desired_leg_q(3));
  r_r2r3_(1,1) = 1.0;
  r_r2r3_(2,0) = -sin(desired_leg_q(3));
  r_r2r3_(2,2) = cos(desired_leg_q(3));

  r_r3r4_(0,0) = cos(desired_leg_q(4));
  r_r3r4_(0,2) = sin(desired_leg_q(4));
  r_r3r4_(1,1) = 1.0;
  r_r3r4_(2,0) = -sin(desired_leg_q(4));
  r_r3r4_(2,2) = cos(desired_leg_q(4));

  r_r4r5_(0,0) = 1.0;
  r_r4r5_(1,1) = cos(desired_leg_q(5));
  r_r4r5_(1,2) = -sin(desired_leg_q(5));
  r_r4r5_(2,1) = sin(desired_leg_q(5));
  r_r4r5_(2,2) = cos(desired_leg_q(5));

  r_tr2_ = trunk_leg_transform_r_ * r_r4r5_.transpose() * r_r3r4_.transpose() * r_r2r3_.transpose();

  desired_leg_q(1) = asin(r_tr2_(2,1));

  double c_rq5_ = -r_tr2(0,1)/cos(qd(1));
  if (c_rq5_ > 1.0)
  {
    c_rq5_ =1.0;
  }
  else if (c_rq5_ < -1.0)
  {
    c_rq5_ = -1.0;
  }

  desired_leg_q(0) = -asin(c_rq5_);
  desired_leg_q(2) = -asin(r_tr2_(2,0)/cos(desired_leg_q(1)))+offset_hip_pitch_;
  desired_leg_q(3) = desired_leg_q(3)- offset_knee_pitch_;
  desired_leg_q(4) = desired_leg_q(4)- offset_ankle_pitch_;

}


}
