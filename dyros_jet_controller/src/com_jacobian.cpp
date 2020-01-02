#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller.h"


namespace dyros_jet_controller
{

void WalkingController::linkMass()
{
  ///////////////////////////////////////////////////////////////////////////////////
  mass_total_ = 0;

  mass_body_(0) =   model_.getLinkMass(0);
  mass_body_(1) =   model_.getLinkMass(13);
  mass_body_(2) =   model_.getLinkMass(14);

  c_waist_[0] = model_.getLinkComPosition(0);
  c_waist_[1] = model_.getLinkComPosition(13);
  c_waist_[2] = model_.getLinkComPosition(14);
  for(unsigned int i = 1; i<7; i++)
  {
    mass_l_leg_(i-1) = model_.getLinkMass(i);
    c_l_leg_[i-1] = model_.getLinkComPosition(i);
  }
  for(unsigned int i = 7; i<13; i++)
  {
    mass_r_leg_(i-7) = model_.getLinkMass(i);
    c_r_leg_[i-7] = model_.getLinkComPosition(i);
  }
  for(unsigned int i = 15; i<22; i++)
  {
    mass_l_arm_(i-15) = model_.getLinkMass(i);
    c_l_arm_[i-15] = model_.getLinkComPosition(i);
  }
  for(unsigned int i = 22; i<29; i++)
  {
    mass_r_arm_(i-22) = model_.getLinkMass(i);
    c_r_arm_[i-22] = model_.getLinkComPosition(i);
  }

  for(unsigned int i=0; i<29; i++)
  {
    mass_total_ += model_.getLinkMass(i);
  }

  cout<<"mass_total: "<<mass_total_<<endl;
  /*
  mass_l_leg_(0) = 1.54216;  //hip yaw link mass
  mass_l_leg_(1) = 1.16907;  //hip roll link mass
  mass_l_leg_(2) = 3.28269;  //hip pitch link mass
  mass_l_leg_(3) = 2.04524;  //Knee pitch link mass
  mass_l_leg_(4) = 1.18450;  //Ankle pitch link mass
  mass_l_leg_(5) = 1.42541;  //Ankle roll link mass
  mass_r_leg_(0) = 1.54216;  //hip yaw link mass
  mass_r_leg_(1) = 1.16907;  //hip roll link mass
  mass_r_leg_(2) = 3.28269;  //hip pitch link mass
  mass_r_leg_(3) = 2.04524;  //Knee pitch link mass
  mass_r_leg_(4) = 1.18450;  //Ankle pitch link mass
  mass_r_leg_(5) = 1.42541;  //Ankle roll link mass
  mass_l_arm_(0) = 0.92826; //Shoulder pitch link mass
  mass_l_arm_(1) = 0.11240; //Shoulder roll link mass
  mass_l_arm_(2) = 1.75149; //Shoulder yaw link mass
  mass_l_arm_(3) = 0.83342; //elbow roll link mass
  mass_l_arm_(4) = 0.52391; //wrist yaw link mass
  mass_l_arm_(5) = 0.07856;  //wrist roll link mass
  mass_l_arm_(6) = 1.20640;  //hand yaw link mass
  mass_r_arm_(0) = 0.92826; //Shoulder pitch link mass
  mass_r_arm_(1) = 0.11240; //Shoulder roll link mass
  mass_r_arm_(2) = 1.75149; //Shoulder yaw link mass
  mass_r_arm_(3) = 0.83342; //elbow roll link mass
  mass_r_arm_(4) = 0.52391; //wrist yaw link mass
  mass_r_arm_(5) = 0.07856; //wrist roll link mass
  mass_r_arm_(6) = 0.80640;//hand yaw link mass
  mass_body_(0) = 3.90994; // base link mass
  mass_body_(1) = 0.18235; //waist pitch link mass
  mass_body_(2) = 14.09938;  //waist yaw link mass
  mass_total_ = 51.315;
  /////////////////////////////////////////////////////////////////////////////////////////////
  c_l_leg_[0](0) = -0.02955;   //Left_Hip_Yaw_com in the Hip Yaw coordiates
  c_l_leg_[0](1) = 0;
  c_l_leg_[0](2) = 0.06943;
  c_l_leg_[1](0) = 0.00513;    //Left_Hip_Roll_com
  c_l_leg_[1](1) = -0.00119;
  c_l_leg_[1](2) = 0;
  c_l_leg_[2](0) = 0.12203;    //Left_Hip_Pitch_com
  c_l_leg_[2](1) = 0.00306;
  c_l_leg_[2](2) = -0.23316;
  c_l_leg_[3](0) = 0.05276;    //Left_Knee_Pitch_com
  c_l_leg_[3](1) = -0.01856;
  c_l_leg_[3](2) = -0.22635;
  c_l_leg_[4](0) = -0.01035;   //Left_Ankle_Pitch_com
  c_l_leg_[4](1) = 0.00568;
  c_l_leg_[4](2) = 0;
  c_l_leg_[5](0) = -0.00989;   //Left_Ankle_Roll_com
  c_l_leg_[5](1) = 0;
  c_l_leg_[5](2) = 0.07224;
  c_r_leg_[0](0) = -0.02955;   //Right_Hip_Yaw_com in the Hip Yaw coordiates
  c_r_leg_[0](1) = 0;
  c_r_leg_[0](2) = 0.06943;
  c_r_leg_[1](0) = 0.00513;    //Right_Hip_Roll_com
  c_r_leg_[1](1) = 0.00119;
  c_r_leg_[1](2) = 0;
  c_r_leg_[2](0) = 0.12203;    //Right_Hip_Pitch_com
  c_r_leg_[2](1) = -0.00306;
  c_r_leg_[2](2) = -0.23316;
  c_r_leg_[3](0) = 0.05276;    //Right_Knee_Pitch_com
  c_r_leg_[3](1) = 0.01856;
  c_r_leg_[3](2) = -0.22635;
  c_r_leg_[4](0) = -0.01035;   //Right_Ankle_Pitch_com
  c_r_leg_[4](1) = 0.00568;
  c_r_leg_[4](2) = 0;
  c_r_leg_[5](0) = -0.00989;   //Right_Ankle_Roll_com
  c_r_leg_[5](1) = 0;
  c_r_leg_[5](2) = 0.07224;
  c_l_arm_[0](0) = -0.00159;     //Left_Sholder_Pitch_com_position
  c_l_arm_[0](1) = -0.00092;
  c_l_arm_[0](2) = 0;
  c_l_arm_[1](0) = 0;            //Left_Sholder_Roll_com_position
  c_l_arm_[1](1) = 0.02958;
  c_l_arm_[1](2) = -0.03197;
  c_l_arm_[2](0) = 0.00022;      //Left_Sholder_Yaw_com_position
  c_l_arm_[2](1) = 0.02910;
  c_l_arm_[2](2) = -0.20695;
  c_l_arm_[3](0) = 0.00013;      //Left_Elbow_Roll_com_position
  c_l_arm_[3](1) = 0.02766;
  c_l_arm_[3](2) = 0.06984;
  c_l_arm_[4](0) = -0.00052833;  //Left_Wrist_Yaw_com_position
  c_l_arm_[4](1) = 0.21354;
  c_l_arm_[4](2) = 0;
  c_l_arm_[5](0) = 0;            //Left_Wrist_Roll_com_position
  c_l_arm_[5](1) = 0.029698;
  c_l_arm_[5](2) = 0;
  c_l_arm_[6](0) = 0;            //Left_Hand_Yaw_com_position
  c_l_arm_[6](1) = 0.20231;
  c_l_arm_[6](2) = 0.0031654;
  c_r_arm_[0](0) = -0.00159;     //Right_Sholder_Pitch_com_position
  c_r_arm_[0](1) = 0.00092;
  c_r_arm_[0](2) = 0;
  c_r_arm_[1](0) = 0;            //Right_Sholder_Roll_com_position
  c_r_arm_[1](1) = -0.02958;
  c_r_arm_[1](2) = -0.03197;
  c_r_arm_[2](0) = 0.00022;    	//Right_Sholder_Yaw_com_position
  c_r_arm_[2](1) = -0.02910;
  c_r_arm_[2](2) = -0.20695;
  c_r_arm_[3](0) = 0.00013;      //Right_Elbow_Roll_com_position
  c_r_arm_[3](1) = -0.02766;
  c_r_arm_[3](2) = 0.06984;
  c_r_arm_[4](0) = -0.00052833;  //Right_Wrist_Yawl_com_position
  c_r_arm_[4](1) = -0.21354;
  c_r_arm_[4](2) = 0;
  c_r_arm_[5](0) = 0;            //Right_Wrist_Roll_com_position
  c_r_arm_[5](1) = -0.029698;
  c_r_arm_[5](2) = 0;
  c_r_arm_[6](0) = 0;            //Right_Hand_Yaw_com_position
  c_r_arm_[6](1) = -0.20231;
  c_r_arm_[6](2) = 0.0031654;
  c_waist_[0](0) = 0;
  c_waist_[0](1) = 0;
  c_waist_[0](2) = 0.08027;
  c_waist_[1](0) = 0;
  c_waist_[1](1) = 0;
  c_waist_[1](2) = 0.14577;
  c_waist_[2](0) = 0.0038;
  c_waist_[2](1) = -0.00415;
  c_waist_[2](2) = 0.11424;
  */

}
void WalkingController::getComJacobian()
{

  Eigen::Matrix<double, 6, 6> j_leg_link_float[12];
  Eigen::Matrix<double, 6, 7> j_arm_link_float[14];
  Eigen::Matrix<double, 3, 6> j_leg_com_link_support[12];
  Eigen::Matrix<double, 3, 7> j_arm_com_link_support[14];
  Eigen::Isometry3d leg_link_transform[12];
  Eigen::Isometry3d arm_link_transform[14];
  Eigen::Matrix3d skew_c_leg;
  Eigen::Matrix3d skew_c_arm;

  Eigen::Matrix6d adjoint_leg_com[12];
  Eigen::Matrix6d adjoint_arm_com[12];


  for(int i=0; i<12; i++)
  {
    j_leg_link_float[i] = model_.getLegLinkJacobian(i);  //left first
    leg_link_transform[i] = model_.getCurrentLinkTransform(i);
  }
  for(int i=0; i<14; i++)
  {
    j_arm_link_float[i] = model_.getArmLinkJacobian(i);  //left first
    arm_link_transform[i] = model_.getCurrentLinkTransform(i+14);
  }

  j_rleg_com_total_support.setZero();  // in the body center coordinates
  j_lleg_com_total_support.setZero();
  j_rarm_com_total_support.setZero();
  j_larm_com_total_support.setZero();



  for(int i=0; i<12; i++)
  {

    if(i<6)
    {
      skew_c_leg = DyrosMath::skew(leg_link_transform[i].linear()*c_l_leg_[i]);
    }
    else
    {
      skew_c_leg = DyrosMath::skew(leg_link_transform[i].linear()*c_r_leg_[i-6]);
    }

    adjoint_leg_com[i].setIdentity();
    adjoint_leg_com[i].block<3, 3>(0, 3) = -skew_c_leg; //Spatial Translation Matrix from i_th link origin to i_th link com position with respect to base coordinate

    adjoint_support_.block<3, 3>(0, 0) = pelv_support_current_.linear();
    adjoint_support_.block<3, 3>(3, 3) = pelv_support_current_.linear();

    j_leg_com_link_support[i] = (adjoint_support_*adjoint_leg_com[i]*j_leg_link_float[i]).block<3, 6>(0, 0);

    if(i<6)
    {
      j_lleg_com_total_support += (mass_l_leg_(i)/mass_total_)*j_leg_com_link_support[i];
    }
    else
    {
      j_rleg_com_total_support += (mass_r_leg_(i-6)/mass_total_)*j_leg_com_link_support[i];
    }
  }

  for(int i=0; i<14; i++)
  {

    if(i<7)
    {
      skew_c_arm = DyrosMath::skew(arm_link_transform[i].linear()*c_l_arm_[i]);
    }
    else
    {
      skew_c_arm = DyrosMath::skew(arm_link_transform[i].linear()*c_r_arm_[i-7]);
    }
    adjoint_arm_com[i].setIdentity();
    adjoint_arm_com[i].block<3, 3>(0, 3) = -skew_c_arm; //Spatial Translation Matrix from i_th link origin to i_th link com position with respect to base coordinate

    adjoint_support_.block<3, 3>(0, 0) = pelv_support_current_.linear();
    adjoint_support_.block<3, 3>(3, 3) = pelv_support_current_.linear();

    j_arm_com_link_support[i] = (adjoint_support_*adjoint_arm_com[i]*j_arm_link_float[i]).block<3, 7>(0, 0);

    if(i<7)
    {
      j_larm_com_total_support += (mass_l_arm_(i)/mass_total_)*j_arm_com_link_support[i];
    }
    else
    {
      j_rarm_com_total_support += (mass_r_arm_(i-7)/mass_total_)*j_arm_com_link_support[i];
    }
  }

  double kc;
  double kp;
  double kd;
  double kf;
  double kw;
  double lambda;
  Eigen::Vector3d r_c1;
  Eigen::Matrix3d skew_r_c1;
  Eigen::Matrix3d skew_r2_r1;
  Eigen::Vector3d error_foot;
  Eigen::Vector4d error_foot_w;
  Eigen::Vector3d swing_foot_w;
  Eigen::Vector3d error_com;
  Eigen::Vector3d error_zmp;
  Eigen::Vector4d error_w;
  Eigen::Vector3d error_moment;

  double switch_l_ft;
  double switch_r_ft;
  kc = 50.0; kp = 0.0; kd = 0.000;  //gains for simulation
  kf = 50.0; kw = 100.0;

  //kc = 300.0; kp = 45.0; kd = 0.005;  //gains for real robot
  //kf = 300.0; kw = 200.0;
  lambda = 0.000;
  error_zmp.setZero();
  error_moment.setZero();
  moment_support_desried_.setZero();

  if(estimator_flag_ == true)
  {
    error_com(0) = com_desired_(0) - X_hat_post_2_(0);
    error_com(1) = com_desired_(1) - X_hat_post_2_(1);
    error_com(2) = com_desired_(2) - com_support_current_(2);
  }
  else
  {
    error_com = com_desired_ - com_support_current_;
  }

  if(l_ft_(2)+r_ft_(2) > 250)
  {
    error_zmp.segment<2>(0) = zmp_desired_ - zmp_measured_;
  }
  else if(walking_tick_%100 == 0)
  {
    cout<<"I'm flying"<<endl;
  }


  if(l_ft_(2) > 10)
  {
    switch_l_ft = 1;
  }
  else
  {
    switch_l_ft = 0;
  }

  if(r_ft_(2) > 10)
  {
    switch_r_ft = 1;
  }
  else
  {
    switch_r_ft = 0;
  }

  moment_support_desried_(0) = zmp_desired_(0)*(l_ft_(2)+r_ft_(2));
  moment_support_desried_(1) = zmp_desired_(1)*(l_ft_(2)+r_ft_(2));



  error_moment = moment_support_desried_ - moment_support_current_;


  disturbance_accel_old_ = disturbance_accel_;


  disturbance_accel_(0) = desired_u_dot_(0) - (switch_l_ft*(-l_ft_(4)) + switch_r_ft*(-r_ft_(4)))/(mass_total_*com_support_current_(2));
  disturbance_accel_(1) = desired_u_dot_(1) - (switch_l_ft*(l_ft_(3)) + switch_r_ft*(r_ft_(3)))/(mass_total_*com_support_current_(2));
  disturbance_accel_(2) = 0;

  //disturbance_accel_ = 0.3*disturbance_accel_ +0.7*disturbance_accel_old_;
  //cout<<"error_moment"<<error_moment<<endl;


  desired_u_old_ = desired_u_;
  desired_u_ = com_dot_desired_ + kc*(error_com) - kp*(error_zmp);
  //desired_u_ = com_dot_desired_ + kc*(error_com) - 1*(error_moment);
  //desired_u_ = com_dot_desired_ + kc*(error_com) - kd*(disturbance_accel_);

  desired_u_dot_ = (desired_u_ - desired_u_old_)*hz_;
  error_w = DyrosMath::rot2Axis(pelv_trajectory_support_.linear()*(pelv_support_current_.linear().transpose()));
  desired_w_ =  kw*(error_w.segment<3>(0)*error_w(3));


  if (foot_step_(current_step_num_, 6) == 1) //left support foot
  {


    r_c1 = com_support_current_ - lfoot_support_current_.translation();
    skew_r_c1 = DyrosMath::skew(r_c1);
    j1_ = adjoint_support_*current_leg_jacobian_l_;
    j2_ = adjoint_support_*current_leg_jacobian_r_;

    j_v1_ = j1_.block<3, 6>(0, 0);
    j_w1_ = j1_.block<3, 6>(3, 0);


    skew_r2_r1 = DyrosMath::skew(pelv_support_current_.linear()*(lfoot_float_current_.translation() - rfoot_float_current_.translation()));
    //Skew(FOOT.L_T.translation()-FOOT.R_T.translation(), skew_r2_r1);

    adjoint_21_.setIdentity();
    adjoint_21_.block<3, 3>(0, 3) = skew_r2_r1; //Spatial Translation Matrix from i_th link origin to i_th link com position with respect to base coordinate

    error_foot = rfoot_trajectory_support_.translation() - rfoot_support_current_.translation();
    error_foot_w = DyrosMath::rot2Axis(rfoot_trajectory_support_.linear() * (rfoot_support_current_.linear().transpose()));

    swing_foot_w.setZero();
    swing_foot_w(2) = rfoot_trajectory_dot_support_(5);

    x2_d_dot_.segment<3>(0) = rfoot_trajectory_dot_support_.segment<3>(0) + kf*(error_foot);
    x2_d_dot_.segment<3>(3) = swing_foot_w + kw*(error_foot_w.segment<3>(0)*error_foot_w(3));


    j_com_psem_ = -j_v1_ + skew_r_c1*j_w1_ + j_lleg_com_total_support + j_rleg_com_total_support*j2_.inverse()*adjoint_21_*j1_;

    desired_c_dot_psem_ = desired_u_ - j_rleg_com_total_support*j2_.inverse()*x2_d_dot_;



    //COM_dot_m = J_COM_PSEM*(_q_sudo_do t.segment<6>(22));
  }
  else //right support foot
  {
    r_c1 = com_support_current_ - rfoot_support_current_.translation();
    skew_r_c1 = DyrosMath::skew(r_c1);
    j1_ = adjoint_support_*current_leg_jacobian_r_;
    j2_ = adjoint_support_*current_leg_jacobian_l_;

    j_v1_ = j1_.block<3, 6>(0, 0);
    j_w1_ = j1_.block<3, 6>(3, 0);


    skew_r2_r1 = DyrosMath::skew(pelv_support_current_.linear()*(rfoot_float_current_.translation() - lfoot_float_current_.translation()));
    //Skew(FOOT.L_T.translation()-FOOT.R_T.translation(), skew_r2_r1);

    adjoint_21_.setIdentity();
    adjoint_21_.block<3, 3>(0, 3) = skew_r2_r1; //Spatial Translation Matrix from i_th link origin to i_th link com position with respect to base coordinate

    error_foot = lfoot_trajectory_support_.translation() - lfoot_support_current_.translation();
    error_foot_w = DyrosMath::rot2Axis(lfoot_trajectory_support_.linear() * (lfoot_support_current_.linear()).transpose());

    swing_foot_w.setZero();
    swing_foot_w(2) = lfoot_trajectory_dot_support_(5);

    x2_d_dot_.segment<3>(0) = lfoot_trajectory_dot_support_.segment<3>(0) + kf*(error_foot);
    x2_d_dot_.segment<3>(3) = swing_foot_w + kw*(error_foot_w.segment<3>(0)*error_foot_w(3));




    j_com_psem_ = -j_v1_ + skew_r_c1*j_w1_ + j_rleg_com_total_support + j_lleg_com_total_support*j2_.inverse()*adjoint_21_*j1_;

    desired_c_dot_psem_ = desired_u_ - j_lleg_com_total_support*j2_.inverse()*x2_d_dot_;



    //COM_dot_m = J_COM_PSEM*(_q_sudo_dot.segment<6>(16));
  }

}



void WalkingController::computeComJacobianControl(Eigen::Vector12d &desired_leg_q_dot)
{
  double lamb = 0.001;

  if (foot_step_(current_step_num_, 6) == 1) // left support foot
  {
    j_total_.block<3, 6>(0, 0) = j_com_psem_;
    j_total_.block<3, 6>(3, 0) = -j_w1_;

    c_total_.segment<3>(0) = desired_c_dot_psem_;
    c_total_.segment<3>(3) = desired_w_;


    desired_leg_q_dot.segment<6>(0) = j_total_.inverse()*c_total_;  //left
    desired_leg_q_dot.segment<6>(6) = j2_.inverse()*(x2_d_dot_ + adjoint_21_*j1_*desired_leg_q_dot.segment<6>(0));
  }
  else //right support foot
  {

    j_total_.block<3, 6>(0, 0) = j_com_psem_;
    j_total_.block<3, 6>(3, 0) = -j_w1_;

    c_total_.segment<3>(0) = desired_c_dot_psem_;
    c_total_.segment<3>(3) = desired_w_;


    desired_leg_q_dot.segment<6>(6) = j_total_.inverse()*c_total_; //right
    desired_leg_q_dot.segment<6>(0) = j2_.inverse()*(x2_d_dot_ + adjoint_21_*j1_*desired_leg_q_dot.segment<6>(6));
  }


}
}