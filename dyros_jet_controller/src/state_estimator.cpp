#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller.h"
#include "cvxgen_6_8_0/cvxgen/solver.h"


namespace dyros_jet_controller
{

void WalkingController::computeZmp()
{
  if(foot_step_(current_step_num_,6)==1 && walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_) // left support and right swing phase
  {
    zmp_r_.setZero();

  }
  else
  {
    //zmp_r_(0) = (-r_ft_(4) - r_ft_(0)*0.0062) / r_ft_(2);
    //zmp_r_(1) = (r_ft_(3) - r_ft_(1)*0.0062) / r_ft_(2);
    zmp_r_(0) = (-r_ft_filtered_(4)) / r_ft_filtered_(2);
    zmp_r_(1) = (r_ft_filtered_(3)) / r_ft_filtered_(2);
  }

  if(foot_step_(current_step_num_,6)==0 && walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_) // right support and left swing phase
  {
    zmp_l_.setZero();

  }
  else
  {
    //zmp_l_(0) = (-l_ft_(4) - l_ft_(0)*0.0062) / l_ft_(2);
    //zmp_l_(1) = (l_ft_(3) - l_ft_(1)*0.0062) / l_ft_(2);
    zmp_l_(0) = (-l_ft_filtered_(4)) / l_ft_filtered_(2);
    zmp_l_(1) = (l_ft_filtered_(3)) / l_ft_filtered_(2);

  }

  zmp_measured_ppre_ = zmp_measured_pre_;
  zmp_measured_pre_ = zmp_measured_;

  if (foot_step_(current_step_num_,6)==1) //left foot support
  {
    zmp_measured_(0) = ((((rfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_r_)(0) + (rfoot_support_current_.translation())(0))*r_ft_(2) + zmp_l_(0)*l_ft_(2)) / (r_ft_(2) + l_ft_(2));
    zmp_measured_(1) = ((((rfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_r_)(1) + (rfoot_support_current_.translation())(1))*r_ft_(2) + zmp_l_(1)*l_ft_(2)) / (r_ft_(2) + l_ft_(2));
    f_ft_support_ = l_ft_.segment<3>(0) + rfoot_support_current_.linear()*r_ft_.segment<3>(0);
  }
  else
  {
    zmp_measured_(0) = ((((lfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_l_)(0) + (lfoot_support_current_.translation())(0))*l_ft_(2) + zmp_r_(0)*r_ft_(2)) / (r_ft_(2) + l_ft_(2));
    zmp_measured_(1) = ((((lfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_l_)(1) + (lfoot_support_current_.translation())(1))*l_ft_(2) + zmp_r_(1)*r_ft_(2)) / (r_ft_(2) + l_ft_(2));
    f_ft_support_ = r_ft_.segment<3>(0) + lfoot_support_current_.linear()*l_ft_.segment<3>(0);
  }
}

void WalkingController::getQpEstimationInputMatrix()
{
  //double mass_total = 51.315;

  if(walking_tick_ == 0)
  {
    solve();
  }

  for (int i = 0; i < 6; i++)
  {
    x_estimation_(i) = vars.x[i];
  }




  Eigen::Vector2d FT_xy;

  FT_xy = r_ft_.topRows(2) + l_ft_.topRows(2);


  if(walking_tick_ == 0)
  {
    com_support_dot_current_.setZero();
    com_support_dot_old_estimation_.setZero();
    com_support_old_estimation_(0) = com_support_current_(0);
    com_support_old_estimation_(1) = com_support_current_(1);
    zmp_old_estimation_ = zmp_measured_;

    com_sim_dot_current_.setZero();
    com_sim_old_ = com_sim_current_;

  }
  else if(walking_tick_ == t_start_ &&  current_step_num_ != 0)
  {
    Eigen::Vector3d com_se_support_old;
    Eigen::Vector3d com_dot_se_support_old;
    Eigen::Vector3d zmp_se_support_old;

    com_se_support_old.setZero();
    com_dot_se_support_old.setZero();
    zmp_se_support_old.setZero();
    com_dot_se_support_old(0) = vars.x[2];
    com_dot_se_support_old(1) = vars.x[3];
    com_se_support_old(0) = vars.x[0];
    com_se_support_old(1) = vars.x[1];
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

      com_sim_old_ = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, com_sim_old_);
      com_support_old_ = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, com_support_old_);

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

      com_sim_old_ = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, com_sim_old_);
      com_support_old_ = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, com_support_old_);

    }

  }
  else if(walking_tick_ != 0)
  {


    //com_support_dot_current_ = (-2*com_support_old_.col(3)+9*com_support_old_.col(2)-18*com_support_old_.col(1)+11*com_support_old_.col(0))*hz_/6;
    com_support_dot_old_estimation_(0) = vars.x[0];
    com_support_dot_old_estimation_(1) = vars.x[1];
    com_support_old_estimation_(0) = vars.x[2];
    com_support_old_estimation_(1) = vars.x[3];
    zmp_old_estimation_(0) = vars.x[4];
    zmp_old_estimation_(1) = vars.x[5];

  }
  else
  {

    com_support_dot_old_estimation_(0) = x_estimation_(0);
    com_support_dot_old_estimation_(1) = x_estimation_(1);
    com_support_old_estimation_(0) = x_estimation_(2);
    com_support_old_estimation_(1) = x_estimation_(3);
    zmp_old_estimation_(0) = x_estimation_(4);
    zmp_old_estimation_(1) = x_estimation_(5);

  }

  if( walking_tick_ !=0)
  {
    com_sim_dot_current_ = (com_sim_current_ - com_sim_old_)*hz_;
    com_support_dot_current_ =(com_support_current_-com_support_old_)*hz_;
  }

  Eigen::Vector6d x_estimation_old;
  x_estimation_old.segment<2>(0) = com_support_old_estimation_;
  x_estimation_old.segment<2>(2) = com_support_dot_old_estimation_;
  x_estimation_old.segment<2>(4) = zmp_old_estimation_;

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
  a_f_(0, 2) = mass_total_*GRAVITY / (zc_);
  a_f_(1, 3) = mass_total_*GRAVITY / (zc_);
  a_f_(0, 4) = -mass_total_*GRAVITY / (zc_);
  a_f_(1, 5) = -mass_total_*GRAVITY / (zc_);
  b_f_ = FT_xy;

  a_noise_.setIdentity();
  b_noise_(0) = com_support_dot_old_estimation_(0);
  b_noise_(1) = com_support_dot_old_estimation_(1);
  b_noise_(2) = com_support_old_estimation_(0);
  b_noise_(3) = com_support_old_estimation_(1);
  b_noise_(4) = zmp_old_estimation_(0);
  b_noise_(5) = zmp_old_estimation_(1);

  /*
  if(walking_tick_ == 0 || walking_tick_ == t_start_ )

  {
    b_noise_(0) = com_support_dot_current_(0);
    b_noise_(1) = com_support_dot_current_(1);
    b_noise_(2) = com_support_current_(0);
    b_noise_(3) = com_support_current_(1);
    b_noise_(4) = zmp_measured_(0);
    b_noise_(5) = zmp_measured_(1);
    std::cout<<"here"<<endl;
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
  */

  /////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix<double, 10, 1> coeff;
  coeff.setIdentity();

  //gain tuning
  /*20180627 real robot gain*/
//  coeff(0) = 2e3;     //kinematic
//  coeff(1) = 1e2;     //c_dot_x
//  coeff(2) = 1e2;     //c_dot_y
//  coeff(3) = 1e2;     //c
//  coeff(4) = 5e3;     //zmp
//  coeff(5) = 1e2;     //approximation
//  coeff(6) = 1e-2;    //FT-accer
//  coeff(7) = 1e2;	    //noise of c_dot
//  coeff(8) = 1e-2;	    //noise of c
//  coeff(9) = 1e4;	    //noise of zmp


  //  /*20180622 real robot gain*/
  //  coeff(0) = 2e3;		//kinematic
  //  coeff(1) = 1e2;		//c_dot_x
  //  coeff(2) = 1e2;		//c_dot_y
  //  coeff(3) = 1e2;		//c
  //  coeff(4) = 5e3;		//zmp
  //  coeff(5) = 2e2;		//approximation
  //  coeff(6) = 1e-3;	//FT-accer
  //  coeff(7) = 1e2;	    //noise of c_dot
  //  coeff(8) = 1e-2;	    //noise of c
  //  coeff(9) = 1e2;	    //noise of zmp

  /*20180510 real robot gain

  coeff(0) = 5e2;		//kinematic
  coeff(1) = 1e3;		//c_dot_x
  coeff(2) = 1e3;		//c_dot_y
  coeff(3) = 1e1;		//c
  coeff(4) = 1e4;		//zmp
  coeff(5) = 8e1;		//approximation
  coeff(6) = 1e-3;	//FT-accer
  coeff(7) = 5e3;	    //noise of c_dot
  coeff(8) = 1e1;	    //noise of c
  coeff(9) = 3e4;	    //noise of zmp
  */
  /*
  if(walking_tick_ == t_start_ )
  {
    coeff(7) = 0.0;	    //noise of c_dot
    coeff(8) = 0.0;	    //noise of c
    coeff(9) = 0.0;	    //noise of zmp
  }
  */

//  coeff(0) = 5e0;		//c_dot_x
//  coeff(2) = 1e2;		//c_dot_y
//  coeff(3) = 1e1;		//c
//  coeff(4) = 3e4;		//zmp
//  coeff(5) = 6e1;		//approximation
//  coeff(6) = 2e-1;	//FT-accer
//  coeff(7) = 0e0;	    //noise of c_dot
//  coeff(8) = 4e0;	    //noise of c
//  coeff(9) = 2e4;	    //noise of zmp

  coeff(0) = 1e3;		//kinematic
  coeff(1) = 1e2;		//c_dot_x
  coeff(2) = 1e2;		//c_dot_y
  coeff(3) = 1e2;		//c
  coeff(4) = 1e4;		//zmp
  coeff(5) = 6e2;		//approximation
  coeff(6) = 2e-1;	//FT-accer
  coeff(7) = 0e0;	    //noise of c_dot
  coeff(8) = 4e0;	    //noise of c
  coeff(9) = 2e4;	    //noise of zmp




//  coeff(0) = 1e3;		//kinematic
//  coeff(1) = 1e2;		//c_dot_x
//  coeff(2) = 1e2;		//c_dot_y
//  coeff(3) = 1e1;		//c
//  coeff(4) = 9e3;		//zmp
//  coeff(5) = 6e2;		//approximation
//  coeff(6) = 2e-1;	//FT-accer
//  coeff(7) = 0e0;	    //noise of c_dot
//  coeff(8) = 4e0;	    //noise of c
//  coeff(9) = 2e4;	    //noise of zmp

//  coeff(0) = 1e3;		//kinematic
//  coeff(1) = 1e2;		//c_dot_x
//  coeff(2) = 1e2;		//c_dot_y
//  coeff(3) = 1e2;		//c
//  coeff(4) = 1e4;		//zmp
//  coeff(5) = 1e2;		//approximation
//  coeff(6) = 1e-2;	//FT-accer
//  coeff(7) = 4e0;	    //noise of c_dot
//  coeff(8) = 1e1;	    //noise of c
//  coeff(9) = 2e4;	    //noise of zmp


  /* for simluation*/

//  coeff(0) = 1e4;		//kinematic
//  coeff(1) = 1e2;		//c_dot_x
//  coeff(2) = 1e2;		//c_dot_y
//  coeff(3) = 1e2;		//c
//  coeff(4) = 1e4;		//zmp
//  coeff(5) = 2e2;		//approximation
//  coeff(6) = 2e-1;	//FT-accer
//  coeff(7) = 1e2;	    //noise of c_dot
//  coeff(8) = 1e0;	    //noise of c
//  coeff(9) = 1e3;	    //noise of zmp





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
//  Eigen::Matrix<double, 6, 6> q;

//  q = a_total_.transpose()*a_total_;

//  for (int i = 0; i < 6; i++)
//    for (int j = 0; j < 6; j++)
//    {
//      params.Q[j + i * 6] = q(j, i);
//    }

//  for (int i = 0; i < 6; i++)
//    params.c[i] = -2*(b_total_.transpose()*a_total_)(i);

//  for (int i = 0; i < 6; i++)
//    for (int j = 0; j < 8; j++)
//    {
//      params.Ai[j + i * 8] = 0;
//    }

  Eigen::Matrix<double, 6, 6> temp_Q;
  Eigen::Matrix<double, 6, 1> temp_x;
  Eigen::Matrix<double, 6, 1> temp_c;


  temp_Q = Q_1_.inverse() + Cd_1_.transpose()*R_1_.inverse()*Cd_1_;
  temp_x = Ad_1_*x_estimation_old+Bd_1_*u_old_1_;
  temp_c = -2*(temp_x.transpose()*Q_1_.inverse() + Y_1_.transpose()*R_1_.inverse()*Cd_1_);
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
    {
      params.Q[j + i * 6] = temp_Q(j, i);
    }

  for (int i = 0; i < 6; i++)
    params.c[i] = temp_c(i);

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 8; j++)
    {
      params.Ai[j + i * 8] = 0;
    }




  if( abs(l_ft_(2)-r_ft_(2))>51*9.81*0.8) //single support
  {
    params.Ai[20] = 1;
    params.Ai[22] = -1;
    params.Ai[29] = 1;
    params.Ai[31] = -1;
    params.Ai[32] = 1;
    params.Ai[34] = -1;
    params.Ai[41] = 1;
    params.Ai[43] = -1;

    params.bi[0] = 0.15;        //zmp x <= 0.15
    params.bi[1] = 0.085;       //zmp y <= 0.17
    params.bi[2] = 0.15;        //zmp x >= -0.15
    params.bi[3] = 0.085;       //zmp y >= -0.17
    params.bi[4] = 0.9;         //com x <= 0.15
    params.bi[5] = 0.9;         //com y <= 0.21
    params.bi[6] = 0.9;         //com x >= -0.15
    params.bi[7] = 0.9;         //com y >= -0.21
  }
  else // double support
  {
    params.bi[6] = 0;
    params.bi[7] = 0;
    if(foot_step_(current_step_num_, 6)==1) //left foot support
    {
      double m = rfoot_support_current_.translation()(1)/rfoot_support_current_.translation()(0);
      if(m < 0)
      {
        params.Ai[32] = 1;
        params.Ai[34] = -1;
        params.Ai[36] = m;
        params.Ai[37] = -m;
        params.Ai[41] = 1;
        params.Ai[43] = -1;
        params.Ai[44] = -1;
        params.Ai[45] = 1;
        params.bi[0] = 0.15 + rfoot_support_current_.translation()(0);
        params.bi[1] = 0.085;
        params.bi[2] = 0.15;
        params.bi[3] = 0.085 - rfoot_support_current_.translation()(1);
        params.bi[4] = -m*0.15 + 0.085;
        params.bi[5] = -m*0.15 + 0.085;
      }
      else
      {
        params.Ai[32] = 1;
        params.Ai[34] = -1;
        params.Ai[36] = m;
        params.Ai[37] = -m;
        params.Ai[41] = 1;
        params.Ai[43] = -1;
        params.Ai[44] = -1;
        params.Ai[45] = 1;
        params.bi[0] = 0.15 ;
        params.bi[1] = 0.085 ;
        params.bi[2] = 0.15 - rfoot_support_current_.translation()(0);
        params.bi[3] = 0.085 - rfoot_support_current_.translation()(1);
        params.bi[4] = m*0.15 + 0.085;
        params.bi[5] = m*0.15 + 0.085;
      }


    }
    else //right foot suppoprt
    {
      double m = lfoot_support_current_.translation()(1)/lfoot_support_current_.translation()(0);
      if(m < 0)
      {
        params.Ai[32] = 1;
        params.Ai[34] = -1;
        params.Ai[36] = m;
        params.Ai[37] = -m;
        params.Ai[41] = 1;
        params.Ai[43] = -1;
        params.Ai[44] = -1;
        params.Ai[45] = 1;
        params.bi[0] = 0.15 ;
        params.bi[1] = 0.085 + lfoot_support_current_.translation()(1);
        params.bi[2] = 0.15 - lfoot_support_current_.translation()(0);
        params.bi[3] = 0.085 ;
        params.bi[4] = -m*0.15 + 0.085;
        params.bi[5] = -m*0.15 + 0.085;
      }
      else
      {
        params.Ai[32] = 1;
        params.Ai[34] = -1;
        params.Ai[36] = m;
        params.Ai[37] = -m;
        params.Ai[41] = 1;
        params.Ai[43] = -1;
        params.Ai[44] = -1;
        params.Ai[45] = 1;
        params.bi[0] = 0.15 + + lfoot_support_current_.translation()(0);
        params.bi[1] = 0.085 + + lfoot_support_current_.translation()(1);
        params.bi[2] = 0.15;
        params.bi[3] = 0.085 ;
        params.bi[4] = +m*0.15 + 0.085;
        params.bi[5] = +m*0.15 + 0.085;
      }
    }
  }





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

void WalkingController::kalmanStateSpace1()
{

  /*
  Ad_1_ = [ 1       0         dT        0         0       0       ]
          [ 0       1         0         dT        0       0       ]
          [ w^2dT   0         1         0         -w^2dT  0       ]
          [ 0       w^2dT     0         1         0       -w^2dT  ]
          [ 0       0         0         0         1       0       ]
          [ 0       0         0         0         0       1       ]

  Bd_1_ = [ 0 ]
          [ 0 ]
          [ 0 ]
          [ 0 ]
          [ 1 ]
          [ 1 ]



  CD_1_ = [ 1         0         0         0         0         0         ]
          [ 0         1         0         0         0         0         ]
          [ 0         0         0         0         1         0         ]
          [ 0         0         0         0         0         1         ]

  */

  double omega_square = GRAVITY/model_.getCurrentCom()(2);


  Q_1_.setIdentity();
  R_1_.setIdentity();

  Q_1_(0, 0) = Q_3_(0, 0);	//com position
  Q_1_(1, 1) = Q_3_(1, 1);

  Q_1_(2, 2) = Q_3_(2, 2);	//com velocity
  Q_1_(3, 3) = Q_3_(3, 3);

  Q_1_(4, 4) = Q_3_(4, 4);	//zmp
  Q_1_(5, 5) = Q_3_(5, 5);


  R_1_(0, 0) = R_3_(0, 0);
  R_1_(1, 1) = R_3_(1, 1);

  R_1_(2, 2) = R_3_(2, 2);
  R_1_(3, 3) = R_3_(3, 3);


  Ad_1_.setIdentity();
  Ad_1_(0, 2) = 1 / hz_;
  Ad_1_(1, 3) = 1 / hz_;
  Ad_1_(2, 0) = omega_square / (hz_);
  Ad_1_(3, 1) = omega_square / (hz_);
  Ad_1_(2, 4) = -omega_square / (hz_);
  Ad_1_(3, 5) = -omega_square / (hz_);
  Ad_1_(4, 4) = 0.0;
  Ad_1_(5, 5) = 0.0;

  Bd_1_.setZero();
  Bd_1_(4, 0) = 1 ;
  Bd_1_(5, 1) = 1 ;


  Cd_1_.setZero();
  Cd_1_(0, 0) = 1;
  Cd_1_(1, 1) = 1;
  Cd_1_(2, 4) = 1;
  Cd_1_(3, 5) = 1;

}


void WalkingController::kalmanFilter1()
{
  Eigen::Vector3d com_kal_support_old;
  Eigen::Vector3d com_dot_kal_support_old;
  Eigen::Vector3d zmp_kal_support_old;
  Eigen::Matrix<double, 6, 6> rotation_stack;

  com_kal_support_old.setZero();
  com_dot_kal_support_old.setZero();
  zmp_kal_support_old.setZero();
  com_dot_kal_support_old(0) = X_hat_post_1_(2);
  com_dot_kal_support_old(1) = X_hat_post_1_(3);
  com_kal_support_old(0) = X_hat_post_1_(0);
  com_kal_support_old(1) = X_hat_post_1_(1);
  zmp_kal_support_old(0) = X_hat_post_1_(4);
  zmp_kal_support_old(1) = X_hat_post_1_(5);

  if(walking_tick_ ==0)
  {
    X_hat_post_old_1_(0) = com_support_current_(0);
    X_hat_post_old_1_(1) = com_support_current_(1);
    X_hat_post_old_1_(2) = com_support_dot_current_(0);
    X_hat_post_old_1_(3) = com_support_dot_current_(1);
    X_hat_post_old_1_(4) = zmp_measured_(0);
    X_hat_post_old_1_(5) = zmp_measured_(1);



    P_post_old_1_.setZero();
  }
  else if (walking_tick_ == t_start_)
  {
    if(current_step_num_ > 0)
    {
      if (foot_step_(current_step_num_, 6) == 0)
      {

        rotation_stack.setIdentity();

        rotation_stack.block<2, 2>(0, 0) = lfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(2, 2) = lfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(4, 4) = lfoot_support_current_.linear().block<2, 2>(0, 0);

        X_hat_post_old_1_(2) = (lfoot_support_current_.linear()*com_dot_kal_support_old)(0);
        X_hat_post_old_1_(3) = (lfoot_support_current_.linear()*com_dot_kal_support_old)(1);
        X_hat_post_old_1_(0) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, com_kal_support_old)(0);
        X_hat_post_old_1_(1) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, com_kal_support_old)(1);
        X_hat_post_old_1_(4) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, zmp_kal_support_old)(0);
        X_hat_post_old_1_(5) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, zmp_kal_support_old)(1);

        P_post_old_1_ = rotation_stack * P_post_1_*rotation_stack.transpose();

      }
      else
      {

        rotation_stack.setIdentity();

        rotation_stack.block<2, 2>(0, 0) = rfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(2, 2) = rfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(4, 4) = rfoot_support_current_.linear().block<2, 2>(0, 0);

        X_hat_post_old_1_(2) = (rfoot_support_current_.linear()*com_dot_kal_support_old)(0);
        X_hat_post_old_1_(3) = (rfoot_support_current_.linear()*com_dot_kal_support_old)(1);
        X_hat_post_old_1_(0) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, com_kal_support_old)(0);
        X_hat_post_old_1_(1) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, com_kal_support_old)(1);
        X_hat_post_old_1_(4) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, zmp_kal_support_old)(0);
        X_hat_post_old_1_(5) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, zmp_kal_support_old)(1);

        P_post_old_1_ = rotation_stack * P_post_1_*rotation_stack.transpose();

      }
    }
    else
    {
      X_hat_post_old_1_ = X_hat_post_1_;
      P_post_old_1_ = P_post_1_;
      K_old_1_ = K_1_;
    }
  }
  else
  {
    X_hat_post_old_1_ = X_hat_post_1_;
    P_post_old_1_ = P_post_1_;
    K_old_1_ = K_1_;
  }



  Y_1_(0) = com_support_current_(0);
  Y_1_(1) = com_support_current_(1);
  Y_1_(2) = zmp_measured_(0);
  Y_1_(3) = zmp_measured_(1);


  u_old_1_(0) = zmp_measured_(0);
  u_old_1_(1) = zmp_measured_(1);

  ///prediction procedure///
  X_hat_prio_1_ = Ad_1_ * X_hat_post_old_1_ + Bd_1_ * u_old_1_;
  P_prio_1_ = Ad_1_ * P_post_old_1_*Ad_1_.transpose() + Q_1_;

  ///crrection procedure///
  Eigen::Matrix<double, 4, 4 > temp_K;



  temp_K = (Cd_1_*P_prio_1_*Cd_1_.transpose() + R_1_).inverse();
  K_1_ = P_prio_1_ * Cd_1_.transpose() * temp_K;
  X_hat_post_1_ = X_hat_prio_1_ + K_1_ * (Y_1_ - Cd_1_ * X_hat_prio_1_);
  P_post_1_ = (Eigen::Matrix<double, 6, 6>::Identity() - K_1_ * Cd_1_)*P_prio_1_;

  //cout << "P_prio_" << P_prio_ << endl;

  //cout << "P_prio_ * Cd_.transpose()" << P_prio_ * Cd_.transpose() << endl;

  //cout << "K_" << K_ << endl;
}

void WalkingController::kalmanStateSpace2()
{
  /*
  Ad_2_ = [ 1       0         dT        0         0       0         0          0   ]
          [ 0       1         0         dT        0       0         0          0   ]
          [ w^2dT   0         1         0         -w^2dT  0         0          0   ]
          [ 0       w^2dT     0         1         0       -w^2dT    0          0   ]
          [ 0       0         0         0         0       0         0          0   ]
          [ 0       0         0         0         0       0         0          0   ]
          [ 0       0         0         0         0       0         1          0   ]
          [ 0       0         0         0         0       0         0          1   ]

  Bd_2_ = [ 0 ]
          [ 0 ]
          [ 0 ]
          [ 0 ]
          [ 1 ]
          [ 1 ]
          [ 0 ]
          [ 0 ]

  cD_2_ = [ 1         0         0         0         0         0         -1        0 ]
          [ 0         1         0         0         0         0         0         -1]
          [ 0         0         0         0         1         0         0         0 ]
          [ 0         0         0         0         0         1         0         0 ]

  */

  double omega_square = GRAVITY/model_.getCurrentCom()(2);


  Q_2_.setIdentity();
  R_2_.setIdentity();

  Q_2_(0, 0) = Q_3_(0, 0);	//com position
  Q_2_(1, 1) = Q_3_(1, 1);

  Q_2_(2, 2) = Q_3_(2, 2);	//com velocity
  Q_2_(3, 3) = Q_3_(3, 3);

  Q_2_(4, 4) = Q_3_(4, 4);	//zmp
  Q_2_(5, 5) = Q_3_(5, 5);

  Q_2_(6, 6) = Q_3_(6, 6);	 //com position error
  Q_2_(7, 7) = Q_3_(7, 7);

  R_2_(0, 0) = R_3_(0, 0);
  R_2_(1, 1) = R_3_(1, 1);

  R_2_(2, 2) = R_3_(2, 2);
  R_2_(3, 3) = R_3_(3, 3);



  Ad_2_.setIdentity();
  Ad_2_(0, 2) = 1 / hz_;
  Ad_2_(1, 3) = 1 / hz_;
  Ad_2_(2, 0) = omega_square / (hz_);
  Ad_2_(3, 1) = omega_square / (hz_);
  Ad_2_(2, 4) = -omega_square / (hz_);
  Ad_2_(3, 5) = -omega_square / (hz_);
  Ad_2_(4, 4) = 0.0;
  Ad_2_(5, 5) = 0.0;

  Bd_2_.setZero();
  Bd_2_(4, 0) = 1 ;
  Bd_2_(5, 1) = 1 ;


  Cd_2_.setZero();
  Cd_2_(0, 0) = 1;
  Cd_2_(1, 1) = 1;
  Cd_2_(0, 6) = -1;
  Cd_2_(1, 7) = -1;
  Cd_2_(2, 4) = 1;
  Cd_2_(3, 5) = 1;
}


void WalkingController::kalmanFilter2()
{


  Eigen::Vector3d com_kal_support_old;
  Eigen::Vector3d com_dot_kal_support_old;
  Eigen::Vector3d zmp_kal_support_old;
  Eigen::Vector3d combias_kal_support_old;

  com_kal_support_old.setZero();
  com_dot_kal_support_old.setZero();
  zmp_kal_support_old.setZero();
  combias_kal_support_old.setZero();
  com_dot_kal_support_old(0) = X_hat_post_2_(2);
  com_dot_kal_support_old(1) = X_hat_post_2_(3);
  com_kal_support_old(0) = X_hat_post_2_(0);
  com_kal_support_old(1) = X_hat_post_2_(1);
  zmp_kal_support_old(0) = X_hat_post_2_(4);
  zmp_kal_support_old(1) = X_hat_post_2_(5);
  combias_kal_support_old(0) = X_hat_post_2_(6);
  combias_kal_support_old(1) = X_hat_post_2_(7);

  if(walking_tick_ ==0)
  {
    X_hat_post_old_2_(0) = com_support_current_(0);
    X_hat_post_old_2_(1) = com_support_current_(1);
    X_hat_post_old_2_(2) = com_support_dot_current_(0);
    X_hat_post_old_2_(3) = com_support_dot_current_(1);
    X_hat_post_old_2_(4) = zmp_measured_(0);
    X_hat_post_old_2_(5) = zmp_measured_(1);
    X_hat_post_old_2_(6) = 0;
    X_hat_post_old_2_(7) = 0;

    P_post_old_2_.setZero();
  }
  else if (walking_tick_ == t_start_)
  {
    if(current_step_num_ > 0)
    {
      if (foot_step_(current_step_num_, 6) == 0)
      {
        Eigen::Matrix<double, 8, 8> rotation_stack;

        rotation_stack.setIdentity();

        rotation_stack.block<2, 2>(0, 0) = lfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(2, 2) = lfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(4, 4) = lfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(6, 6) = lfoot_support_current_.linear().block<2, 2>(0, 0);

        X_hat_post_old_2_(2) = (lfoot_support_current_.linear()*com_dot_kal_support_old)(0);
        X_hat_post_old_2_(3) = (lfoot_support_current_.linear()*com_dot_kal_support_old)(1);
        X_hat_post_old_2_(0) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, com_kal_support_old)(0);
        X_hat_post_old_2_(1) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, com_kal_support_old)(1);
        X_hat_post_old_2_(4) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, zmp_kal_support_old)(0);
        X_hat_post_old_2_(5) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, zmp_kal_support_old)(1);
        X_hat_post_old_2_(6) = (lfoot_support_current_.linear()*combias_kal_support_old)(0);
        X_hat_post_old_2_(7) = (lfoot_support_current_.linear()*combias_kal_support_old)(1);

        P_post_old_2_ = rotation_stack * P_post_2_*rotation_stack.transpose();

      }
      else
      {
        Eigen::Matrix<double, 8, 8> rotation_stack;

        rotation_stack.setIdentity();

        rotation_stack.block<2, 2>(0, 0) = rfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(2, 2) = rfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(4, 4) = rfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(6, 6) = rfoot_support_current_.linear().block<2, 2>(0, 0);

        X_hat_post_old_2_(2) = (rfoot_support_current_.linear()*com_dot_kal_support_old)(0);
        X_hat_post_old_2_(3) = (rfoot_support_current_.linear()*com_dot_kal_support_old)(1);
        X_hat_post_old_2_(0) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, com_kal_support_old)(0);
        X_hat_post_old_2_(1) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, com_kal_support_old)(1);
        X_hat_post_old_2_(4) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, zmp_kal_support_old)(0);
        X_hat_post_old_2_(5) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, zmp_kal_support_old)(1);
        X_hat_post_old_2_(6) = (rfoot_support_current_.linear()*combias_kal_support_old)(0);
        X_hat_post_old_2_(7) = (rfoot_support_current_.linear()*combias_kal_support_old)(1);

        P_post_old_2_ = rotation_stack * P_post_2_*rotation_stack.transpose();

      }
    }
    else
    {
      X_hat_post_old_2_ = X_hat_post_2_;
      P_post_old_2_ = P_post_2_;
      K_old_2_ = K_2_;
    }
  }
  else
  {
    X_hat_post_old_2_ = X_hat_post_2_;
    P_post_old_2_ = P_post_2_;
    K_old_2_ = K_2_;
  }



  Y_2_(0) = com_support_current_(0);
  Y_2_(1) = com_support_current_(1);
  Y_2_(2) = zmp_measured_(0);
  Y_2_(3) = zmp_measured_(1);


  u_old_2_(0) = zmp_measured_(0);
  u_old_2_(1) = zmp_measured_(1);

  ///prediction procedure///
  X_hat_prio_2_ = Ad_2_ * X_hat_post_old_2_ + Bd_2_ * u_old_2_;
  P_prio_2_ = Ad_2_ * P_post_old_2_*Ad_2_.transpose() + Q_2_;

  ///crrection procedure///
  Eigen::Matrix<double, 4, 4 > temp_K;



  temp_K = (Cd_2_*P_prio_2_*Cd_2_.transpose() + R_2_).inverse();
  K_2_ = P_prio_2_ * Cd_2_.transpose() * temp_K;
  X_hat_post_2_ = X_hat_prio_2_ + K_2_ * (Y_2_ - Cd_2_ * X_hat_prio_2_);
  P_post_2_ = (Eigen::Matrix<double, 8, 8>::Identity() - K_2_ * Cd_2_)*P_prio_2_;

  //cout << "P_prio_" << P_prio_ << endl;

  //cout << "P_prio_ * Cd_.transpose()" << P_prio_ * Cd_.transpose() << endl;

  //cout << "K_" << K_ << endl;
}

void WalkingController::kalmanStateSpace3()
{
  /*
  Ad_3_ = [ 1       0         dT        0         0       0         0          0          0         0]
          [ 0       1         0         dT        0       0         0          0          0         0]
          [ w^2dT   0         1         0         -w^2dT  0         0          0          dT        0]
          [ 0       w^2dT     0         1         0       -w^2dT    0          0          0         dT]
          [ 0       0         0         0         0       0         0          0          0         0]
          [ 0       0         0         0         0       0         0          0          0         0]
          [ 0       0         0         0         0       0         1          0          0         0]
          [ 0       0         0         0         0       0         0          1          0         0]
          [ 0       0         0         0         0       0         0          0          1         0]
          [ 0       0         0         0         0       0         0          0          0         1]

  Bd_3_ = [ 0 ]
          [ 0 ]
          [ 0 ]
          [ 0 ]
          [ 1 ]
          [ 1 ]
          [ 0 ]
          [ 0 ]
          [ 0 ]
          [ 0 ]

  cD_3_ = [ 1         0         0         0         0         0         -1        0         0         0 ]
          [ 0         1         0         0         0         0         0         -1        0         0 ]
          [ 0         0         0         0         1         0         0         0         0         0 ]
          [ 0         0         0         0         0         1         0         0         0         0 ]
          [ w^2dT     0         0         0         -w^2dT    0         0         0         1         0 ]
          [ 0         w^2dT     0         0         0         -w^2dT    0         0         0         1 ]

  */

  double omega_square = GRAVITY/model_.getCurrentCom()(2);

  double mass_total = 51.315;

  Q_3_.setIdentity();
  R_3_.setIdentity();

  Q_3_(0, 0) = 1e-5;	//com position
  Q_3_(1, 1) = 1e-5;

  Q_3_(2, 2) = 1e-6;	//com velocity
  Q_3_(3, 3) = 1e-6;

  Q_3_(4, 4) = 1e-7;	//zmp
  Q_3_(5, 5) = 1e-7;

  Q_3_(6, 6) = 1e-9;	 //com position error
  Q_3_(7, 7) = 1e-9;

  Q_3_(8, 8) = 1e-8;	 //model error
  Q_3_(9, 9) = 1e-8;

  R_3_(0, 0) = 1e-6;   //com
  R_3_(1, 1) = 1e-6;

  R_3_(2, 2) = 1e-9;    //zmp
  R_3_(3, 3) = 1e-9;

  R_3_(4, 4) = 1e-4;    //imu
  R_3_(5, 5) = 1e-4;


  /*
  /// 20180716 real robot Covariance gain

  Q_3_(0, 0) = 1e-4;	//com position
  Q_3_(1, 1) = 1e-4;

  Q_3_(2, 2) = 1e-5;	//com velocity
  Q_3_(3, 3) = 1e-5;

  Q_3_(4, 4) = 1e-6;	//zmp
  Q_3_(5, 5) = 1e-6;

  Q_3_(6, 6) = 1e-8;	 //com position error
  Q_3_(7, 7) = 1e-8;

  Q_3_(8, 8) = 1e-8;	 //model error
  Q_3_(9, 9) = 1e-8;

  R_3_(0, 0) = 1e-5;   //com
  R_3_(1, 1) = 1e-5;

  R_3_(2, 2) = 1e-9;    //zmp
  R_3_(3, 3) = 1e-9;

  R_3_(4, 4) = 1e-7;    //imu
  R_3_(5, 5) = 1e-7;
*/


  //Q_(0, 0) = 1e-0;	//com position
  //Q_(1, 1) = 1e-0;
  //
  //Q_(2, 2) = 1e-2;	//com velocity
  //Q_(3, 3) = 1e-2;
  //
  //Q_(4, 4) = 1e-3;	//zmp
  //Q_(5, 5) = 1e-3;
  //
  //Q_(6, 6) = 1e1;	//com position error
  //Q_(7, 7) = 1e1;
  //
  //R_(0, 0) = 1e-2;
  //R_(1, 1) = 1e-2;
  //
  //R_(2, 2) = 1e-6;
  //R_(3, 3) = 1e-6;



  Ad_3_.setIdentity();
  Ad_3_(0, 2) = 1 / hz_;
  Ad_3_(1, 3) = 1 / hz_;
  Ad_3_(2, 0) = omega_square / (hz_);
  Ad_3_(3, 1) = omega_square / (hz_);
  Ad_3_(2, 4) = -omega_square / (hz_);
  Ad_3_(3, 5) = -omega_square / (hz_);
  Ad_3_(2, 8) = 1 / hz_;
  Ad_3_(3, 9) = 1 / hz_;
  Ad_3_(4, 4) = 0.0;
  Ad_3_(5, 5) = 0.0;

  Bd_3_.setZero();
  Bd_3_(4, 0) = 1 ;
  Bd_3_(5, 1) = 1 ;


  Cd_3_.setZero();
  Cd_3_(0, 0) = 1;
  Cd_3_(1, 1) = 1;
  Cd_3_(0, 6) = -1;
  Cd_3_(1, 7) = -1;
  Cd_3_(2, 4) = 1;
  Cd_3_(3, 5) = 1;
  Cd_3_(4, 0) = omega_square / (hz_);
  Cd_3_(5, 1) = omega_square / (hz_);
  Cd_3_(4, 4) = -omega_square / (hz_);
  Cd_3_(5, 5) = -omega_square / (hz_);
  Cd_3_(4, 8) = 1;
  Cd_3_(5, 9) = 1;

}


void WalkingController::kalmanFilter3()
{


  Eigen::Vector3d com_kal_support_old;
  Eigen::Vector3d com_dot_kal_support_old;
  Eigen::Vector3d zmp_kal_support_old;
  Eigen::Vector3d combias_kal_support_old;
  Eigen::Vector3d modelbias_kal_support_old;

  Eigen::Matrix<double, 10, 10> rotation_stack;

  com_kal_support_old.setZero();
  com_dot_kal_support_old.setZero();
  zmp_kal_support_old.setZero();
  combias_kal_support_old.setZero();
  modelbias_kal_support_old.setZero();
  com_dot_kal_support_old(0) = X_hat_post_3_(2);
  com_dot_kal_support_old(1) = X_hat_post_3_(3);
  com_kal_support_old(0) = X_hat_post_3_(0);
  com_kal_support_old(1) = X_hat_post_3_(1);
  zmp_kal_support_old(0) = X_hat_post_3_(4);
  zmp_kal_support_old(1) = X_hat_post_3_(5);
  combias_kal_support_old(0) = X_hat_post_3_(6);
  combias_kal_support_old(1) = X_hat_post_3_(7);
  modelbias_kal_support_old(0) = X_hat_post_3_(8);
  modelbias_kal_support_old(1) = X_hat_post_3_(9);

  if(walking_tick_ ==0)
  {
    X_hat_post_old_3_(0) = com_support_current_(0);
    X_hat_post_old_3_(1) = com_support_current_(1);
    X_hat_post_old_3_(2) = com_support_dot_current_(0);
    X_hat_post_old_3_(3) = com_support_dot_current_(1);
    X_hat_post_old_3_(4) = zmp_measured_(0);
    X_hat_post_old_3_(5) = zmp_measured_(1);
    X_hat_post_old_3_(6) = 0;
    X_hat_post_old_3_(7) = 0;
    X_hat_post_old_3_(8) = 0;
    X_hat_post_old_3_(9) = 0;

    P_post_old_3_.setZero();
  }
  else if (walking_tick_ == t_start_)
  {
    if(current_step_num_ > 0)
    {
      if (foot_step_(current_step_num_, 6) == 0)
      {


        rotation_stack.setIdentity();

        rotation_stack.block<2, 2>(0, 0) = lfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(2, 2) = lfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(4, 4) = lfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(6, 6) = lfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(8, 8) = lfoot_support_current_.linear().block<2, 2>(0, 0);

        X_hat_post_old_3_(2) = (lfoot_support_current_.linear()*com_dot_kal_support_old)(0);
        X_hat_post_old_3_(3) = (lfoot_support_current_.linear()*com_dot_kal_support_old)(1);
        X_hat_post_old_3_(0) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, com_kal_support_old)(0);
        X_hat_post_old_3_(1) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, com_kal_support_old)(1);
        X_hat_post_old_3_(4) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, zmp_kal_support_old)(0);
        X_hat_post_old_3_(5) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, zmp_kal_support_old)(1);
        X_hat_post_old_3_(6) = (lfoot_support_current_.linear()*combias_kal_support_old)(0);
        X_hat_post_old_3_(7) = (lfoot_support_current_.linear()*combias_kal_support_old)(1);
        X_hat_post_old_3_(8) = (lfoot_support_current_.linear()*modelbias_kal_support_old)(0);
        X_hat_post_old_3_(9) = (lfoot_support_current_.linear()*modelbias_kal_support_old)(1);

        P_post_old_3_ = rotation_stack * P_post_3_*rotation_stack.transpose();

      }
      else
      {

        rotation_stack.setIdentity();

        rotation_stack.block<2, 2>(0, 0) = rfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(2, 2) = rfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(4, 4) = rfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(6, 6) = rfoot_support_current_.linear().block<2, 2>(0, 0);
        rotation_stack.block<2, 2>(8, 8) = lfoot_support_current_.linear().block<2, 2>(0, 0);

        X_hat_post_old_3_(2) = (rfoot_support_current_.linear()*com_dot_kal_support_old)(0);
        X_hat_post_old_3_(3) = (rfoot_support_current_.linear()*com_dot_kal_support_old)(1);
        X_hat_post_old_3_(0) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, com_kal_support_old)(0);
        X_hat_post_old_3_(1) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, com_kal_support_old)(1);
        X_hat_post_old_3_(4) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, zmp_kal_support_old)(0);
        X_hat_post_old_3_(5) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, zmp_kal_support_old)(1);
        X_hat_post_old_3_(6) = (rfoot_support_current_.linear()*combias_kal_support_old)(0);
        X_hat_post_old_3_(7) = (rfoot_support_current_.linear()*combias_kal_support_old)(1);
        X_hat_post_old_3_(8) = (rfoot_support_current_.linear()*modelbias_kal_support_old)(0);
        X_hat_post_old_3_(9) = (rfoot_support_current_.linear()*modelbias_kal_support_old)(1);

        P_post_old_3_ = rotation_stack * P_post_3_*rotation_stack.transpose();

      }
    }
    else
    {
      X_hat_post_old_3_ = X_hat_post_3_;
      P_post_old_3_ = P_post_3_;
      K_old_3_ = K_3_;
    }
  }
  else
  {
    X_hat_post_old_3_ = X_hat_post_3_;
    P_post_old_3_ = P_post_3_;
    K_old_3_ = K_3_;
  }



  Y_3_(0) = com_support_current_(0);
  Y_3_(1) = com_support_current_(1);
  Y_3_(2) = zmp_measured_(0);
  Y_3_(3) = zmp_measured_(1);
  Y_3_(4) = imu_acc_(0);
  Y_3_(5) = imu_acc_(1);

  //cout <<"accel_sim_current_"<<accel_sim_current_<<endl;


  u_old_3_(0) = zmp_measured_(0);
  u_old_3_(1) = zmp_measured_(1);

  ///prediction procedure///
  X_hat_prio_3_ = Ad_3_ * X_hat_post_old_3_ + Bd_3_ * u_old_3_;
  P_prio_3_ = Ad_3_ * P_post_old_3_*Ad_3_.transpose() + Q_3_;

  ///corection procedure///
  Eigen::Matrix<double, 6, 6 > temp_K;



  temp_K = (Cd_3_*P_prio_3_*Cd_3_.transpose() + R_3_).inverse();
  K_3_ = P_prio_3_ * Cd_3_.transpose() * temp_K;
  X_hat_post_3_ = X_hat_prio_3_ + K_3_ * (Y_3_ - Cd_3_ * X_hat_prio_3_);
  P_post_3_ = (Eigen::Matrix<double, 10, 10>::Identity() - K_3_ * Cd_3_)*P_prio_3_;

  //cout << "P_prio_" << P_prio_ << endl;

  //cout << "P_prio_ * Cd_.transpose()" << P_prio_ * Cd_.transpose() << endl;

  //cout << "K_" << K_ << endl;
}

/////////////////////////////////////////ExtendedKalmanFilter1//////////////////////////////////////////////////
/// \brief WalkingController::EKF1Init
/// this code is from github: "https://github.com/mrsp/serow"
///
void WalkingController::EKF1Init()
{

  F  = Eigen::Matrix<double,12,12>::Zero();
  Fd  = Eigen::Matrix<double,12,12>::Zero();
  I = Eigen::Matrix<double,12,12>::Identity();


  Q = Eigen::Matrix<double,12,12>::Zero();


  //Measurement Noise
  x = Eigen::Matrix<double,12,1>::Zero();
  f = Eigen::Matrix<double,12,1>::Zero();
  P = Eigen::Matrix<double,12,12>::Identity();
  z = Eigen::Matrix<double,9,1>::Zero();
  R = Eigen::Matrix<double,9,9>::Zero();
  H = Eigen::Matrix<double,9,12>::Zero();
  K = Eigen::Matrix<double,12,9>::Zero();
  P.block<3,3>(0,0) = 1e-6 * Eigen::Matrix<double,3,3>::Identity();
  P.block<3,3>(3,3) = 1e-2 * Eigen::Matrix<double,3,3>::Identity();
  P.block<3,3>(6,6) = 1e-1 * Eigen::Matrix<double,3,3>::Identity();
  P.block<3,3>(9,9) = Eigen::Matrix<double,3,3>::Zero();

  comX = 0.000;
  comY = 0.000;
  comZ = 0.000;

  velX = 0.000;
  velY = 0.000;
  velZ = 0.000;

  fX = 0.000;
  fY = 0.000;
  fZ = 0.000;

  com_x_error_ = 0.000;
  com_y_error_ = 0.000;
  com_z_error_ = 0.000;

  firstrun = true;

  std::cout << "Non-linear CoM Estimator Initialized!" << std::endl;
}


void WalkingController::EKF1Predict(Eigen::Vector3d COP_, Eigen::Vector3d fN_, Eigen::Vector3d ti_, Eigen::Vector3d L_)
{
  ti = ti_;
  COP = COP_;
  fN = fN_;
  L = L_;
  /*check support foot change*/
  Eigen::Matrix<double, 12, 12> rotation_stack;

  if (walking_tick_ == t_start_)
  {
    if(current_step_num_ > 0)
    {
      if (foot_step_(current_step_num_, 6) == 0) // right support
      {


        rotation_stack.setIdentity();

        rotation_stack.block<3, 3>(0, 0) = lfoot_support_current_.linear();
        rotation_stack.block<3, 3>(3, 3) = lfoot_support_current_.linear();
        rotation_stack.block<3, 3>(6, 6) = lfoot_support_current_.linear();
        rotation_stack.block<3, 3>(9, 9) = lfoot_support_current_.linear();

        x.segment<3>(0) = DyrosMath::multiplyIsometry3dVector3d(lfoot_support_current_, x.segment<3>(0));
        x.segment<3>(3) = (lfoot_support_current_.linear()*x.segment<3>(3));
        x.segment<3>(6) = (lfoot_support_current_.linear()*x.segment<3>(6));
        x.segment<3>(9) = (lfoot_support_current_.linear()*x.segment<3>(9));

        P = rotation_stack * P*rotation_stack.transpose();
      }
      else // left support
      {
        rotation_stack.setIdentity();

        rotation_stack.block<3, 3>(0, 0) = rfoot_support_current_.linear();
        rotation_stack.block<3, 3>(3, 3) = rfoot_support_current_.linear();
        rotation_stack.block<3, 3>(6, 6) = rfoot_support_current_.linear();
        rotation_stack.block<3, 3>(9, 9) = rfoot_support_current_.linear();

        x.segment<3>(0) = DyrosMath::multiplyIsometry3dVector3d(rfoot_support_current_, x.segment<3>(0));
        x.segment<3>(3) = (rfoot_support_current_.linear()*x.segment<3>(3));
        x.segment<3>(6) = (rfoot_support_current_.linear()*x.segment<3>(6));
        x.segment<3>(9) = (rfoot_support_current_.linear()*x.segment<3>(9));

        P = rotation_stack * P*rotation_stack.transpose();
      }
    }
  }


  /*Predict Step*/
  F.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
  tmp = x(2)-COP(2);

  F(3, 0) = (fN(2)) / (m * tmp);

  F(3, 2) = -((fN(2)) * (x(0) - COP(0))) / (m * tmp * tmp) +
   (L(1)) / (m * tmp * tmp);

  F(3, 6) = 1.000 / m;

  F(3, 8) = (x(0) - COP(0)) / (m * tmp);

  F(4, 1) = (fN(2)) / (m * tmp);

  F(4, 2) = - (fN(2)) * ( x(1) - COP(1) ) / (m * tmp * tmp) -
  (L(0)) / (m * tmp * tmp);

  F(4, 7) =  1.000 / m;

  F(4, 8) = (x(1) - COP(1)) / (m * tmp);

  F(5, 8) =   1.000 / m;


  //Discretization
  Q(0, 0) = com_q * com_q;
  Q(1, 1) = Q(0,0);
  Q(2, 2) = Q(0,0);

  Q(3, 3) = comd_q * comd_q;
  Q(4, 4) = Q(3, 3);
  Q(5, 5) = Q(3, 3);

  Q(6, 6) = fd_q * fd_q;
  Q(7, 7) = Q(6, 6);
  Q(8, 8) = Q(6, 6);

  Q(9, 9) = comerr_q * comerr_q;
  Q(10, 10) = Q(9, 9);
  Q(11, 11) = Q(9, 9);

  Fd = I;
  Fd.noalias() += F * dt ;



  P = Fd * P * Fd.transpose() + Q;


  //Forward Euler Integration of dynamics f
  f(0) = x(3);
  f(1) = x(4);
  f(2) = x(5);

  f(3) = (x(0) - COP(0)) / (m * tmp) * (fN(2) + x(8)) + x(6) / m - (L(1)) / (m * tmp);
  f(4) = (x(1) - COP(1)) / (m * tmp) * (fN(2) + x(8)) + x(7) / m + (L(0)) / (m * tmp);
  f(5) = (fN(2) + x(8)) / m - g;


  x  += f * dt;

  EKF1UpdateVars();
}



void WalkingController::EKF1Update(Eigen::Vector3d Acc, Eigen::Vector3d Pos, Eigen::Vector3d Gyro, Eigen::Vector3d Gyrodot)
{

  /* Update Step */
  //Compute the CoM Acceleration
  Acc += Gyro.cross(Gyro.cross(Pos)) + Gyrodot.cross(Pos);


  tmp = x(2)-COP(2);

  z.segment<3>(0).noalias() = Pos - (x.segment<3>(0) - Eigen::Vector3d( 0, 0, x(11)));

  z(3) = Acc(0) - ( (x(0) - COP(0)) / (m * tmp) * (fN(2)) + x(6) / m - (L(1)) / (m * tmp) );
  z(4) = Acc(1) - ( (x(1) - COP(1)) / (m * tmp) * (fN(2)) + x(7) / m + (L(0)) / (m * tmp) );
  z(5) = Acc(2) - ( (fN(2) + x(8)) / m - g );

  z(6) = fN(0) - ( (x(0)-COP(0))*fN(2)/tmp - L(1)/tmp );
  z(7) = fN(1) - ( (x(1)-COP(1))*fN(2)/tmp + L(0)/tmp );
  z(8) = ( m*(Acc(2) + g)- fN(2)) - ( x(8));
  //z(8) = ( contact_moment_(2) - L(2)) - ( (x(1)*COP(0) - x(0)*COP(1))*fN(2)/tmp + (x(0)*L(0) + x(1)*L(1))/tmp );


  H = Eigen::Matrix<double,9,12>::Zero();
  H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
  H(2,11) = -1;
  //H.block<2,2>(0,9) = -Eigen::Matrix2d::Identity();

  H(3, 0) = (fN(2)) / (m * tmp);
  H(3, 2) =  -((fN(2)) * (x(0) - COP(0))) / (m * tmp * tmp) + (L(1)) / (m * tmp * tmp);
  H(3, 6) = 1.000 / m;
  //H(3, 8) = (x(0) - COP(0)) / (m * tmp);

  H(4, 1) = (fN(2)) / (m * tmp);
  H(4, 2) = - ( fN(2)) * ( x(1) - COP(1) ) / (m * tmp * tmp) - (L(0)) / (m * tmp * tmp);
  H(4, 7) = 1.000 / m;
  //H(4, 8) = (x(1) - COP(1)) / (m * tmp);
  H(5, 8) = 1.000 / m;

  H(6, 0) = fN(2)/tmp;
  H(6, 2) =  -((fN(2)) * (x(0) - COP(0))) / (tmp * tmp) + (L(1)) / (tmp * tmp);

  H(7, 1) = fN(2)/tmp;
  H(7, 2) = -((fN(2)) * (x(0) - COP(0))) / (tmp * tmp) - (L(0)) / (tmp * tmp);

  H(8, 8) = -1;
  //H(8, 0) = -COP(1)*fN(2)/tmp + L(0)/tmp;
  //H(8, 1) = COP(0)*fN(2)/tmp + L(1)/tmp;
  //H(8, 2) = -(COP(0)*x(1) - COP(1)*x(0))*fN(2)/(tmp * tmp) - (x(0)*L(0)+x(1)*L(1))/(tmp*tmp);


  R(0, 0) = com_r * com_r;
  R(1, 1) = R(0, 0);
  R(2, 2) = R(0, 0);

  R(3, 3) = comdd_r * comdd_r;
  R(4, 4) = R(3, 3);
  R(5, 5) = R(3, 3);

  R(6, 6) = ft_r*ft_r;
  R(7, 7) = R(6, 6);
  R(8, 8) = R(6, 6);

  S = R;
  S.noalias() += H * P * H.transpose();
  K.noalias() = P * H.transpose() * S.inverse();

  x += K * z;
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
  EKF1UpdateVars();
}


void WalkingController::EKF1UpdateVars()
{
  comX = x(0);
  comY = x(1);
  comZ = x(2);

  velX = x(3);
  velY = x(4);
  velZ = x(5);

  fX = x(6);
  fY = x(7);
  fZ = x(8);

  com_x_error_ = x(9);
  com_y_error_ = x(10);
  com_z_error_ = x(11);
}


void WalkingController::gyrodotFilter()
{
  if (!firstGyrodot_) {
    //Compute numerical derivative
    imu_ang_dot_ = (imu_ang_ - imu_ang_old_)*hz_;
    moving_avg_filter_[0]->filter(imu_ang_dot_(0));
    moving_avg_filter_[1]->filter(imu_ang_dot_(1));
    moving_avg_filter_[2]->filter(imu_ang_dot_(2));

    imu_ang_dot_(0)=moving_avg_filter_[0]->x;
    imu_ang_dot_(1)=moving_avg_filter_[1]->x;
    imu_ang_dot_(2)=moving_avg_filter_[2]->x;
  }
  else {
    imu_ang_dot_.setZero();
    firstGyrodot_ = false;
  }
  imu_ang_old_ = imu_ang_;
}

}
