#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller.h"
#include "cvxgen_6_8_0/cvxgen/solver.h"


namespace dyros_jet_controller
{

void WalkingController::getQpEstimationInputMatrix()
{
  double mass_total = 51.315;

  if(walking_tick_ == 0)
  {
    solve();
  }

  for (int i = 0; i < 6; i++)
  {
    x_estimation_(i) = vars.x[i];
  }

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
  a_f_(0, 2) = mass_total*GRAVITY / (zc_);
  a_f_(1, 3) = mass_total*GRAVITY / (zc_);
  a_f_(0, 4) = -mass_total*GRAVITY / (zc_);
  a_f_(1, 5) = -mass_total*GRAVITY / (zc_);
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
    cout << "first set" << endl;
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


  cout<<"Ad_2_"<<Ad_2_<<endl;
  cout<<"Bd_2_"<<Bd_2_<<endl;
  cout<<"Cd_2_"<<Cd_2_<<endl;
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
          [ 0         1         0         0         0         0         0         -1        0         -1]
          [ 0         0         0         0         1         0         0         0         0         0 ]
          [ 0         0         0         0         0         1         0         0         0         0 ]
          [ w^2dT0  0         0         0         -w^2dT  0         0         0         0         0 ]
          [ 0         w^2dT   0         0         0         -w^2dT  0         0         0         0 ]

  */

  double omega_square = GRAVITY/model_.getCurrentCom()(2);

  double mass_total = 51.315;

  Q_3_.setIdentity();
  R_3_.setIdentity();

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



}
