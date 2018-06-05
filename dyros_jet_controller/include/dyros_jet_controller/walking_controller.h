#ifndef WALKING_CONTROLLER_H
#define WALKING_CONTROLLER_H


#include "dyros_jet_controller/dyros_jet_model.h"
#include "math_type_define.h"
#include <vector>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <thread>
#include <mutex>

#define ZERO_LIBRARY_MODE


const int FILE_CNT = 10;

const std::string FILE_NAMES[FILE_CNT] =
{
  ///change this directory when you use this code on the other computer///
  "/home/dg/data/walking/0_desired_zmp_.txt",
  "/home/dg/data/walking/1_desired_com_.txt",
  "/home/dg/data/walking/2_desired_q_.txt",
  "/home/dg/data/walking/3_real_q_.txt",
  "/home/dg/data/walking/4_desired_swingfoot_.txt",
  "/home/dg/data/walking/5_desired_pelvis_trajectory_.txt",
  "/home/dg/data/walking/6_current_com_pelvis_trajectory_.txt",
  "/home/dg/data/walking/7_current_foot_trajectory_.txt",
  "/home/dg/data/walking/8_estimation_variables_.txt",
  "/home/dg/data/walking/9_ft_sensor_.txt"
};

using namespace std;
namespace dyros_jet_controller
{

class WalkingController
{
public:
  fstream file[FILE_CNT];


  static constexpr unsigned int PRIORITY = 8;


  WalkingController(DyrosJetModel& model, const VectorQd& current_q, const Eigen::Vector12d& current_q_ext, const double hz, const double& control_time) :
    total_dof_(DyrosJetModel::HW_TOTAL_DOF), model_(model), current_q_(current_q), current_q_ext_(current_q_ext), hz_(hz), current_time_(control_time), start_time_{}, end_time_{}, slowcalc_thread_(&WalkingController::slowCalc, this), calc_start_flag_(false) , calc_update_flag_(false)
  {

    for(int i=0; i<FILE_CNT;i++)
    {
      file[i].open(FILE_NAMES[i].c_str(),ios_base::out);
    }
    file[0]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"zmp_desired_(0)"<<"\t"<<"zmp_desired_(1)"<<"\t"<<"foot_step_(current_step_num_, 0)"<<"\t"<<"foot_step_(current_step_num_, 1)"<<"\t"<<"foot_step_support_frame_(current_step_num_, 0)"<<"\t"<<"foot_step_support_frame_(current_step_num_, 1)"<<"\t"<<"foot_step_support_frame_(current_step_num_, 2)"<<endl;
    file[1]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"com_desired_(0)"<<"\t"<<"com_desired_(1)"<<"\t"<<"com_desired_(2)"<<"\t"<<"com_dot_desired_(0)"<<"\t"<<"com_dot_desired_(1)"<<"\t"<<"com_dot_desired_(2)"<<"\t"<<"com_support_init_(0)"<<"\t"<<"com_support_init_(0)"<<"\t"<<"com_support_init_(0)"<<endl;
    file[2]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"desired_leg_q_(0)"<<"\t"<<"desired_leg_q_(1)"<<"\t"<<"desired_leg_q_(2)"<<"\t"<<"desired_leg_q_(3)"<<"\t"<<"desired_leg_q_(4)"<<"\t"<<"desired_leg_q_(5)"<<"\t"<<"desired_leg_q_(6)"<<"\t"<<"desired_leg_q_(7)"<<"\t"<<"desired_leg_q_(8)"<<"\t"<<"desired_leg_q_(9)"<<"\t"<<"desired_leg_q_(10)"<<"\t"<<"desired_leg_q_(11)"<<endl;
    file[3]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"current_q_(0)"<<"\t"<<"current_q_(1)"<<"\t"<<"current_q_(2)"<<"\t"<<"current_q_(3)"<<"\t"<<"current_q_(4)"<<"\t"<<"current_q_(5)"<<"\t"<<"current_q_(6)"<<"\t"<<"current_q_(7)"<<"\t"<<"current_q_(8)"<<"\t"<<"current_q_(9)"<<"\t"<<"current_q_(10)"<<"\t"<<"current_q_(11)"<<endl;
    file[4]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"rfoot_trajectory_support_.translation()(0)"<<"\t"<<"rfoot_trajectory_support_.translation()(1)"<<"\t"<<"rfoot_trajectory_support_.translation()(2)"<<"\t"<<"lfoot_trajectory_support_.translation()(0)"<<"\t"<<"lfoot_trajectory_support_.translation()(1)"<<"\t"<<"lfoot_trajectory_support_.translation()(2)"<<"\t"<<"rfoot_support_init_.translation()(0)"<<"\t"<<"rfoot_support_init_.translation()(1)"<<"\t"<<"rfoot_support_init_.translation()(2)"<<endl;
    file[5]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"pelv_trajectory_support_.translation()(0)"<<"\t"<<"pelv_trajectory_support_.translation()(1)"<<"\t"<<"pelv_trajectory_support_.translation()(2)"<<endl;
    file[6]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"com_support_current_(0)"<<"\t"<<"com_support_current_(1)"<<"\t"<<"com_support_current_(2)"
          <<"\t"<<"pelv_support_current_.translation()(0)"<<"\t"<<"pelv_support_current_.translation()(1)"<<"\t"<<"pelv_support_current_.translation()(2)"<<"\t"<<"com_support_dot_current_(0)"<<"\t"<<"com_support_dot_current_(1)"<<"\t"<<"com_support_dot_current_(2)"
         <<"\t"<<"com_sim_current_(0)"<<"\t"<<"com_sim_current_(1)"<<"\t"<<"com_sim_current_(2)"<<endl;
    file[7]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"rfoot_support_current_.translation()(0)"<<"\t"<<"rfoot_support_current_.translation()(1)"<<"\t"<<"rfoot_support_current_.translation()(2)"
          <<"\t"<<"lfoot_support_current_.translation()(0)"<<"\t"<<"lfoot_support_current_.translation()(1)"<<"\t"<<"lfoot_support_current_.translation()(2)"<<endl;
    file[8]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"vars.x[0]"<<"\t"<<"vars.x[1]"<<"\t"<<"vars.x[2]"<<"\t"<<"vars.x[3]"<<"\t"<<"vars.x[4]"<<"\t"<<"vars.x[5]"<<"\t"<<"zmp_measured_(0)"<<"\t"<<"zmp_measured_(1)"<<"\t"<<"zmp_r_(0)"<<"\t"<<"zmp_r_(1)"<<"\t"<<"zmp_l_(0)"<<"\t"<<"zmp_l_(1)"<<endl;
    file[9]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"r_ft_(0)"<<"\t"<<"r_ft_(1)"<<"\t"<<"r_ft_(2)"<<"\t"<<"r_ft_(3)"<<"\t"<<"r_ft_(4)"<<"\t"<<"r_ft_(5)"<<"\t"<<"l_ft_(0)"<<"\t"<<"l_ft_(1)"<<"\t"<<"l_ft_(2)"<<"\t"<<"l_ft_(3)"<<"\t"<<"l_ft_(4)"<<"\t"<<"l_ft_(5)"<<endl;
  }
  //WalkingController::~WalkingController()
  //{
  //  for(int i=0; i<FILE_CNT;i++)
  //  {
  //    if(file[i].is_open())
  //      file[i].close();
  //  }
  //}



  void compute();
  void setTarget(int walk_mode, bool hip_compensation, bool lqr, int ik_mode, bool heel_toe,
                 bool is_right_foot_swing, double x, double y, double z, double height, double theta,
                 double step_length, double step_length_y);
  void setEnable(bool enable);
  void updateControlMask(unsigned int *mask);
  void writeDesired(const unsigned int *mask, VectorQd& desired_q);

  void parameterSetting();
  //functions in compute
  void getRobotState();
  void getComTrajectory();
  void getZmpTrajectory();
  void getPelvTrajectory();
  void getFootTrajectory();
  void computeIkControl(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& desired_leg_q);
  void computeJacobianControl(Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector3d float_lleg_transform_euler, Eigen::Vector3d float_rleg_transform_euler, Eigen::Vector12d& desired_leg_q_dot);
  void compensator();

  void supportToFloatPattern();
  void updateNextStepTime();
  void updateInitialState();

  //functions for getFootStep()
  void calculateFootStepTotal();
  void calculateFootStepSeparate();

  //functions for getZMPTrajectory()
  void floatToSupportFootstep();
  void addZmpOffset();
  void zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num);
  void onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);

  //functions in compensator()
  void hipCompensator(); //reference Paper: http://dyros.snu.ac.kr/wp-content/uploads/2017/01/ICHR_2016_JS.pdf
  void hipCompensation();

  //PreviewController
  void modifiedPreviewControl();
  void previewControl(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs,
                      Eigen::Vector3d ys, double ux_1 , double uy_1 ,
                      double& ux, double& uy, double gi, Eigen::VectorXd gp_l,
                      Eigen::Matrix1x3d gx, Eigen::Matrix3d a, Eigen::Vector3d b,
                      Eigen::Matrix1x3d c, Eigen::Vector3d &xd, Eigen::Vector3d &yd);
  void previewControlParameter(double dt, int NL, Eigen::Matrix4d& k, Eigen::Vector3d com_support_init_,
                               double& gi, Eigen::VectorXd& gp_l, Eigen::Matrix1x3d& gx, Eigen::Matrix3d& a,
                               Eigen::Vector3d& b, Eigen::Matrix1x3d& c);
  void vibrationControl(const Eigen::Vector12d desired_leg_q, Eigen::Vector12d &output);
  void massSpringMotorModel(double spring_k, double damping_d, double motor_k, Eigen::MatrixXd & mass, Eigen::MatrixXd& a, Eigen::MatrixXd& b, Eigen::MatrixXd& c);
  void discreteModel(Eigen::MatrixXd& a, Eigen::MatrixXd& b, Eigen::MatrixXd& c, int np, double dt, Eigen::MatrixXd& ad, Eigen::MatrixXd& bd, Eigen::MatrixXd& cd, Eigen::MatrixXd& ad_total, Eigen::MatrixXd& bd_total);
  void riccatiGain(Eigen::MatrixXd& ad_total, Eigen::MatrixXd& bd_total, Eigen::Matrix<double, 12*4, 12*4>& q, Eigen::Matrix12d& r, Eigen::Matrix<double, 12, 12*4>& k);
  void slowCalc();

private:

  const double hz_;
  const double &current_time_; // updated by control_base
  unsigned int walking_tick_ = 0;
  double walking_time_ = 0;

  //sensorData
  Eigen::Vector6d r_ft_;
  Eigen::Vector6d l_ft_;

  //parameterSetting()
  double t_last_;
  double t_start_;
  double t_start_real_;
  double t_temp_;
  double t_imp_;
  double t_rest_init_;
  double t_rest_last_;
  double t_double1_;
  double t_double2_;
  double t_total_;
  double foot_height_;
  double com_height_;

  bool com_control_mode_;
  bool com_update_flag_; // frome A to B
  bool gyro_frame_flag_;

  int ik_mode_;
  int walk_mode_;
  bool hip_compensator_mode_;
  bool lqr_compensator_mode_;
  int heel_toe_mode_;
  int is_right_foot_swing_;

  bool walking_enable_;
  bool joint_enable_[DyrosJetModel::HW_TOTAL_DOF];
  double step_length_x_;
  double step_length_y_;

  //double step_angle_theta_;
  double target_x_;
  double target_y_;
  double target_z_;
  double target_theta_;
  double total_step_num_;
  double current_step_num_;

  Eigen::MatrixXd foot_step_;
  Eigen::MatrixXd foot_step_support_frame_;
  Eigen::MatrixXd foot_step_support_frame_offset_;

  Eigen::MatrixXd ref_zmp_;


  VectorQd start_q_;
  VectorQd desired_q_;
  VectorQd target_q_;
  const VectorQd& current_q_;  
  const Eigen::Vector12d& current_q_ext_;



  //const double &current_time_;
  const unsigned int total_dof_;
  double start_time_[DyrosJetModel::HW_TOTAL_DOF];
  double end_time_[DyrosJetModel::HW_TOTAL_DOF];

  //Step initial state variable//
  Eigen::Isometry3d pelv_support_init_;
  Eigen::Isometry3d lfoot_support_init_;
  Eigen::Isometry3d rfoot_support_init_;
  Eigen::Isometry3d pelv_float_init_;
  Eigen::Isometry3d lfoot_float_init_;
  Eigen::Isometry3d rfoot_float_init_;

  Eigen::Vector3d pelv_support_euler_init_;
  Eigen::Vector3d lfoot_support_euler_init_;
  Eigen::Vector3d rfoot_support_euler_init_;
  VectorQd q_init_;

  Eigen::Vector6d supportfoot_float_init_;
  Eigen::Vector6d supportfoot_support_init_;
  Eigen::Vector6d supportfoot_support_init_offset_;
  Eigen::Vector6d swingfoot_float_init_;
  Eigen::Vector6d swingfoot_support_init_;
  Eigen::Vector6d swingfoot_support_init_offset_;

  Eigen::Isometry3d pelv_suppprt_start_;

  Eigen::Vector3d com_float_init_;
  Eigen::Vector3d com_support_init_;

  double lfoot_zmp_offset_;   //have to be initialized
  double rfoot_zmp_offset_;
  Eigen::Vector3d com_offset_;

  //Step current state variable//
  Eigen::Vector3d com_support_current_;
  Eigen::Vector3d com_sim_current_;

  Eigen::Isometry3d pelv_support_current_;
  Eigen::Isometry3d lfoot_support_current_;
  Eigen::Isometry3d rfoot_support_current_;

  Eigen::Vector3d com_float_current_;
  Eigen::Isometry3d pelv_float_current_;
  Eigen::Isometry3d lfoot_float_current_;
  Eigen::Isometry3d rfoot_float_current_;

  Eigen::Matrix6d current_leg_jacobian_l_;
  Eigen::Matrix6d current_leg_jacobian_r_;

  DyrosJetModel &model_;


  //desired variables
  Eigen::Vector12d desired_leg_q_;
  Eigen::Vector12d desired_leg_q_dot_;
  Eigen::Vector3d com_desired_;
  Eigen::Vector3d com_dot_desired_;
  Eigen::Vector2d zmp_desired_;

  Eigen::Isometry3d rfoot_trajectory_support_;  //local frame
  Eigen::Isometry3d lfoot_trajectory_support_;
  Eigen::Vector3d rfoot_trajectory_euler_support_;
  Eigen::Vector3d lfoot_trajectory_euler_support_;
  Eigen::Vector6d rfoot_trajectory_dot_support_; //x,y,z translation velocity & roll, pitch, yaw velocity
  Eigen::Vector6d lfoot_trajectory_dot_support_;

  Eigen::Isometry3d pelv_trajectory_support_; //local frame
  Eigen::Isometry3d pelv_trajectory_float_; //pelvis frame

  Eigen::Isometry3d rfoot_trajectory_float_;  //pelvis frame
  Eigen::Isometry3d lfoot_trajectory_float_;
  Eigen::Vector3d rfoot_trajectory_euler_float_;
  Eigen::Vector3d lfoot_trajectory_euler_float_;
  Eigen::Vector3d rfoot_trajectory_dot_float_;
  Eigen::Vector3d lfoot_trajectory_dot_float_;

  //getComTrajectory() variables
  double xi_;
  double yi_;
  Eigen::Vector3d xs_;
  Eigen::Vector3d ys_;
  Eigen::Vector3d xd_;
  Eigen::Vector3d yd_;

  //Preview Control
  double ux_, uy_, ux_1_, uy_1_;
  double zc_;
  double gi_;
  double zmp_start_time_; //원래 코드에서는 start_time, zmp_ref 시작되는 time같음
  Eigen::Matrix4d k_;
  Eigen::VectorXd gp_l_;
  Eigen::Matrix1x3d gx_;
  Eigen::Matrix3d a_;
  Eigen::Vector3d b_;
  Eigen::Matrix1x3d c_;


  //resolved momentum control
  Eigen::Vector3d p_ref_;
  Eigen::Vector3d l_ref_;

  //Gravitycompensate
  Eigen::Vector12d joint_offset_angle_;
  Eigen::Vector12d grav_ground_torque_;

  //vibrationCotrol
  std::mutex slowcalc_mutex_;
  std::thread slowcalc_thread_;

  Eigen::Vector12d current_motor_q_leg_;
  Eigen::Vector12d current_link_q_leg_;
  Eigen::Vector12d pre_motor_q_leg_;
  Eigen::Vector12d pre_link_q_leg_;
  Eigen::Vector12d lqr_output_;
  Eigen::Vector12d lqr_output_pre_;

  VectorQd thread_q_;
  unsigned int thread_tick_;

  Eigen::Matrix<double, 48, 1> x_bar_right_;
  Eigen::MatrixXd kkk_copy_;
  Eigen::MatrixXd ad_total_copy_;
  Eigen::MatrixXd bd_total_copy_;
  Eigen::MatrixXd ad_copy_;
  Eigen::MatrixXd bd_copy_;

  Eigen::MatrixXd ad_right_;
  Eigen::MatrixXd bd_right_;
  Eigen::MatrixXd ad_total_right_;
  Eigen::MatrixXd bd_total_right_;
  Eigen::MatrixXd kkk_motor_right_;

  Eigen::Vector12d dist_prev_;

  bool calc_start_flag_;
  bool calc_update_flag_;

  Eigen::MatrixXd mass_matrix_;
  Eigen::MatrixXd mass_matrix_pc_;
  Eigen::MatrixXd mass_matrix_sel_;
  Eigen::MatrixXd a_right_mat_;
  Eigen::MatrixXd b_right_mat_;
  Eigen::MatrixXd c_right_mat_;
  Eigen::MatrixXd a_disc_;
  Eigen::MatrixXd b_disc_;
  Eigen::MatrixXd a_disc_total_;
  Eigen::MatrixXd b_disc_total_;
  Eigen::MatrixXd kkk_;

  //////////////////StateEstimation/////////////////////
  Eigen::Matrix<double, 18, 6> a_total_;
  Eigen::Matrix<double, 2, 6> a_kin_;
  Eigen::Matrix<double, 2, 6> a_c_dot_;
  Eigen::Matrix<double, 2, 6> a_c_;
  Eigen::Matrix<double, 2, 6> a_zmp_;
  Eigen::Matrix<double, 2, 6> a_c_c_dot_;
  Eigen::Matrix<double, 2, 6> a_f_;
  Eigen::Matrix<double, 6, 6> a_noise_;
  Eigen::Matrix<double, 18, 1> b_total_;
  Eigen::Matrix<double, 2, 1> b_kin_;
  Eigen::Matrix<double, 2, 1> b_c_dot_;
  Eigen::Matrix<double, 2, 1> b_c_;
  Eigen::Matrix<double, 2, 1> b_zmp_;
  Eigen::Matrix<double, 2, 1> b_c_c_dot_;
  Eigen::Matrix<double, 2, 1> b_f_;
  Eigen::Matrix<double, 6, 1> b_noise_;

  Eigen::Vector3d com_support_dot_current_;//from support foot

  Eigen::Matrix<double, 3, 4> com_float_old_;  // [com(k) com(k-1) com(k-2) com(k-3)]
  Eigen::Matrix<double, 3, 4> com_float_dot_old_;
  Eigen::Matrix<double, 3, 4> com_support_old_;  // [con(k) com(k-1) com(k-2) com(k-3)]
  Eigen::Matrix<double, 3, 4> com_support_dot_old_;
  Eigen::Vector2d com_support_dot_old_estimation_;
  Eigen::Vector2d com_support_old_estimation_;


  Eigen::Vector2d zmp_r_;
  Eigen::Vector2d zmp_l_;
  Eigen::Vector2d zmp_measured_;
  Eigen::Vector2d zmp_old_estimation_;

  Eigen::Vector6d x_estimation_;

  void getEstimationInputMatrix();
  ////////////////////////////////////////////////////////
};

}
#endif // WALKING_CONTROLLER_H
