#ifndef WALKING_CONTROLLER_H
#define WALKING_CONTROLLER_H


#include "dyros_jet_controller/dyros_jet_model.h"
#include "moving_average_filter.h"
#include "math_type_define.h"
#include <vector>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <thread>
#include <mutex>

#define ZERO_LIBRARY_MODE


const int FILE_CNT = 16;

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
  "/home/dg/data/walking/8_QPestimation_variables_.txt",
  "/home/dg/data/walking/9_ft_sensor_.txt",
  "/home/dg/data/walking/10_ext_encoder_.txt",
  "/home/dg/data/walking/11_kalman_estimator2_.txt",
  "/home/dg/data/walking/12_kalman_estimator1_.txt",
  "/home/dg/data/walking/13_kalman_estimator3_.txt",
  "/home/dg/data/walking/14_grav_torque_.txt",
  "/home/dg/data/walking/15_ekf1_state.txt"
};

using namespace std;
namespace dyros_jet_controller
{

class WalkingController
{
public:
  fstream file[FILE_CNT];


  static constexpr unsigned int PRIORITY = 8;


  WalkingController(DyrosJetModel& model, const VectorQd& current_q, const double hz, const double& control_time) :
    total_dof_(DyrosJetModel::HW_TOTAL_DOF), model_(model), current_q_(current_q), hz_(hz), current_time_(control_time), start_time_{}, end_time_{}, slowcalc_thread_(&WalkingController::slowCalc, this), calc_update_flag_(false), calc_start_flag_(false), ready_for_thread_flag_(false), ready_for_compute_flag_(false), foot_step_planner_mode_(false), walking_end_foot_side_ (false), foot_plan_walking_last_(false), foot_last_walking_end_(false)
  {
    walking_state_send = false;
    walking_end_ = false;
    for(int i=0; i<FILE_CNT;i++)
    {
      file[i].open(FILE_NAMES[i].c_str(),ios_base::out);
    }
    file[0]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"ref_zmp_(0, 0)"<<"\t"<<"ref_zmp_(0, 1)"<<"\t"<<"zmp_desired_(0)"<<"\t"<<"zmp_desired_(1)"<<"\t"<<"zmp_measured_(0)"<<"\t"<<"zmp_measured_(1)"<<"\t"<<"zmp_measured_inverse_(0)"<<"\t"<<"zmp_measured_inverse_(1)"
          <<"\t"<<"zmp_dist_(0)"<<"\t"<<"zmp_dist_(1)"<<"\t"<<"foot_step_(current_step_num_, 0)"<<"\t"<<"foot_step_(current_step_num_, 1)"<<"\t"<<"foot_step_support_frame_(current_step_num_, 0)"<<"\t"<<"foot_step_support_frame_(current_step_num_, 1)"<<"\t"<<"foot_step_support_frame_(current_step_num_, 2)"<<endl;
    file[1]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"com_desired_(0)"<<"\t"<<"com_desired_(1)"<<"\t"<<"com_desired_(2)"<<"\t"<<"com_dot_desired_(0)"<<"\t"<<"com_dot_desired_(1)"<<"\t"<<"com_dot_desired_(2)"<<"\t"<<"com_support_init_(0)"<<"\t"<<"com_support_init_(0)"<<"\t"<<"com_support_init_(0)"<<endl;
    file[2]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"desired_leg_q_(0)"<<"\t"<<"desired_leg_q_(1)"<<"\t"<<"desired_leg_q_(2)"<<"\t"<<"desired_leg_q_(3)"<<"\t"<<"desired_leg_q_(4)"<<"\t"<<"desired_leg_q_(5)"<<"\t"<<"desired_leg_q_(6)"<<"\t"<<"desired_leg_q_(7)"<<"\t"<<"desired_leg_q_(8)"<<"\t"<<"desired_leg_q_(9)"<<"\t"<<"desired_leg_q_(10)"<<"\t"<<"desired_leg_q_(11)"<<endl;
    file[3]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"current_q_(0)"<<"\t"<<"current_q_(1)"<<"\t"<<"current_q_(2)"<<"\t"<<"current_q_(3)"<<"\t"<<"current_q_(4)"<<"\t"<<"current_q_(5)"<<"\t"<<"current_q_(6)"<<"\t"<<"current_q_(7)"<<"\t"<<"current_q_(8)"<<"\t"<<"current_q_(9)"<<"\t"<<"current_q_(10)"<<"\t"<<"current_q_(11)"<<endl;
    file[4]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"rfoot_trajectory_support_.translation()(0)"<<"\t"<<"rfoot_trajectory_support_.translation()(1)"<<"\t"<<"rfoot_trajectory_support_.translation()(2)"<<"\t"<<"lfoot_trajectory_support_.translation()(0)"<<"\t"<<"lfoot_trajectory_support_.translation()(1)"<<"\t"<<"lfoot_trajectory_support_.translation()(2)"<<"\t"<<"rfoot_support_init_.translation()(0)"<<"\t"<<"rfoot_support_init_.translation()(1)"<<"\t"<<"rfoot_support_init_.translation()(2)"<<endl;
    file[5]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"pelv_trajectory_support_.translation()(0)"<<"\t"<<"pelv_trajectory_support_.translation()(1)"<<"\t"<<"pelv_trajectory_support_.translation()(2)"<<endl;
    file[6]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"com_support_current_(0)"<<"\t"<<"com_support_current_(1)"<<"\t"<<"com_support_current_(2)"
          <<"\t"<<"pelv_support_current_.translation()(0)"<<"\t"<<"pelv_support_current_.translation()(1)"<<"\t"<<"pelv_support_current_.translation()(2)"<<"\t"<<"com_support_dot_current_(0)"<<"\t"<<"com_support_dot_current_(1)"<<"\t"<<"com_support_dot_current_(2)"
         <<"\t"<<"com_sim_current_(0)"<<"\t"<<"com_sim_current_(1)"<<"\t"<<"com_sim_current_(2)"<<"\t"<<"com_sim_dot_current_(0)"<<"\t"<<"com_sim_dot_current_(1)"<<"\t"<<"com_sim_dot_current_(2)"<<endl;
    file[7]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"rfoot_support_current_.translation()(0)"<<"\t"<<"rfoot_support_current_.translation()(1)"<<"\t"<<"rfoot_support_current_.translation()(2)"
          <<"\t"<<"lfoot_support_current_.translation()(0)"<<"\t"<<"lfoot_support_current_.translation()(1)"<<"\t"<<"lfoot_support_current_.translation()(2)"<<endl;
    file[8]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"vars.x[0]"<<"\t"<<"vars.x[1]"<<"\t"<<"vars.x[2]"<<"\t"<<"vars.x[3]"<<"\t"<<"vars.x[4]"<<"\t"<<"vars.x[5]"<<"\t"<<"zmp_measured_(0)"<<"\t"<<"zmp_measured_(1)"<<"\t"<<"zmp_r_(0)"<<"\t"<<"zmp_r_(1)"<<"\t"<<"zmp_l_(0)"<<"\t"<<"zmp_l_(1)"<<endl;
    file[9]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"r_ft_(0)"<<"\t"<<"r_ft_(1)"<<"\t"<<"r_ft_(2)"<<"\t"<<"r_ft_(3)"<<"\t"<<"r_ft_(4)"<<"\t"<<"r_ft_(5)"<<"\t"<<"l_ft_(0)"<<"\t"<<"l_ft_(1)"<<"\t"<<"l_ft_(2)"<<"\t"<<"l_ft_(3)"<<"\t"<<"l_ft_(4)"<<"\t"<<"l_ft_(5)"<<endl;
    file[10]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"current_link_q_leg_(0)"<<"\t"<<"current_link_q_leg_(1)"<<"\t"<<"current_link_q_leg_(2)"<<"\t"<<"current_link_q_leg_(3)"<<"\t"<<"current_link_q_leg_(4)"<<"\t"<<"current_link_q_leg_(5)"<<"\t"<<
              "current_link_q_leg_(6)"<<"\t"<<"current_link_q_leg_(7)"<<"\t"<<"current_link_q_leg_(8)"<<"\t"<<"current_link_q_leg_(9)"<<"\t"<<"current_link_q_leg_(10)"<<"\t"<<"current_link_q_leg_(11)"<<endl;
    file[11]<<"walking_tick_"<<"\t"<<"X_hat_post_2_(0)"<<"\t"<<"X_hat_post_2_(1)"<<"\t"<<"X_hat_post_2_(2)"<<"\t"<<"X_hat_post_2_(3)"<<"\t"<<"X_hat_post_2_(4)"<<"\t"<<"X_hat_post_2_(5)"<<"\t"<<"X_hat_post_2_(6)"<<"\t"<<"X_hat_post_2_(7)"<<endl;
    file[12]<<"walking_tick_"<<"\t"<<"X_hat_post_1_(0)"<<"\t"<<"X_hat_post_1_(1)"<<"\t"<<"X_hat_post_1_(2)"<<"\t"<<"X_hat_post_1_(3)"<<"\t"<<"X_hat_post_1_(4)"<<"\t"<<"X_hat_post_1_(5)"<<endl;
    file[13]<<"walking_tick_"<<"\t"<<"X_hat_post_3_(0)"<<"\t"<<"X_hat_post_3_(1)"<<"\t"<<"X_hat_post_3_(2)"<<"\t"<<"X_hat_post_3_(3)"<<"\t"<<"X_hat_post_3_(4)"<<"\t"<<"X_hat_post_3_(5)"<<endl;
    file[14]<<"walking_tick_"<<"\t"<<"grav_ground_torque_(0)"<<"\t"<<"grav_ground_torque_(1)"<<"\t"<<"grav_ground_torque_(2)"<<"\t"<<"grav_ground_torque_(3)"<<"\t"<<"grav_ground_torque_(4)"<<"\t"<<"grav_ground_torque_(5)"<<endl;
    file[15]<<"walking_tick_"<<"\t"<<"comX"<<"\t"<<"comY"<<"\t"<<"comZ"<<"\t"<<"velX"<<"\t"<<"velY"<<"\t"<<"velZ"<<"\t"<<"fX"<<"\t"<<"fY"<<"\t"<<"fZ"<<"\t"<<"com_x_error_"<<"\t"<<"com_y_error_"<<"\t"<<"com_z_error_"<<endl;
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
  void setFootPlan(int footnum, int startfoot, Eigen::MatrixXd footpose);
  void updateControlMask(unsigned int *mask);
  void writeDesired(const unsigned int *mask, VectorQd& desired_q);

  void parameterSetting();
  void initWalkingWholeBody();

  //functions in compute
  void getRobotState();
  void getComTrajectory();
  void getZmpTrajectory();
  void getComTrajectorySimpleLIPM();
  void getZmpTrajectorySimpleLIPM();
  void getPelvTrajectory();
  void getFootTrajectory();
  void computeIkControl(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& desired_leg_q);
  void computeJacobianControl(Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector3d float_lleg_transform_euler, Eigen::Vector3d float_rleg_transform_euler, Eigen::Vector12d& desired_leg_q_dot);
  void compensator();

  void linkMass();
  void linkInertia();
  Eigen::Matrix3d inertiaTensorTransform(Eigen::Matrix3d local_inertia, double mass, Eigen::Isometry3d transformation);
  void getComJacobian();
  void computeComJacobianControl(Eigen::Vector12d &desired_leg_q_dot);

  void supportToFloatPattern();
  void updateNextStepTime();
  void updateInitialState();

  //functions for getFootStep()
  void calculateFootStepTotal();
  void calculateFootStepSeparate();
  void usingFootStepPlanner();

  //functions for getZMPTrajectory()
  void floatToSupportFootstep();
  void addZmpOffset();
  void zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num);
  void onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);
  void computeZmp();
  void zmpCompensator();
  void inverseZmpPlant(Eigen::Vector2d zmp, Eigen::Vector2d zmp_pre, Eigen::Vector2d zmp_ppre, double fc, double damping_ratio, double hz);
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
  //LQR && External Encoder
  void vibrationControl(const Eigen::Vector12d desired_leg_q, Eigen::Vector12d &output);
  void massSpringMotorModel(double spring_k, double damping_d, double motor_k, Eigen::Matrix12d & mass, Eigen::Matrix<double, 36, 36>& a, Eigen::Matrix<double, 36, 12>& b, Eigen::Matrix<double, 12, 36>& c);
  void discreteModel(Eigen::Matrix<double, 36, 36>& a, Eigen::Matrix<double, 36, 12>& b, Eigen::Matrix<double, 12, 36>& c, int np, double dt,
                     Eigen::Matrix<double, 36, 36>& ad, Eigen::Matrix<double, 36, 12>& bd, Eigen::Matrix<double, 12, 36>& cd,
                     Eigen::Matrix<double, 48, 48>& ad_total, Eigen::Matrix<double, 48, 12>& bd_total);
  void riccatiGain(Eigen::Matrix<double, 48, 48>& ad_total, Eigen::Matrix<double, 48, 12>& bd_total, Eigen::Matrix<double, 48, 48>& q, Eigen::Matrix12d& r, Eigen::Matrix<double, 12, 48>& k);
  void slowCalc();
  void slowCalcContent();



  void discreteRiccatiEquationInitialize(Eigen::MatrixXd a, Eigen::MatrixXd b);
  Eigen::MatrixXd discreteRiccatiEquationLQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd R, Eigen::MatrixXd Q);
  Eigen::MatrixXd discreteRiccatiEquationPrev(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd r, Eigen::MatrixXd q);

  VectorQd desired_q_not_compensated_;

  bool walking_end_foot_side_;
  bool walking_end_;
  bool foot_plan_walking_last_;
  bool foot_last_walking_end_;
  bool walking_state_send;


  //ImpedanceControl
  void impedancefootUpdate();
  void impedanceControl();

private:

  const double hz_;
  const double &current_time_; // updated by control_base
  unsigned int walking_tick_ = 0;
  double walking_time_ = 0;

  //sensorData
  Eigen::Vector6d r_ft_;
  Eigen::Vector6d l_ft_;
  Eigen::Vector6d r_ft_pre_;
  Eigen::Vector6d l_ft_pre_;
  Eigen::Vector6d r_ft_ppre_;
  Eigen::Vector6d l_ft_ppre_;

  Eigen::Vector6d r_ft_filtered_;
  Eigen::Vector6d l_ft_filtered_;
  Eigen::Vector6d r_ft_filtered_pre_;
  Eigen::Vector6d l_ft_filtered_pre_;
  Eigen::Vector6d r_ft_filtered_ppre_;
  Eigen::Vector6d l_ft_filtered_ppre_;

  Eigen::Vector3d imu_acc_;
  Eigen::Vector3d imu_ang_;
  Eigen::Vector3d imu_ang_old_;
  Eigen::Vector3d imu_ang_dot_;
  Eigen::Vector3d imu_grav_rpy_;

  Eigen::Vector3d f_ft_support_;
  Eigen::Vector3d moment_support_desried_;
  Eigen::Vector3d moment_support_current_;

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
  bool ready_for_thread_flag_;
  bool ready_for_compute_flag_;
  bool estimator_flag_;

  int ik_mode_;
  int walk_mode_;
  bool hip_compensator_mode_;
  bool lqr_compensator_mode_;
  int heel_toe_mode_;
  int is_right_foot_swing_;
  bool foot_step_planner_mode_;

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
  int foot_step_plan_num_;
  int foot_step_start_foot_;
  Eigen::MatrixXd foot_pose_;

  Eigen::MatrixXd foot_step_;
  Eigen::MatrixXd foot_step_support_frame_;
  Eigen::MatrixXd foot_step_support_frame_offset_;

  Eigen::MatrixXd ref_zmp_;
  Eigen::MatrixXd compensated_zmp_;


  VectorQd start_q_;
  VectorQd desired_q_;
  VectorQd target_q_;
  const VectorQd& current_q_;



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
  Eigen::Vector3d com_support_dot_current_;//from support foot

  ///simulation
  Eigen::Vector3d com_sim_current_;
  Eigen::Vector3d com_sim_dot_current_;
  Eigen::Isometry3d lfoot_sim_global_current_;
  Eigen::Isometry3d rfoot_sim_global_current_;
  Eigen::Isometry3d base_sim_global_current_;
  Eigen::Isometry3d lfoot_sim_float_current_;
  Eigen::Isometry3d rfoot_sim_float_current_;
  Eigen::Isometry3d supportfoot_float_sim_current_;

  Eigen::Isometry3d supportfoot_float_current_;
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
  Eigen::Vector2d zmp_desired_pre_;
  Eigen::Vector2d zmp_desired_ppre_;

  Eigen::Vector2d zmp_desired_filtered_;
  Eigen::Vector2d zmp_desired_filtered_pre_;
  Eigen::Vector2d zmp_desired_filtered_ppre_;

  Eigen::Vector2d zmp_dist_;
  Eigen::Vector2d zmp_dist_pre_;
  Eigen::Vector2d zmp_dist_ppre_;

  Eigen::Vector2d zmp_dist_filtered_;
  Eigen::Vector2d zmp_dist_filtered_pre_;
  Eigen::Vector2d zmp_dist_filtered_ppre_;

  Eigen::Vector2d zmp_measured_inverse_;
  Eigen::Vector2d zmp_measured_inverse_pre_;
  Eigen::Vector2d zmp_measured_inverse_ppre_;

  Eigen::Vector2d zmp_measured_pre_;
  Eigen::Vector2d zmp_measured_ppre_;

  Eigen::Isometry3d rfoot_trajectory_support_;  //local frame
  Eigen::Isometry3d lfoot_trajectory_support_;
  Eigen::Vector3d rfoot_trajectory_euler_support_;
  Eigen::Vector3d lfoot_trajectory_euler_support_;
  Eigen::Vector6d rfoot_trajectory_dot_support_; //x,y,z translation velocity & roll, pitch, yaw derivative
  Eigen::Vector6d lfoot_trajectory_dot_support_;

  Eigen::Isometry3d pelv_trajectory_support_; //local frame
  Eigen::Isometry3d pelv_trajectory_float_; //pelvis frame

  Eigen::Isometry3d rfoot_trajectory_float_;  //pelvis frame
  Eigen::Isometry3d lfoot_trajectory_float_;
  Eigen::Vector3d rfoot_trajectory_euler_float_;
  Eigen::Vector3d lfoot_trajectory_euler_float_;
  Eigen::Vector3d rfoot_trajectory_dot_float_;
  Eigen::Vector3d lfoot_trajectory_dot_float_;

  //comJacobian variables
  Eigen::Matrix<double, 6, 1> mass_l_leg_;
  Eigen::Matrix<double, 6, 1> mass_r_leg_;
  Eigen::Matrix<double, 7, 1> mass_l_arm_;
  Eigen::Matrix<double, 7, 1> mass_r_arm_;
  Eigen::Matrix<double, 3, 1> mass_body_;
  double mass_total_;

  Eigen::Matrix3d inertia_link_float_[29];
  Eigen::Matrix3d inertia_total_;

  Eigen::Vector3d c_l_leg_[6];
  Eigen::Vector3d c_r_leg_[6];
  Eigen::Vector3d c_l_arm_[7];
  Eigen::Vector3d c_r_arm_[7];
  Eigen::Vector3d c_waist_[3];

  Eigen::Matrix6d adjoint_support_;
  Eigen::Matrix6d adjoint_21_;
  Eigen::Vector3d disturbance_accel_;
  Eigen::Vector3d disturbance_accel_old_;
  Eigen::Vector3d desired_w_;
  Eigen::Vector3d desired_u_;
  Eigen::Vector3d desired_u_old_;
  Eigen::Vector3d desired_u_dot_;
  Eigen::Vector6d x2_d_dot_;

  Eigen::Matrix<double, 3, 6> j_rleg_com_total_support;
  Eigen::Matrix<double, 3, 6> j_lleg_com_total_support;

  Eigen::Matrix<double, 3, 7> j_rarm_com_total_support;
  Eigen::Matrix<double, 3, 7> j_larm_com_total_support;

  Eigen::Matrix6d j1_;
  Eigen::Matrix6d j2_;
  Eigen::Matrix<double, 3, 6> j_v1_;
  Eigen::Matrix<double, 3, 6> j_w1_;

  Eigen::Matrix<double, 3, 6> j_com_psem_;
  Eigen::Vector3d desired_c_dot_psem_;

  Eigen::Matrix6d j_total_;
  Eigen::Vector6d c_total_;


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
  Eigen::Matrix<double, 12, 48> kkk_copy_;
  Eigen::Matrix<double, 48, 48> ad_total_copy_;
  Eigen::Matrix<double, 48, 12> bd_total_copy_;
  Eigen::Matrix<double, 36, 36> ad_copy_;
  Eigen::Matrix<double, 36, 12> bd_copy_;

  Eigen::Matrix<double, 36, 36> ad_right_;
  Eigen::Matrix<double, 36, 12> bd_right_;
  Eigen::Matrix<double, 48, 48> ad_total_right_;
  Eigen::Matrix<double, 48, 12> bd_total_right_;
  Eigen::Matrix<double, 12, 48> kkk_motor_right_;

  Eigen::Vector12d dist_prev_;

  bool calc_update_flag_;
  bool calc_start_flag_;


  Eigen::Matrix<double, 18, 18> mass_matrix_;
  Eigen::Matrix<double, 18, 18> mass_matrix_pc_;
  Eigen::Matrix<double, 12, 12> mass_matrix_sel_;
  Eigen::Matrix<double, 36, 36> a_right_mat_;
  Eigen::Matrix<double, 36, 12> b_right_mat_;
  Eigen::Matrix<double, 12, 36> c_right_mat_;
  Eigen::Matrix<double, 36, 36> a_disc_;
  Eigen::Matrix<double, 36, 12> b_disc_;
  Eigen::Matrix<double, 48, 48> a_disc_total_;
  Eigen::Matrix<double, 48, 12> b_disc_total_;
  Eigen::Matrix<double, 48, 48> kkk_;


  //////////////////QP based StateEstimation/////////////////////
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


  Eigen::Vector3d com_float_old_;
  Eigen::Vector3d com_float_dot_old_;
  Eigen::Vector3d com_support_old_;
  Eigen::Vector3d com_support_dot_old_;
  Eigen::Vector3d com_sim_old_;
  Eigen::Vector2d com_support_dot_old_estimation_;
  Eigen::Vector2d com_support_old_estimation_;


  Eigen::Vector2d zmp_r_;
  Eigen::Vector2d zmp_l_;
  Eigen::Vector2d zmp_measured_;
  Eigen::Vector2d zmp_old_estimation_;

  Eigen::Vector6d x_estimation_;


  //Riccati variable
  Eigen::MatrixXd Z11;
  Eigen::MatrixXd Z12;
  Eigen::MatrixXd Z21;
  Eigen::MatrixXd Z22;
  Eigen::MatrixXd temp1;
  Eigen::MatrixXd temp2;
  Eigen::MatrixXd temp3;
  std::vector<double> eigVal_real; //eigen valueÀÇ real°ª
  std::vector<double> eigVal_img; //eigen valueÀÇ img°ª
  std::vector<Eigen::VectorXd> eigVec_real; //eigen vectorÀÇ real°ª
  std::vector<Eigen::VectorXd> eigVec_img; //eigen vectorÀÇ img°ª
  Eigen::MatrixXd Z;
  Eigen::VectorXd deigVal_real;
  Eigen::VectorXd deigVal_img;
  Eigen::MatrixXd deigVec_real;
  Eigen::MatrixXd deigVec_img;
  Eigen::MatrixXd tempZ_real;
  Eigen::MatrixXd tempZ_img;
  Eigen::MatrixXcd U11_inv;
  Eigen::MatrixXcd X;
  Eigen::MatrixXd X_sol;

  Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType Z_eig;
  Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType es_eig;
  Eigen::MatrixXcd tempZ_comp;
  Eigen::MatrixXcd U11;
  Eigen::MatrixXcd U21;

  void getQpEstimationInputMatrix();
  ////////////////////////////////////////////////////////


  /////////////////////////Kalman Filter1///////////////////////
  Eigen::Matrix<double, 6, 6> Ad_1_;
  Eigen::Matrix<double, 6, 2> Bd_1_;
  Eigen::Matrix<double, 4, 6> Cd_1_;
  Eigen::Matrix<double, 6, 6> Q_1_;
  Eigen::Matrix<double, 4, 4> R_1_;


  Eigen::Matrix<double, 6, 1> X_hat_prio_1_;
  Eigen::Matrix<double, 6, 1> X_hat_post_1_;
  Eigen::Matrix<double, 6, 1> X_hat_prio_old_1_;
  Eigen::Matrix<double, 6, 1> X_hat_post_old_1_;

  Eigen::Matrix<double, 4, 1> Y_1_;



  Eigen::Matrix<double, 6, 6> P_prio_1_;
  Eigen::Matrix<double, 6, 6> P_post_1_;
  Eigen::Matrix<double, 6, 6> P_prio_old_1_;
  Eigen::Matrix<double, 6, 6> P_post_old_1_;

  Eigen::Matrix<double, 6, 4> K_1_;
  Eigen::Matrix<double, 6, 4> K_old_1_;


  Eigen::Matrix<double, 2, 1> u_old_1_;

  void kalmanFilter1();
  void kalmanStateSpace1();
  //////////////////////////////////////////////////////////////


  /////////////////////////Kalman Filter2///////////////////////

  Eigen::Matrix<double, 8, 8> Ad_2_;
  Eigen::Matrix<double, 8, 2> Bd_2_;
  Eigen::Matrix<double, 4, 8> Cd_2_;
  Eigen::Matrix<double, 8, 8> Q_2_;
  Eigen::Matrix<double, 4, 4> R_2_;


  Eigen::Matrix<double, 8, 1> X_hat_prio_2_;
  Eigen::Matrix<double, 8, 1> X_hat_post_2_;
  Eigen::Matrix<double, 8, 1> X_hat_prio_old_2_;
  Eigen::Matrix<double, 8, 1> X_hat_post_old_2_;

  Eigen::Matrix<double, 4, 1> Y_2_;



  Eigen::Matrix<double, 8, 8> P_prio_2_;
  Eigen::Matrix<double, 8, 8> P_post_2_;
  Eigen::Matrix<double, 8, 8> P_prio_old_2_;
  Eigen::Matrix<double, 8, 8> P_post_old_2_;

  Eigen::Matrix<double, 8, 4> K_2_;
  Eigen::Matrix<double, 8, 4> K_old_2_;


  Eigen::Matrix<double, 2, 1> u_old_2_;


  void kalmanFilter2();
  void kalmanStateSpace2();
  //////////////////////////////////////////////////////////////


  /////////////////////////Kalman Filter3///////////////////////

  Eigen::Matrix<double, 10, 10> Ad_3_;
  Eigen::Matrix<double, 10, 2> Bd_3_;
  Eigen::Matrix<double, 6, 10> Cd_3_;
  Eigen::Matrix<double, 10, 10> Q_3_;
  Eigen::Matrix<double, 6, 6> R_3_;


  Eigen::Matrix<double, 10, 1> X_hat_prio_3_;
  Eigen::Matrix<double, 10, 1> X_hat_post_3_;
  Eigen::Matrix<double, 10, 1> X_hat_prio_old_3_;
  Eigen::Matrix<double, 10, 1> X_hat_post_old_3_;

  Eigen::Matrix<double, 6, 1> Y_3_;



  Eigen::Matrix<double, 10, 10> P_prio_3_;
  Eigen::Matrix<double, 10, 10> P_post_3_;
  Eigen::Matrix<double, 10, 10> P_prio_old_3_;
  Eigen::Matrix<double, 10, 10> P_post_old_3_;

  Eigen::Matrix<double, 10, 6> K_3_;
  Eigen::Matrix<double, 10, 6> K_old_3_;


  Eigen::Matrix<double, 2, 1> u_old_3_;

  void kalmanFilter3();
  void kalmanStateSpace3();
  //////////////////////////////////////////////////////////////

  ////////////////////////Extended Kalman Filter1////////////////////

private:

  Eigen::Matrix<double, 12, 12> F, P, I, Q, Fd;

  Eigen::Vector3d COP, fN, L, ti, contact_moment_;

  Eigen::Matrix<double, 9, 12> H;

  Eigen::Matrix<double, 12, 9> K;

  Eigen::Matrix<double, 9, 9> R, S;


  Eigen::Matrix<double, 9, 1> z;

  double tmp;

  void EKF1UpdateVars();

  MovingAverageFilter **moving_avg_filter_;


public:

  Eigen::Matrix<double, 12, 1> x, f;

  double comd_q, com_q, fd_q, comerr_q, com_r, comdd_r, ft_r;

  double dt, m, g, I_xx,I_yy;

    double bias_fx, bias_fy, bias_fz;
  bool firstrun;
  bool firstGyrodot_;
  void EKF1Init();

  void EKF1Setdt(double dtt) {
    dt = dtt;
  }

  void EKF1SetParams(double m_, double I_xx_, double I_yy_, double g_,
                     double com_q_, double comd_q_, double fd_q_, double comerr_q_, double com_r_, double comdd_r_, double ft_r_)
  {
    m = m_;
    I_xx = I_xx_;
    I_yy = I_yy_;
    g = g_;
    com_q = com_q_;
    comd_q = comd_q_;
    fd_q = fd_q_;
    comerr_q = comerr_q_;
    com_r = com_r_;
    comdd_r = comdd_r_;
    ft_r = ft_r_;
  }

  void EKF1SetCoMPos(Eigen::Vector3d pos) {
    x(0) = pos(0);
    x(1) = pos(1);
    x(2) = pos(2);
  }
  void EKF1SetCoMExternalForce(Eigen::Vector3d force) {
    x(6) = force(0);
    x(7) = force(1);
    x(8) = force(2);
  }

  void EKF1Predict(Eigen::Vector3d COP_, Eigen::Vector3d fN_, Eigen::Vector3d ti_, Eigen::Vector3d L_);
  void EKF1Update(Eigen::Vector3d Acc, Eigen::Vector3d Pos, Eigen::Vector3d Gyro, Eigen::Vector3d Gyrodot);
  void EKF1UpdateWithEnc(Eigen::Vector3d Pos);
  void EKF1UpdateWithImu(Eigen::Vector3d Acc, Eigen::Vector3d Pos, Eigen::Vector3d Gyro);  
  void gyrodotFilter();

  double comX, comY, comZ, velX, velY, velZ, fX, fY, fZ, com_x_error_, com_y_error_, com_z_error_;


  ///////////////////////////////////////////////////////////////////

};

}
#endif // WALKING_CONTROLLER_H
