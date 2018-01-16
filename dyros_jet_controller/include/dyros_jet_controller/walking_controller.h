#ifndef WALKING_CONTROLLER_H
#define WALKING_CONTROLLER_H


#include "dyros_jet_controller/dyros_jet_model.h"
#include "math_type_define.h"
#include <vector>

using namespace std;
namespace dyros_jet_controller
{

class WalkingController
{
public:


  static constexpr unsigned int PRIORITY = 8;


  WalkingController(DyrosJetModel& model, const VectorQd& current_q, const double hz, const double& control_time) :
    total_dof_(DyrosJetModel::HW_TOTAL_DOF), model_(model), current_q_(current_q), hz_(hz), current_time_(control_time), start_time_{}, end_time_{} {}



  void compute(VectorQd* desired_q);
  void setTarget(int walk_mode, std::vector<bool> compensator_mode, int ik_mode, bool heel_toe,
                 bool is_right_foot_swing, double x, double y, double z, double theta,
                 double step_length, double step_length_y);
  void setEnable(bool enable);
  void updateControlMask(unsigned int *mask);
  void writeDesired(const unsigned int *mask, VectorQd& desired_q);

  void parameterSetting();
  //functions in compute
  void getFootStep();
  void getComTrajectory();
  void getZmpTrajectory();
  void getPelvTrajectory();
  void getFootTrajectory();
  void computeIkControl(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::VectorLXd& desired_leg_q);
  void computeJacobianControl(Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::VectorLXd& desired_leg_q_dot);
  void compensator();

  void updateInitialState();


  //functions for getFootStep()
  void calculateFootStepTotal();
  void calculateFootStepSeparate();

  //functions for getZMPTrajectory()
  void floatToSupportFootstep();
  void addZmpOffset();
  void zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num);
  void onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);

  //PreviewController
  //void previewControl(Eigen::Vector3d com_support_init_);
  void previewControlParameter(double dt, int NL, Eigen::Matrix4d& k, double& gi, Eigen::VectorXd& gp_l, Eigen::Matrix1x3d& gx, Eigen::Matrix3d& a, Eigen::Vector3d& b, Eigen::Matrix1x3d& c);
  Eigen::Matrix4d discreteRicattiEquation(Eigen::Matrix4d a, Eigen::Vector4d b, double r, Eigen::Matrix4d q);


private:

  const double hz_;
  const double &current_time_; // updated by control_base
  double walking_tick_ = 0;
  double walking_time_ = 0;

  //parameterSetting()
  double t_last_;
  double t_start_;
  double t_temp_;
  double t_rest_init_;
  double t_rest_last_;
  double t_double1_;
  double t_double2_;
  double t_total_;
  double foot_height_;


  int ik_mode_;
  int walk_mode_;
  std::vector<bool> compensator_mode_;
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
  Eigen::MatrixXd foot_step_;
  Eigen::MatrixXd foot_step_support_frame_;
  Eigen::MatrixXd foot_step_support_frame_offset_;

  Eigen::MatrixXd ref_zmp_;

  double current_step_num_;

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
  VectorQd q_init_;

  Eigen::Vector3d supportfoot_float_init_;
  Eigen::Vector3d supportfoot_support_init_;
  Eigen::Vector3d supportfoot_support_init_offset_;
  Eigen::Vector3d swingfoot_float_init_;
  Eigen::Vector3d swingfoot_support_init_;
  Eigen::Vector3d swingfoot_support_init_offset_;

  Eigen::Vector3d com_float_init_;
  Eigen::Vector3d com_support_init_;

  double lfoot_zmp_offset_;   //have to be initialized
  double rfoot_zmp_offset_;
  Eigen::Vector3d com_offset_;

  //Step current state variable//
  Eigen::Vector3d com_support_cuurent_;
  Eigen::Isometry3d pelv_support_cuurent_;
  Eigen::Isometry3d lfoot_support_cuurent_;
  Eigen::Isometry3d rfoot_support_cuurent_;
  Eigen::Vector3d com_float_cuurent_;
  Eigen::Isometry3d pelv_float_cuurent_;
  Eigen::Isometry3d lfoot_float_cuurent_;
  Eigen::Isometry3d rfoot_float_cuurent_;



  DyrosJetModel &model_;
  Eigen::Isometry3d currnet_leg_transform_[2];
  Eigen::Isometry3d currnet_leg_transform_l_;
  Eigen::Isometry3d currnet_leg_transform_r_;

  Eigen::Matrix6d current_leg_jacobian_[2];
  Eigen::Matrix6d current_leg_jacobian_l_;
  Eigen::Matrix6d current_leg_jacobian_r_;

  Eigen::VectorLXd desired_leg_q_;
  Eigen::VectorLXd desired_leg_q_dot_;

  //Preview Control
  double zc;




};

}
#endif // WALKING_CONTROLLER_H
