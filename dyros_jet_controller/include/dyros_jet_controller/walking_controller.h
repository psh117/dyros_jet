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

  WalkingController(DyrosJetModel& model,const VectorQd& current_q, const double hz, const double& control_time) :
    total_dof_(DyrosJetModel::HW_TOTAL_DOF), dyros_model_(model), current_q_(current_q), hz_(hz), current_time_(control_time), start_time_{}, end_time_{} {}


  void compute(VectorQd* desired_q);
  void setTarget(int walk_mode, std::vector<bool> compensator_mode, int ik_mode, bool heel_toe,
                 bool is_right_foot_swing, double x, double y, double z, double theta,
                 double step_length, double step_length_y);
//  void setTarget(unsigned int joint_number, double target, double duration);
  void setEnable(bool enable);
  void updateControlMask(unsigned int *mask);
  void writeDesired(const unsigned int *mask, VectorQd& desired_q);

  //functions in compute
  void getFootStep();
  void getCOMTrajectory();
  void getZMPTrajectory();
  void computeIKControl();
  void computeJacobianControl();
  void compensator();

  //functions for getFootStep()
  void calculateFootStepTotal();
  void calculateFootStepSeparate();

  //functions for getZMPTrajectory()
  void floatToSupportFootstep();

  void updateInitialState();



private:
  DyrosJetModel &dyros_model_;

  const double hz_;
  const double &current_time_; // updated by control_base
  double walking_tick = 0;
  double walking_time = 0;

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

  double current_step_num_;

  bool walking_enable_;


  VectorQd start_q_;
  VectorQd desired_q_;
  VectorQd target_q_;
  const VectorQd& current_q_;
  //const double &current_time_;
  const unsigned int total_dof_;
  double start_time_[DyrosJetModel::HW_TOTAL_DOF];
  double end_time_[DyrosJetModel::HW_TOTAL_DOF];

  //Step initial state variable//
  Eigen::Vector3d com_suppport_init_;
  Eigen::Isometry3d pelv_support_init_;
  Eigen::Isometry3d lfoot_support_init_;
  Eigen::Isometry3d rfoot_support_init_;
  Eigen::Vector3d com_float_init_;
  Eigen::Isometry3d pelv_float_init_;
  Eigen::Isometry3d lfoot_float_init_;
  Eigen::Isometry3d rfoot_float_init_;
  VectorQd q_init_;

  //Step initial state variable//
  Eigen::Vector3d com_support_cuurent_;
  Eigen::Isometry3d pelv_support_cuurent_;
  Eigen::Isometry3d lfoot_support_cuurent_;
  Eigen::Isometry3d rfoot_support_cuurent_;
  Eigen::Vector3d com_float_cuurent_;
  Eigen::Isometry3d pelv_float_cuurent_;
  Eigen::Isometry3d lfoot_float_cuurent_;
  Eigen::Isometry3d rfoot_float_cuurent_;

  Eigen::Vector3d supportfoot_float_init;
  Eigen::Vector3d supportfoot_support_init;
  Eigen::Vector3d swingfoot_float_init;
  Eigen::Vector3d swingfoot_suppport_init;
};

}
#endif // WALKING_CONTROLLER_H
