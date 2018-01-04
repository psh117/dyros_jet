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

  //functions in compute
  void getFootStep();
  void getCOMTrajectory();
  void getZMPTrajectory();
  void computeIKControl(Eigen::VectorLXd *desired_leg_q);
  void computeJacobianControl();
  void compensator();

  //functions for getFootStep()
  void calculateFootStepTotal();
  void calculateFootStepSeparate();



private:

  const double hz_;
  const double &current_time_; // updated by control_base


  bool walking_enable_;
  bool joint_enable_[DyrosJetModel::HW_TOTAL_DOF];
  double step_length_x_;
  double step_length_y_;
  //double step_angle_theta_;
  double target_x_;
  double target_y_;
  double target_z_;
  double target_theta_;
  double step_num_;
  Eigen::MatrixXd foot_step_;

  VectorQd start_q_;
  VectorQd desired_q_;
  VectorQd target_q_;
  const VectorQd& current_q_;
  //const double &current_time_;
  const unsigned int total_dof_;
  double start_time_[DyrosJetModel::HW_TOTAL_DOF];
  double end_time_[DyrosJetModel::HW_TOTAL_DOF];

  DyrosJetModel &model_;
  Eigen::Isometry3d currnet_leg_transform_[2];
  Eigen::Isometry3d currnet_leg_transform_l_;
  Eigen::Isometry3d currnet_leg_transform_r_;

};

}
#endif // WALKING_CONTROLLER_H
