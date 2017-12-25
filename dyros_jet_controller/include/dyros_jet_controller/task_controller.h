#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H

#include <array>
#include <Eigen/Geometry>
#include "dyros_jet_controller/dyros_jet_model.h"
#include "math_type_define.h"

namespace dyros_jet_controller
{

class TaskController
{
public:

  static constexpr unsigned int PRIORITY = 16;  ///< Joint priority

  TaskController(DyrosJetModel& model, const VectorQd& current_q, const double hz, const double& control_time) :
    model_(model), current_q_(current_q), hz_(hz), control_time_(control_time), start_time_{}, end_time_{} {}
  void plan();
  void compute();
  void setTarget(DyrosJetModel::EndEffector ee, Eigen::HTransform target, double start_time, double end_time);
  void setTarget(DyrosJetModel::EndEffector ee, Eigen::HTransform target, double duration);
  void setEnable(DyrosJetModel::EndEffector ee, bool enable);
  void updateControlMask(unsigned int *mask);
  void writeDesired(const unsigned int *mask, VectorQd& desired_q);
  //void setTarget(DyrosJetModel::EndEffector ee, )
private:
  void computeCLIK();

  void taskCLIKControl(DyrosJetModel::EndEffector ee);

  unsigned int total_dof_;

  bool ee_enabled_[4];

  // Eigen::HTransform current_transform_[4]; // --> Use model_.getCurrentTransform()

  // motion time
  const double hz_;
  const double &control_time_; // updated by control_base
  double start_time_[4];
  double end_time_[4];

  Eigen::HTransform start_transform_[4];
  Eigen::HTransform previous_transform_[4];
  Eigen::HTransform desired_transform_[4];
  Eigen::HTransform target_transform_[4];

  Eigen::Vector3d start_x_dot_;
  Eigen::Vector3d x_prev_;  //< Previous x

  DyrosJetModel &model_;

  VectorQd desired_q_;
  const VectorQd &current_q_;  // updated by control_base



};


}
#endif // TASK_CONTROLLER_H
