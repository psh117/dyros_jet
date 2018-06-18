#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include "dyros_jet_controller/dyros_jet_model.h"
#include "math_type_define.h"

namespace dyros_jet_controller
{


class JointController
{
public:
  static constexpr unsigned int PRIORITY = 64;  ///< Joint priority

  JointController(const VectorQd& current_q, const double& control_time);
  void compute();
  void setTarget(unsigned int joint_number, double target, double start_time, double end_time);
  void setTarget(unsigned int joint_number, double target, double duration);
  void setEnable(unsigned int joint_number, bool enable);
  void updateControlMask(unsigned int *mask);
  void writeDesired(const unsigned int *mask, VectorQd& desired_q);
  bool isEnabled(int index) { return joint_enable_[index]; }
private:
  bool joint_enable_[DyrosJetModel::HW_TOTAL_DOF];

  const unsigned int total_dof_;
  VectorQd start_q_;
  VectorQd desired_q_;
  VectorQd target_q_;
  const VectorQd& current_q_;
  const double &current_time_;
  double start_time_[DyrosJetModel::HW_TOTAL_DOF];
  double end_time_[DyrosJetModel::HW_TOTAL_DOF];
};

} // namespace dyros_jet_controller

#endif // JOINT_CONTROLLER_H
