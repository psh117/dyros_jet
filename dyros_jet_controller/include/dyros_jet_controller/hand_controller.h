#ifndef HAND_CONTROLLER_H
#define HAND_CONTROLLER_H

#include "dyros_jet_controller/dyros_jet_model.h"
#include "math_type_define.h"

namespace dyros_jet_controller
{


class HandController
{
public:
  static constexpr unsigned int PRIORITY = 64;  ///< Joint priority // need to fix

  HandController(const Vector4d& current_q_hand, const double& control_time);
  void compute();
  void setTarget(unsigned int joint_number, double target, double start_time, double end_time);
  void setTarget(unsigned int joint_number, double target, double duration);
  void setEnable(unsigned int joint_number, bool enable);
  void updateControlMask(unsigned int *mask);
  void writeDesired(const unsigned int *mask, Vector4d& desired_q_hand);
  bool isEnabled(int index) { return joint_enable_[index]; }
private:
  bool joint_enable_[DyrosJetModel::HW_HAND_DOF];

  const unsigned int total_dof_;
  VectorQd start_q_;
  VectorQd desired_q_;
  VectorQd target_q_;
  const Vector4d& current_q_;
  const double &current_time_;
  double start_time_[DyrosJetModel::HW_HAND_DOF];
  double end_time_[DyrosJetModel::HW_HAND_DOF];
};

}
#endif // HAND_CONTROLLER_H
