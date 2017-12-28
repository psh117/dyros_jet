#ifndef WALKING_CONTROLLER_H
#define WALKING_CONTROLLER_H

#include "dyros_jet_controller/dyros_jet_model.h"
#include "math_type_define.h"

namespace dyros_jet_controller
{

class WalkingController
{
public:


  static constexpr unsigned int PRIORITY = 2;

  void WalkingController::initWalkingPose(VectorQd& desired_q);
  void WalkingController::compute(VectorQd& desired_q);
  void WalkingController::writeDesired(const unsigned int *mask, VectorQd& desired_q);

private:
  bool joint_enable_[DyrosJetModel::HW_TOTAL_DOF];

  VectorQd start_q_;
  VectorQd desired_q_;
  VectorQd target_q_;
  const double &current_time_;
  const unsigned int total_dof_;
  const unsigned int RA_HAND = 31;

}

}
