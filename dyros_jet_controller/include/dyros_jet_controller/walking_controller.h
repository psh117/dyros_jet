#ifndef WALKING_CONTROLLER_H
#define WALKING_CONTROLLER_H

#include "dyros_jet_controller/dyros_jet_model.h"
#include "math_type_define.h"

namespace dyros_jet_controller
{

class WalkingController
{
public:


  static constexpr unsigned int PRIORITY = 8;

  WalkingController(const VectorQd& current_q, const double hz, const double& control_time) :
    total_dof_(DyrosJetModel::HW_TOTAL_DOF), current_q_(current_q), hz_(hz), control_time_(control_time), start_time_{}, end_time_{} {}

  void initWalkingPose(VectorQd& desired_q);
  void compute(VectorQd& desired_q);
  void setTarget(unsigned int joint_number, double target, double start_time, double end_time);
  void setTarget(unsigned int joint_number, double target, double duration);
  void setEnable(unsigned int joint_number, bool enable);
  void updateControlMask(unsigned int *mask);
  void writeDesired(const unsigned int *mask, VectorQd& desired_q);

  //functions in compute
  void getFootStep();
  void getCOMTrajectory();
  void getZMPTrajectory();
  void computeIKControl();
  void computeJacobianControl();
  void compensator();

private:

  const double hz_;
  const double &control_time_; // updated by control_base

  bool joint_enable_[DyrosJetModel::HW_TOTAL_DOF];

  VectorQd start_q_;
  VectorQd desired_q_;
  VectorQd target_q_;
  const VectorQd& current_q_;
  //const double &current_time_;
  const unsigned int total_dof_;
  const unsigned int ra_hand_ = 30;

  double start_time_[DyrosJetModel::HW_TOTAL_DOF];
  double end_time_[DyrosJetModel::HW_TOTAL_DOF];

};

}
#endif // WALKING_CONTROLLER_H
