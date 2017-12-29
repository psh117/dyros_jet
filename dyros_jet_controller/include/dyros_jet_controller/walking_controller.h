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


  static constexpr unsigned int PRIORITY = 2;

  WalkingController(const VectorQd& current_q, const double hz, const double& control_time) :
    total_dof_(DyrosJetModel::HW_TOTAL_DOF), current_q_(current_q), hz_(hz), current_time_(control_time), start_time_{}, end_time_{} {}

  void compute(VectorQd* desired_q);
  void setTarget(int walk_mode, std::vector<bool> compensator_mode,int ik_mode, bool heel_toe, bool first_foot_step, double x, double y, double z, double theta, double step_length);
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

private:

  const double hz_;
 // const double &control_time_; // updated by control_base

  bool walking_enable_;


  VectorQd start_q_;
  VectorQd desired_q_;
  VectorQd target_q_;
  const VectorQd& current_q_;
  const double &current_time_;
  const unsigned int total_dof_;

  double start_time_[DyrosJetModel::HW_TOTAL_DOF];
  double end_time_[DyrosJetModel::HW_TOTAL_DOF];

};

}
#endif // WALKING_CONTROLLER_H
