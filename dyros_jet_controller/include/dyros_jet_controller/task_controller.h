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

  static constexpr unsigned int PRIORITY = 16;

  TaskController(DyrosJetModel& model, const VectorQd& current_q) :
    model_(model), current_q_(current_q) , start_time_{}, end_time_{} {}
  void plan();
  void loop();
  void setTarget(DyrosJetModel::EndEffector ee, Eigen::HTransform target);
  void setEnable(DyrosJetModel::EndEffector ee, bool enable);
  void updateControlMask(unsigned int *mask);
  //void setTarget(DyrosJetModel::EndEffector ee, )
private:
  void armComputeLoop();
  void legComputeLoop();

  void taskCLIKControl(DyrosJetModel::EndEffector ee);

  unsigned int total_dof_;

  bool ee_enabled_[4];

  // Eigen::HTransform current_transform_[4]; // --> Use model_.getCurrentTransform()

  // motion time
  double start_time_[4];
  double end_time_[4];

  Eigen::HTransform desired_transform_[4];
  Eigen::HTransform target_transform_[4];


  DyrosJetModel &model_;

  VectorQd desired_q_;
  const VectorQd &current_q_;



};


}
#endif // TASK_CONTROLLER_H
