#ifndef DYROS_JET_MODEL_H
#define DYROS_JET_MODEL_H

#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "math_type_define.h"

class DyrosJetModel
{
public:
  DyrosJetModel();
  void test();

  enum EndEffector {
    EE_LEFT_FOOT, EE_RIGHT_FOOT,
    EE_LEFT_HAND, EE_RIGHT_HAND,};

  static constexpr size_t MODEL_DOF = 28;
  static constexpr const char* EE_NAME[4] =
      {"L_AnckleRoll_Link", "R_AnckleRoll_Link",
       "L_HandYaw_Link", "R_HandYaw_Link" };

  unsigned int end_effector_id_[4];
  unsigned int joint_start_index_[4];

  void getTransformEndEffector(EndEffector ee, const Eigen::VectorXd& q, bool update_kinematics,
                                  Eigen::Vector3d* position, Eigen::Matrix3d* rotation);
private:
  RigidBodyDynamics::Model model_;
  Eigen::Vector28d q_;
  Eigen::Vector3d base_position_;
};

#endif // DYROS_JET_MODEL_H
