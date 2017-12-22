#ifndef DYROS_JET_MODEL_H
#define DYROS_JET_MODEL_H

#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "math_type_define.h"


namespace dyros_jet_controller
{



class DyrosJetModel
{
public:
  DyrosJetModel();
  void test();

  enum EndEffector : unsigned int {
    EE_LEFT_FOOT, EE_RIGHT_FOOT,
    EE_LEFT_HAND, EE_RIGHT_HAND};

  static constexpr size_t HW_TOTAL_DOF = 32;
  static constexpr size_t MODEL_DOF = 28;

  static constexpr const char* EE_NAME[4] =
      {"L_AnckleRoll_Link", "R_AnckleRoll_Link",
       "L_HandYaw_Link", "R_HandYaw_Link" };

  unsigned int end_effector_id_[4];
  unsigned int joint_start_index_[4];

  // Calc Jacobian, Transformation
  void updateKinematics(const Eigen::VectorXd& q);

  void getTransformEndEffector(EndEffector ee, Eigen::HTransform* transform_matrix);
  void getTransformEndEffector(EndEffector ee, Eigen::Vector3d* position, Eigen::Matrix3d* rotation);

  void getTransformEndEffector(EndEffector ee, const Eigen::VectorXd& q, bool update_kinematics,
                                  Eigen::Vector3d* position, Eigen::Matrix3d* rotation);


  void getJacobianMatrix6DoF(EndEffector ee, Eigen::Matrix<double, 6, 6> *jacobian);
  void getJacobianMatrix7DoF(EndEffector ee, Eigen::Matrix<double, 6, 7> *jacobian);

  const Eigen::HTransform& getCurrentTrasmfrom(EndEffector ee) { return currnet_transform_[ee]; }
  const Eigen::Matrix<double, 6, 6>& getLegJacobian(EndEffector ee) { return leg_jacobian_[ee]; }
  const Eigen::Matrix<double, 6, 7>& getArmJacobian(EndEffector ee) { return arm_jacobian_[ee-2]; }

private:
  RigidBodyDynamics::Model model_;

  Eigen::Vector28d q_;
  Eigen::Vector3d base_position_;

  Eigen::HTransform currnet_transform_[4];

  Eigen::Matrix<double, 6, 6> leg_jacobian_[2];
  Eigen::Matrix<double, 6, 7> arm_jacobian_[2];


};

typedef Eigen::Matrix<double, DyrosJetModel::HW_TOTAL_DOF, 1> VectorQd;


}
#endif // DYROS_JET_MODEL_H
