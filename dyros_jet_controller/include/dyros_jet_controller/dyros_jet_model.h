#ifndef DYROS_JET_MODEL_H
#define DYROS_JET_MODEL_H

#include <string>
#include <map>

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

  enum EndEffector : unsigned int {
    EE_LEFT_FOOT, EE_RIGHT_FOOT,
    EE_LEFT_HAND, EE_RIGHT_HAND};

  static constexpr size_t HW_TOTAL_DOF = 32;
  static constexpr size_t MODEL_WITH_VJOINT_DOF = 34;
  static constexpr size_t MODEL_DOF = 28;

  static const std::string JOINT_NAME[HW_TOTAL_DOF];
  static const int JOINT_ID[HW_TOTAL_DOF];

  static constexpr const char* EE_NAME[4] =
      {"L_AnkleRoll_Link", "R_AnkleRoll_Link",
       "L_HandYaw_Link", "R_HandYaw_Link" };

  unsigned int end_effector_id_[4];
  const unsigned int joint_start_index_[4];

  void test();
  // Calc Jacobian, Transformation
  void updateKinematics(const Eigen::VectorXd &q ///< input q (size must be DyrosJetModel::MODEL_DOF)
                        );

  void getTransformEndEffector(EndEffector ee, Eigen::Isometry3d* transform_matrix);
  void getTransformEndEffector(EndEffector ee, Eigen::Vector3d* position, Eigen::Matrix3d* rotation);

  void getTransformEndEffector(EndEffector ee, const Eigen::VectorXd& q, bool update_kinematics,
                                  Eigen::Vector3d* position, Eigen::Matrix3d* rotation);

  void getJacobianMatrix6DoF(EndEffector ee, Eigen::Matrix<double, 6, 6> *jacobian);
  void getJacobianMatrix7DoF(EndEffector ee, Eigen::Matrix<double, 6, 7> *jacobian);
  void getJacobianMatrix12DoF(EndEffector ee, Eigen::Matrix<double, 6, 12> *jacobian);  ///< @brief get jacobian matrix with virtual link (6 DOF)
  void getInertiaMatrix34DoF(Eigen::Matrix<double, 34, 34> *inertia); ///< @brief get whole body inertia matrix with virtual link (34 DOF)
  void getInertiaMatrix18DoF(Eigen::Matrix<double, 18, 18> *leg_inertia); ///< @brief get leg inertia matrix with virtual link (18 DOF)



  const Eigen::Isometry3d& getCurrentTrasmfrom(EndEffector ee) { return currnet_transform_[ee]; }
  const Eigen::Matrix<double, 6, 6>& getLegJacobian(EndEffector ee) { return leg_jacobian_[ee]; }
  const Eigen::Matrix<double, 6, 12>& getLegWithVLinkJacobian(EndEffector ee) { return leg_with_vlink_jacobian_[ee]; }
  const Eigen::Matrix<double, 6, 7>& getArmJacobian(EndEffector ee) { return arm_jacobian_[ee-2]; }

private:
  RigidBodyDynamics::Model model_;

  Eigen::Matrix<double, MODEL_WITH_VJOINT_DOF, 1> q_;
  Eigen::Vector3d base_position_;

  Eigen::Isometry3d currnet_transform_[4];

  Eigen::Matrix<double, 6, 6> leg_jacobian_[2];
  Eigen::Matrix<double, 6, 12> leg_with_vlink_jacobian_[2];
  Eigen::Matrix<double, 6, 7> arm_jacobian_[2];
  Eigen::Matrix<double, MODEL_WITH_VJOINT_DOF, MODEL_WITH_VJOINT_DOF> A_;
  Eigen::Matrix<double, 18, 18> leg_A_;
  Eigen::MatrixXd A_temp_;


};

typedef Eigen::Matrix<double, DyrosJetModel::HW_TOTAL_DOF, 1> VectorQd;


}
#endif // DYROS_JET_MODEL_H
