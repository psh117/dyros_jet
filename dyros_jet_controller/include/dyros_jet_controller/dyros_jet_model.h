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
  static constexpr size_t MODEL_DOF = 28;
  static constexpr size_t MODEL_WITH_VIRTUAL_DOF = 34;
  static constexpr size_t HW_HAND_DOF = 4;

  static const std::string JOINT_NAME[HW_TOTAL_DOF];
  static const int JOINT_ID[HW_TOTAL_DOF];

  static constexpr const char* EE_NAME[4] =
      {"L_AnkleRoll_Link", "R_AnkleRoll_Link",
       "L_HandYaw_Link", "R_HandYaw_Link" };

  unsigned int end_effector_id_[4];
  const unsigned int joint_start_index_[4];

  void test();

  std::map<std::string, size_t> joint_name_map_;
  inline size_t getIndex(const std::string& joint_name) const
  {
    return joint_name_map_.at(joint_name);
  }
  // Calc Jacobian, Transformation
  void updateKinematics(const Eigen::VectorXd &q);
  void updateSensorData(const Eigen::Vector6d &r_ft, const Eigen::Vector6d &l_ft);

  void getTransformEndEffector(EndEffector ee, Eigen::Isometry3d* transform_matrix);
  void getTransformEndEffector(EndEffector ee, Eigen::Vector3d* position, Eigen::Matrix3d* rotation);

  void getTransformEndEffector(EndEffector ee, const Eigen::VectorXd& q, bool update_kinematics,
                                  Eigen::Vector3d* position, Eigen::Matrix3d* rotation);


  void getJacobianMatrix6DoF(EndEffector ee, Eigen::Matrix<double, 6, 6> *jacobian);
  void getJacobianMatrix7DoF(EndEffector ee, Eigen::Matrix<double, 6, 7> *jacobian);

  void getCenterOfMassPosition(Eigen::Vector3d* position);

  void getLegMassMatrix18Dof(Eigen::Matrix<double, 18, 18> *massmatrix);

  const Eigen::Isometry3d& getCurrentTrasmfrom(EndEffector ee) { return currnet_transform_[ee]; }
  const Eigen::Matrix<double, 6, 6>& getLegJacobian(EndEffector ee) { return leg_jacobian_[ee]; }
  const Eigen::Matrix<double, 6, 7>& getArmJacobian(EndEffector ee) { return arm_jacobian_[ee-2]; }
  const Eigen::Vector3d getCurrentCom(){ return com_;}
  const Eigen::Vector6d getRightFootForce() {return r_ft_wrench_;}
  const Eigen::Vector6d getLeftFootForce() {return l_ft_wrench_;}
  const Eigen::Matrix<double, 18, 18>& getLegMassMatrix(){ return leg_massmatrix_;}

private:
  RigidBodyDynamics::Model model_;

  Eigen::Vector28d q_;
  Eigen::Vector3d base_position_;

  Eigen::Isometry3d currnet_transform_[4];

  Eigen::Matrix<double, 6, 6> leg_jacobian_[2];
  Eigen::Matrix<double, 6, 7> arm_jacobian_[2];
  Eigen::Matrix28d A_;
  Eigen::MatrixXd A_temp_;
  Eigen::Vector6d r_ft_wrench_;
  Eigen::Vector6d l_ft_wrench_;

  Eigen::Vector3d com_;
  Eigen::Matrix18d leg_massmatrix_; //With Virtual Joint

};

typedef Eigen::Matrix<double, DyrosJetModel::HW_TOTAL_DOF, 1> VectorQd;


}
#endif // DYROS_JET_MODEL_H
