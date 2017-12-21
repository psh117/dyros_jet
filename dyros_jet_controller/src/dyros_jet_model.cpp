#include "dyros_jet_controller/dyros_jet_model.h"



// https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
constexpr const char* DyrosJetModel::EE_NAME[4];

DyrosJetModel::DyrosJetModel()
{
  base_position_.setZero();

  std::string desc_package_path = ros::package::getPath("dyros_jet_description");
  std::string urdf_path = desc_package_path + "/robots/dyros_jet_robot.urdf";

  ROS_INFO("Loading DYROS JET description from = %s",urdf_path.c_str());
  RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_, true);
  ROS_INFO("Successfully loaded.");
  ROS_INFO("Total DoF = %d", model_.dof_count);
  ROS_INFO("Total DoF = %d", model_.q_size);
  //model_.mJoints[0].)
  if(model_.dof_count != MODEL_DOF)
  {
    ROS_WARN("The DoF in the model file and the code do not match.");
    ROS_WARN("Model file = %d, Code = %d", model_.dof_count, (int)MODEL_DOF);
  }


  for (size_t i=0; i<4; i++)
  {
    end_effector_id_[i] = model_.GetBodyId(EE_NAME[i]);
    ROS_INFO("%s: id - %d",EE_NAME[i], end_effector_id_[i]);
    std::cout << model_.mBodies[end_effector_id_[i]].mCenterOfMass << std::endl;
  }
}



void DyrosJetModel::getTransformEndEffector
(EndEffector ee, const Eigen::VectorXd& q, bool update_kinematics,
 Eigen::Vector3d* position, Eigen::Matrix3d* rotation)
{
  Eigen::Vector28d q_new;
  q_new = q_;
  switch (ee)
  {
  case EE_LEFT_FOOT:
  case EE_RIGHT_FOOT:
    q_new.segment<6>(joint_start_index_[ee]) = q;
    break;
  case EE_LEFT_HAND:
  case EE_RIGHT_HAND:
    q_new.segment<7>(joint_start_index_[ee]) = q;
    break;
  }
  if (update_kinematics)
  {
    q_ = q_new;
  }
  *position = RigidBodyDynamics::CalcBaseToBodyCoordinates
      (model_,q_new,end_effector_id_[ee], base_position_, update_kinematics);
  *rotation = RigidBodyDynamics::CalcBodyWorldOrientation(
        model_, q_new, end_effector_id_[ee], update_kinematics);
  // RigidBodyDynamics::Calcpo
  // model_.mBodies[0].mCenterOfMass
}


void DyrosJetModel::test()
{

  //RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,);

  //model_.GetBodyId()
  ROS_INFO("URDF LOAD COMPLETE!!");

}
