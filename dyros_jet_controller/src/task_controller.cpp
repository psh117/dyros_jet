#include "dyros_jet_controller/task_controller.h"
#include "dyros_jet_controller/dyros_jet_model.h"

namespace dyros_jet_controller
{

constexpr unsigned int TaskController::PRIORITY;

void TaskController::compute()
{
  // update state
  computeCLIK();
  //
}
void TaskController::setTarget(DyrosJetModel::EndEffector ee, Eigen::Isometry3d target, double start_time, double end_time)
{
  start_transform_[ee] = model_.getCurrentTransform(ee);
  target_transform_[ee] = target;
  start_time_[ee] = start_time;
  end_time_[ee] = end_time;
  x_prev_[ee] = start_transform_[ee].translation();
  target_arrived_[ee] = false;
  if (ee < 2)  // Legs
  {
    Eigen::Vector6d x = model_.getLegJacobian(ee) *
        current_q_dot_.segment<6>(model_.joint_start_index_[ee]);
    start_x_dot_[ee] = x.head<3>();
    start_w_[ee] = x.tail<3>();
  }
  else
  {
    Eigen::Vector6d x = model_.getArmJacobian(ee) *
        current_q_dot_.segment<7>(model_.joint_start_index_[ee]);
    start_x_dot_[ee] = x.head<3>();
    start_w_[ee] = x.tail<3>();
  }
}

void TaskController::setTarget(DyrosJetModel::EndEffector ee, Eigen::Isometry3d target, double duration)
{
  setTarget(ee, target, control_time_, control_time_ + duration);
}
void TaskController::setEnable(DyrosJetModel::EndEffector ee, bool enable)
{
  ee_enabled_[ee] = enable;
}
void TaskController::updateControlMask(unsigned int *mask)
{
  unsigned int index = 0;
  for(int i=0; i<total_dof_; i++)
  {
    if(i < 6)
    {
      index = 0;
    }
    else if (i < 6 + 6)
    {
      index = 1;
    }
    else if (i < 6 + 6 + 2)
    {
      continue; // waist
    }
    else if (i < 6 + 6 + 2 + 7)
    {
      index = 2;
    }
    else if (i < 6 + 6 + 2 + 7 + 7)
    {
      index = 3;
    }

    if(ee_enabled_[index])
    {
      if (mask[i] >= PRIORITY * 2)
      {
        // Higher priority task detected
        ee_enabled_[index] = false;
        target_transform_[index] = model_.getCurrentTransform((DyrosJetModel::EndEffector)index);
        end_time_[index] = control_time_;
        if (index < 2)  // Legs
        {
          desired_q_.segment<6>(model_.joint_start_index_[index]) = current_q_.segment<6>(model_.joint_start_index_[index]);
        }
        else
        {
          desired_q_.segment<7>(model_.joint_start_index_[index]) = current_q_.segment<7>(model_.joint_start_index_[index]);
        }
        //setTarget((DyrosJetModel::EndEffector)index, model_.getCurrentTransform((DyrosJetModel::EndEffector)index), 0); // Stop moving
        target_arrived_[index] = true;
      }
      mask[i] = (mask[i] | PRIORITY);
    }
    else
    {
      mask[i] = (mask[i] & ~PRIORITY);
      //setTarget((DyrosJetModel::EndEffector)index, model_.getCurrentTransform((DyrosJetModel::EndEffector)index), 0); // Stop moving
      target_arrived_[index] = true;
    }
  }
}

void TaskController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
{
  for(unsigned int i=0; i<total_dof_; i++)
  {
    if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
    {
      desired_q(i) = desired_q_(i);
    }
  }
}

// Jacobian OK. Translation OK.
void TaskController::computeCLIK()
{
  const double inverse_damping = 1e-4;
  const double phi_gain = 0.5;
  const double kp = 200;
  const double epsilon = 1e-3;

  // Arm
  for(unsigned int i=0; i<4; i++)
  {
    if(ee_enabled_[i])
    {
      // For short names
      const auto &x_0 = start_transform_[i].translation();
      const auto &rot_0 = start_transform_[i].linear();

      const auto &x = model_.getCurrentTransform((DyrosJetModel::EndEffector)(i)).translation();
      const auto &rot = model_.getCurrentTransform((DyrosJetModel::EndEffector)(i)).linear();

      //debug_ << control_time_ << "\t" << x(0) << "\t" << x(1) << "\t" <<x(2) << std::endl;

      const auto &x_target = target_transform_[i].translation();
      const auto &rot_target = target_transform_[i].linear();

      Eigen::Vector3d x_cubic;
      Eigen::Vector6d x_dot_desired;
      Eigen::Matrix3d rot_cubic;
      x_cubic = DyrosMath::cubicVector<3>(control_time_,
                                          start_time_[i],
                                          end_time_[i],
                                          x_0,
                                          x_target,
                                          Eigen::Vector3d::Zero(),//start_x_dot_[i],
                                          Eigen::Vector3d::Zero());

      rot_cubic = DyrosMath::rotationCubic(control_time_,
                                           start_time_[i],
                                           end_time_[i],
                                           start_w_[i],
                                           Eigen::Vector3d::Zero(),
                                           rot_0,
                                           rot_target);
      /*
      std::cout << " rot 0 ----" << std::endl;
      std::cout << rot_0 << std::endl;
      std::cout << " rot cubic ----" << std::endl;
      std::cout << rot_cubic << std::endl;
      std::cout << " rot target ----" << std::endl;
      std::cout << rot_target << std::endl;*/
      Eigen::Vector6d x_error;
      x_error.head<3>() = (x_cubic - x);
      //x_error.tail<3>().setZero();
      x_error.tail<3>() =  - phi_gain * DyrosMath::getPhi(rot, rot_cubic);
      x_dot_desired.head<3>() = x_cubic - x_prev_[i];
      x_dot_desired.tail<3>().setZero();

      x_error = x_error; //* hz_;
      x_dot_desired = x_dot_desired * hz_;
      x_prev_[i] = x_cubic;

      if (x_error.norm() < epsilon && control_time_ >= end_time_[i]) // target arrived, cubic finished
      {
        target_arrived_[i] = true;
        ee_enabled_[i] = false;
        ROS_INFO("target arrived - %d End effector", i);
        continue;
      }
      if (i < 2)  // Legs
      {
        const auto &J = model_.getLegJacobian((DyrosJetModel::EndEffector)(i));
        const auto &q = current_q_.segment<6>(model_.joint_start_index_[i]);

        auto J_inverse = J.transpose() *
            (inverse_damping * Eigen::Matrix6d::Identity() +
             J * J.transpose()).inverse();

        desired_q_.segment<6>(model_.joint_start_index_[i])
            = (J_inverse * (x_dot_desired + x_error * kp)) / hz_ + q;
      }
      else    // Arms
      {
        const auto &J = model_.getArmJacobian((DyrosJetModel::EndEffector)(i));
        const auto &q = current_q_.segment<7>(model_.joint_start_index_[i]);


        auto J_inverse = J.transpose() *
            (inverse_damping * Eigen::Matrix6d::Identity() +
             J * J.transpose()).inverse();


        desired_q_.segment<7>(model_.joint_start_index_[i]) =
            (J_inverse * (x_dot_desired + x_error * kp)) / hz_ + q;

        /*
        auto tmp = (J_inverse * (x_dot_desired + x_error * kp)) / hz_;

          std::cout << "Jacobian : " << std::endl;
          std::cout << J << std::endl;
          std::cout << "Jacobian.inv : "<< std::endl;
          std::cout << J_inverse << std::endl;
          std::cout << "desired_q_ : " << std::endl << desired_q_.segment<7>(model_.joint_start_index_[i]) << std::endl;
          std::cout << "x : " << std::endl << x << std::endl;
          std::cout << "q : " << std::endl << current_q_.segment<7>(model_.joint_start_index_[i]) << std::endl;
          std::cout << "calc : " << std::endl << tmp << std::endl;
          std::cout << "rot : " << std::endl << rot << std::endl;
          std::cout << "rot : " << std::endl << rot_target << std::endl;
          std::cout << "x_cubic : " << std::endl << x_cubic << std::endl;
          std::cout << "x_error : " << std::endl << x_error << std::endl;
          std::cout << "x_dot_desired : " << std::endl << x_dot_desired << std::endl;
        */
      }
    }
  }
}

}
