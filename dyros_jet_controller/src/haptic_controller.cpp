#include "dyros_jet_controller/haptic_controller.h"
#include "dyros_jet_controller/dyros_jet_model.h"

namespace dyros_jet_controller
{

constexpr unsigned int HapticController::PRIORITY;

void HapticController::compute()
{
  // update state
  computeCLIK();
  //
}
void HapticController::setTarget(DyrosJetModel::EndEffector ee, Eigen::Isometry3d target, double start_time, double end_time)
{
  start_transform_[ee] = model_.getCurrentTransform(ee);
  target_transform_[ee] = target;
  start_time_[ee] = start_time;
  end_time_[ee] = end_time;
  x_prev_[ee] = start_transform_[ee].translation();
  target_arrived_[ee] = false;
}
void HapticController::setTarget(DyrosJetModel::EndEffector ee, Eigen::Isometry3d target, double duration)
{
  // Trick: When msg->duration is 0.0 it means that haptic buttion is positive edge triggered
  if(duration == 1.0)
  {
      rot_init_ = model_.getCurrentTransform(ee).linear();
  }

  setTarget(ee, target, control_time_, control_time_ + duration);
}
void HapticController::setEnable(DyrosJetModel::EndEffector ee, bool enable)
{
  ee_enabled_[ee] = enable;
}
void HapticController::updateControlMask(unsigned int *mask)
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

void HapticController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
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
void HapticController::computeCLIK()
{
    const double inverse_damping = 1e-4;
    const double phi_gain = 0.5;
    const double kp = 200;
    const double epsilon = 1e-3;

    // Arm
    for(unsigned int i=2; i<4; i++)
    {
        if(ee_enabled_[i])
        {
            // For short names

            const auto &x = model_.getCurrentTransform((DyrosJetModel::EndEffector)(i)).translation();
            const auto &rot = model_.getCurrentTransform((DyrosJetModel::EndEffector)(i)).linear();

            //debug_ << control_time_ << "\t" << x(0) << "\t" << x(1) << "\t" <<x(2) << std::endl;

            const auto &x_target = target_transform_[i].translation();

            Eigen::Vector6d x_dot_desired;
            Eigen::Vector6d x_error;
            x_error.head<3>() = (x_target - x);
            //x_error.tail<3>().setZero();
            x_error.tail<3>() =  - phi_gain * DyrosMath::getPhi(rot, rot_init_);
            x_dot_desired.head<3>() = x_target - x_prev_[i];
            x_dot_desired.tail<3>().setZero();

            x_error = x_error; //* hz_;
            x_dot_desired = x_dot_desired * hz_;
            x_prev_[i] = x_target;

            if (x_error.norm() < epsilon && control_time_ >= end_time_[i]) // target arrived, cubic finished
            {
                target_arrived_[i] = true;
                ee_enabled_[i] = false;
                ROS_INFO("target arrived - %d End effector", i);
                continue;
            }
            const auto &J = model_.getArmJacobian((DyrosJetModel::EndEffector)(i));
            const auto &q = current_q_.segment<7>(model_.joint_start_index_[i]);


            auto J_inverse = J.transpose() *
                    (inverse_damping * Eigen::Matrix6d::Identity() +
                     J * J.transpose()).inverse();


            desired_q_.segment<7>(model_.joint_start_index_[i]) =
                    (J_inverse * (x_dot_desired + x_error * kp)) / hz_ + q;


        }
    }
}

}
