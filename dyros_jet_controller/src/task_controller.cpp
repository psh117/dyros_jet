#include "dyros_jet_controller/task_controller.h"
#include "dyros_jet_controller/dyros_jet_model.h"

namespace dyros_jet_controller
{

static constexpr unsigned int TaskController::PRIORITY;

void TaskController::loop()
{
  // update state

  // compute next q

  //
}
void TaskController::setTarget(DyrosJetModel::EndEffector ee, Eigen::HTransform target)
{
  target_transform_[ee] = target;
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
    else if (i < 6 + 6 + 7)
    {
      index = 2;
    }
    else if (i < 6 + 6 + 7 + 7)
    {
      index = 3;
    }

    if(ee_enabled_[index])
    {
      if(mask[i] < PRIORITY)
      {
        mask[i] = PRIORITY;
      }

    }
  }
}

void TaskController::armComputeLoop()
{
  for(unsigned int i=0; i<2; i++)
  {
    if(ee_enabled_[i+2])
    {

    }
  }
  if (ee_enabled_[DyrosJetModel::EE_LEFT_HAND])
  {

  }
}
void TaskController::legComputeLoop()
{

}
}
