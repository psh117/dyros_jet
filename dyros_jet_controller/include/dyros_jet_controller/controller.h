#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "dyros_jet_controller/dyros_jet_model.h"
#include "math_type_define.h"

namespace dyros_jet_controller
{


class Controller
{
public:
  Controller(const unsigned long priority) : priority_(priority) {}
  virtual void compute()=0;

  virtual void updateControlMask(unsigned int *mask)=0;
  virtual void writeDesired(const unsigned int *mask, VectorQd& desired_q)=0;

protected:
  inline void setMask(unsigned int *mask)
  {
    *mask = (*mask | priority_);
  }
  inline void resetMask(unsigned int *mask)
  {
    *mask = (*mask & ~priority_);
  }
  inline bool isAvailable(const unsigned int *mask)
  {
    return (*mask >= priority_ && *mask < priority_ * 2);
  }

private:
  const unsigned long priority_;

};

} // namespace dyros_jet_controller

#endif // CONTROLLER_H
