#ifndef QUADRATICPROGRAM_H
#define QUADRATICPROGRAM_H

#include <iostream>
#include "math_type_define.h"
#include <Eigen/Dense>
#include "qpOASES.hpp"

using namespace Eigen;
using namespace std;
using namespace qpOASES;

class CQuadraticProgram
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  CQuadraticProgram();
  virtual ~CQuadraticProgram();

public:

private:

};


#endif //QUADRATICPROGRAM_H
