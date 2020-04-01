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

<<<<<<< HEAD
  void InitializeProblemSize(const int &num_var, const int &num_cons);
  void QpOption();
  VectorXd SolveQPoases(const int &num_max_iter);
  void UpdateMinProblem(const MatrixXd &H, const VectorXd &g);
  void UpdateSubjectToAx(const MatrixXd &A, const VectorXd &lbA, const VectorXd &ubA);
  void UpdateSubjectToX(const VectorXd &lb, const VectorXd &ub);
  void PrintMinProb();
  void PrintSubjectToAx();
  void PrintSubjectTox();

public:
 
=======
public:
  void Initialize();
>>>>>>> b22719c91c3d101d33a1f8e495aed81daf35be03
private:
  int _num_var;
  int _num_cons;
  bool _bInitialized;

  bool _bInitialized;
  int _num_var, _num_cons;
  SQProblem _QPprob;
  MatrixXd _H;
  MatrixXd _A;
  VectorXd _g;
  VectorXd _lb;
  VectorXd _ub;
  VectorXd _lbA;
  VectorXd _ubA;
  Options _options;

  void Initialize();
};


#endif //QUADRATICPROGRAM_H
