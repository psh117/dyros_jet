#include "dyros_jet_controller/quadraticprogram.h"

CQuadraticProgram::CQuadraticProgram()
{
    Initialize();
}
CQuadraticProgram::~CQuadraticProgram()
{
}
void CQuadraticProgram::Initialize()
{
  _bInitialized = false;
  _num_var = 1;
  _num_cons = 1;
}
