/* Produced by CVXGEN, 2018-03-22 04:11:37 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.Ai[0])-rhs[1]*(params.Ai[8])-rhs[2]*(params.Ai[16])-rhs[3]*(params.Ai[24])-rhs[4]*(params.Ai[32])-rhs[5]*(params.Ai[40]);
  lhs[1] = -rhs[0]*(params.Ai[1])-rhs[1]*(params.Ai[9])-rhs[2]*(params.Ai[17])-rhs[3]*(params.Ai[25])-rhs[4]*(params.Ai[33])-rhs[5]*(params.Ai[41]);
  lhs[2] = -rhs[0]*(params.Ai[2])-rhs[1]*(params.Ai[10])-rhs[2]*(params.Ai[18])-rhs[3]*(params.Ai[26])-rhs[4]*(params.Ai[34])-rhs[5]*(params.Ai[42]);
  lhs[3] = -rhs[0]*(params.Ai[3])-rhs[1]*(params.Ai[11])-rhs[2]*(params.Ai[19])-rhs[3]*(params.Ai[27])-rhs[4]*(params.Ai[35])-rhs[5]*(params.Ai[43]);
  lhs[4] = -rhs[0]*(params.Ai[4])-rhs[1]*(params.Ai[12])-rhs[2]*(params.Ai[20])-rhs[3]*(params.Ai[28])-rhs[4]*(params.Ai[36])-rhs[5]*(params.Ai[44]);
  lhs[5] = -rhs[0]*(params.Ai[5])-rhs[1]*(params.Ai[13])-rhs[2]*(params.Ai[21])-rhs[3]*(params.Ai[29])-rhs[4]*(params.Ai[37])-rhs[5]*(params.Ai[45]);
  lhs[6] = -rhs[0]*(params.Ai[6])-rhs[1]*(params.Ai[14])-rhs[2]*(params.Ai[22])-rhs[3]*(params.Ai[30])-rhs[4]*(params.Ai[38])-rhs[5]*(params.Ai[46]);
  lhs[7] = -rhs[0]*(params.Ai[7])-rhs[1]*(params.Ai[15])-rhs[2]*(params.Ai[23])-rhs[3]*(params.Ai[31])-rhs[4]*(params.Ai[39])-rhs[5]*(params.Ai[47]);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.Ai[0])-rhs[1]*(params.Ai[1])-rhs[2]*(params.Ai[2])-rhs[3]*(params.Ai[3])-rhs[4]*(params.Ai[4])-rhs[5]*(params.Ai[5])-rhs[6]*(params.Ai[6])-rhs[7]*(params.Ai[7]);
  lhs[1] = -rhs[0]*(params.Ai[8])-rhs[1]*(params.Ai[9])-rhs[2]*(params.Ai[10])-rhs[3]*(params.Ai[11])-rhs[4]*(params.Ai[12])-rhs[5]*(params.Ai[13])-rhs[6]*(params.Ai[14])-rhs[7]*(params.Ai[15]);
  lhs[2] = -rhs[0]*(params.Ai[16])-rhs[1]*(params.Ai[17])-rhs[2]*(params.Ai[18])-rhs[3]*(params.Ai[19])-rhs[4]*(params.Ai[20])-rhs[5]*(params.Ai[21])-rhs[6]*(params.Ai[22])-rhs[7]*(params.Ai[23]);
  lhs[3] = -rhs[0]*(params.Ai[24])-rhs[1]*(params.Ai[25])-rhs[2]*(params.Ai[26])-rhs[3]*(params.Ai[27])-rhs[4]*(params.Ai[28])-rhs[5]*(params.Ai[29])-rhs[6]*(params.Ai[30])-rhs[7]*(params.Ai[31]);
  lhs[4] = -rhs[0]*(params.Ai[32])-rhs[1]*(params.Ai[33])-rhs[2]*(params.Ai[34])-rhs[3]*(params.Ai[35])-rhs[4]*(params.Ai[36])-rhs[5]*(params.Ai[37])-rhs[6]*(params.Ai[38])-rhs[7]*(params.Ai[39]);
  lhs[5] = -rhs[0]*(params.Ai[40])-rhs[1]*(params.Ai[41])-rhs[2]*(params.Ai[42])-rhs[3]*(params.Ai[43])-rhs[4]*(params.Ai[44])-rhs[5]*(params.Ai[45])-rhs[6]*(params.Ai[46])-rhs[7]*(params.Ai[47]);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.Q[0])+rhs[1]*(2*params.Q[6])+rhs[2]*(2*params.Q[12])+rhs[3]*(2*params.Q[18])+rhs[4]*(2*params.Q[24])+rhs[5]*(2*params.Q[30]);
  lhs[1] = rhs[0]*(2*params.Q[1])+rhs[1]*(2*params.Q[7])+rhs[2]*(2*params.Q[13])+rhs[3]*(2*params.Q[19])+rhs[4]*(2*params.Q[25])+rhs[5]*(2*params.Q[31]);
  lhs[2] = rhs[0]*(2*params.Q[2])+rhs[1]*(2*params.Q[8])+rhs[2]*(2*params.Q[14])+rhs[3]*(2*params.Q[20])+rhs[4]*(2*params.Q[26])+rhs[5]*(2*params.Q[32]);
  lhs[3] = rhs[0]*(2*params.Q[3])+rhs[1]*(2*params.Q[9])+rhs[2]*(2*params.Q[15])+rhs[3]*(2*params.Q[21])+rhs[4]*(2*params.Q[27])+rhs[5]*(2*params.Q[33]);
  lhs[4] = rhs[0]*(2*params.Q[4])+rhs[1]*(2*params.Q[10])+rhs[2]*(2*params.Q[16])+rhs[3]*(2*params.Q[22])+rhs[4]*(2*params.Q[28])+rhs[5]*(2*params.Q[34]);
  lhs[5] = rhs[0]*(2*params.Q[5])+rhs[1]*(2*params.Q[11])+rhs[2]*(2*params.Q[17])+rhs[3]*(2*params.Q[23])+rhs[4]*(2*params.Q[29])+rhs[5]*(2*params.Q[35]);
}
void fillq(void) {
  work.q[0] = params.c[0];
  work.q[1] = params.c[1];
  work.q[2] = params.c[2];
  work.q[3] = params.c[3];
  work.q[4] = params.c[4];
  work.q[5] = params.c[5];
}
void fillh(void) {
  work.h[0] = params.bi[0];
  work.h[1] = params.bi[1];
  work.h[2] = params.bi[2];
  work.h[3] = params.bi[3];
  work.h[4] = params.bi[4];
  work.h[5] = params.bi[5];
  work.h[6] = params.bi[6];
  work.h[7] = params.bi[7];
}
void fillb(void) {
}
void pre_ops(void) {
}
