///////////////////////////////////////////////////////////////////////////////
//
//  (C) 2001-2015 Force Dimension
//  All Rights Reserved.
//
///////////////////////////////////////////////////////////////////////////////



#ifndef CMacrosGLH
#define CMacrosGLH



#if defined(WIN32) || defined(WIN64)
#include "windows.h"
#endif
#include "Eigen/Eigen"
#ifdef MACOSX
#include "GLUT/glut.h"
#else
#include "GL/glut.h"
#endif



//  Matrix class used to express position or translation.
//  On the OpenGL side 4x4 matrices are required to perform all
//  geometric transformations. cMatrixGL provides a structure
//  which encapsulates all the necessary functionalities to
//  generate 4x4 OpenGL transformation matrices by passing 3D
//  position vectors and rotation matrices. cMatrixGL also provides
//  OpenGL calls to push, multiply and pop matrices off the OpenGL stack.
//  OpenGL Matrices are COLUMN major.

struct cMatrixGL
{
private:

	double  m[4][4];

public:

	// returns a pointer to the matrix array in memory
	const double* pMatrix() const { return m[0]; }

	// creates OpenGL translation matrix from a position vector passed as parameter
	void set(const Eigen::Vector3d& a_pos) {
     m[0][0] = 1.0;       m[0][1] = 0.0;        m[0][2] = 0.0;        m[0][3] = 0.0;
     m[1][0] = 0.0;       m[1][1] = 1.0;        m[1][2] = 0.0;        m[1][3] = 0.0;
     m[2][0] = 0.0;       m[2][1] = 0.0;        m[2][2] = 1.0;        m[2][3] = 0.0;
     m[3][0] = a_pos(0);  m[3][1] = a_pos(1);   m[3][2] = a_pos(2);   m[3][3] = 1.0;
   }

	// creates OpenGL translation matrix from vector giving translation
	void set(const Eigen::Vector3d& a_pos, const double a_rot[3][3]) {
    m[0][0] = a_rot[0][0];  m[0][1] = a_rot[1][0];  m[0][2] = a_rot[2][0];  m[0][3] = 0.0;
    m[1][0] = a_rot[0][1];  m[1][1] = a_rot[1][1];  m[1][2] = a_rot[2][1];  m[1][3] = 0.0;
    m[2][0] = a_rot[0][2];  m[2][1] = a_rot[1][2];  m[2][2] = a_rot[2][2];  m[2][3] = 0.0;
    m[3][0] = a_pos(0);     m[3][1] = a_pos(1);     m[3][2] = a_pos(2);     m[3][3] = 1.0;
	}

  // creates OpenGL translation matrix from vector giving translation
  void set(const Eigen::Vector3d& a_pos, const Eigen::Matrix3d& a_rot) {
    m[0][0] = a_rot(0,0);  m[0][1] = a_rot(1,0);  m[0][2] = a_rot(2,0);  m[0][3] = 0.0;
    m[1][0] = a_rot(0,1);  m[1][1] = a_rot(1,1);  m[1][2] = a_rot(2,1);  m[1][3] = 0.0;
    m[2][0] = a_rot(0,2);  m[2][1] = a_rot(1,2);  m[2][2] = a_rot(2,2);  m[2][3] = 0.0;
    m[3][0] = a_pos(0);    m[3][1] = a_pos(1);    m[3][2] = a_pos(2);    m[3][3] = 1.0;
  }

  // push current OpenGL on the stack and multiply with this cMatrixGL matrix
	void glMatrixPushMultiply() {
	  glPushMatrix();
	  glMultMatrixd( (const double *)pMatrix() );
	}

	// pop current OpenGL matrix off the stack
	void glMatrixPop() {
	  glPopMatrix();
	}

};



#endif
