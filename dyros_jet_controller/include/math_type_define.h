#ifndef MATH_TYPE_DEFINE_H
#define MATH_TYPE_DEFINE_H


#define RAD2DEG (0.01745329251994329576923690768489)
// constexpr size_t MAX_DOF=50;

#include <Eigen/Dense>

#define GRAVITY 9.80665
#define MAX_DOF 50U
namespace Eigen
{

// Eigen default type definition
#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
  typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix;  \
  typedef Matrix<Type, Size, 1>    Vector##SizeSuffix##TypeSuffix;  \
  typedef Matrix<Type, 1, Size>    RowVector##SizeSuffix##TypeSuffix;

typedef double	rScalar;

EIGEN_MAKE_TYPEDEFS(rScalar, d, 5, 5)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 6, 6)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 7, 7)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 12, 12)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 28, 28)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 30, 30)

typedef Transform<rScalar, 3, Eigen::Isometry> HTransform;
typedef Matrix<rScalar, 6, 3>	Matrix6x3d;
typedef Matrix<rScalar, -1, 1, 0, MAX_DOF, 1> VectorJXd;
typedef Matrix<rScalar, -1, -1, 0, MAX_DOF, MAX_DOF> MatrixJXd;

}

namespace DyrosMath
{

static double cubic(double time,     ///< Current time
             double time_0,   ///< Start time
             double time_f,   ///< End time
             double x_0,      ///< Start state
             double x_f,      ///< End state
             double x_dot_0,  ///< Start state dot
             double x_dot_f   ///< End state dot
             )
{
  double x_t;

  if (time < time_0)
  {
    x_t = x_0;
  }
  else if (time > time_f)
  {
    x_t = x_f;
  }
  else
  {
    double elapsed_time = time - time_0;
    double total_time = time_f - time_0;
    double total_time2 = total_time * total_time;  // pow(t,2)
    double total_time3 = total_time2 * total_time; // pow(t,3)
    double total_x    = x_f - x_0;

    x_t = x_0 + x_dot_0 * elapsed_time

        + (3 * total_x / total_time2
           - 2 * x_dot_0 / total_time2
           - x_dot_f / total_time2)
        * elapsed_time * elapsed_time

        + (-2 * total_x / total_time3 +
           (x_dot_0 + x_dot_f) / total_time3)
        * elapsed_time * elapsed_time * elapsed_time;
  }

  return x_t;
}

static Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation,
                       Eigen::Matrix3d desired_rotation)
{
  Eigen::Vector3d phi;
  Eigen::Vector3d s[3], v[3], w[3];

  for (int i = 0; i < 3; i++) {
    v[i] = current_rotation.block<3, 1>(0, i);
    w[i] = desired_rotation.block<3, 1>(0, i);
    s[i] = v[i].cross(w[i]);
  }
  phi = s[0] + s[1] + s[2];
  phi = -0.5* phi;

  return phi;
}
}

#endif